#!/usr/bin/env python3
"""Capture SDRangel's APSK constellation, sweep TX amplitude and restore live video.

Run with SDRangel and tx_dvbs.py already running, using the saved /tmp state:
  python3 -B host/measure_apsk.py --output /absolute/path --seconds 8
No firmware is flashed. Existing playback preload and UDP tee are preserved.
SDRangel is restarted briefly with a drawing-only probe, then without it.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request
from zoneinfo import ZoneInfo

from apsk_centroids import analyze, export, ideal_points, load_pixels, reference_crosses
from measure_spectra import REPO, running, save_json, start_tx, stop_previous, stop_tx

BASE = "http://127.0.0.1:8091/sdrangel"
TX_STATE = Path("/tmp/esp32_datv_tx_current.json")
RX_STATE = Path("/tmp/esp32_datv_receiver_current.json")
LAUNCHER = REPO.parent / "odbiornik/uruchom_sdrangel.sh"


def api(path, method="GET", data=None):
    request = urllib.request.Request(BASE + path, None if data is None else json.dumps(data).encode(),
                                     {"Content-Type": "application/json"}, method=method)
    with urllib.request.urlopen(request, timeout=8) as response:
        return json.load(response)


def log(value):
    print(datetime.datetime.now(ZoneInfo("Europe/Warsaw")).isoformat(timespec="seconds"), value, flush=True)


def arg(command, key, default=None):
    return command[command.index(key) + 1] if key in command else default


def replace_arg(command, key, value):
    command = list(command)
    if key in command:
        command[command.index(key) + 1] = str(value)
    else:
        command += [key, str(value)]
    return command


def stop_receiver(pid, index):
    if not running(pid):
        try:
            os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            pass
        return
    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    if Path(os.fsdecode(cmd[0])).name != "sdrangel":
        raise RuntimeError("Saved receiver PID does not belong to SDRangel")
    try:
        api(f"/deviceset/{index}/device/run", "DELETE")
    except OSError:
        pass
    time.sleep(.6)
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 15
    while running(pid):
        if time.monotonic() > deadline:
            raise RuntimeError("SDRangel did not exit")
        time.sleep(.1)
    # The launcher's pgrep also sees zombies. Reap our own child before
    # starting the next receiver, otherwise the launcher refuses to run.
    try:
        os.waitpid(pid, os.WNOHANG)
    except ChildProcessError:
        pass


def start_receiver(environment, output, device, channel, was_running):
    with output.open("wb") as stream:
        process = subprocess.Popen([str(LAUNCHER)], cwd=LAUNCHER.parent, env=environment,
                                   stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    state = dict(pid=process.pid, log=str(output), launcher=str(LAUNCHER), udp_tee=environment.get("DATV_UDP_TEE", "8882"))
    save_json(RX_STATE, state)
    try:
        return configure_receiver(process, output, device, channel, was_running)
    except BaseException:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise


def configure_receiver(process, output, device, channel, was_running):
    deadline = time.monotonic() + 35
    index = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("SDRangel exited: " + output.read_text(errors="replace")[-1000:])
        try:
            for i, item in enumerate(api("/devicesets")["deviceSets"]):
                if item["samplingDevice"]["hwType"] == "HackRF" and any(c["id"] == "DATVDemod" for c in item.get("channels", [])):
                    index = i
            if index is not None:
                break
        except (OSError, KeyError):
            pass
        time.sleep(.5)
    if index is None:
        raise RuntimeError("SDRangel did not load HackRF/DATV")
    time.sleep(1)
    api(f"/deviceset/{index}/device/run", "DELETE")
    time.sleep(.5)
    api(f"/deviceset/{index}/device/settings", "PATCH", device)
    api(f"/deviceset/{index}/channel/0/settings", "PATCH", channel)
    time.sleep(1)
    if was_running:
        api(f"/deviceset/{index}/device/run", "POST")
    return process, index


def capture_case(name, amp, a, points, mod, fec, index, cases):
    time.sleep(5)  # reacquire after a transmitter or gain change
    first = points.stat().st_size // 4 if points.exists() else 0
    reports = []
    started = time.monotonic()
    while time.monotonic() - started < a.seconds:
        time.sleep(2)
        reports.append(api(f"/deviceset/{index}/channel/0/report"))
    count = points.stat().st_size // 4 - first if points.exists() else 0
    ideal = ideal_points(mod, fec)
    result, z, labels, centers = analyze(load_pixels(points, first=first, count=count), ideal)
    result.update(modulation=mod.upper(), fec=fec, tx_amp=amp, capture_seconds=time.monotonic() - started,
                  first=first, count=count, input_file=str(points), receiver_reports=reports,
                  receiver_settings=api(f"/deviceset/{index}/device/settings"),
                  receiver_reference_crosses=reference_crosses(str(points) + ".refs"))
    export(a.output / name, result, z, ideal, labels, centers)
    cases.append(dict(name=name, amp=amp, samples=result["samples"], payload_gain=result["payload_gain"],
                      rings=result["rings"], centroid_error_after_common_gain_percent=result["centroid_error_after_common_gain_percent"]))
    save_json(a.output / "summary.json", cases)
    log(f"{name}: gain {result['payload_gain']:.5f}; radii {[round(r['measured_radius'], 5) for r in result['rings']]}; gamma {[round(r['measured_ratio_to_inner'], 5) for r in result['rings']]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--amps", default="300,160")
    parser.add_argument("--skip-rx-gain-check", action="store_true")
    a = parser.parse_args()
    a.output = a.output.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    previous_tx, previous_rx = json.loads(TX_STATE.read_text()), json.loads(RX_STATE.read_text())
    original_command = previous_tx["command"]
    mod, fec = arg(original_command, "--mod", "qpsk"), arg(original_command, "--fec", "1/2")
    if mod not in ("16apsk", "32apsk"):
        raise RuntimeError("The current transmitter must use APSK")
    amp = int(arg(original_command, "--amp", 400 if mod == "32apsk" else 420))
    if not running(previous_tx["pid"]):
        raise RuntimeError("The saved transmitter is not running")
    index = next(i for i, item in enumerate(api("/devicesets")["deviceSets"]) if item["samplingDevice"]["hwType"] == "HackRF")
    device = api(f"/deviceset/{index}/device/settings")
    channel = api(f"/deviceset/{index}/channel/0/settings")
    was_running = api(f"/deviceset/{index}/device/run")["state"] == "running"
    old_vars = dict(x.split(b"=", 1) for x in Path(f"/proc/{previous_rx['pid']}/environ").read_bytes().split(b"\0") if b"=" in x)
    environment = dict(os.environ)
    environment["DATV_UDP_TEE"] = os.fsdecode(old_vars.get(b"DATV_UDP_TEE", b"8882"))
    # The launcher itself prepends its playback fix. Keep any other preloads.
    old_preload = os.fsdecode(old_vars.get(b"LD_PRELOAD", b""))
    other_preloads = [p for p in old_preload.split(":") if p and not p.endswith("libsdrangel_datv_stream_fix.so")]
    environment["LD_PRELOAD"] = ":".join(other_preloads)
    snapshot = dict(tx=previous_tx, rx=previous_rx, device=device, channel=channel, running=was_running)
    save_json(a.output / "before.json", snapshot)
    probe = a.output / "probe.so"
    subprocess.run(["g++", "-std=c++17", "-O2", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror",
                    str(REPO / "host/apsk_probe.cpp"), "-ldl", "-o", str(probe)], check=True)
    points = a.output / "points.s16"
    probe_env = dict(environment, LD_PRELOAD=":".join([str(probe)] + other_preloads), DATV_APSK_CAPTURE=str(points))
    receiver, tx, rx_stopped, tx_stopped = None, None, False, False
    cases = []
    try:
        stop_receiver(previous_rx["pid"], index)
        rx_stopped = True
        receiver, index = start_receiver(probe_env, a.output / "receiver_probe.log", device, channel, True)
        capture_case("baseline", amp, a, points, mod, fec, index, cases)
        if not a.skip_rx_gain_check:
            low_vga = max(0, int(device["hackRFInputSettings"]["vgaGain"]) - 8)
            api(f"/deviceset/{index}/device/settings", "PATCH", dict(deviceHwType="HackRF", direction=0, hackRFInputSettings=dict(vgaGain=low_vga)))
            try:
                capture_case("rx_gain_minus8", amp, a, points, mod, fec, index, cases)
            finally:
                api(f"/deviceset/{index}/device/settings", "PATCH", device)
        for new_amp in map(int, a.amps.split(",")):
            if not tx_stopped:
                stop_previous(previous_tx)
                tx_stopped = True
            else:
                stop_tx(tx)
                tx = None
            command = replace_arg(original_command, "--amp", new_amp)
            tx, ack, _ = start_tx(command, a.output / f"tx_amp{new_amp}.log")
            save_json(TX_STATE, dict(previous_tx, pid=tx.pid, command=command, log=str(a.output / f"tx_amp{new_amp}.log")))
            log(ack)
            capture_case(f"amp{new_amp}", new_amp, a, points, mod, fec, index, cases)
    finally:
        try:
            if tx_stopped:
                stop_tx(tx)
                tx, ack, _ = start_tx(original_command, a.output / "restored_video.log")
                save_json(TX_STATE, dict(previous_tx, pid=tx.pid, command=original_command, log=str(a.output / "restored_video.log")))
                log("Restored TX: " + ack)
        finally:
            if rx_stopped:
                current_pid = receiver.pid if receiver is not None else json.loads(RX_STATE.read_text())["pid"]
                if current_pid != previous_rx["pid"]:
                    stop_receiver(current_pid, index)
                receiver, index = start_receiver(environment, a.output / "restored_receiver.log", device, channel, was_running)
                log(f"Restored receiver PID {receiver.pid} without the measurement probe")


if __name__ == "__main__":
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    main()
