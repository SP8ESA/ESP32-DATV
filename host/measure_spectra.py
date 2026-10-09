#!/usr/bin/env python3
"""Measure the QO-100 symbol-rate set with a tinySA Ultra+ and restore live TX.

Run from the repository: python3 -B host/measure_spectra.py --output DIR
The default is three power-averaged sweeps, with three-bin display smoothing.
Raw sweeps, averaged CSVs, figures, TX logs and settings remain in DIR.
No firmware is flashed and no receiver settings are changed.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

os.environ.setdefault("MPLCONFIGDIR", "/tmp/esp32_datv_matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import serial

REPO = Path(__file__).resolve().parents[1]
ESP_PORT = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_10:00:3B:DE:F9:64-if00"
SA_PORT = "/dev/serial/by-id/usb-tinysa.org_tinySA4_400-if00"
RATES = (1000000, 500000, 333000, 250000, 125000, 66000, 33000)
FEC = {"qpsk": "1/2", "8psk": "3/5", "16apsk": "2/3", "32apsk": "3/4"}
CENTRE = 2370e6
ROW = re.compile(r"^(\d{9,10}) (-?\d+\.\d+e[+-]\d+) ")


def now():
    return datetime.datetime.now(ZoneInfo("Europe/Warsaw")).isoformat(timespec="seconds")


def log(message):
    print(now(), message, flush=True)


def save_json(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2) + "\n")
    temp.replace(path)


def smooth(dbm):
    return 10 * np.log10(np.convolve(np.pad(10 ** (dbm / 10), 1, mode="edge"), np.ones(3) / 3, mode="valid"))


class TinySA:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=.2)

    def cmd(self, command, timeout=45):
        self.s.reset_input_buffer()
        self.s.write((command + "\r").encode())
        deadline, buf = time.monotonic() + timeout, b""
        while time.monotonic() < deadline:
            buf += self.s.read(65536)
            if buf.endswith(b"ch> "):
                return buf.decode(errors="replace")
        raise RuntimeError(f"tinySA command timed out: {command}")

    def scan(self, f0, f1, points=450):
        for _ in range(3):
            answer = self.cmd(f"scan {int(f0)} {int(f1)} {points} 3")
            # tinySA's formatter can round a mantissa to 10 and emit ':'
            # ('0' + 10), e.g. -:.000000e+00 for exactly -10 dBm.
            answer = re.sub(r"([+-]?):(\.\d+e[+-]\d+)", r"\g<1>10\2", answer)
            rows = [m.groups() for m in map(ROW.match, answer.splitlines()) if m]
            if len(rows) == points:
                return np.array([int(f) for f, y in rows], float), np.array([float(y) for f, y in rows])
        raise RuntimeError("tinySA returned incomplete scans three times")


def running(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


def stop_previous(state):
    pid = state["pid"]
    if not running(pid):
        raise RuntimeError("The saved live transmitter is no longer running")
    actual = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")[:-1]
    if actual != [os.fsencode(s) for s in state["command"]]:
        raise RuntimeError("Saved PID belongs to a different process")
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 25
    while running(pid):
        if time.monotonic() >= deadline:
            raise RuntimeError("Previous transmitter did not stop")
        time.sleep(.1)


def stop_tx(tx):
    if tx is not None and tx.poll() is None:
        tx.send_signal(signal.SIGINT)
        tx.wait(timeout=25)


def start_tx(command, path):
    with path.open("wb") as stream:
        tx = subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
                              stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            text = path.read_text(errors="replace")
            if tx.poll() is not None:
                raise RuntimeError("TX exited: " + text[-1500:])
            ack = next((s for s in text.splitlines() if s.startswith("OK QPSKT")), None)
            fills = re.findall(r"ESP buffer (\d+)\.\.(\d+) pairs", text)
            if ack and fills and int(fills[-1][1]) > 500:
                return tx, ack, dict(zip(ack.split()[2::2], ack.split()[3::2]))
            time.sleep(.2)
        raise RuntimeError("TX did not start streaming: " + text[-1500:])
    except BaseException:
        stop_tx(tx)
        raise


def acquire(sa, tx, rate, half_span, rbw, passes, prefix, align=False):
    start = time.monotonic()
    sa.cmd(f"rbw {rbw:g}")
    response = sa.cmd("rbw")
    segments = math.ceil(2 * half_span / (449 * rbw * 1000 * .75))
    edges = np.linspace(CENTRE - half_span, CENTRE + half_span, segments + 1)
    scans, centres, grid = [], [], None
    for sweep in range(passes):
        if tx.poll() is not None:
            raise RuntimeError("TX stopped during measurement")
        fs, ys = [], []
        for k in range(segments):
            f, y = sa.scan(edges[k], edges[k + 1])
            fs.append(f[1:] if k else f)
            ys.append(y[1:] if k else y)
        f, y = np.concatenate(fs), np.concatenate(ys)
        if grid is not None and not np.array_equal(grid, f):
            raise RuntimeError("Frequency grid changed between sweeps")
        grid = f
        p = 10 ** (y / 10)
        mask = abs(f - CENTRE) < max(rate * 1.2, rbw * 3000)
        weights = np.maximum(p[mask] - np.percentile(p, 20), 0)
        if not weights.sum():
            raise RuntimeError("No signal in the expected channel")
        centres.append(float(np.sum(f[mask] * weights) / weights.sum()))
        scans.append(y)
        log(f"{prefix.name}: sweep {sweep + 1}/{passes}, {segments} segments")
    raw = np.array(scans)
    reference = float(np.median(centres))
    powers = 10 ** (raw / 10)
    if align:
        powers = np.array([np.interp(grid + c - reference, grid, row) for c, row in zip(centres, powers)])
    mean = 10 * np.log10(powers.mean(axis=0))
    filtered = smooth(mean)
    central = abs(grid - reference) <= max(rate, rbw * 2000)
    top = float(filtered[central].max())
    max_step = float(np.diff(grid).max())
    if max_step > rbw * 1000 + 1:
        raise RuntimeError("Frequency step exceeded requested RBW")
    np.savez_compressed(prefix.with_suffix(".npz"), frequency_hz=grid, sweeps_dbm=raw,
                        centres_hz=np.array(centres), mean_dbm=mean)
    info = dict(rbw_khz=rbw, rbw_response=response, passes=passes, segments_per_pass=segments,
                points=len(grid), max_step_hz=max_step, centroid_alignment=align,
                reference_hz=reference, centroid_drift_hz=float(np.ptp(centres)),
                signal_top_dbm=top, raw_file=prefix.with_suffix(".npz").name,
                duration_seconds=round(time.monotonic() - start, 2))
    return grid, mean, info


def plot_and_export(output, mod, baud, values, wide, zoom, passes):
    suffix = "1MBd" if baud == 1000000 else f"{baud // 1000}kBd"
    png = f"spectrum_{'' if mod == 'qpsk' else mod.upper() + '_'}{suffix}.png"
    stem = f"{mod}_{suffix}"
    reference = zoom[2]["reference_hz"]
    traces = []
    for kind, (f, y, info) in zip(("wide", "zoom"), (wide, zoom)):
        relative = y - info["signal_top_dbm"]
        csv_path = output / f"{stem}_{kind}.csv"
        np.savetxt(csv_path, np.column_stack((f, y, relative)), delimiter=",", fmt=("%.0f", "%.5f", "%.5f"),
                   header="frequency_hz,mean_dbm,relative_db", comments="")
        traces.append(((f - reference) / 1e6, smooth(relative)))
    fw, yw = traces[0]
    fz, yz = traces[1]
    dac = float(values["OUT"])
    half_window = max(.10e6, 1.4 * baud)
    floor_mask = abs(fw) > max(.2, 2 * baud / 1e6)
    for k in range(1, math.ceil(15e6 / dac) + 1):
        floor_mask &= abs(abs(fw) - k * dac / 1e6) > half_window / 1e6
    floor = float(np.percentile(yw[floor_mask], 20))
    aliases = []
    for sign in (-1, 1):
        mask = abs(fw - sign * dac / 1e6) < half_window / 1e6
        i = int(np.argmax(np.where(mask, yw, -np.inf)))
        aliases.append(dict(frequency_offset_mhz=float(fw[i]), level_db=float(yw[i]),
                            near_floor=bool(yw[i] - floor < 6)))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.0), dpi=150)
    fig.patch.set_facecolor("#fdfdfb")
    rate_label = "1 MS/s" if baud == 1000000 else f"{baud // 1000} kS/s"
    for ax in axes:
        ax.set_facecolor("#fdfdfb")
        ax.grid(color="#e4e4e1", linewidth=.8)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylabel("dB relative to signal top")
        ax.set_ylim(-70, 5)
    axes[0].plot(fw, yw, color="#2a78d6", linewidth=1.3)
    axes[0].set_xlim(-15, 15)
    axes[0].set_xlabel("Offset from signal centre (MHz)")
    axes[0].set_title(f"{mod.upper()}, {rate_label} — 30 MHz / RBW 30 kHz", loc="left", fontsize=12)
    for sign, alias in zip((-1, 1), aliases):
        note = f"alias {alias['level_db']:.0f} dB" + (" (near floor)" if alias["near_floor"] else "")
        xp, yp = alias["frequency_offset_mhz"], alias["level_db"]
        axes[0].annotate(note, (xp, yp), (xp + sign * 2.1, min(yp + 16, -10)), ha="center", fontsize=9,
                         arrowprops=dict(arrowstyle="-", color="#333333", linewidth=.7))
    use_khz = 2 * zoom[2]["max_step_hz"] * zoom[2]["points"] < 2e6
    scale = 1000 if use_khz else 1
    unit = "kHz" if use_khz else "MHz"
    axes[1].plot(fz * scale, yz, color="#2a78d6", linewidth=1.3)
    axes[1].set_xlim(fz.min() * scale, fz.max() * scale)
    axes[1].set_xlabel(f"Offset from signal centre ({unit})")
    span = (zoom[0][-1] - zoom[0][0]) / (1000 if use_khz else 1e6)
    axes[1].set_title(f"{mod.upper()}, {rate_label} — {span:g} {unit} / RBW {zoom[2]['rbw_khz']:g} kHz", loc="left", fontsize=12)
    fig.text(.5, .025, f"tinySA Ultra+ · 2370 MHz · AMP {values.get('AMP', '300')} · {passes} sweeps averaged in power · 3-bin smoothing · "
             f"DAC {dac / 1e6:g} MS/s / {values['SPS']} SPS · {now()[:10]}", ha="center", fontsize=9, color="#444444")
    fig.tight_layout(rect=(0, .05, 1, 1), w_pad=2.2)
    fig.savefig(output / png, facecolor=fig.get_facecolor())
    plt.close(fig)
    return dict(figure=png, csv_files=[f"{stem}_wide.csv", f"{stem}_zoom.csv"],
                aliases=aliases, wide_floor_db=floor, reference_hz=reference)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--state", type=Path, default=Path("/tmp/esp32_datv_tx_current.json"))
    parser.add_argument("--passes", type=int, default=3)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--sa-port", default=SA_PORT)
    parser.add_argument("--modulations", nargs="+", choices=tuple(FEC), default=["qpsk", "8psk", "16apsk"])
    parser.add_argument("--rates", nargs="+", type=int, choices=RATES)
    parser.add_argument("--amp", type=int, default=300)
    parser.add_argument("--source", choices=("null", "video"), default="null")
    parser.add_argument("--pilots", action="store_true")
    parser.add_argument("--apsk-pl", choices=("outer", "unit"), default="unit")
    args = parser.parse_args()
    if not 1 <= args.passes <= 30:
        parser.error("--passes must be 1..30")
    if not 1 <= args.amp <= 480:
        parser.error("--amp must be 1..480")
    if "32apsk" in args.modulations and args.rates and any(rate > 250000 for rate in args.rates):
        parser.error("32APSK supports up to 250000 symbols/s")
    jobs = [(mod, baud) for mod in dict.fromkeys(args.modulations)
            for baud in dict.fromkeys(args.rates or RATES) if mod != "32apsk" or baud <= 250000]
    settings = dict(modulations=list(dict.fromkeys(args.modulations)),
                    symbol_rates_baud=list(dict.fromkeys(args.rates or RATES)),
                    amp=args.amp, source="live video" if args.source == "video" else "scrambled null TS packets",
                    pilots=args.pilots, apsk_pl=args.apsk_pl)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    previous = json.loads(args.state.read_text())
    save_json(output / "previous_video.json", previous)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    meta_path = output / "measurement.json"
    if args.resume and meta_path.exists():
        metadata = json.loads(meta_path.read_text())
        if metadata["firmware_commit"] != commit or metadata["passes"] != args.passes:
            raise RuntimeError("Cannot resume with different firmware or averaging")
        if any(metadata.get(key, value) != value for key, value in settings.items()):
            raise RuntimeError("Cannot resume with different modulation, source or transmitter settings")
    else:
        metadata = dict(started_at=now(), firmware_commit=commit, frequency_mhz=2370,
                        **settings, ppm=12, analyzer_internal_attenuation_db=10,
                        passes=args.passes, smoothing_bins=3, normalization="each panel relative to its smoothed central signal top",
                        bandplan="https://wiki.batc.org.uk/QO-100_WB_Bandplan", rates=[])
    metadata["worker_pid"] = os.getpid()
    image = REPO / "firmware/build/esp32_datv.bin"
    if image.exists():
        digest = hashlib.sha256(image.read_bytes()).hexdigest()
        if args.resume and metadata.get("firmware_sha256", digest) != digest:
            raise RuntimeError("Cannot resume with a different firmware image")
        metadata["firmware_sha256"] = digest
    sa = TinySA(args.sa_port)
    tx, stopped = None, False
    try:
        metadata["instrument"] = sa.cmd("version")
        metadata["instrument_previous_attenuation"] = sa.cmd("attenuate")
        metadata["instrument_previous_calculation"] = sa.cmd("calc")
        metadata["instrument_calculation"] = sa.cmd("calc off")
        metadata["attenuation_response"] = sa.cmd("attenuate 10")
        save_json(meta_path, metadata)
        stop_previous(previous)
        stopped = True
        finished = {(r["modulation"], r["requested_baud"]) for r in metadata["rates"]}
        for mod in dict.fromkeys(args.modulations):
            fec = FEC[mod]
            for _, baud in (job for job in jobs if job[0] == mod):
                if (mod, baud) in finished:
                    continue
                stem = f"{mod}_{'1MBd' if baud == 1000000 else str(baud // 1000) + 'kBd'}"
                log(f"{len(metadata['rates']) + 1}/{len(jobs)}: {mod.upper()} {baud} Bd")
                path = output / f"tx_{stem}.log"
                command = ["/usr/bin/python3", "-u", "-B", "host/tx_dvbs.py", "--freq", "2370.000", "--baud", str(baud),
                           "--mod", mod, "--fec", fec, "--frame", "normal", "--amp", str(args.amp), "--ppm", "12", "--port", ESP_PORT]
                if args.source == "null":
                    command.append("--null")
                if mod != "qpsk":
                    command.append("--dvbs2")
                    if args.pilots:
                        command.append("--pilots")
                if mod == "32apsk":
                    command += ["--apsk-pl", args.apsk_pl]
                tx, ack, values = start_tx(command, path)
                if values["MOD"] != mod.upper() or float(values["BAUD"]) != baud or values["AMP"] != str(args.amp):
                    raise RuntimeError("Unexpected TX mode: " + ack)
                log(ack)
                rbw = 10 if baud >= 333000 else 3 if baud >= 125000 else 1
                wide = acquire(sa, tx, baud, 15e6, 30, args.passes, output / f"{stem}_wide")
                zoom = acquire(sa, tx, baud, 3 * baud, rbw, args.passes, output / f"{stem}_zoom", align=True)
                stop_tx(tx)
                tx = None
                text = path.read_text(errors="replace")
                summary = next((s for s in text.splitlines() if "TX END" in s), None)
                if summary is None:
                    raise RuntimeError("TX did not finish cleanly: " + text[-1500:])
                result = plot_and_export(output, mod, baud, values, wide, zoom, args.passes)
                result.update(modulation=mod, standard="DVB-S" if mod == "qpsk" else "DVB-S2", fec=fec, frame=None if mod == "qpsk" else "normal",
                              pilots=args.pilots if mod != "qpsk" else False, apsk_pl=args.apsk_pl if mod == "32apsk" else None,
                              amp=args.amp, requested_baud=baud, actual_baud=float(values["BAUD"]), dac_hz=float(values["OUT"]), sps=int(values["SPS"]),
                              period=int(values["PERIOD"]), command=command, acknowledgement=ack, tx_summary=summary,
                              buffer_reports=[list(map(int, row)) for row in re.findall(r"ESP buffer (\d+)\.\.(\d+) pairs", text)],
                              wide=wide[2], zoom=zoom[2])
                metadata["rates"].append(result)
                save_json(meta_path, metadata)
                log(f"Saved {result['figure']}, first aliases {[round(a['level_db'], 1) for a in result['aliases']]}")
        metadata["completed_at"] = now()
        save_json(meta_path, metadata)
    finally:
        try:
            stop_tx(tx)
        finally:
            sa.s.close()
            if stopped:
                log("Restoring previous live video transmitter")
                path = output / "restored_video.log"
                restored_tx, ack, values = start_tx(previous["command"], path)
                restored = dict(previous, pid=restored_tx.pid, log=str(path))
                save_json(args.state, restored)
                save_json(output / "restored_video.json", restored)
                log(f"Restored PID {restored_tx.pid}: {ack}")


if __name__ == "__main__":
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    main()
