#!/usr/bin/env python3
"""DATV transmitter: encodes an MPEG transport stream to DVB-S symbols and streams them to the ESP32-DATV firmware.

The ESP does the root-raised-cosine filtering and the I/Q DAC output; this script does everything before that
(see dvbs.py) and keeps the ESP's symbol buffer filled.

Transport stream sources (default: the demo film in ../media, looped, encoded on the fly by ffmpeg):
  --film FILE       any video file ffmpeg can read, encoded to H.264 + MP2 sized for the channel capacity
  --test            ffmpeg test pattern + 800 Hz tone
  --ts FILE         a ready transport stream (looped); it must have a constant bit rate (null-padded)
  --ts -            transport stream on stdin, e.g.
                      ffmpeg -re -i film.mp4 ... -f mpegts -muxrate 880000 - | python3 tx_dvbs.py --ts -
  --camera DEVICE   V4L2 camera; --camera-size/--camera-fps/--camera-format select capture parameters
  --null            only null packets: a valid but empty multiplex (a good receiver lock test)
  --cw              no DVB-S at all: a constant symbol, i.e. an unmodulated carrier on the centre frequency (for tuning and level checks)
When the source cannot keep up the script pads with null packets, so symbols never stop.

  python3 tx_dvbs.py --freq 2402.000 --baud 1000000 --fec 1/2
  Receiver: centre = --freq, symbol rate 1000 kS/s, FEC 1/2 (or auto). No lock: try --invert or --swap-iq.

DVB-S2 instead of DVB-S (QPSK or 8PSK, normal or short frames, pilots optional):
  python3 tx_dvbs.py --freq 2402.000 --baud 500000 --dvbs2 --fec 2/3 [--frame short] [--pilots]
  python3 tx_dvbs.py --freq 2402.000 --baud 500000 --dvbs2 --mod 8psk --fec 2/3     (8PSK: 200..400 kBd clean, up to 500 kBd, code rates 3/5 .. 9/10)
  python3 tx_dvbs.py --freq 2402.000 --baud 1000000 --dvbs2 --mod 8psk --fec 3/5    (8PSK at 1 MBd: 3 bits per symbol on the USB link, 375 kB/s)
  python3 tx_dvbs.py --freq 2402.000 --baud 400000 --dvbs2 --mod 16apsk --fec 2/3   (16APSK: 2..500 kBd, code rates 2/3 .. 9/10; needs an SNR of 12..16 dB at the receiver)
  Receiver: DVB-S2, QPSK, 8PSK or 16APSK, the same symbol rate, code rate, roll-off 0.35.

Amateur radio use only, within the limits of your licence. The firmware refuses frequencies outside 2300..2450 MHz.
"""
import argparse
import json
import math
import signal
from dataclasses import dataclass
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dvbs  # noqa: E402
import dvbs2  # noqa: E402
import esp_link  # noqa: E402
from tx_media import build_media_command, encoding_settings, validate_source

CPU_HZ = 160_000_000
P8_MIN_PERIOD = 75                 # floor interval, including fractional-deadline overhead in the C loop
A16_MIN_PERIOD = 80                # measured margin for symbol, sample-clock and USB work
A32_MIN_PERIOD = 100               # slower 32APSK C path; 250 kS/s has a dedicated 8 MS/s loop
A32_MAX_BAUD = 250_000             # one USB byte per symbol: the link carries about 260 kB/s
DEMO_FILM = os.path.join(HERE, "..", "media", "sintel_trailer.mp4")


class TsSource:
    """Delivers 188-byte transport stream packets; take(n) pads with null packets when the source runs dry."""

    def __init__(self, kind, arg, cap, width=640, video_k=0, output_fps=0,
                 camera_size="640x480", camera_fps=30, camera_format="auto",
                 audio_source="none", audio_device="default"):
        self.q = queue.Queue(maxsize=400)
        self.kind, self.arg = kind, arg
        self.nulls = self.pkts = 0
        self.proc = None
        self.stop_event = threading.Event()
        self.error = None
        self.reader = None
        self.input = None
        if kind == "null":
            return
        validate_source(kind, arg, camera_size, camera_fps, camera_format, audio_source)
        command = build_media_command(kind, arg, cap, width, video_k, output_fps,
                                      camera_size, camera_fps, camera_format, audio_source, audio_device)
        if command:
            self.proc = subprocess.Popen(command, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, start_new_session=True)
            self.input = self.proc.stdout
            if kind != "ts":
                v = encoding_settings(cap, width, video_k, output_fps)
                print(f"ffmpeg: {v.width}x video {v.video_bps / 1000:.0f} kb/s at {v.fps:g} fps + audio {v.audio_k} kb/s in a {v.mux / 1000:.0f} kb/s multiplex (channel capacity {cap / 1000:.0f} kb/s)")
        elif arg == "-":
            self.input = sys.stdin.buffer
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        buf = b""
        try:
            while not self.stop_event.is_set():
                if self.input is None:
                    with open(self.arg, "rb") as f:
                        while not self.stop_event.is_set():
                            data = f.read(188 * 64)
                            if not data:
                                break
                            buf = self._push(buf + data)
                else:
                    data = self.input.read(188 * 16)
                    if not data:
                        if not self.stop_event.is_set():
                            self.error = "Source stream ended; check the FFmpeg messages above"
                        return
                    buf = self._push(buf + data)
        except (OSError, ValueError) as e:
            if not self.stop_event.is_set():
                self.error = str(e)

    def _push(self, buf):
        while len(buf) >= 188 * 2 and not self.stop_event.is_set():
            if buf[0] != 0x47 or buf[188] != 0x47:
                i = buf.find(b"\x47", 1)
                buf = buf[i:] if i > 0 else b""
                continue
            n = len(buf) // 188
            for k in range(n):
                while not self.stop_event.is_set():
                    try:
                        self.q.put(buf[188 * k:188 * k + 188], timeout=.1)
                        break
                    except queue.Full:
                        continue
            buf = buf[188 * n:]
        return buf

    def check(self):
        if self.error:
            raise RuntimeError(self.error)
        if self.proc is not None and self.proc.poll() is not None:
            raise RuntimeError(f"FFmpeg stopped (exit {self.proc.returncode}); check the source and log")

    def take(self, n):
        out = []
        for _ in range(n):
            try:
                out.append(self.q.get_nowait())
                self.pkts += 1
            except queue.Empty:
                out.append(dvbs.NULL_PACKET)
                self.nulls += 1
        return b"".join(out)

    def close(self):
        self.stop_event.set()
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=2)
            self.proc.stdout.close()
        if self.reader is not None:
            self.reader.join(timeout=.5)


def auto_sps(baud):
    """Samples per symbol. The firmware has hand-scheduled loops that update the DAC every 20 CPU cycles (8 MS/s) or 24 cycles
    (6.67 MS/s): sps = 8 at exactly 1 MBd, and for 16..232 samples per symbol the one of the two periods that gets the symbol rate
    closest (within 1 %); other rates use the C loops at up to 4 MS/s. The firmware derives the period from baud * sps the same way."""
    if baud == 1_000_000:
        return 8
    best = None
    for period in (20, 24):
        sps = round(CPU_HZ / (period * baud))
        if 16 <= sps <= 232:
            err = abs(CPU_HZ / (period * sps) / baud - 1)
            if err <= 0.01 and (best is None or err < best[0] - 1e-9):
                best = (err, sps)
    if best:
        return best[1]
    return 16 if baud * 16 <= 4_000_000 else 8 if baud * 8 <= 4_000_000 else 4


def auto_sps_16apsk(baud):
    """Use 8 SPS at 1 MBd, 16..24 SPS near 8 MS/s, or a 4..24-SPS rational C clock."""
    if baud == 1_000_000:
        return 8
    if not 2000 <= baud <= 500_000:
        raise SystemExit("16APSK: --baud must be 2000 .. 500000 Bd or 1000000 Bd")
    sps = round(CPU_HZ / (20 * baud))
    if 16 <= sps <= 24 and abs(CPU_HZ / (20 * sps) / baud - 1) <= 0.01:
        return sps
    for sps in range(24, 3, -1):
        if CPU_HZ // (baud * sps) >= A16_MIN_PERIOD:
            return sps
    raise SystemExit("16APSK: no sample rate fits the firmware timing budget")


def auto_sps_32apsk(baud):
    """250 kS/s uses 32 SPS and an 8 MS/s assembly loop. Other rates use the
    six-tap C loop with 4..64 SPS and fractional deadlines, at most 1.6 MS/s."""
    if not 2000 <= baud <= A32_MAX_BAUD:
        raise SystemExit(f"32APSK: --baud must be 2000 .. {A32_MAX_BAUD} Bd (one USB byte per symbol)")
    if baud == 250000:
        return 8000000 // baud
    for sps in range(64, 3, -1):
        if CPU_HZ // (baud * sps) >= A32_MIN_PERIOD:
            return sps
    raise SystemExit("32APSK: no sample rate fits the firmware timing budget")


def auto_sps_8psk(baud):
    """8PSK runs at 8 MS/s in the assembly loops (lutg_psk8.S: 16..64 samples per symbol, i.e. 125..500 kBd, the rate within 1 %; lutg_p8s8.S: 1 MBd, 8 samples
    per symbol, 3 bits per symbol on the USB link). Below 125 kBd the tables of 1024 x S bytes would not fit at 8 MS/s: a C loop with a slower DAC (a store every
    75 or more cycles) takes S = 16..64 samples per symbol. Fractional cycle deadlines give the requested average rate."""
    if baud == 1_000_000:
        return 8
    sps = round(CPU_HZ / (20 * baud))
    if 16 <= sps <= 64 and abs(CPU_HZ / (20 * sps) / baud - 1) <= 0.01:
        return sps
    for sps in range(64, 15, -1):
        if CPU_HZ // (baud * sps) >= P8_MIN_PERIOD:
            return sps
    raise SystemExit(f"8PSK: --baud must be 1000000, {CPU_HZ // 20 // 64} .. {CPU_HZ // 20 // 16} Bd (8 MS/s) or below that down to about 10000 Bd (slower DAC, a C loop), e.g. 1000000, 500000, 250000, 125000, 66000, 33000")


def output_baud(baud, sps, modulation):
    """Match sample-clock averaging in the PSK/APSK C loops and symbol-clock averaging in the generic assembly loops."""
    sample_hz = baud * sps
    period = (CPU_HZ + sample_hz // 2) // sample_hz
    if modulation == "8psk" and period != 20 and 16 <= sps <= 64 and CPU_HZ // sample_hz >= P8_MIN_PERIOD:
        return float(baud)
    if modulation == "16apsk" and period != 20 and 4 <= sps <= 24 and CPU_HZ // sample_hz >= A16_MIN_PERIOD:
        return float(baud)
    if modulation == "32apsk" and 4 <= sps <= 64 and CPU_HZ // sample_hz >= A32_MIN_PERIOD:
        return float(baud)
    if modulation == "32apsk" and (baud, sps) in ((250000, 32),):
        return float(baud)
    generic = (modulation == "8psk" and 16 <= sps <= 64 and period == 20) or (modulation == "qpsk" and 16 <= sps <= 232 and period in (20, 24)) or (modulation == "16apsk" and 16 <= sps <= 24 and period == 20)
    if generic and CPU_HZ // baud == period * sps:
        return float(baud)
    return CPU_HZ / (period * sps)


def load_cal(path):
    """DC and I/Q trims; the default file contains neutral corrections."""
    if not path or not os.path.exists(path):
        return [0.0, 0.0], 1.0, 0.0
    c = json.load(open(path))
    print(f"calibration {os.path.basename(path)}: DC {c['dc']} codes, Q gain {c['iq_gain']}, Q phase {c['iq_phase_deg']} deg")
    return c["dc"], c["iq_gain"], c["iq_phase_deg"]


def feed_symbols(ready, stop_enc, src, enc, cw_byte=None):
    """Encode ahead of USB writes, retaining each block until the queue accepts it."""
    while not stop_enc.is_set():
        data = bytes([cw_byte]) * 4096 if cw_byte is not None else enc.encode(src.take(8))
        while not stop_enc.is_set():
            try:
                ready.put(data, timeout=0.1)
                break
            except queue.Full:
                continue


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, default=2402.000, help="centre of the spectrum [MHz] (13 cm band)")
    ap.add_argument("--baud", type=int, default=1000000, help="symbol rate [Bd], 2000..1000000 (e.g. 33000 for narrow-band DATV)")
    ap.add_argument("--fec", default="1/2", help="code rate: DVB-S 1/2 2/3 3/4 5/6 7/8; with --dvbs2 1/4 1/3 2/5 1/2 3/5 2/3 3/4 4/5 5/6 8/9 9/10 (short frames: no 9/10)")
    ap.add_argument("--dvbs2", action="store_true", help="DVB-S2 instead of DVB-S")
    ap.add_argument("--mod", default="qpsk", choices=("qpsk", "8psk", "16apsk", "32apsk"),
                    help="DVB-S2 modulation (8PSK: code rates 3/5 2/3 3/4 5/6 8/9 9/10, 10..500 kBd and 1 MBd; 16APSK: code rates 2/3 3/4 4/5 5/6 8/9 9/10, 2..500 kBd and 1 MBd; "
                         "32APSK: code rates 3/4 4/5 5/6 8/9 9/10, 2..250 kBd)")
    ap.add_argument("--frame", default="normal", choices=("normal", "short"), help="DVB-S2 FECFRAME size (normal 64800 bits, short 16200)")
    ap.add_argument("--pilots", action="store_true", help="DVB-S2 pilots (36 symbols after every 16 slots)")
    ap.add_argument("--apsk-pl", choices=("outer", "unit"), default="outer", help="32APSK PL symbol radius: outer (default, preserves outer-radius-one normalization) or unit (E=1 payload, compatible with SDRangel)")
    ap.add_argument("--sps", type=int, default=0, help="DAC samples per symbol (32APSK: 4..64; 16APSK: 4..24; QPSK: 4/8/16 or 16..232; 8PSK: 8 or 16..64); 0 = automatic: the hand-scheduled 8 / 6.67 MS/s loops when they fit the symbol rate, else the most that keeps the output at or below 4 MS/s")
    ap.add_argument("--ifm", type=int, default=0, help="centre = LO + ifm * baud (puts the LO leakage outside the signal)")
    ap.add_argument("--amp", type=int, default=0, help="peak amplitude in DAC codes (1..480); 0 = 300 for QPSK, 400 for 32APSK, 420 for 8PSK/16APSK")
    ap.add_argument("--pa-enable", action="store_true", help="assert GPIO3 during TX to enable an external PA")
    ap.add_argument("--target", type=int, default=0, help="ESP buffer fill [pairs of 2 bytes]: default 3000 for QPSK, 2500 for 16APSK/slow 32APSK, 3500 for 32APSK at 250 kS/s; at 1 MBd, 6000 for 8PSK or 7000 for 16APSK")
    ap.add_argument("--width", type=int, default=640, help="--film: picture width")
    ap.add_argument("--video-k", type=int, default=0, help="--film/--test: video bit rate [kb/s] (0 = from the channel capacity)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--film", help="video file to encode and loop (default: the demo film)")
    g.add_argument("--camera", help="V4L2 capture device, e.g. /dev/video0")
    g.add_argument("--test", action="store_true", help="ffmpeg test pattern")
    g.add_argument("--ts", help="transport stream file, or - for stdin")
    g.add_argument("--null", action="store_true", help="null packets only")
    g.add_argument("--cw", action="store_true", help="unmodulated carrier on the centre frequency (constant symbol, no DVB-S)")
    ap.add_argument("--fps", type=float, default=0, help="encoded video FPS; 0 = automatic from channel capacity")
    ap.add_argument("--camera-size", default="640x480", help="V4L2 capture resolution")
    ap.add_argument("--camera-fps", type=float, default=30, help="V4L2 capture frame rate")
    ap.add_argument("--camera-format", choices=("auto", "mjpeg", "yuyv422", "nv12", "h264"), default="auto")
    ap.add_argument("--audio-source", choices=("none", "pulse", "alsa"), default="none", help="camera audio; none supplies silence")
    ap.add_argument("--audio-device", default="default", help="PulseAudio or ALSA capture device")
    ap.add_argument("--dc-i", type=float, help="override I DC correction [DAC codes]")
    ap.add_argument("--dc-q", type=float, help="override Q DC correction [DAC codes]")
    ap.add_argument("--iq-gain", type=float, help="override Q gain (0.7..1.3)")
    ap.add_argument("--iq-phase", type=float, help="override Q phase correction [degrees], -40..40")
    ap.add_argument("--invert", action="store_true", help="invert the spectrum (Q -> -Q)")
    ap.add_argument("--swap-iq", action="store_true", help="swap I and Q")
    ap.add_argument("--ppm", type=float, default=0.0, help="crystal error of YOUR board [ppm] (the PLL assumes exactly 40 MHz); measure it with a receiver")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--cal", default=os.path.join(HERE, "cal.json"), help="DC / I/Q trim file; --no-cal disables it")
    ap.add_argument("--no-cal", action="store_true")
    ap.add_argument("--port")
    return ap


@dataclass(frozen=True)
class Transmission:
    baud: float
    sample_hz: float
    capacity: float
    a16s8: bool
    a32fast: bool


def resolve_transmission(a):
    """Resolve automatic settings and validate before opening USB or a source."""
    if not all(math.isfinite(v) for v in (a.freq, a.ppm, a.seconds, a.fps, a.camera_fps)):
        raise ValueError("Frequency, correction and timing values must be finite")
    if not esp_link.BAND[0] <= a.freq <= esp_link.BAND[1]:
        raise ValueError("Transmission only in the 13 cm band (2300..2450 MHz)")
    if not 2000 <= a.baud <= 1000000:
        raise ValueError("Symbol rate must be 2000..1000000 Bd")
    if a.mod != "qpsk" and not a.dvbs2:
        raise ValueError(f"{a.mod.upper()} needs DVB-S2")
    if a.apsk_pl == "unit" and a.mod != "32apsk":
        raise ValueError("Unit PL points require DVB-S2 32APSK")
    if not 0 <= a.amp <= 480 or not 0 <= a.seconds <= 86400 or abs(a.ifm) > 6:
        raise ValueError("AMP: 0..480; duration: 0..86400 seconds; IF multiplier: -6..6")
    if abs(a.ppm) > 1000:
        raise ValueError("Crystal correction must be -1000..1000 ppm")
    if not a.amp:
        a.amp = 400 if a.mod == "32apsk" else 420 if a.mod != "qpsk" else 300
    if a.mod == "16apsk" and not 2000 <= a.baud <= 500000:
        auto_sps_16apsk(a.baud)
    if a.mod == "32apsk" and not 2000 <= a.baud <= A32_MAX_BAUD:
        auto_sps_32apsk(a.baud)
    if not a.sps:
        a.sps = auto_sps_32apsk(a.baud) if a.mod == "32apsk" else auto_sps_16apsk(a.baud) if a.mod == "16apsk" else auto_sps_8psk(a.baud) if a.mod == "8psk" else auto_sps(a.baud)
    if not 4 <= a.sps <= 232:
        raise ValueError("Samples per symbol must be 4..232, or 0 for automatic")
    a32fast = a.mod == "32apsk" and (a.baud, a.sps) == (250000, 32)
    if a.mod == "32apsk" and (a.sps > 64 or (not a32fast and CPU_HZ // (a.baud * a.sps) < A32_MIN_PERIOD)):
        raise ValueError("32APSK: use 32 SPS at 250 kS/s, or 4..64 SPS with at least 100 cycles per sample")
    a16s8 = a.mod == "16apsk" and a.baud == 1000000
    if a16s8 and a.sps != 8:
        raise ValueError("16APSK at 1 MS/s requires 8 SPS")
    sample_hz = a.baud * a.sps
    period = (CPU_HZ + sample_hz // 2) // sample_hz
    floor = CPU_HZ // (a.baud * a.sps)
    if a.mod == "16apsk" and not a16s8 and not (16 <= a.sps <= 24 and period == 20) and not (4 <= a.sps <= 24 and floor >= A16_MIN_PERIOD):
        raise ValueError("Unsupported 16APSK sampling; use automatic SPS")
    if a.mod == "8psk" and not (a.sps == 8 and period == 20) and not (16 <= a.sps <= 64 and (period == 20 or floor >= P8_MIN_PERIOD)):
        raise ValueError("Unsupported 8PSK sampling; use automatic SPS")
    if a.mod == "qpsk" and not ((a.sps == 8 and period == 20) or (16 <= a.sps <= 232 and period in (20, 24)) or (a.sps in (4, 8, 16) and period >= 16)):
        raise ValueError("Unsupported QPSK sampling; use automatic SPS")
    if not a.target:
        a.target = 6000 if a.mod == "8psk" and a.sps == 8 else (7000 if a16s8 else 2500) if a.mod == "16apsk" else 3500 if a32fast else 2500 if a.mod == "32apsk" else 3000
    max_target = 4000 if a.mod == "32apsk" else 8000 if a16s8 else 6000
    if not 64 <= a.target <= max_target:
        raise ValueError(f"Buffer target must be 64..{max_target} pairs, or 0 for automatic")
    baud_act = output_baud(a.baud, a.sps, a.mod)
    if abs(a.ifm * baud_act) + baud_act * .675 >= baud_act * a.sps / 2:
        raise ValueError("IF offset is too high for the selected sampling rate")
    lo = (a.freq * 1e6 - a.ifm * baud_act) / (1 + a.ppm * 1e-6)
    low = lo + min(0, a.ifm * baud_act) - baud_act * .675
    high = lo + max(0, a.ifm * baud_act) + baud_act * .675
    if low < esp_link.BAND[0] * 1e6 or high > esp_link.BAND[1] * 1e6:
        raise ValueError("The signal and LO must fit in 2300..2450 MHz; move away from the band edge")
    if a.dvbs2:
        cap = dvbs2.ts_rate(baud_act, a.fec, a.frame, a.pilots, a.mod)
    else:
        if a.fec not in dvbs.PUNCT:
            raise ValueError(f"DVB-S FEC: {', '.join(dvbs.PUNCT)}")
        cap = dvbs.ts_rate(baud_act, a.fec)
    for name, low, high in (("dc_i", -512, 512), ("dc_q", -512, 512), ("iq_gain", .7, 1.3), ("iq_phase", -40, 40)):
        value = getattr(a, name)
        if value is not None and (not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f"{name}: {low}..{high}")
    return Transmission(baud_act, baud_act * a.sps, cap, a16s8, a32fast)


def source_kind(a):
    return "null" if a.null or a.cw else "camera" if a.camera else "ts" if a.ts else "test" if a.test else "film"


def main(argv=None):
    a = build_parser().parse_args(argv)
    try:
        mode = resolve_transmission(a)
    except ValueError as e:
        raise SystemExit(str(e))
    baud_act, cap, a16s8, a32fast = mode.baud, mode.capacity, mode.a16s8, mode.a32fast
    if_hz = round(a.ifm * baud_act)
    dc, iq_g, iq_p = ([0.0, 0.0], 1.0, 0.0) if a.no_cal else load_cal(a.cal)
    dc = [a.dc_i if a.dc_i is not None else dc[0], a.dc_q if a.dc_q is not None else dc[1]]
    iq_g = a.iq_gain if a.iq_gain is not None else iq_g
    iq_p = a.iq_phase if a.iq_phase is not None else iq_p
    if not all(math.isfinite(v) for v in (*dc, iq_g, iq_p)) or not .7 <= iq_g <= 1.3 or not -40 <= iq_p <= 40:
        raise SystemExit("Invalid DC/IQ calibration")
    dc4 = [round(16 * v) for v in dc]
    gq, ph = round(iq_g * 1e4), round(iq_p * 1e3)
    kind = source_kind(a)
    try:
        if a.dvbs2:
            cap = dvbs2.ts_rate(baud_act, a.fec, a.frame, a.pilots, a.mod)
            enc = dvbs2.Encoder(a.fec, a.frame, a.pilots, swap_iq=a.swap_iq, invert=a.invert, mod=a.mod, bits3=a.sps == 8, apsk_pl=a.apsk_pl)
        else:
            if a.fec not in dvbs.PUNCT:
                raise ValueError(f"DVB-S code rate: {', '.join(dvbs.PUNCT)} (DVB-S2: --dvbs2)")
            cap = dvbs.ts_rate(baud_act, a.fec)
            enc = dvbs.Encoder(a.fec, swap_iq=a.swap_iq, invert=a.invert)
    except ValueError as e:
        raise SystemExit(str(e))
    value = a.camera if kind == "camera" else a.ts if kind == "ts" else a.film or DEMO_FILM
    try:
        src = TsSource(kind, value, cap, a.width, a.video_k, a.fps,
                       a.camera_size, a.camera_fps, a.camera_format, a.audio_source, a.audio_device)
    except ValueError as e:
        raise SystemExit(str(e))
    khz = round((a.freq * 1e6 - if_hz) / 1000 / (1 + a.ppm * 1e-6))
    link = None
    stop_enc = threading.Event()
    try:
        link = esp_link.Link(esp_link.find_port(a.port))
        src.check()
        link.configure_pa(a.pa_enable)
        secs = min(86400, math.ceil(a.seconds) + 30) if a.seconds else 86400
        cmd = {"8psk": "PSK8T", "16apsk": "A16T", "32apsk": "A32T"}.get(a.mod, "QPSKT")
        gam = (f" {round(100 * dvbs2.APSK16_GAMMA[a.fec])}" if a.mod == "16apsk"                       # ring ratio R2 / R1 x 100 of the code rate
               else " %d %d" % tuple(round(100 * x) for x in dvbs2.APSK32_GAMMA[a.fec]) if a.mod == "32apsk" else "")     # R2 / R1 and R3 / R1 x 100
        info = link.start(f"{cmd} {khz / 1000:.3f} {a.baud} {a.sps} {a.amp} {secs} {a.ifm} {a.target} {dc4[0]} {dc4[1]} {gq} {ph}{gam}")
        kv = dict(zip(info.split()[2::2], info.split()[3::2]))
        if link.pa_gpio is not None and kv.get("PA") != str(int(a.pa_enable)):
            raise SystemExit("ESP PA enable output does not match the requested state")
        if a.mod == "32apsk" and a.apsk_pl == "unit" and int(kv.get("PLPTS", "0")) < 4:
            raise SystemExit("This firmware has no unit-radius PL points; build and flash firmware/ first")
        lo_true = float(kv["LO"]) * (1 + a.ppm * 1e-6)
        std = f"DVB-S2 {a.mod.upper()} {a.frame} frame{', pilots' if a.pilots else ''}" if a.dvbs2 else "DVB-S"
        print(f"{info}\n" + ("UNMODULATED CARRIER (--cw)\n" if a.cw else "") + f"spectrum centre {(lo_true + if_hz) / 1e6:.6f} MHz (requested {a.freq:.6f}), {baud_act:.1f} Bd, "
              f"{std} FEC {a.fec}, RRC 0.35 occupies {baud_act * 1.35 / 1e3:.0f} kHz, TS capacity {cap / 1e3:.0f} kb/s")
        # Large writes keep USB busy while the encoder prepares the next frame.
        # At 1 MBd 16APSK needs 500 kB/s; its loop requires full 64-byte packets.
        min_todo, max_chunk = (400, 4096) if a16s8 else (64, 2048) if a32fast else (400, 2048) if a.sps == 8 and a.mod == "8psk" else (32, 1024)
        buf = b""
        # The encoder runs in its own thread, ahead of the USB writes: the kernel takes about one write at a time, so encoding between two writes leaves the
        # link idle (a loop that encodes and writes alternately delivered 360 kB/s, a pre-encoded stream 420 kB/s)
        ready = queue.Queue(maxsize=48)
        cw_byte = (0xFF if a.mod == "qpsk" else 0x00) if a.cw else None
        threading.Thread(target=feed_symbols, args=(ready, stop_enc, src, enc, cw_byte), daemon=True).start()
        t_start = t_rep = time.time()
        fmin, fmax = 10 ** 9, 0
        while not a.seconds or time.time() - t_start < a.seconds:
            src.check()
            link.poll()
            if b"TX END" in link.text:
                break
            if link.reports:
                fmin, fmax = min(fmin, link.fill), max(fmax, link.fill)
            todo = a.target - (link.fill + link.sent_since)               # in pairs of 2 bytes
            if todo >= min_todo:
                nb = 2 * min(todo, max_chunk)
                while len(buf) < nb:
                    try:
                        buf += ready.get(timeout=0.05)
                    except queue.Empty:
                        break
                nb = min(nb, len(buf))
                if a16s8 or a32fast:
                    nb -= nb % 64    # full USB packets for the 16APSK 1 MS/s and 32APSK 8 MS/s loops
                if nb:
                    link.send(buf[:nb])                                   # raw symbol bytes, no framing
                    buf = buf[nb:]
                    link.sent_since += nb // 2
            else:
                time.sleep(0.0005)
            if time.time() - t_rep > 2:
                print(f"  ESP buffer {fmin}..{fmax} pairs, TS packets: {src.pkts} from the source, {src.nulls} null")
                fmin, fmax, t_rep = 10 ** 9, 0, time.time()
    except KeyboardInterrupt:
        pass
    finally:
        stop_enc.set()
        src.close()
        if link is not None:
            print(link.finish(send_stop=not (a16s8 or a32fast)))


if __name__ == "__main__":
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    main()
