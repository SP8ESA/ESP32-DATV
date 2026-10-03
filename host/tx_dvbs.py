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
  --null            only null packets: a valid but empty multiplex (a good receiver lock test)
  --cw              no DVB-S at all: a constant symbol, i.e. an unmodulated carrier on the centre frequency (for tuning and level checks)
When the source cannot keep up the script pads with null packets, so symbols never stop.

  python3 tx_dvbs.py --freq 2402.000 --baud 1000000 --fec 1/2
  Receiver: centre = --freq, symbol rate 1000 kS/s, FEC 1/2 (or auto). No lock: try --invert or --swap-iq.

Amateur radio use only, within the limits of your licence. The firmware refuses frequencies outside 2300..2450 MHz.
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dvbs  # noqa: E402
import esp_link  # noqa: E402

CPU_HZ = 160e6
DEMO_FILM = os.path.join(HERE, "..", "media", "sintel_trailer.mp4")


class TsSource:
    """Delivers 188-byte transport stream packets; take(n) pads with null packets when the source runs dry."""

    def __init__(self, kind, arg, baud, fec, width=640, video_k=0):
        self.q = queue.Queue(maxsize=400)
        self.kind = kind
        self.arg = arg
        self.nulls = 0
        self.pkts = 0
        self.proc = None
        if kind == "null":
            return
        cap = dvbs.ts_rate(baud, fec)
        mux = int(cap * 0.965)                                      # slightly below capacity: the rest is null padding
        # budget by channel capacity: audio, picture size / frame rate and the PSI repetition shrink for narrow channels
        if mux >= 600_000:
            aud, ach, ar, fps, pat, w = 96, 2, 48000, 25, 0.2, width
        elif mux >= 200_000:
            aud, ach, ar, fps, pat, w = 32, 1, 24000, 15, 0.5, min(width, 320)
        else:
            aud, ach, ar, fps, pat, w = (8 if mux < 40_000 else 16), 1, 16000, 10, 1.0, min(width, 160)
        psi = int(3 * 188 * 8 / pat)                                 # PAT + PMT + SDT packets, bit/s
        vb = video_k * 1000 if video_k else int((mux - aud * 1000 - psi) * 0.88)
        if vb < 6000:
            raise SystemExit(f"channel capacity {cap / 1000:.1f} kb/s is too small for video: use a higher symbol rate or FEC")
        common = ["-c:v", "libx264", "-preset", "veryfast", "-profile:v", "main", "-g", str(2 * fps), "-bf", "2",
                  "-b:v", str(vb), "-maxrate", str(vb), "-bufsize", str(vb // 2), "-x264-params", "nal-hrd=cbr:force-cfr=1",
                  "-c:a", "mp2", "-b:a", f"{aud}k", "-ac", str(ach), "-ar", str(ar), "-pix_fmt", "yuv420p",
                  "-f", "mpegts", "-muxrate", str(mux), "-pcr_period", "40" if aud > 50 else "100", "-pat_period", str(pat),
                  "-mpegts_flags", "+resend_headers",
                  "-metadata", "service_provider=ESP32-DATV", "-metadata", "service_name=ESP32-C3 DATV", "-"]
        if kind == "film":
            cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-re", "-stream_loop", "-1", "-i", arg,
                   "-vf", f"scale={w}:-2,fps={fps}"] + common
        elif kind == "test":
            cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-re", "-f", "lavfi", "-i", f"testsrc2=size={w}x{w * 9 // 16}:rate={fps}",
                   "-f", "lavfi", "-i", f"sine=frequency=800:sample_rate={ar}"] + common
        else:
            cmd = None
        f = None
        if cmd:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, start_new_session=True)
            f = self.proc.stdout
            print(f"ffmpeg: {w}x video {vb / 1000:.0f} kb/s at {fps} fps + audio {aud} kb/s in a {mux / 1000:.0f} kb/s multiplex (channel capacity {cap / 1000:.0f} kb/s)")
        elif arg == "-":
            f = sys.stdin.buffer
        threading.Thread(target=self._read, args=(f,), daemon=True).start()

    def _read(self, f):
        buf = b""
        while True:
            if f is None:                                           # a file: loop it
                with open(self.arg, "rb") as fh:
                    while True:
                        d = fh.read(188 * 64)
                        if not d:
                            break
                        buf = self._push(buf + d)
                continue
            d = f.read(188 * 16)
            if not d:
                return                                              # end of input: null padding from now on
            buf = self._push(buf + d)

    def _push(self, buf):
        while len(buf) >= 188 * 2:                                  # align on the 0x47 sync byte every 188 bytes
            if buf[0] != 0x47 or buf[188] != 0x47:
                i = buf.find(b"\x47", 1)
                buf = buf[i:] if i > 0 else b""
                continue
            n = len(buf) // 188
            for k in range(n):
                self.q.put(buf[188 * k:188 * k + 188])
            buf = buf[188 * n:]
        return buf

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
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


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


def load_cal(path):
    """DC and I/Q imbalance trim (measured on the author's board; see README for how to calibrate yours)."""
    if not path or not os.path.exists(path):
        return [0.0, 0.0], 1.0, 0.0
    c = json.load(open(path))
    print(f"calibration {os.path.basename(path)}: DC {c['dc']} codes, Q gain {c['iq_gain']}, Q phase {c['iq_phase_deg']} deg")
    return c["dc"], c["iq_gain"], c["iq_phase_deg"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, default=2402.000, help="centre of the spectrum [MHz] (13 cm band)")
    ap.add_argument("--baud", type=int, default=1000000, help="symbol rate [Bd], 2000..1000000 (e.g. 33000 for narrow-band DATV)")
    ap.add_argument("--fec", default="1/2", choices=list(dvbs.PUNCT))
    ap.add_argument("--sps", type=int, default=0, help="DAC samples per symbol (4, 8, 16, or 16..232 when baud * sps is 8 or 6.67 MHz); 0 = automatic: the hand-scheduled 8 / 6.67 MS/s loops when they fit the symbol rate, else the most that keeps the output at or below 4 MS/s")
    ap.add_argument("--ifm", type=int, default=0, help="centre = LO + ifm * baud (puts the LO leakage outside the signal)")
    ap.add_argument("--amp", type=int, default=300, help="peak amplitude in DAC codes (1..480)")
    ap.add_argument("--target", type=int, default=3000, help="ESP symbol buffer fill to hold [pairs of 8 symbols]")
    ap.add_argument("--film", help="video file to encode and loop (default: the demo film)")
    ap.add_argument("--width", type=int, default=640, help="--film: picture width")
    ap.add_argument("--video-k", type=int, default=0, help="--film/--test: video bit rate [kb/s] (0 = from the channel capacity)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--test", action="store_true", help="ffmpeg test pattern")
    g.add_argument("--ts", help="transport stream file, or - for stdin")
    g.add_argument("--null", action="store_true", help="null packets only")
    g.add_argument("--cw", action="store_true", help="unmodulated carrier on the centre frequency (constant symbol, no DVB-S)")
    ap.add_argument("--invert", action="store_true", help="invert the spectrum (Q -> -Q)")
    ap.add_argument("--swap-iq", action="store_true", help="swap I and Q")
    ap.add_argument("--ppm", type=float, default=0.0, help="crystal error of YOUR board [ppm] (the PLL assumes exactly 40 MHz); measure it with a receiver")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--cal", default=os.path.join(HERE, "cal.json"), help="DC / I/Q trim file; --no-cal disables it")
    ap.add_argument("--no-cal", action="store_true")
    ap.add_argument("--port")
    a = ap.parse_args()
    if not esp_link.BAND[0] <= a.freq <= esp_link.BAND[1]:
        raise SystemExit("transmission only in the 13 cm band (2300..2450 MHz)")
    if not a.sps:
        a.sps = auto_sps(a.baud)
    period = round(CPU_HZ / (a.baud * a.sps))
    baud_act = CPU_HZ / (period * a.sps)
    if abs(baud_act / a.baud - 1) > 0.01:
        print(f"note: {a.sps} samples per symbol give {baud_act:.1f} Bd, not {a.baud}")
    if_hz = round(a.ifm * baud_act)
    dc, iq_g, iq_p = ([0.0, 0.0], 1.0, 0.0) if a.no_cal else load_cal(a.cal)
    dc4 = [round(16 * v) for v in dc]
    gq, ph = round(iq_g * 1e4), round(iq_p * 1e3)
    kind = "null" if a.null or a.cw else "ts" if a.ts else "test" if a.test else "film"
    src = TsSource(kind, a.ts if kind == "ts" else a.film or DEMO_FILM, a.baud, a.fec, a.width, a.video_k)
    enc = dvbs.Encoder(a.fec, swap_iq=a.swap_iq, invert=a.invert)
    khz = round((a.freq * 1e6 - if_hz) / 1000 / (1 + a.ppm * 1e-6))
    link = esp_link.Link(esp_link.find_port(a.port))
    try:
        secs = int(a.seconds) + 30 if a.seconds else 86400
        info = link.start(f"QPSKT {khz / 1000:.3f} {a.baud} {a.sps} {a.amp} {secs} {a.ifm} {a.target} {dc4[0]} {dc4[1]} {gq} {ph}")
        kv = dict(zip(info.split()[2::2], info.split()[3::2]))
        lo_true = float(kv["LO"]) * (1 + a.ppm * 1e-6)
        print(f"{info}\n" + ("UNMODULATED CARRIER (--cw)\n" if a.cw else "") + f"spectrum centre {(lo_true + if_hz) / 1e6:.6f} MHz (requested {a.freq:.6f}), {baud_act:.1f} Bd, FEC {a.fec}, "
              f"RRC 0.35 occupies {baud_act * 1.35 / 1e3:.0f} kHz, TS capacity {dvbs.ts_rate(baud_act, a.fec) / 1e3:.0f} kb/s")
        buf = b""
        t_start = t_rep = time.time()
        fmin, fmax = 10 ** 9, 0
        while not a.seconds or time.time() - t_start < a.seconds:
            link.poll()
            if link.reports:
                fmin, fmax = min(fmin, link.fill), max(fmax, link.fill)
            todo = a.target - (link.fill + link.sent_since)               # in pairs of 2 bytes (8 symbols)
            if todo >= 32:
                nb = 2 * min(todo, 1024)
                while len(buf) < nb:
                    buf += b"\xFF" * nb if a.cw else enc.encode(src.take(8))      # 0xFF = four symbols (+1, +1): a carrier
                link.send(buf[:nb])                                       # raw symbol bytes, no framing
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
        src.close()
        print(link.finish())


if __name__ == "__main__":
    main()
