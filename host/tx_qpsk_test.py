#!/usr/bin/env python3
"""QPSK spectrum test: random symbols through the ESP32-DATV modulator (no DVB-S framing).

By default the ESP generates the random bits itself (a PRBS), so the USB link is not involved. With --stream the PC sends
random symbol bytes over USB instead, which exercises the same path as the real DATV transmitter.

  python3 tx_qpsk_test.py --freq 2402.000 --baud 1000000 --seconds 60
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import esp_link  # noqa: E402
from tx_dvbs import auto_sps  # noqa: E402

CPU_HZ = 160e6


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, default=2402.000, help="centre of the spectrum [MHz] (13 cm band)")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--sps", type=int, default=0, choices=(0, 4, 8, 16), help="0 = automatic (as tx_dvbs.py: 8 at 1 MBd)")
    ap.add_argument("--ifm", type=int, default=0, help="centre = LO + ifm * baud")
    ap.add_argument("--amp", type=int, default=300)
    ap.add_argument("--stream", action="store_true", help="random symbol bytes from the PC over USB instead of the on-chip PRBS")
    ap.add_argument("--target", type=int, default=3000)
    ap.add_argument("--ppm", type=float, default=0.0, help="crystal error of your board [ppm]")
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--cal", default=os.path.join(HERE, "cal.json"))
    ap.add_argument("--no-cal", action="store_true")
    ap.add_argument("--port")
    a = ap.parse_args()
    if not esp_link.BAND[0] <= a.freq <= esp_link.BAND[1]:
        raise SystemExit("transmission only in the 13 cm band (2300..2450 MHz)")
    if not a.sps:
        a.sps = auto_sps(a.baud)
    if a.sps * a.baud > 4_000_000 and not a.stream:
        print("8 MS/s takes the symbols from USB: --stream switched on (random symbols from the PC)")
        a.stream = True
    period = round(CPU_HZ / (a.baud * a.sps))
    baud_act = CPU_HZ / (period * a.sps)
    if_hz = round(a.ifm * baud_act)
    dc, gq, ph = [0.0, 0.0], 1.0, 0.0
    if not a.no_cal and os.path.exists(a.cal):
        c = json.load(open(a.cal))
        dc, gq, ph = c["dc"], c["iq_gain"], c["iq_phase_deg"]
    khz = round((a.freq * 1e6 - if_hz) / 1000 / (1 + a.ppm * 1e-6))
    link = esp_link.Link(esp_link.find_port(a.port))
    try:
        secs = int(a.seconds) + 30 if a.seconds else 86400
        info = link.start(f"QPSKT {khz / 1000:.3f} {a.baud} {a.sps} {a.amp} {secs} {a.ifm} {a.target if a.stream else 0} "
                          f"{round(16 * dc[0])} {round(16 * dc[1])} {round(gq * 1e4)} {round(ph * 1e3)}")
        kv = dict(zip(info.split()[2::2], info.split()[3::2]))
        lo_true = float(kv["LO"]) * (1 + a.ppm * 1e-6)
        print(f"{info}\nspectrum centre {(lo_true + if_hz) / 1e6:.6f} MHz, {baud_act:.1f} Bd, RRC 0.35 occupies {baud_act * 1.35 / 1e3:.0f} kHz")
        t0 = time.time()
        while not a.seconds or time.time() - t0 < a.seconds:
            if a.stream:
                link.poll()
                todo = a.target - (link.fill + link.sent_since)
                if todo >= 32:
                    nb = 2 * min(todo, 1024)
                    link.send(np.random.randint(0, 256, nb, dtype=np.uint8).tobytes())
                    link.sent_since += nb // 2
                else:
                    time.sleep(0.0005)
            else:
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(link.finish())


if __name__ == "__main__":
    main()
