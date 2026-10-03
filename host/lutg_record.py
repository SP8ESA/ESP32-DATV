#!/usr/bin/env python3
"""Sends one command to the ESP32-DATV firmware and prints everything it answers until the recording of the lutg words / timing
build ("SLED" and "TX END" lines) is complete. Only for the LUTG_REC builds, see gen_lutg.py and test_lutg_words.c.

  python3 lutg_record.py "QPSKT 2370.000 500000 16 300 5 0 0" [--port /dev/ttyACM0]
"""
import argparse
import time

import serial

import esp_link


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command")
    ap.add_argument("--port")
    ap.add_argument("--wait", type=float, default=15.0)
    a = ap.parse_args()
    s = serial.Serial(esp_link.find_port(a.port), 115200, timeout=0)    # opening the port resets the C3
    time.sleep(1.2)
    for _ in range(10):
        s.write(b"\nINFO\n")
        t0, got = time.time(), b""
        while time.time() - t0 < 0.7 and b"ESP32DATV" not in got:
            got += s.read(4096)
            time.sleep(0.02)
        if b"ESP32DATV" in got:
            break
    else:
        raise SystemExit("the ESP does not answer INFO")
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write((a.command + "\n").encode())
    t0, buf = time.time(), b""
    while time.time() - t0 < a.wait:
        buf += s.read(65536)
        time.sleep(0.01)
        if b"SLED" in buf and b"TX END" in buf:
            time.sleep(0.2)
            buf += s.read(65536)
            break
    print(buf.decode(errors="replace"))


if __name__ == "__main__":
    main()
