"""USB link to the ESP32-DATV firmware (native USB Serial/JTAG of the ESP32-C3, shows up as /dev/ttyACM*).

Text commands go down, a raw symbol stream goes down after QPSKT, and the ESP sends back 4-byte fill reports
(B7, fill_lo, fill_hi, underruns) interleaved with text lines.
"""
import glob
import time

import serial

BAND = (2300.0, 2450.0)          # MHz, the 13 cm amateur band; the firmware refuses anything else too


def find_port(port=None):
    if port:
        return port
    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        raise SystemExit("no /dev/ttyACM*: is the board plugged in?")
    return ports[0]


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0)   # opening the port may reset the C3
        time.sleep(1.2)
        for _ in range(8):                                # wait for the firmware (boot can take a while)
            self.s.write(b"\nINFO\n")
            t0 = time.time()
            got = b""
            while time.time() - t0 < 0.7 and b"ESP32DATV" not in got:
                got += self.s.read(4096)
                time.sleep(0.02)
            if b"ESP32DATV" in got:
                break
        else:
            raise SystemExit("the ESP does not answer INFO (flash firmware/ first, or reset the board)")
        time.sleep(0.1)
        self.s.reset_input_buffer()
        self.buf = b""
        self.text = b""
        self.fill = 0              # symbol pairs (2 bytes = 8 symbols) in the ESP ring, last report
        self.under = 0             # ring underruns (saturates at 255)
        self.sent_since = 0        # pairs sent since the last report
        self.reports = 0

    def poll(self):
        data = self.s.read(65536)
        if data:
            self.buf += data
        b, i = self.buf, 0
        while i < len(b):
            if b[i] == 0xB7:
                if i + 4 > len(b):
                    break
                self.fill = b[i + 1] | b[i + 2] << 8
                self.under = b[i + 3]
                self.sent_since = 0
                self.reports += 1
                i += 4
            else:
                self.text += b[i:i + 1]
                i += 1
        self.buf = b[i:]

    def start(self, cmd, timeout=8.0):
        """Send a command and wait for its 'OK QPSKT ...' line (returned) or an ERR."""
        self.s.write((cmd + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.poll()
            for ln in self.text.decode(errors="replace").splitlines():
                if ln.startswith("OK QPSKT"):
                    return ln
                if ln.startswith("ERR"):
                    raise SystemExit(f"ESP: {ln}")
            time.sleep(0.01)
        raise SystemExit(f"no answer from the ESP: {self.text[-200:]!r}")

    def send(self, data):
        self.s.write(data)

    def finish(self, timeout=5.0):
        """Stop byte (ends the PRBS test at once; a streaming transmitter stops 0.5 s after the last byte), then the 'TX END' line."""
        try:
            self.s.write(b"\xA5\xFF")
        except serial.SerialException:
            pass
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.poll()
            if b"TX END" in self.text and self.text.rstrip().endswith(b")"):
                break
            time.sleep(0.02)
        lines = [ln for ln in self.text.decode(errors="replace").splitlines() if "TX END" in ln]
        self.s.close()
        return "\n".join(lines) if lines else "no summary from the ESP"
