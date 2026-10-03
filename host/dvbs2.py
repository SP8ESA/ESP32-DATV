#!/usr/bin/env python3
"""DVB-S2 encoder (ETSI EN 302 307) for the ESP32-DATV transmitter: MPEG transport stream -> QPSK symbols.

Scope: QPSK (all code rates: normal 1/4 ... 9/10, short 1/4 ... 8/9) and 8PSK (3/5 ... 9/10, short without 9/10), normal (64800 bit) and
short (16200 bit) FECFRAMEs, CCM, one transport stream, roll-off 0.35 (the ESP filters), pilots on or off (16APSK / 32APSK are not possible
with the ESP's modulator).
Chain: mode adaptation (CRC-8 in the sync byte position, BBHEADER) -> BB scrambler -> BCH -> LDPC -> bit interleaver (8PSK) -> mapping ->
PLHEADER (SOF + PLSCODE, pi/2 BPSK) -> pilots -> PL scrambler (Gold code 0).
Output for QPSK: every symbol is one of four points, the same packed stream as dvbs.py: 4 symbols per byte, bit 0 = I level, bit 1 = Q
level (1 = +1), first symbol in the low bits. Output for 8PSK: every symbol is one of eight points e^(j pi k / 4), k = 0..7 (the angle in
45 degree steps), 2 symbols per byte, the first one in the low nibble.

dvbs2_ldpc.json holds the LDPC parity address tables of the standard (annex B and C): for every group of 360 information bits the
addresses of the parity bits it is added to. They were read out of an independent encoder (gr-dtv) by encoding unit vectors, and the
whole chain was checked bit-exact against gr-dtv with host/dvbs2_vs_gnuradio.py.

Self-test (no other software needed):  python3 dvbs2.py
"""
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TABLES = json.load(open(os.path.join(HERE, "dvbs2_ldpc.json")))

# (k_bch, t) for the normal frames; short frames: k_bch below, t = 12 for all
NORMAL = {"1/4": (16008, 12), "1/3": (21408, 12), "2/5": (25728, 12), "1/2": (32208, 12), "3/5": (38688, 12), "2/3": (43040, 10),
          "3/4": (48408, 12), "4/5": (51648, 12), "5/6": (53840, 10), "8/9": (57472, 8), "9/10": (58192, 8)}
SHORT = {"1/4": (3072, 12), "1/3": (5232, 12), "2/5": (6312, 12), "1/2": (7032, 12), "3/5": (9552, 12), "2/3": (10632, 12),
         "3/4": (11712, 12), "4/5": (12432, 12), "5/6": (13152, 12), "8/9": (14232, 12)}
PARAMS = {"normal": NORMAL, "short": SHORT}
MODCOD = {"1/4": 1, "1/3": 2, "2/5": 3, "1/2": 4, "3/5": 5, "2/3": 6, "3/4": 7, "4/5": 8, "5/6": 9, "8/9": 10, "9/10": 11}
MODCOD_8PSK = {"3/5": 12, "2/3": 13, "3/4": 14, "5/6": 15, "8/9": 16, "9/10": 17}
FEC_RATES = list(NORMAL)
BITS = {"qpsk": 2, "8psk": 3}
# 8PSK: DVB-S2 bit triple (b0 b1 b2) -> angle index k (the point e^(j pi k / 4)); read out of gr-dtv's modulator
PSK8_K = (1, 0, 4, 5, 2, 7, 3, 6)


def rates(mod="qpsk", frame="normal"):
    """The code rates that exist for this modulation and frame size."""
    r = list(PARAMS[frame])
    return [x for x in r if x in MODCOD_8PSK] if mod == "8psk" else r
NULL_PACKET = bytes([0x47, 0x1F, 0xFF, 0x10]) + b"\xFF" * 184


def frame_info(fec, frame="normal", pilots=False, mod="qpsk"):
    """k_bch, t, n_ldpc, bits of the data field (DFL) and symbols of the PLFRAME."""
    if mod not in BITS:
        raise ValueError("modulation: qpsk, 8psk")
    if frame not in PARAMS or fec not in rates(mod, frame):
        raise ValueError(f"DVB-S2 {mod.upper()} {frame} frame: FEC {', '.join(rates(mod, frame))}")
    kbch, t = PARAMS[frame][fec]
    n = 64800 if frame == "normal" else 16200
    nsym_data = n // BITS[mod]
    slots = nsym_data // 90
    nsym = 90 + nsym_data + (36 * ((slots - 1) // 16) if pilots else 0)
    return dict(kbch=kbch, t=t, nldpc=n, dfl=kbch - 80, plframe=nsym)


def ts_rate(baud, fec, frame="normal", pilots=False, mod="qpsk"):
    """Useful transport-stream bit rate [bit/s]: the data field carries 188-byte user packets (CRC-8 in the place of the sync byte)."""
    fi = frame_info(fec, frame, pilots, mod)
    return baud * fi["dfl"] / fi["plframe"]


# ---------------------------------------------------------------- CRC-8 (BBHEADER and user packets): x^8+x^7+x^6+x^4+x^2+1
def _crc8_table():
    tab = []
    for b in range(256):
        c = b
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        tab.append(c)
    return tab


_CRC8 = _crc8_table()


def crc8(data):
    c = 0
    for b in data:
        c = _CRC8[c ^ b]
    return c


# ---------------------------------------------------------------- BB scrambler: 1 + x^14 + x^15, initial 100101010000000
def _bb_prbs(nbits):
    reg = [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    out = np.zeros(nbits, np.uint8)
    for i in range(nbits):
        o = reg[13] ^ reg[14]
        out[i] = o
        reg = [o] + reg[:-1]
    return out


_BBPRBS = {}


def bb_prbs(kbch):
    if kbch not in _BBPRBS:
        _BBPRBS[kbch] = _bb_prbs(kbch)
    return _BBPRBS[kbch]


# ---------------------------------------------------------------- BCH: generator polynomial from the minimal polynomials of a^1..a^2t
def _gf_tables(m, prim):
    exp = [0] * (2 ** m - 1)
    x = 1
    for i in range(2 ** m - 1):
        exp[i] = x
        x <<= 1
        if x >> m:
            x ^= prim
    log = {v: i for i, v in enumerate(exp)}
    return exp, log


def _poly_mul_gf2(a, b):
    r = 0
    while b:
        if b & 1:
            r ^= a
        a <<= 1
        b >>= 1
    return r


def bch_generator(m, prim, t):
    """g(x) as an int (bit i = coefficient of x^i): product of the minimal polynomials of a^1, a^3, ..., a^(2t-1)."""
    exp, log = _gf_tables(m, prim)
    n = 2 ** m - 1
    g = 1
    seen = set()
    for i in range(1, 2 * t, 2):
        coset, j = [], i
        while j not in coset:
            coset.append(j)
            j = (2 * j) % n
        if coset[0] in seen:
            continue
        seen.add(coset[0])
        # minimal polynomial = prod (x - a^j) over the coset, coefficients in GF(2^m) that come out in GF(2)
        poly = [1]                                    # poly[k] = coefficient of x^k (field elements as ints)
        for j in coset:
            root = exp[j]
            new = [0] * (len(poly) + 1)
            for k, c in enumerate(poly):
                new[k + 1] ^= c
                if c:
                    new[k] ^= exp[(log[c] + j) % n]
            poly = new
        assert all(c in (0, 1) for c in poly)
        mp = sum(c << k for k, c in enumerate(poly))
        g = _poly_mul_gf2(g, mp)
    return g


class Bch:
    def __init__(self, frame, t):
        m, prim = (16, 0x1002D) if frame == "normal" else (14, 0x402B)
        self.g = bch_generator(m, prim, t)
        self.deg = self.g.bit_length() - 1             # n - k = m * t
        self.mask = (1 << self.deg) - 1
        # byte-wise table: remainder of (b * x^deg) mod g
        tab = []
        for b in range(256):
            r = b << (self.deg - 8) if self.deg >= 8 else b
            r = b << self.deg
            for bit in range(self.deg + 7, self.deg - 1, -1):
                if r >> bit & 1:
                    r ^= self.g << (bit - self.deg)
            tab.append(r & self.mask)
        self.tab = tab

    def parity(self, data):
        """data: bytes (k_bch / 8 bytes, first bit = highest power) -> parity as bytes (deg / 8 bytes)."""
        s = 0
        sh = self.deg - 8
        tab, mask = self.tab, self.mask
        for b in data:
            s = ((s << 8) & mask) ^ tab[((s >> sh) ^ b) & 0xFF]
        return s.to_bytes(self.deg // 8, "big")


# ---------------------------------------------------------------- LDPC: irregular repeat-accumulate encoder with the annex B / C tables
class Ldpc:
    def __init__(self, frame, fec):
        rows = TABLES[frame][fec]
        self.k = len(rows) * 360
        self.n = 64800 if frame == "normal" else 16200
        self.nk = self.n - self.k
        q = self.nk // 360
        i = np.arange(360)
        self.idx = [[(np.array(x) + q * i) % self.nk for x in row] for row in rows]
        self.idx = [[(x + q * i) % self.nk for x in row] for row in rows]

    def encode(self, bits):
        """bits: uint8 array of k bits -> codeword (information + parity), n bits."""
        p = np.zeros(self.nk, np.uint8)
        for g, row in enumerate(self.idx):
            bg = bits[360 * g:360 * g + 360]
            for ix in row:
                p[ix] ^= bg
        p = np.bitwise_xor.accumulate(p)
        return np.concatenate((bits, p))


# ---------------------------------------------------------------- physical layer
def _bits(value, n):
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


SOF = _bits(0x18D2E82, 26)
PLS_SCRAMBLE = _bits(0x719D83C953422DFA, 64)
RM_ROWS = (0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF, 0x0000FFFF, 0xFFFFFFFF)


def plscode(modcod, short, pilots):
    """The 64 PLSCODE bits: MODCOD (5 bits) + TYPE (short, pilots) through the (64,7) code and the scrambling sequence."""
    pls = (modcod << 2) | (2 if short else 0) | (1 if pilots else 0)
    b = _bits(pls, 7)                                  # b1..b7, b1 = MSB
    cw = 0
    for i in range(6):
        if b[i]:
            cw ^= RM_ROWS[5 - i] if False else RM_ROWS[i]
    w = _bits(cw, 32)
    out = []
    for i in range(32):
        out += [w[i], w[i] ^ b[6]]
    return [o ^ s for o, s in zip(out, PLS_SCRAMBLE)]


def _gold_rotation(n=0, length=33282):
    """PL scrambling sequence: R(i) in 0..3, the symbol is multiplied by exp(j * R * pi / 2)."""
    N = 2 ** 18 - 1
    x = [0] * (N + 18 + 2)
    y = [1] * 18 + [0] * (N + 2)
    x[0] = 1
    for i in range(N + 2 - 18):
        x[i + 18] = x[i + 7] ^ x[i]
        y[i + 18] = y[i + 10] ^ y[i + 7] ^ y[i + 5] ^ y[i]
    z = np.array([x[(i + n) % N] ^ y[i] for i in range(N)], np.uint8)
    i = np.arange(length)
    return (z[i] + 2 * z[(i + 131072) % N]).astype(np.uint8)


_ROT = None


def rotation(n):
    global _ROT
    if _ROT is None or len(_ROT) < n:
        _ROT = _gold_rotation(0, max(n, 33282))
    return _ROT[:n]


class Encoder:
    def __init__(self, fec="1/2", frame="normal", pilots=False, swap_iq=False, invert=False, mod="qpsk", bits3=False):
        fi = frame_info(fec, frame, pilots, mod)
        self.fec, self.frame, self.pilots, self.mod = fec, frame, pilots, mod
        self.bits3 = bits3 and mod == "8psk"            # 8PSK at 1 MBd (lutg_p8s8.S): 3 bits per symbol, 8 symbols = 3 bytes; otherwise a nibble per symbol
        self.kbch, self.dfl, self.nsym = fi["kbch"], fi["dfl"], fi["plframe"]
        self.bch = Bch(frame, fi["t"])
        self.ldpc = Ldpc(frame, fec)
        assert self.ldpc.k == self.kbch + self.bch.deg
        self.swap_iq, self.invert = swap_iq, invert
        self.crc_prev = 0
        self.pend = np.zeros(0, np.uint8)               # symbols that do not fill a byte yet
        self.stream = bytearray()                       # user packets with the sync byte replaced by the CRC-8 of the previous one
        self.pos = 0                                    # stream byte number of stream[0]
        self.hdr = np.array(SOF + plscode((MODCOD_8PSK if mod == "8psk" else MODCOD)[fec], frame == "short", pilots), np.uint8)
        # pi/2 BPSK header symbols: even index (1+j)/sqrt2, odd index (-1+j)/sqrt2, times (1 - 2 b)
        even = np.arange(90) % 2 == 0
        a = 1 - 2 * self.hdr.astype(np.int8)
        self.hdr_i = np.where(even, a, -a).astype(np.int8)
        self.hdr_q = a.astype(np.int8)
        self.hdr_k = np.array([{(1, 1): 1, (-1, 1): 3, (-1, -1): 5, (1, -1): 7}[(int(i), int(q))] for i, q in zip(self.hdr_i, self.hdr_q)], np.uint8)

    @property
    def packets_per_frame(self):
        return self.dfl / 1504.0

    def _need_bytes(self):
        return self.dfl // 8

    def push(self, ts):
        """Add transport stream packets (bytes, a multiple of 188, every packet starting with 0x47)."""
        p = np.frombuffer(ts, np.uint8).reshape(-1, 188)
        if not np.all(p[:, 0] == 0x47):
            raise ValueError("TS packet without the 0x47 sync byte")
        for row in p:
            self.stream.append(self.crc_prev)
            self.stream += row[1:].tobytes()
            self.crc_prev = crc8(row[1:].tobytes())

    def ready(self):
        return len(self.stream) >= self._need_bytes()

    def bbframe(self):
        """The next BBFRAME (k_bch bits as bytes), BB scrambled."""
        nb = self._need_bytes()
        data = bytes(self.stream[:nb])
        first = -(-self.pos // 188) * 188               # stream byte number of the first user packet that starts in this frame
        syncd = (first - self.pos) * 8 if first < self.pos + nb else 65535
        del self.stream[:nb]
        self.pos += nb
        h = bytearray([0xF0, 0x00, 0x05, 0xE0, self.dfl >> 8, self.dfl & 255, 0x47, syncd >> 8, syncd & 255])
        h.append(crc8(h))
        bits = np.unpackbits(np.frombuffer(bytes(h) + data, np.uint8))
        return np.packbits(bits ^ bb_prbs(self.kbch)).tobytes()

    def plframe(self, bb):
        """BBFRAME bytes -> PLFRAME symbols as (I, Q) arrays of +-1."""
        par = self.bch.parity(bb)
        bits = np.unpackbits(np.frombuffer(bb + par, np.uint8))
        cw = self.ldpc.encode(bits)                      # QPSK: no bit interleaver
        si = 1 - 2 * cw[0::2].astype(np.int8)             # b0 -> I, b1 -> Q, bit 0 = +
        sq = 1 - 2 * cw[1::2].astype(np.int8)
        if self.pilots:                                  # 36 pilot symbols (1+j)/sqrt2 after every 16 slots, none after the last
            slots = len(si) // 90
            ai, aq = [], []
            for s in range(0, slots, 16):
                e = min(s + 16, slots)
                ai.append(si[90 * s:90 * e]); aq.append(sq[90 * s:90 * e])
                if e < slots:
                    ai.append(np.ones(36, np.int8)); aq.append(np.ones(36, np.int8))
            si, sq = np.concatenate(ai), np.concatenate(aq)
        r = rotation(len(si))                            # PL scrambling: multiply by 1, j, -1, -j
        ci = np.where(r == 0, si, np.where(r == 1, -sq, np.where(r == 2, -si, sq)))
        cq = np.where(r == 0, sq, np.where(r == 1, si, np.where(r == 2, -sq, -si)))
        return np.concatenate((self.hdr_i, ci)).astype(np.int8), np.concatenate((self.hdr_q, cq)).astype(np.int8)

    def plframe8(self, bb):
        """8PSK: BBFRAME bytes -> the PLFRAME as angle indices k (symbol = e^(j pi k / 4))."""
        par = self.bch.parity(bb)
        cw = self.ldpc.encode(np.unpackbits(np.frombuffer(bb + par, np.uint8)))
        cols = cw.reshape(3, len(cw) // 3)               # bit interleaver: written by columns, read by rows (columns reversed for 3/5)
        c = cols[::-1] if self.fec == "3/5" else cols
        k = np.array(PSK8_K, np.uint8)[(c[0] << 2 | c[1] << 1 | c[2]).astype(np.uint8)]
        if self.pilots:                                  # 36 pilot symbols (1+j)/sqrt2 = k 1 after every 16 slots, none after the last
            slots = len(k) // 90
            parts = []
            for s0 in range(0, slots, 16):
                e = min(s0 + 16, slots)
                parts.append(k[90 * s0:90 * e])
                if e < slots:
                    parts.append(np.ones(36, np.uint8))
            k = np.concatenate(parts)
        k = (k + 2 * rotation(len(k))) % 8               # PL scrambler: a quarter turn per unit of R
        return np.concatenate((self.hdr_k, k)).astype(np.uint8)

    def pack8(self, k):
        """Angle indices -> bytes, two symbols per byte, the first in the low nibble (an odd symbol waits for the next frame), or with bits3
        three bits per symbol (up to 7 symbols wait for the next frame)."""
        if self.invert:
            k = (8 - k) % 8                              # Q -> -Q
        if self.swap_iq:
            k = (2 - k) % 8                              # I <-> Q: the angle becomes 90 degrees minus the angle
        s = np.concatenate((self.pend, k.astype(np.uint8)))
        if self.bits3:                                   # a bit stream, symbol i at bits 3i..3i+2 (low bits first): 8 symbols make 3 bytes
            nr = len(s) // 8
            self.pend = s[8 * nr:]
            v = (s[:8 * nr].reshape(nr, 8).astype(np.uint32) << (3 * np.arange(8, dtype=np.uint32))).sum(axis=1)
            return np.stack(((v & 255), (v >> 8) & 255, (v >> 16) & 255), axis=1).astype(np.uint8).tobytes()
        nb = len(s) // 2
        self.pend = s[2 * nb:]
        q = s[:2 * nb].reshape(nb, 2)
        return (q[:, 0] | q[:, 1] << 4).astype(np.uint8).tobytes()

    def pack(self, si, sq):
        """+-1 symbols -> packed bytes (I level in bit 0, Q level in bit 1 of every 2 bit symbol); a PLFRAME is not a whole number
        of bytes, so up to 3 symbols wait for the next frame."""
        li, lq = (si > 0).astype(np.uint8), (sq > 0).astype(np.uint8)
        if self.invert:
            lq = 1 - lq
        if self.swap_iq:
            li, lq = lq, li
        s = np.concatenate((self.pend, li | (lq << 1)))
        nb = len(s) // 4
        self.pend = s[4 * nb:]
        q = s[:4 * nb].reshape(nb, 4)
        return (q[:, 0] | q[:, 1] << 2 | q[:, 2] << 4 | q[:, 3] << 6).astype(np.uint8).tobytes()

    def frames(self, ts):
        """Push packets and return the packed symbols of every PLFRAME that is complete now."""
        self.push(ts)
        out = []
        while self.ready():
            if self.mod == "8psk":
                out.append(self.pack8(self.plframe8(self.bbframe())))
            else:
                out.append(self.pack(*self.plframe(self.bbframe())))
        return b"".join(out)

    encode = frames                                     # the same call as dvbs.Encoder.encode


# ---------------------------------------------------------------- self-test
def _golden_ts():
    """120 transport stream packets from a small xorshift generator (the same on every numpy version)."""
    x = 0x2545F491
    out = bytearray()
    for _ in range(120):
        out.append(0x47)
        for _ in range(187):
            x ^= (x << 13) & 0xFFFFFFFF
            x ^= x >> 17
            x ^= (x << 5) & 0xFFFFFFFF
            out.append(x >> 11 & 255)
    return bytes(out)


GOLDEN = {('normal', '1/2', False): 'b6e638f7edd4f2f0', ('normal', '3/4', True): '0d3788b63e1cdb7d', ('short', '2/3', False): '7a12e0b7f21f23e2', ('short', '1/4', True): 'fad3eb819a6cd97a'}
GOLDEN_8PSK = {('normal', '2/3', True): '181d2d8334e78f7b', ('short', '3/5', False): '85f75b336f7d6f59'}


def selftest():
    rng = np.random.default_rng(5)
    ok = True
    # 1. LDPC: the encoded word satisfies all parity checks (H built from the tables, independent of the encoder code path)
    for frame in ("normal", "short"):
        for fec in PARAMS[frame]:
            L = Ldpc(frame, fec)
            info = rng.integers(0, 2, L.k, dtype=np.uint8)
            cw = L.encode(info)
            chk = np.zeros(L.nk, np.uint8)
            q = L.nk // 360
            for g, row in enumerate(TABLES[frame][fec]):
                for x in row:
                    for i in range(360):
                        chk[(x + q * i) % L.nk] ^= info[360 * g + i]
            # check node j: XOR of the information bits tied to it plus parity bits j and j-1 (accumulator): zero for a valid word
            par = cw[L.k:]
            res = chk ^ par ^ np.concatenate(([0], par[:-1]))
            good = not res.any()
            ok &= good
            print(f"LDPC {frame:6s} {fec:5s}: n={L.n} k={L.k} checks {'ok' if good else 'FAILED'}")
    # 2. BCH: the code word is divisible by g(x), and so are the first words of a valid BBFRAME
    for frame, fec in (("normal", "1/2"), ("normal", "2/3"), ("normal", "9/10"), ("short", "1/4"), ("short", "8/9")):
        k, t = PARAMS[frame][fec]
        b = Bch(frame, t)
        msg = rng.integers(0, 256, k // 8, dtype=np.uint8).tobytes()
        cw = int.from_bytes(msg + b.parity(msg), "big")
        r = cw
        while r.bit_length() > b.deg:
            r ^= b.g << (r.bit_length() - 1 - b.deg)
        print(f"BCH {frame} {fec}: g degree {b.deg}, code word divisible by g(x): {'ok' if r == 0 else 'FAILED'}")
        ok &= r == 0
    # 3. frame lengths, header, pilots
    ts = bytearray()
    for i in range(200):
        p = bytearray(rng.integers(0, 256, 188, dtype=np.uint8).tobytes())
        p[0] = 0x47
        ts += p
    for frame, fec, pil in (("normal", "1/2", False), ("normal", "3/4", True), ("short", "2/3", False), ("short", "1/4", True)):
        e = Encoder(fec, frame, pil)
        t0 = time.time()
        sym = e.frames(bytes(ts))
        n = (len(sym) * 4 + len(e.pend)) // e.nsym
        print(f"{frame} {fec} pilots {pil}: {n} PLFRAMEs of {e.nsym} symbols from {len(ts) // 188} packets in {time.time() - t0:.2f} s, TS rate at 1 MBd {ts_rate(1e6, fec, frame, pil) / 1e3:.0f} kb/s")
        ok &= len(sym) * 4 + len(e.pend) == n * e.nsym and n >= 1
    # 4. regression: hashes of the symbol streams of four modes for a fixed transport stream; these streams were compared with
    #    GNU Radio (dvbs2_vs_gnuradio.py) and decoded over the air by SatDump
    import hashlib
    for (frame, fec, pil), want in GOLDEN.items():
        got = hashlib.sha256(Encoder(fec, frame, pil).frames(_golden_ts())).hexdigest()[:16]
        ok &= got == want
        print(f"golden {frame} {fec} pilots {int(pil)}: {got} {'ok' if got == want else 'DIFFERENT, expected ' + want}")
    for (frame, fec, pil), want in GOLDEN_8PSK.items():
        got = hashlib.sha256(Encoder(fec, frame, pil, mod="8psk").frames(_golden_ts())).hexdigest()[:16]
        ok &= got == want
        print(f"golden 8PSK {frame} {fec} pilots {int(pil)}: {got} {'ok' if got == want else 'DIFFERENT, expected ' + want}")
    print("self-test", "passed" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if selftest() else 1)
