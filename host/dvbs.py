#!/usr/bin/env python3
"""DVB-S encoder (ETSI EN 300 421) for the ESP32-DATV transmitter: MPEG transport stream -> QPSK symbols.

Chain: energy dispersal (PRBS 1+x^14+x^15, restarted every 8 packets, inverted sync), RS(204,188,t=8), Forney
convolutional interleaver (I=12, M=17), K=7 convolutional code (G1=171 -> X, G2=133 -> Y) with puncturing 1/2, 2/3, 3/4,
5/6, 7/8, QPSK mapping (bit 0 = +1; I = X, Q = Y). Output: 4 symbols per byte, bit 0 = I level, bit 1 = Q level
(1 = +1), the first symbol in time in the lowest bits: this is what the QPSKT modulator in the ESP reads.

Self-test (encoder -> Viterbi decoder -> de-interleaver -> RS syndromes):  python3 dvbs.py
The encoder was also verified bit-exact against the independent leandvb decoder, in simulation and over the air.
"""
import sys
import time

import numpy as np

# ---- GF(256), x^8 + x^4 + x^3 + x^2 + 1 ----
_EXP = np.zeros(512, np.uint8)
_LOG = np.zeros(256, np.int32)
_x = 1
for _i in range(255):
    _EXP[_i] = _x
    _LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
_EXP[255:510] = _EXP[:255]


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return int(_EXP[_LOG[a] + _LOG[b]])


MUL = np.zeros((256, 256), np.uint8)
for _a in range(1, 256):
    for _b in range(1, 256):
        MUL[_a, _b] = _EXP[_LOG[_a] + _LOG[_b]]


def rs_generator():
    """g(x) = (x + a^0)(x + a^1) ... (x + a^15), highest power first (17 coefficients)."""
    g = [1]
    for i in range(16):
        r = int(_EXP[i])
        ng = [0] * (len(g) + 1)
        for k, c in enumerate(g):
            ng[k] ^= c
            ng[k + 1] ^= gf_mul(c, r)
        g = ng
    return np.array(g, np.uint8)


GEN = rs_generator()


def rs_parity(data):
    """data: (n, 188) uint8 -> (n, 16) parity (shortened code RS(255,239) -> RS(204,188))."""
    n = data.shape[0]
    R = np.zeros((n, 16), np.uint8)
    g = GEN[1:][None, :]
    for i in range(data.shape[1]):
        fb = data[:, i] ^ R[:, 0]
        R[:, :15] = R[:, 1:]
        R[:, 15] = 0
        R ^= MUL[fb[:, None], g]
    return R


def prbs_bytes():
    """1503 PRBS bytes (init 100101010000000, output = stages 14 xor 15)."""
    reg = [1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    bits = []
    for _ in range(1503 * 8):
        o = reg[13] ^ reg[14]
        bits.append(o)
        reg = [o] + reg[:-1]
    return np.packbits(np.array(bits, np.uint8))


PRBS = prbs_bytes()

# puncturing patterns: (X, Y) per period of input bits
PUNCT = {
    "1/2": ([1], [1]),
    "2/3": ([1, 0], [1, 1]),
    "3/4": ([1, 0, 1], [1, 1, 0]),
    "5/6": ([1, 0, 1, 0, 1], [1, 1, 0, 1, 0]),
    "7/8": ([1, 0, 0, 0, 1, 0, 1], [1, 1, 1, 1, 0, 1, 0]),
}
FEC_RATE = {"1/2": 1 / 2, "2/3": 2 / 3, "3/4": 3 / 4, "5/6": 5 / 6, "7/8": 7 / 8}
NULL_PACKET = bytes([0x47, 0x1F, 0xFF, 0x10]) + b"\xFF" * 184


def ts_rate(baud, fec):
    """Useful transport-stream bit rate [bit/s] at a given symbol rate."""
    return baud * 2 * FEC_RATE[fec] * 188 / 204


class Encoder:
    def __init__(self, fec="1/2", swap_iq=False, invert=False):
        if fec not in PUNCT:
            raise ValueError("FEC: " + ", ".join(PUNCT))
        self.fec = fec
        self.xp, self.yp = (np.array(p, bool) for p in PUNCT[fec])
        self.swap_iq = swap_iq
        self.invert = invert                 # spectrum inversion: Q -> -Q
        self.pkt = 0                         # packet number within the group of 8
        self.hist = np.zeros(204 * 11, np.uint8)
        self.cstate = np.zeros(6, np.uint8)
        self.cpos = 0                        # position within the puncturing period
        self.pend_bit = np.zeros(0, np.uint8)
        self.pend_sym = np.zeros(0, np.uint8)

    def encode(self, ts):
        """ts: bytes, a multiple of 188 (every packet starts with 0x47) -> packed symbol bytes."""
        p = np.frombuffer(ts, np.uint8).reshape(-1, 188).copy()
        n = len(p)
        if n == 0:
            return b""
        if not np.all(p[:, 0] == 0x47):
            raise ValueError("TS packet without the 0x47 sync byte")
        # energy dispersal: the PRBS runs through all 8 packets (sync bytes skipped), the first sync is inverted
        k = (self.pkt + np.arange(n)) % 8
        for g in range(8):
            rows = np.nonzero(k == g)[0]
            if len(rows):
                p[rows, 1:] ^= PRBS[188 * g:188 * g + 187]
        p[k == 0, 0] = 0xB8
        self.pkt = (self.pkt + n) % 8
        # RS
        cw = np.concatenate((p, rs_parity(p)), axis=1).reshape(-1)
        # Forney interleaver: out(t) = in(t - 204 * (t mod 12))
        x = np.concatenate((self.hist, cw))
        t = np.arange(len(cw))
        out = x[len(self.hist) + t - 204 * (t % 12)]
        self.hist = x[-204 * 11:].copy()
        # convolutional code
        bits = np.unpackbits(out)
        b = np.concatenate((self.cstate, bits))
        self.cstate = bits[-6:].copy()
        X = b[6:] ^ b[5:-1] ^ b[4:-2] ^ b[3:-3] ^ b[:-6]                      # 171: delays 0, 1, 2, 3, 6
        Y = b[6:] ^ b[4:-2] ^ b[3:-3] ^ b[1:-5] ^ b[:-6]                      # 133: delays 0, 2, 3, 5, 6
        P = len(self.xp)
        ph = (self.cpos + np.arange(len(bits))) % P
        self.cpos = (self.cpos + len(bits)) % P
        keep = np.stack((self.xp[ph], self.yp[ph]), axis=1)
        coded = np.stack((X, Y), axis=1)[keep]                                 # X1 Y1 Y2 ... in time order
        s = np.concatenate((self.pend_bit, coded))
        ns = len(s) // 2
        self.pend_bit = s[2 * ns:]
        xi, yq = s[0:2 * ns:2], s[1:2 * ns:2]
        li, lq = 1 - xi, 1 - yq                                                # bit 0 -> +1
        if self.invert:
            lq = 1 - lq
        if self.swap_iq:
            li, lq = lq, li
        sym = np.concatenate((self.pend_sym, (li | (lq << 1)).astype(np.uint8)))
        nb = len(sym) // 4
        self.pend_sym = sym[4 * nb:]
        q = sym[:4 * nb].reshape(nb, 4)
        return (q[:, 0] | q[:, 1] << 2 | q[:, 2] << 4 | q[:, 3] << 6).astype(np.uint8).tobytes()


# ---------------------------------------------------------------- self-test: reference decoder

def _viterbi_half(xy):
    """Hard-decision decoder for the rate 1/2 code: xy (n, 2) bits X, Y -> n information bits (initial state 0)."""
    n = len(xy)
    ns = np.arange(64)                                   # state = the 6 previous bits, bit 0 = newest
    bit = ns & 1
    pred = (ns >> 1, (ns >> 1) | 32)                     # the two predecessor states of every state
    ox, oy = [], []
    for pr in pred:
        r7 = (pr << 1) | bit                             # bit 0 = x_n, bit 1 = x_{n-1}, ...
        ox.append(((r7 >> 0) ^ (r7 >> 1) ^ (r7 >> 2) ^ (r7 >> 3) ^ (r7 >> 6)) & 1)
        oy.append(((r7 >> 0) ^ (r7 >> 2) ^ (r7 >> 3) ^ (r7 >> 5) ^ (r7 >> 6)) & 1)
    pm = np.full(64, 10 ** 6)
    pm[0] = 0
    prev = np.zeros((n, 64), np.int16)
    for i in range(n):
        m0 = pm[pred[0]] + (ox[0] != xy[i, 0]) + (oy[0] != xy[i, 1])
        m1 = pm[pred[1]] + (ox[1] != xy[i, 0]) + (oy[1] != xy[i, 1])
        ch = m1 < m0
        pm = np.where(ch, m1, m0)
        prev[i] = np.where(ch, pred[1], pred[0])
    st = int(np.argmin(pm))
    bits = np.zeros(n, np.uint8)
    for i in range(n - 1, -1, -1):
        bits[i] = st & 1
        st = prev[i, st]
    return bits


def selftest():
    rng = np.random.default_rng(3)
    n = 24
    ts = bytearray()
    for i in range(n):
        pk = bytearray(rng.integers(0, 256, 188, dtype=np.uint8).tobytes())
        pk[0] = 0x47
        ts += pk
    enc = Encoder("1/2")
    t0 = time.time()
    sym = enc.encode(bytes(ts))
    print(f"encoded {n} TS packets in {time.time() - t0:.3f} s -> {len(sym)} symbol bytes ({len(sym) * 4} symbols, expected {n * 204 * 8})")
    # PRBS: the first bytes of the dispersal sequence (hand-derived from the register definition: 0000 0011 1111 0110 ...)
    print("PRBS first bytes:", " ".join(f"{b:02X}" for b in PRBS[:6]), "(expected 03 F6 ...)")
    # RS: cross-check against an independent library when it is installed
    try:
        import reedsolo
        rsc = reedsolo.RSCodec(16, nsize=255, fcr=0, prim=0x11D, generator=2)
        d = rng.integers(0, 256, (5, 188), dtype=np.uint8)
        mine = rs_parity(d)
        ok = all(bytes(rsc.encode(bytes(d[i]))[188:]) == bytes(mine[i]) for i in range(5))
        print("RS parity matches reedsolo:", ok)
    except ImportError:
        print("reedsolo not installed - RS is checked through syndromes only")
    # closed-loop decoding
    s = np.frombuffer(sym, np.uint8)
    s = np.stack((s & 3, s >> 2 & 3, s >> 4 & 3, s >> 6 & 3), axis=1).reshape(-1)
    li, lq = s & 1, s >> 1 & 1
    xy = np.stack((1 - li, 1 - lq), axis=1)
    bits = _viterbi_half(xy)
    by = np.packbits(bits)
    # the decoder output is the interleaved stream; de-interleave: branch j delays by 204 * (11 - j), so with the
    # interleaver every byte is delayed by 11 * 204
    L = len(by)
    t = np.arange(204 * 11, L)
    de = by[t - 204 * (11 - (t % 12))]                   # de[k] = input byte number k
    total = len(de) // 204 - 1                           # the last packet may not be complete yet
    good = 0
    for k in range(total):
        pk = de[204 * k:204 * (k + 1)]
        syn = 0
        for i in range(16):                              # syndromes: the code polynomial evaluated at a^0..a^15 is 0
            r = 0
            for c in pk:
                r = gf_mul(r, int(_EXP[i])) ^ int(c)
            syn |= r
        if syn == 0:
            good += 1
    print(f"RS: {good}/{total} packets error-free after Viterbi and de-interleaving")
    return good == total and total > 0


if __name__ == "__main__":
    ok = selftest()
    sys.exit(0 if ok else 1)
