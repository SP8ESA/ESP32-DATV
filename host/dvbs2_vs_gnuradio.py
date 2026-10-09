#!/usr/bin/env python3
"""Compares dvbs2.py with the DVB-S2 transmitter of GNU Radio (gr-dtv) stage by stage: BBFRAME, BB scrambler, BCH, LDPC, QPSK mapping,
PLFRAME (header, pilots, PL scrambler). Needs GNU Radio 3.10 with gr-dtv; only for development.

  python3 dvbs2_vs_gnuradio.py [normal|short [fec [pilots [qpsk|8psk|16apsk|32apsk]]]]     (no arguments: a set of QPSK, 8PSK, 16APSK and 32APSK modes)

16APSK and 32APSK: the header and the pilots are sent on the outer ring (radius r2, not 1), so the PLFRAME is compared on the data symbols exactly and on the
header and pilot symbols by their phase only.
"""
import sys

import numpy as np
from gnuradio import blocks, dtv, gr

import dvbs2

RATES = {"1/4": dtv.C1_4, "1/3": dtv.C1_3, "2/5": dtv.C2_5, "1/2": dtv.C1_2, "3/5": dtv.C3_5, "2/3": dtv.C2_3, "3/4": dtv.C3_4,
         "4/5": dtv.C4_5, "5/6": dtv.C5_6, "8/9": dtv.C8_9, "9/10": dtv.C9_10}
FRAMES = {"normal": dtv.FECFRAME_NORMAL, "short": dtv.FECFRAME_SHORT}


def gr_chain(ts, fec, frame, pilots, upto, mod="qpsk"):
    fs, r = FRAMES[frame], RATES[fec]
    gmod = {"8psk": dtv.MOD_8PSK, "16apsk": dtv.MOD_16APSK, "32apsk": dtv.MOD_32APSK}.get(mod, dtv.MOD_QPSK)
    tb = gr.top_block()
    src = blocks.vector_source_b(list(ts), False)
    order = [dtv.dvb_bbheader_bb(dtv.STANDARD_DVBS2, fs, r, dtv.RO_0_35, dtv.INPUTMODE_NORMAL, dtv.INBAND_OFF, 0, 0),
             dtv.dvb_bbscrambler_bb(dtv.STANDARD_DVBS2, fs, r),
             dtv.dvb_bch_bb(dtv.STANDARD_DVBS2, fs, r),
             dtv.dvb_ldpc_bb(dtv.STANDARD_DVBS2, fs, r, dtv.MOD_OTHER),
             dtv.dvbs2_interleaver_bb(fs, r, gmod),
             dtv.dvbs2_modulator_bc(fs, r, gmod, dtv.INTERPOLATION_OFF),
             dtv.dvbs2_physical_cc(fs, r, gmod, dtv.PILOTS_ON if pilots else dtv.PILOTS_OFF, 0)]
    names = ["bbh", "scr", "bch", "ldpc", "itl", "mod", "phys"]
    blks = order[:names.index(upto) + 1]
    sink = blocks.vector_sink_c() if upto in ("mod", "phys") else blocks.vector_sink_b()
    tb.connect(src, blks[0])
    for a, b in zip(blks[:-1], blks[1:]):
        tb.connect(a, b)
    tb.connect(blks[-1], sink)
    tb.run()
    return np.array(sink.data())


def compare(fec, frame, pilots, npk=240, nframes=2, mod="qpsk", apsk_pl="outer"):
    rng = np.random.default_rng(hash((fec, frame, pilots, mod)) & 0xFFFF)
    ts = bytearray()
    for i in range(npk):
        p = bytearray(rng.integers(0, 256, 188, dtype=np.uint8).tobytes())
        p[0] = 0x47
        ts += p
    e = dvbs2.Encoder(fec, frame, pilots, mod=mod, apsk_pl=apsk_pl)
    e.push(bytes(ts))
    kb, nl = e.kbch, e.ldpc.n
    mine = {"scr": [], "bch": [], "ldpc": [], "mod": [], "phys": []}
    for _ in range(nframes):
        bb = e.bbframe()
        bits = np.unpackbits(np.frombuffer(bb, np.uint8))
        mine["scr"].append(bits)
        par = e.bch.parity(bb)
        bchbits = np.unpackbits(np.frombuffer(bb + par, np.uint8))
        mine["bch"].append(bchbits)
        cw = e.ldpc.encode(bchbits)
        mine["ldpc"].append(cw)
        if mod == "32apsk":
            c5 = cw.reshape(5, len(cw) // 5)
            pts = dvbs2.apsk32_points(fec)
            mine["mod"].append(pts[(c5[0] << 4 | c5[1] << 3 | c5[2] << 2 | c5[3] << 1 | c5[4]).astype(np.uint8)])
            plpts = np.concatenate((pts, np.exp(1j * np.radians([45, 135, -135, -45]))))
            mine["phys"].append(plpts[e.plframe32(bb)])
        elif mod == "16apsk":
            c4 = cw.reshape(4, len(cw) // 4)
            pts = dvbs2.apsk16_points(fec)
            mine["mod"].append(pts[(c4[0] << 3 | c4[1] << 2 | c4[2] << 1 | c4[3]).astype(np.uint8)])
            mine["phys"].append(pts[e.plframe16(bb)])
        elif mod == "8psk":
            cols = cw.reshape(3, len(cw) // 3)
            c = cols[::-1] if fec == "3/5" else cols
            kk = np.array(dvbs2.PSK8_K)[c[0] << 2 | c[1] << 1 | c[2]]
            mine["mod"].append(np.exp(1j * np.pi * kk / 4))
            mine["phys"].append(np.exp(1j * np.pi * e.plframe8(bb) / 4))
        else:
            mine["mod"].append((1 - 2 * cw[0::2].astype(np.float64)) / np.sqrt(2) + 1j * (1 - 2 * cw[1::2].astype(np.float64)) / np.sqrt(2))
            # PLFRAME from the same BBFRAME (the encoder state is only consumed by bbframe)
            si, sq = e.plframe(bb)
            mine["phys"].append((si + 1j * sq) / np.sqrt(2))
    ok = True
    res = []
    for stage in ("scr", "bch", "ldpc", "mod", "phys"):
        g = gr_chain(bytes(ts), fec, frame, pilots, stage, mod)
        m = np.concatenate(mine[stage])
        if stage == "phys":
            g = g[0::2]                                         # gr-dtv zero-stuffs the PLFRAME by two
        n = min(len(m), len(g))
        if stage == "phys" and mod in ("16apsk", "32apsk") and apsk_pl == "outer":
            # data symbols exactly; header (90 symbols) and pilots (36 after every 16 slots): the phase
            nd = len(m) // nframes
            mask = np.zeros(nd, bool)
            mask[:90] = True
            if pilots:
                slots = (64800 if frame == "normal" else 16200) // (4 if mod == "16apsk" else 5) // 90
                pos = 90
                for s0 in range(0, slots, 16):
                    e0 = min(s0 + 16, slots)
                    pos += 90 * (e0 - s0)
                    if e0 < slots:
                        mask[pos:pos + 36] = True
                        pos += 36
            mask = np.tile(mask, nframes)[:n]
            good = np.allclose(g[:n][~mask], m[:n][~mask], atol=1e-5) and np.allclose(np.angle(g[:n][mask]), np.angle(m[:n][mask]), atol=1e-5)
        elif stage in ("mod", "phys"):
            good = np.allclose(g[:n], m[:n], atol=1e-5)
        else:
            good = np.array_equal(g[:n].astype(np.uint8), m[:n])
        res.append(f"{stage}:{'ok' if good else 'DIFF'}({n})")
        ok &= good
        if not good:
            bad = np.nonzero(~np.isclose(g[:n], m[:n], atol=1e-5) if stage in ("mod", "phys") else g[:n].astype(np.uint8) != m[:n])[0]
            res.append(f"first diff at {bad[0]}")
            break
    print(f"{mod:5s} {frame:6s} {fec:5s} pilots {int(pilots)}: " + " ".join(res), flush=True)
    return ok


if __name__ == "__main__":
    if len(sys.argv) > 1:
        modes = [(sys.argv[2] if len(sys.argv) > 2 else "1/2", sys.argv[1], len(sys.argv) > 3 and sys.argv[3] == "1", 240, 2, sys.argv[4] if len(sys.argv) > 4 else "qpsk")]
    else:
        modes = [("1/2", "normal", False), ("1/2", "normal", True), ("1/4", "normal", False), ("2/3", "normal", True), ("3/4", "normal", False), ("5/6", "normal", True),
                 ("9/10", "normal", False), ("1/4", "short", False), ("1/2", "short", True), ("8/9", "short", False)]
        modes += [(f, fr, p, 240, 2, "8psk") for f, fr, p in (("3/5", "normal", False), ("2/3", "normal", True), ("5/6", "normal", False), ("9/10", "normal", True),
                                                              ("3/5", "short", True), ("8/9", "short", False))]
        modes += [(f, fr, p, 240, 2, "16apsk") for f, fr, p in (("2/3", "normal", False), ("3/4", "normal", True), ("4/5", "normal", False), ("5/6", "normal", True),
                                                                ("8/9", "normal", False), ("9/10", "normal", True), ("2/3", "short", True), ("8/9", "short", False))]
        modes += [(f, fr, p, 240, 2, "32apsk") for f, fr, p in (("3/4", "normal", False), ("4/5", "normal", True), ("5/6", "normal", False), ("8/9", "normal", True),
                                                                ("9/10", "normal", False), ("3/4", "short", True), ("8/9", "short", False))]
    good = all([compare(*m) for m in modes])
    print("ALL MATCH" if good else "MISMATCH")
    sys.exit(0 if good else 1)
