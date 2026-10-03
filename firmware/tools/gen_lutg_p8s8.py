#!/usr/bin/env python3
"""Generates main/lutg_p8s8.S: the cycle-deterministic 8PSK / RRC modulator for 1 MBd (8 samples per symbol, one DAC store every 20 CPU
cycles = 8 MS/s). Function lutg_p8s8_run_p20.

The generic 8PSK loop (gen_lutg.py --psk8) needs S >= 16 slots per symbol for its per-symbol work, so 1 MBd gets its own, fully unrolled
loop with no inner loop at all: one pass = 8 symbols = 64 slots (the same shape as lut8.S).

Table layout (qpsk_lut.h, lut_build_p8): 4 groups of 2 symbols, 64 rows each, the 64 words of one (group, sample) are 256 bytes apart from
the next sample. The context holds the group bases PLUS 1024 bytes, so a row pointer = base + 4 * idx points at sample 4 and the 8 samples
are reached with the immediates -1024 .. +768 (no pointer adjustment inside a symbol, no inner loop).

Symbol stream (3 bits per symbol, the USB bytes are the stream as is): symbol i occupies bits 3i .. 3i+2 of the byte stream, least
significant bit first. A record of 3 bytes = 8 symbols, so a pass consumes exactly one record. Link budget: 3 Mb/s = 375 kB/s of the
about 430 kB/s the USB port takes when the loop reads one byte per symbol (the nibble format of the generic loop would need 500 kB/s).
The symbols are taken from a bit window in s9 (V valid bits, new bytes are merged on top of them), so a symbol costs 4 instructions and
no record boundary has to be handled: V is tracked statically by this generator (the merge shifts are immediates).

Per symbol k of a pass (the work for the symbol k + 1, in slots 0..5; pointers alternate between the sets s1..s4 / s5..s8):
  slot 0     r_k out of the window into the history a2, next-symbol row pointer of group 0
  slot 1,2   row pointers of the groups 1, 2, 3   (the pointer of a group = base + 4 * ((a2 >> 6g) & 63))
  slot 1..5  USB: room in the ring, is a byte waiting, read it, put it into the ring (N copy)
  slot 6,7   even symbols: spare, merges of the three bytes of the next record (symbols 2, 4, 6) / bookkeeping;
             odd symbols: D (measure) and E (jump into the sync sled), i.e. a sync every two symbols
Two code copies: N (normal, reads the USB) and M (maintenance: silence check, then the fill report and the underrun count, written to the USB
FIFO in the symbols after each other, then the flush); M replaces every (mper + 1)th pass. (gen_lutg.py has three maintenance copies of one
symbol each; a pass here is 8 symbols and the code of a copy is 5.6 KB, which the IRAM can not afford four times.)

usage (from firmware/):  python3 tools/gen_lutg_p8s8.py > main/lutg_p8s8.S
  --rec timing | words   the recording builds (main.c: #define LUTG_REC 1 | 2), see gen_lutg.py; the records of a pass are 64 consecutive words
  --raw                  no padding at all
Padding per slot in lutg_p8s8_pads.json ({"20": {"N5": 1, ...}}, keys <copy><slot 0..63>; D and E are slots 14, 15, 30, 31, 46, 47, 62, 63).
"""
import json
import os
import re
import sys

REC = "none"
if "--rec" in sys.argv:
    REC = sys.argv[sys.argv.index("--rec") + 1]
assert REC in ("none", "timing", "words")
SCHED = "--nosched" not in sys.argv
RAW = "--raw" in sys.argv
HERE = os.path.dirname(os.path.abspath(__file__))
PADFILE = os.path.join(HERE, "lutg_p8s8_pads.json")
PADS_ALL = json.load(open(PADFILE)) if os.path.exists(PADFILE) else {}

P = 20
S = 8
NSYM = 8                 # symbols per pass
SLED = 24
SYNC = 2 * S * P         # cycles between two sync points (a pair of symbols)
COPIES = ("N", "M")
CTX = dict(T0=0, T1=4, T2=8, T3=12, RING=16, QW=20, QR=24, TN=28, NSYM=32, UNDER=36, MPER=40, MCNT=44, TRX=48, QWSEEN=52, LIM=56, LIM1=60,
           EXITC=64, LATE=68, S64M=72, SPER=76, RLIM=80, NS_N=84, NS_A=88, NS_B=92, NS_C=96, SLEDREC=100, REC=104, RECP=108)
KERN = 8
o = []
emit = o.append
EFFECTIVE = {}
PADS = {}
uid = [0]


def lab(name):
    uid[0] += 1
    return f".L{name}{uid[0]}"


def nops(n):
    """n one-cycle nops in as few bytes as possible: 2 byte c.nops in even numbers (the next instruction stays 4 byte aligned), an odd one is a 4 byte nop.
    The IRAM is the scarce resource here: the saved bytes keep the heap below the RF dump bank big enough for the symbol ring."""
    if n <= 0:
        return ""
    t = "    nop\n" if n % 2 else ""
    if n // 2:
        t += f"    .option push\n    .option rvc\n    .rept {2 * (n // 2)}\n    c.nop\n    .endr\n    .option pop\n"
    return t.rstrip("\n")


class Slot:
    def __init__(self, key, pads):
        self.key, self.pads = key, pads
        self.lines = []
        self.cost = 0.0

    def add(self, lines, cost):
        self.lines += lines if isinstance(lines, list) else [lines]
        self.cost += cost

    def finish(self):
        est = round(P - self.cost)
        n = 0 if RAW else self.pads.get(self.key, est)
        if n < 0:
            sys.stderr.write(f"WARNING slot {self.key}: estimated {self.cost:.1f} cycles, over the period\n")
            n = 0
        EFFECTIVE[self.key] = n
        src = self.lines
        pre = 0 if RAW else self.pads.get(self.key + "p", 0)
        grp = 3 if REC != "none" else 2                 # lines of the store group (production: the store and one block of two nops)
        if pre and src[0].startswith("    sw   t3, 0(a6)"):
            EFFECTIVE[self.key + "p"] = pre
            src = src[:grp] + [nops(pre)] + src[grp:]
        lines = schedule(src) if SCHED else src
        if n:
            lines = lines + [nops(n)]
        return "\n".join(lines)


def store_group(n):
    off = 4 * n
    if REC == "timing":
        return ["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", f"    sw   ra, {off}(a0)"], 3
    if REC == "words":
        return ["    sw   t3, 0(a6)", "    mv   ra, t3", f"    sw   ra, {off}(a0)"], 3
    return ["    sw   t3, 0(a6)", nops(2)], 3


def kern(imm, regs):
    r = regs
    return [f"    lw   t0, {imm}({r[0]})", f"    lw   t1, {imm}({r[1]})", f"    lw   t2, {imm}({r[2]})", f"    lw   t4, {imm}({r[3]})",
            "    add  t0, t0, t1", "    add  t0, t0, t2", "    add  t0, t0, t4", "    xor  t3, t0, a5"]


SETS = (("s1", "s2", "s3", "s4"), ("s5", "s6", "s7", "s8"))


def cur(k):
    return SETS[k % 2]


def nxt(k):
    return SETS[(k + 1) % 2]


# ------------------------------------------------------------------ instruction scheduler (as in gen_lutg.py)
REG_RE = re.compile(r"\b(ra|sp|gp|tp|t[0-6]|s(?:1[01]|[0-9])|a[0-7])\b")
LOADS = ("lw", "lbu", "lhu", "lb", "lh")
STORES = ("sw", "sb", "sh")


def parse_ins(line):
    t = line.strip()
    if not t or t.startswith((".", "/*")) or t.endswith(":"):
        return None
    op, _, rest = t.partition(" ")
    if op in ("beqz", "bnez", "bne", "beq", "blt", "bge", "bltu", "bgeu", "bgtz", "blez", "j", "jal", "jalr", "ret", "la"):
        return None
    regs = REG_RE.findall(rest)
    if op in STORES:
        return (op, None, regs, "store")
    if op in LOADS:
        return (op, regs[0], regs[1:], "load")
    if op == "nop":
        return (op, None, [], "alu")
    return (op, regs[0], regs[1:], "alu")


def schedule(lines):
    out, seg = [], []

    def flush():
        if not seg:
            return
        n = len(seg)
        ins = [parse_ins(l) for l in seg]
        dep = [set() for _ in range(n)]
        for j in range(n):
            oj, dj, sj, kj = ins[j]
            for i in range(j):
                oi, di, si, ki = ins[i]
                if (di and di in sj) or (dj and dj in si) or (di and dj and di == dj):
                    dep[j].add(i)
                if ("store" in (ki, kj)) and kj != "alu" and ki != "alu":
                    dep[j].add(i)
        done, order = set(), []
        last_load = None
        while len(order) < n:
            cand = [j for j in range(n) if j not in done and dep[j] <= done]
            ok = [j for j in cand if last_load is None or last_load not in ins[j][2]]
            loads = [j for j in ok if ins[j][3] == "load"]
            pick = loads[0] if loads else ok[0] if ok else cand[0]
            done.add(pick)
            order.append(pick)
            last_load = ins[pick][1] if ins[pick][3] == "load" else None
        out.extend(seg[j] for j in order)
        seg.clear()

    for l in lines:
        if "\n" in l or parse_ins(l) is None:
            flush()
            out.append(l)
        else:
            seg.append(l)
    flush()
    return out


# ------------------------------------------------------------------ events: (lines, cycles). Registers: t0-t4 belong to the kern, s9 = bit window,
# a2 = history, a7 = bytes consumed, s11 = bytes written, a4 = room flag, tp = byte read from the USB, t5/t6/gp temporaries
def ev_ext():
    """symbol r_k out of the window into the history"""
    return ["    andi t6, s9, 7", "    srli s9, s9, 3", "    slli a2, a2, 3", "    or   a2, a2, t6"], 4


def ev_nptr(k, g):
    """row pointer of group g for the symbol k + 1 (into the set that is not in use): base + 4 * ((a2 >> 6g) & 63)"""
    dest = nxt(k)[g]
    sh = {0: "slli t6, a2, 2", 1: "srli t6, a2, 4", 2: "srli t6, a2, 10", 3: "srli t6, a2, 16"}[g]
    return [f"    lw   {dest}, T{g}(s0)", f"    {sh}", "    andi t6, t6, 252", f"    add  {dest}, {dest}, t6"], 4


def ev_rm():
    return ["    lw   t6, RLIM(s0)", "    sub  t5, s11, a7", "    slt  a4, t5, t6"], 3


def ev_av():
    return ["    lw   t5, 4(a3)", "    srli t5, t5, 2"], 7


def ev_avc():
    return ["    andi t5, t5, 1", "    and  a4, a4, t5"], 2


def ev_rd(stubs):
    nd, back = lab("nd"), lab("rb")
    stubs.append(f"{nd}:\n{nops(PADS.get('RDNOPS', 2))}\n    j    {back}")
    return [f"    beqz a4, {nd}", "    lw   tp, 0(a3)", f"{back}:"], 7


def ev_sb():
    return ["    lw   t6, RING(s0)", "    slli t5, s11, 18", "    srli t5, t5, 18", "    add  t5, t5, t6", "    sb   tp, 0(t5)", "    add  s11, s11, a4"], 6


def ev_addr(i):
    """t5 = ring address of byte i of the next record"""
    L = ["    lw   t6, RING(s0)"]
    if i:
        L.append(f"    addi t5, a7, {i}")
        src = "t5"
    else:
        src = "a7"
    L += [f"    slli t5, {src}, 18", "    srli t5, t5, 18", "    add  t5, t5, t6"]
    return L, len(L)


def ev_load(v):
    """merge the byte at t5 into the window above its v valid bits"""
    return ["    lbu  t5, 0(t5)", f"    slli t5, t5, {v}", "    or   s9, s9, t5"], 4


def ev_have():
    return ["    sub  gp, s11, a7", "    addi gp, gp, -2", "    slt  gp, zero, gp"], 3


def ev_adv():
    return ["    slli t5, gp, 1", "    add  t5, t5, gp", "    add  a7, a7, t5"], 3


def ev_under():
    return ["    lw   t5, UNDER(s0)", "    xori t6, gp, 1", "    add  t5, t5, t6", "    sw   t5, UNDER(s0)"], 4


def ev_tgt():
    return [f"    addi a1, a1, {SYNC}"], 1


def ev_nsym(stubs):
    x = lab("x")
    stubs.append(f"{x}:\n    j    lutg_p8s8_exit")
    return ["    lw   tp, NSYM(s0)", f"    addi tp, tp, -{NSYM}", "    sw   tp, NSYM(s0)", f"    beqz tp, {x}"], 4


def ev_mcn_a():
    return ["    lw   tp, MCNT(s0)", "    addi tp, tp, -1", "    sw   tp, MCNT(s0)", "    seqz tp, tp"], 4


def ev_mcn_b():
    return ["    slli tp, tp, 2", "    add  tp, tp, s0", "    lw   s10, NS_N(tp)"], 3


def ev_sil1():
    return ["    lw   t5, QWSEEN(s0)", "    sub  t5, s11, t5", "    snez t5, t5", "    neg  t5, t5", "    csrr t6, 0x7e2"], 5


def ev_sil2():
    return ["    lw   gp, TRX(s0)", "    xor  tp, gp, t6", "    and  tp, tp, t5", "    xor  gp, gp, tp", "    sw   gp, TRX(s0)", "    sw   s11, QWSEEN(s0)",
            "    sub  t6, t6, gp"], 7


def ev_sil3a(stubs):
    x = lab("s")
    stubs.append(f"{x}:\n    li   t5, 2\n    sw   t5, EXITC(s0)\n    j    lutg_p8s8_exit")
    return ["    lw   t5, LIM(s0)", f"    bltu t5, t6, {x}", "    snez t6, s11", "    neg  t6, t6"], 4


def ev_sil3b():
    return ["    lw   gp, LIM1(s0)", "    xor  tp, t5, gp", "    and  tp, tp, t6", "    xor  t5, t5, tp", "    sw   t5, LIM(s0)"], 5


def ev_next(nsname, reload=False):
    L = [f"    lw   s10, {nsname}(s0)"]
    if reload:
        L += ["    lw   t5, MPER(s0)", "    sw   t5, MCNT(s0)"]
    return L, 3 if reload else 1


def ev_rep_calc():
    """fill report: marker 0xB7 in gp, fill (pairs of bytes) low byte in tp, high byte in a4"""
    return ["    sub  t5, s11, a7", "    srli t5, t5, 1", "    li   gp, 0xB7", "    andi tp, t5, 255", "    srli a4, t5, 8"], 5


def ev_wr(reg):
    return [f"    sw   {reg}, 0(a3)"], 8.1


def ev_rep_under():
    """underruns: count in tp, 1 (the flush) in a4"""
    return ["    lw   tp, UNDER(s0)", "    andi tp, tp, 255", "    li   a4, 1"], 3


# ------------------------------------------------------------------ the plan of one code copy: {slot: [events]}
MERGE_SYMS = (2, 4, 6)            # the symbols whose slot 6 merges byte 0, 1, 2 of the next record into the window
V0 = 24                           # valid bits in the window at the start of a pass (the prologue loads a whole record)


def build_pass(c, stubs):
    pl = {n: [] for n in range(NSYM * S)}

    def at(k, j, *evs):
        pl[8 * k + j] += list(evs)

    v = V0
    for k in range(NSYM):
        v -= 3
        at(k, 0, ev_ext(), ev_nptr(k, 0))
        at(k, 1, ev_nptr(k, 1), ev_nptr(k, 2))
        if c == "N":
            sp = stubs[k // 2]
            at(k, 2, ev_nptr(k, 3), ev_rm())
            at(k, 3, ev_av())
            at(k, 4, ev_avc(), ev_rd(sp))
            at(k, 5, ev_sb())
        else:
            at(k, 2, ev_nptr(k, 3))
        if k in MERGE_SYMS:
            i = MERGE_SYMS.index(k)
            at(k, 7 if (c == "M" and k == 2) else 6, ev_addr(i), ev_load(v))
            v += 8
        if k % 2 == 1:
            at(k, 5, ev_tgt())
    assert v == V0, f"window not periodic: {v}"
    at(0, 6, ev_nsym(stubs[0]))
    at(0, 7, ev_under())                      # counts the record flag (gp) of the previous pass; nothing between touches gp (the maintenance events use it later in the pass)
    at(6, 7, ev_have(), ev_adv())
    if c == "N":
        at(4, 7, ev_mcn_a(), ev_mcn_b())
    else:
        at(2, 3, ev_sil1())                       # silence check
        at(2, 4, ev_sil2())
        at(2, 5, ev_sil3a(stubs[1]))
        at(2, 6, ev_sil3b())
        at(3, 5, ev_rep_calc())                   # fill report: marker, low byte, high byte
        at(4, 3, ev_wr("gp"))
        at(4, 4, ev_wr("tp"))
        at(4, 5, ev_wr("a4"))
        at(5, 5, ev_rep_under())                  # underruns, then the flush
        at(6, 3, ev_wr("tp"))
        at(6, 4, (["    sw   a4, 4(a3)"], 8.1))
        at(4, 7, ev_next("NS_N", reload=True))
    return pl


def copy_code(c, pads):
    stubs = [[] for _ in range(NSYM // 2)]
    pl = build_pass(c, stubs)
    out = []
    for m in range(NSYM // 2):
        sled_end = f".LsE{c}{m}"
        out.append(f"    .align 4\nlutg_p8s8_copy_{c}{m}:\n{nops(SLED)}\n{sled_end}:")
        for jj in range(2 * S):
            n = 16 * m + jj
            k, j = n // S, n % S
            s = Slot(f"{c}{n}", pads)
            st, sc = store_group(n)
            s.add(st, sc)
            if jj == 14:                                   # D: measure
                s.add(kern(768, cur(k)), KERN)
                hand, back = f".Lh{c}{m}", f".Lb{c}{m}"
                last = m == NSYM // 2 - 1
                tgt = ["    sub  t6, s10, t5"] if last else [f"    la   t6, .LsE{c}{m + 1}", "    sub  t6, t6, t5"]
                s.add(["    csrr t5, 0x7e2", "    sub  t5, a1, t5", f"    sltiu t6, t5, {SLED + 1}", f"    beqz t6, {hand}", f"{back}:", "    slli t5, t5, 1"] + tgt,
                      5 + (1 if last else 3))
                if REC == "timing":
                    s.add("    sw   t5, SLEDREC(s0)", 1)
                else:
                    s.add("    nop", 1)
                out.append(f"/* {c} slot {n} (D, measure) */\n" + s.finish())
                stubs[m].append(f"""{hand}:
    bgtz t5, {hand}e
    lw   t6, LATE(s0)
    addi t6, t6, 1
    sw   t6, LATE(s0)
    li   t5, 0
    j    {back}
{hand}e:
    csrr t6, 0x7e2
    sub  t5, a1, t6
    li   t6, {SLED}
    blt  t6, t5, {hand}e
    j    {back}""")
                continue
            if jj == 15:                                   # E: jump into the sled
                s.add(kern(-1024, cur(k + 1)), KERN)
                s.add(f"    addi a0, a0, {4 * NSYM * S}" if (REC != "none" and m == NSYM // 2 - 1) else "    nop", 1)
                s.add("    jalr x0, 0(t6)", 3)
                out.append(f"/* {c} slot {n} (E, jump into the sled) */\n" + "\n".join(s.lines))
                EFFECTIVE[f"{c}{n}_cost"] = s.cost
                out += stubs[m]
                continue
            imm = (j + 1 - 4) * 256 if j < S - 1 else -1024
            s.add(kern(imm, cur(k) if j < S - 1 else cur(k + 1)), KERN)
            for L, cst in pl[n]:
                s.add(L, cst)
            out.append(f"/* {c} slot {n} */\n" + s.finish())
    return "\n".join(out)


def function():
    pads = PADS_ALL.get(str(P), {})
    global PADS
    PADS = pads
    f = [f"""
    .global lutg_p8s8_run_p20
    .type lutg_p8s8_run_p20, @function
    .align 4
lutg_p8s8_run_p20:
    addi sp, sp, -64
    sw   ra, 0(sp)
    sw   s0, 4(sp)
    sw   s1, 8(sp)
    sw   s2, 12(sp)
    sw   s3, 16(sp)
    sw   s4, 20(sp)
    sw   s5, 24(sp)
    sw   s6, 28(sp)
    sw   s7, 32(sp)
    sw   s8, 36(sp)
    sw   s9, 40(sp)
    sw   s10, 44(sp)
    sw   s11, 48(sp)
    sw   gp, 52(sp)
    sw   tp, 56(sp)
    mv   s0, a0
    li   a3, 0x60043000         /* USB Serial/JTAG */
    li   a5, 0x80200            /* offset binary -> two's complement, both fields */
    li   a6, 0x3fcb0000         /* held DAC word */
    lw   s11, QW(s0)
    lw   a7, QR(s0)
    li   a2, 0                  /* symbol history */
    lw   s1, T0(s0)             /* row pointers of the first symbol (history 0), base + 1024 */
    lw   s2, T1(s0)
    lw   s3, T2(s0)
    lw   s4, T3(s0)
    lw   a0, REC(s0)
    lw   t0, -1024(s1)
    lw   t1, -1024(s2)
    lw   t2, -1024(s3)
    lw   t4, -1024(s4)
    add  t0, t0, t1
    add  t0, t0, t2
    add  t0, t0, t4
    xor  t3, t0, a5             /* word of sample 0 */
    la   t5, .LsEN0
    sw   t5, NS_N(s0)
    la   t5, .LsEM0
    sw   t5, NS_A(s0)           /* NS_A: the sled of the maintenance copy M */
    /* the first record into the window (a whole record: V0 = {V0} bits) */
    sub  gp, s11, a7
    addi gp, gp, -2
    slt  gp, zero, gp
    lw   t6, RING(s0)
    slli t5, a7, 18
    srli t5, t5, 18
    add  t5, t5, t6
    lbu  s9, 0(t5)
    addi t5, a7, 1
    slli t5, t5, 18
    srli t5, t5, 18
    add  t5, t5, t6
    lbu  t5, 0(t5)
    slli t5, t5, 8
    or   s9, s9, t5
    addi t5, a7, 2
    slli t5, t5, 18
    srli t5, t5, 18
    add  t5, t5, t6
    lbu  t5, 0(t5)
    slli t5, t5, 16
    or   s9, s9, t5
    slli t5, gp, 1
    add  t5, t5, gp
    add  a7, a7, t5
    lw   a1, TN(s0)             /* start of the first symbol; the target of the first measure is two symbols later (added in slot 13) */
    addi a1, a1, {-pads.get('KD', 24)}
.Lent:                          /* wait for the start (spin until at most SLED cycles are left, then the sled) */
    csrr t5, 0x7e2
    lw   t6, TN(s0)
    sub  t5, t6, t5
    li   t6, {SLED}
    blt  t6, t5, .Lent
    bgez t5, .Lentok
    li   t5, 0
.Lentok:
    slli t5, t5, 1
    la   t6, .LsEN0
    sub  t6, t6, t5
    jalr x0, 0(t6)
"""]
    for c in COPIES:
        f.append(copy_code(c, pads))
    f.append("""lutg_p8s8_exit:
    sw   s11, QW(s0)
    sw   a7, QR(s0)
    sw   a0, RECP(s0)
    lw   ra, 0(sp)
    lw   s0, 4(sp)
    lw   s1, 8(sp)
    lw   s2, 12(sp)
    lw   s3, 16(sp)
    lw   s4, 20(sp)
    lw   s5, 24(sp)
    lw   s6, 28(sp)
    lw   s7, 32(sp)
    lw   s8, 36(sp)
    lw   s9, 40(sp)
    lw   s10, 44(sp)
    lw   s11, 48(sp)
    lw   gp, 52(sp)
    lw   tp, 56(sp)
    addi sp, sp, 64
    li   a0, 0
    ret
    .size lutg_p8s8_run_p20, . - lutg_p8s8_run_p20
""")
    return "\n".join(f)


emit("/* Generated by gen_lutg_p8s8.py - do not edit by hand (rec mode: " + REC + ").\n"
     " * 1 MBd 8PSK / RRC modulator for the ESP32-C3: 8 samples per symbol, one DAC word every 20 CPU cycles (8 MS/s), 3 bit symbols in the USB stream,\n"
     " * tables of 4 groups x 64 rows x 8 words (RRC span 8). Context: struct lutg_ctx_t in main.c. */")
for k, v in CTX.items():
    emit(f"    .equ {k}, {v}")
emit('    .section .iram1.lutg_p8s8, "ax"\n    .option norvc')
emit(function())
print("\n".join(o))
json.dump({str(P): EFFECTIVE}, open(os.path.join(HERE, "lutg_p8s8_pads_used.json"), "w"), indent=0)
