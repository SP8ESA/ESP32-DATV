#!/usr/bin/env python3
"""Generates main/lutg.S: the cycle-deterministic QPSK/RRC modulator for any number of samples per symbol S >= 16 (S is a run-time value),
one DAC store every P = 20 or 24 CPU cycles (8 or 6.67 MS/s at 160 MHz). One function per P: lutg_run_p20, lutg_run_p24.

Table layout (see qpsk_lut.h, lut_build_t): 4 groups of 2 symbols, 16 rows each, transposed: the 16 words of one (group, sample) are
64 bytes apart from the next sample, so four pointers advance by 64 per sample. One sample = 4 loads, 3 adds, 1 xor, 1 store.

A symbol is S slots of exactly P cycles between stores:
  slots 0 .. E-1   unrolled, hold the per-symbol work (the "events"), immediate table offsets
  slots E .. S-3   one 1-slot loop (4 loads + 4 pointer increments + adds + branch)
  slot S-2   (D)   measures the cycle counter against the absolute symbol schedule and prepares a jump into a nop sled
  slot S-1   (E)   loads the first word of the next symbol and jumps into the sled of the next symbol's code copy
Four code copies: N (normal), M1, M2, M3 (maintenance: silence check, fill report written to the USB FIFO in the symbols after
each other). Each starts with a 24-nop sled; the computed jump into it absorbs whatever deviates from the schedule (USB register
accesses vary by a cycle), so the symbol rate is exact (S*P cycles) and no slot but E ever has a variable length.

usage (from firmware/):  python3 tools/gen_lutg.py > main/lutg.S
  --rec timing   instead of driving the DAC every slot stamps the cycle counter right after the real DAC store and records it
                 (build with "#define LUTG_REC 1" in main.c); the end of the run prints, for every symbol, the slots whose store-to-store
                 spacing differs from P ("slot:cycles") and the nominal length of the sync sled (SLED)
  --rec words    ... records the DAC words (LUTG_REC 2), to compare with the reference model on the PC (host/test_lutg_words.c)
  --raw          no padding at all: the timing build then shows the true cost of every slot
Per-slot padding lives in lutg_pads.json ({"20": {"N5": 1, ...}, "24": {...}}, nop counts; N A B C = the code copies, 0..12 the
unrolled slots, L the loop, D the measuring slot). How it was tuned, with host/lutg_record.py on a timing build:
  1. raw build, run QPSKT 2370.000 250000 32 (P = 20) and 66000 101 (P = 24): pad = P - cost for every slot (D is only valid in a
     padded build: without padding the sync is permanently late);
  2. rebuild padded, run again, correct the pads by (P - spacing), repeat until every slot is P. The slots that read or write the USB
     registers (N7 = the avail read, B8..B10 and C8, C9 = the report writes) need a scan: an access completes on an edge of the 48 MHz
     USB clock, so one nop more can change the spacing by 2 cycles; take the pad for which the spacing is exactly P.
The run-time symbol rate does not depend on any of this (the sync sled makes every symbol S * P cycles); the padding only keeps the
individual DAC stores equally spaced. Cost rules measured on this core: ALU, load, store 1 cycle; a load whose result is used by the
next instruction +1; taken branch 3; j 2; jalr 3; USB register read ~6, USB register write ~8 (it blocks the next bus access).
"""
import json
import os
import sys

REC = "none"
if "--rec" in sys.argv:
    REC = sys.argv[sys.argv.index("--rec") + 1]
assert REC in ("none", "timing", "words")
SCHED = "--nosched" not in sys.argv
RAW = "--raw" in sys.argv          # no padding at all: the timing build then shows the true cost of every slot
HERE = os.path.dirname(os.path.abspath(__file__))
PADFILE = os.path.join(HERE, "lutg_pads.json")
PADS_ALL = json.load(open(PADFILE)) if os.path.exists(PADFILE) else {}

E = 13               # unrolled event slots per symbol (S >= E + 3)
SLED = 24            # nops in each sync sled
MIN_S = E + 3
COPIES = ("N", "A", "B", "C")        # N normal, A/B/C = M1/M2/M3
NS_INDEX = {c: i for i, c in enumerate(COPIES)}

# context layout (struct lutg_ctx_t in main.c)
CTX = dict(T0=0, T1=4, T2=8, T3=12, RING=16, QW=20, QR=24, TN=28, NSYM=32, UNDER=36, MPER=40, MCNT=44, TRX=48, QWSEEN=52, LIM=56, LIM1=60,
           EXITC=64, LATE=68, S64M=72, SPER=76, RLIM=80, NS_N=84, NS_A=88, NS_B=92, NS_C=96, SLEDREC=100, REC=104, RECP=108)

# cost model in cycles (measured on the C3, see README): alu/load/store 1, APB read ~6, APB write ~8, taken branch 3, j 2, jalr 3
KERN = 8
o = []
emit = o.append
EFFECTIVE = {}
CUR_P = 0
PADS = {}
OUTLINE = []
uid = [0]


def lab(name):
    uid[0] += 1
    return f".L{name}{uid[0]}"


class Slot:
    """Lines of one slot and their estimated cycles; pad() adds the nops that make it exactly P."""

    def __init__(self, P, key, pads):
        self.P, self.key, self.pads = P, key, pads
        self.lines = []
        self.cost = 0.0

    def add(self, lines, cost):
        self.lines += lines if isinstance(lines, list) else [lines]
        self.cost += cost

    def finish(self, extra_note=""):
        est = round(self.P - self.cost)
        n = 0 if RAW else self.pads.get(self.key, est)           # a tuned absolute count wins over the estimate
        if n < 0:
            sys.stderr.write(f"WARNING P={self.P} slot {self.key}: estimated {self.cost:.1f} cycles, over the period\n")
            n = 0
        EFFECTIVE.setdefault(str(self.P), {})[self.key] = n
        lines = schedule(self.lines) if SCHED else self.lines
        if n:
            lines = lines + [f"    .rept {n}\n    nop\n    .endr"]
        return "\n".join(lines)


def store_group(off, ptr="a0"):
    """The DAC store of a slot with the recording that goes with it. The timing build stamps the cycle counter right AFTER the real store
    (a pending APB write stalls the store, which production feels too), then records it: the group has the same 3 instructions
    as production's store + 2 nops, so both builds take the same cycles."""
    if REC == "timing":
        return ["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", f"    sw   ra, {off}({ptr})"], 3
    if REC == "words":
        return ["    sw   t3, 0(a6)", "    mv   ra, t3", f"    sw   ra, {off}({ptr})"], 3
    return ["    sw   t3, 0(a6)", "    nop", "    nop"], 3


def store_unrolled(j):
    return store_group(4 * j)


def kern(imm, regs):
    r = regs
    return [f"    lw   t0, {imm}({r[0]})", f"    lw   t1, {imm}({r[1]})", f"    lw   t2, {imm}({r[2]})", f"    lw   t4, {imm}({r[3]})",
            "    add  t0, t0, t1", "    add  t0, t0, t2", "    add  t0, t0, t4", "    xor  t3, t0, a5"]


Q = ("s1", "s2", "s3", "s4")        # current row pointers (entries of sample j + 1)
NQ = ("s5", "s6", "s7", "s8")       # next symbol's row pointers (sample 0)



# ------------------------------------------------------------------ instruction scheduler: separates a load from the instruction that uses it (+1 cycle stall on this core)
import re
REG_RE = re.compile(r"\b(ra|sp|gp|tp|t[0-6]|s(?:1[01]|[0-9])|a[0-7])\b")
LOADS = ("lw", "lbu", "lhu", "lb", "lh")
STORES = ("sw", "sb", "sh")


def parse_ins(line):
    """(op, dst, srcs, kind) for a movable instruction, None for anything that must keep its place (labels, branches, directives)."""
    t = line.strip()
    if not t or t.startswith((".", "/*")) or t.endswith(":"):
        return None
    op, _, rest = t.partition(" ")
    if op in ("beqz", "bnez", "bne", "beq", "blt", "bge", "bltu", "bgeu", "bgtz", "j", "jal", "jalr", "ret", "la"):
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
    """Reorders each run of movable instructions (greedy list scheduling, program-order dependencies kept, stores stay in order with every
    other memory access) so that a load is not followed at once by its consumer."""
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
                if (di and di in sj) or (dj and dj in si) or (di and dj and di == dj):      # RAW, WAR, WAW
                    dep[j].add(i)
                if ("store" in (ki, kj)) and kj != "alu" and ki != "alu":                     # memory order around stores
                    dep[j].add(i)
        done, order = set(), []
        last_load = None
        while len(order) < n:
            cand = [j for j in range(n) if j not in done and dep[j] <= done]
            ok = [j for j in cand if last_load is None or last_load not in ins[j][2]]
            loads = [j for j in ok if ins[j][3] == "load"]
            pick = loads[0] if loads else ok[0] if ok else cand[0]       # loads first: their latency then overlaps the following instructions
            done.add(pick)
            order.append(pick)
            last_load = ins[pick][1] if ins[pick][3] == "load" else None
        out.extend(seg[j] for j in order)
        seg.clear()

    for l in lines:
        # a multi-line string element (pad block) is a barrier
        if "\n" in l or parse_ins(l) is None:
            flush()
            out.append(l)
        else:
            seg.append(l)
    flush()
    return out


# ------------------------------------------------------------------ events (lines, cycles); registers: t5 t6 a4 gp tp are free, t0-t4 belong to the kern
def a_mv():
    return [f"    mv   {q}, {n}" for q, n in zip(Q, NQ)], 4


def a_tgt():                          # next schedule target
    return ["    lw   tp, SPER(s0)", "    add  a1, a1, tp"], 2


def a_qend():                         # end value of s1 for the loop (s1 still at the row base)
    return ["    lw   t5, S64M(s0)", "    add  s9, s1, t5"], 2


def a_fill():                         # gp = ring has a symbol (not an underrun), a4 = ring has room for a USB packet
    return ["    lw   t6, RLIM(s0)", "    slli t5, s11, 2", "    sub  t5, t5, a7", "    slt  gp, zero, t5", "    slt  a4, t5, t6"], 5


def a_nsym(c, r="s5"):                # symbols left; leave when done (s5..s8 are free until the next-pointer events write them)
    return [f"    lw   {r}, NSYM(s0)", f"    addi {r}, {r}, -1", f"    sw   {r}, NSYM(s0)", f"    beqz {r}, .Lx{c}{CUR_P}"], 4


def a_xa():                           # t6 = ring byte holding the next symbol
    return ["    lw   t6, RING(s0)", "    srli t5, a7, 2", "    slli t5, t5, 18", "    srli t5, t5, 18", "    add  t5, t5, t6", "    lbu  t6, 0(t5)"], 6


def a_xb():                           # take the symbol out of the byte, shift it into the history, advance unless underrun
    return ["    andi t5, a7, 3", "    slli t5, t5, 1", "    srl  t6, t6, t5", "    andi t6, t6, 3", "    slli a2, a2, 2", "    or   a2, a2, t6", "    add  a7, a7, gp"], 7


def a_under():
    return ["    lw   t5, UNDER(s0)", "    xori t6, gp, 1", "    add  t5, t5, t6", "    sw   t5, UNDER(s0)"], 4


def a_nptr(g, r=("t5", "t6")):        # next symbol's row pointer of group g: T_g + 4 * ((C >> 4g) & 15)
    a, b = r
    sh = {0: f"    slli {a}, a2, 2", 1: f"    srli {a}, a2, 2", 2: f"    srli {a}, a2, 6", 3: f"    srli {a}, a2, 10"}[g]
    return [f"    lw   {b}, T{g}(s0)", sh, f"    andi {a}, {a}, 60", f"    add  {NQ[g]}, {a}, {b}"], 4


def a_mcn_a():                        # maintenance countdown; tp = 1 when the next symbol starts the maintenance pass
    return ["    lw   tp, MCNT(s0)", "    addi tp, tp, -1", "    sw   tp, MCNT(s0)", "    seqz tp, tp"], 4


def a_mcn_b():                        # s10 = sled end of the next copy (N or A)
    return ["    slli tp, tp, 2", "    add  tp, tp, s0", "    lw   s10, NS_N(tp)"], 3


def a_av():                           # USB: is a byte waiting (consumed in the next slot, an APB read costs ~6 cycles)
    return ["    lw   t5, 4(a3)"], 6


def a_avc():
    return ["    srli t5, t5, 2", "    andi t5, t5, 1", "    and  a4, a4, t5"], 3


def a_rd(c, stubs):                   # read the byte if there is one and the ring has room (balanced with the skip path)
    nd, back = lab("nd"), lab("rb")
    stubs.append(f"{nd}:\n    .rept {PADS.get('RDNOPS', 2)}\n    nop\n    .endr\n    j    {back}")
    return [f"    beqz a4, {nd}", "    lw   gp, 0(a3)", f"{back}:"], 7


def a_sb():                           # byte into the ring (harmless garbage when there was none: qw does not move)
    return ["    lw   t6, RING(s0)", "    slli t5, s11, 18", "    srli t5, t5, 18", "    add  t5, t5, t6", "    sb   gp, 0(t5)", "    add  s11, s11, a4"], 6


def a_qadj():                         # row pointers for the loop: sample E + 1
    return [f"    addi {q}, {q}, {64 * (E + 1)}" for q in Q], 4


# maintenance
def a_sil1():
    return ["    lw   t5, QWSEEN(s0)", "    sub  t5, s11, t5", "    snez t5, t5", "    neg  t5, t5", "    csrr t6, 0x7e2"], 5


def a_sil2():
    return ["    lw   gp, TRX(s0)", "    xor  tp, gp, t6", "    and  tp, tp, t5", "    xor  gp, gp, tp", "    sw   gp, TRX(s0)", "    sw   s11, QWSEEN(s0)",
            "    sub  t6, t6, gp"], 7


def a_sil3a(c):
    return ["    lw   t5, LIM(s0)", f"    bltu t5, t6, .Ls{c}{CUR_P}", "    snez t6, s11", "    neg  t6, t6"], 4


def a_sil3b():
    return ["    lw   gp, LIM1(s0)", "    xor  tp, t5, gp", "    and  tp, tp, t6", "    xor  t5, t5, tp", "    sw   t5, LIM(s0)"], 5


def a_next(nsname, reload=False):
    L = [f"    lw   s10, {nsname}(s0)"]
    if reload:
        L += ["    lw   t5, MPER(s0)", "    sw   t5, MCNT(s0)"]
    return L, 3 if reload else 1


def a_rep_calc():
    return ["    slli t5, s11, 2", "    sub  t5, t5, a7", "    srli t5, t5, 3", "    li   gp, 0xB7"], 4


def a_wr(prep):                       # one USB FIFO write (the FIFO ignores writes when it is full)
    return prep + ["    sw   gp, 0(a3)"], 8.1 + len(prep)


def a_flush():
    return ["    li   gp, 1", "    sw   gp, 4(a3)"], 9.1


def a_rep_under():
    return ["    lw   gp, UNDER(s0)", "    andi gp, gp, 255"], 2


# ------------------------------------------------------------------ one code copy
def copy_code(P, c, pads):
    """Returns the text of copy c: sled, E unrolled slots, the loop, D and E slots, out-of-line stubs."""
    stubs = []
    out = []
    sled_end = f".LsE{c}{P}"
    out.append(f"    .align 4\nlutg{P}_copy_{c}:\n    .rept {SLED}\n    nop\n    .endr\n{sled_end}:")
    sch = {j: [] for j in range(E)}
    sch[0] = [a_mv, a_tgt]               # a_mv first (before the kern loads)
    sch[1] = [a_fill]
    sch[2] = [a_xa]
    sch[3] = [a_xb]
    sch[4] = [a_under, lambda: a_nptr(0, ("tp", "s9"))]
    sch[5] = [lambda: a_nptr(1), lambda: a_nptr(2, ("tp", "s9"))]
    if c == "N":
        sch[6] = [lambda: a_nptr(3), a_mcn_a]
        sch[7] = [a_av]
        sch[8] = [a_avc, a_mcn_b]
        sch[9] = [lambda: a_rd(c, stubs)]
        sch[10] = [a_sb]
    elif c == "A":                       # silence check
        sch[6] = [lambda: a_nptr(3), lambda: a_next("NS_B")]
        sch[7] = [a_sil1]
        sch[8] = [a_sil2]
        sch[9] = [lambda: a_sil3a(c)]
        sch[10] = [a_sil3b]
    elif c == "B":                       # fill report: marker, fill lo, fill hi
        sch[6] = [lambda: a_nptr(3), lambda: a_next("NS_C")]
        sch[7] = [a_rep_calc]
        sch[8] = [lambda: a_wr([])]
        sch[9] = [lambda: a_wr(["    andi gp, t5, 255"])]
        sch[10] = [lambda: a_wr(["    srli gp, t5, 8"])]
    else:                                # C: underruns, flush
        sch[6] = [lambda: a_nptr(3), lambda: a_next("NS_N", reload=True)]
        sch[7] = [a_rep_under]
        sch[8] = [lambda: a_wr([])]
        sch[9] = [a_flush]
    sch[11] = [lambda: a_nsym(c, "tp"), a_qend]
    sch[12] = [a_qadj]
    for j in range(E):
        s = Slot(P, f"{c}{j}", pads)
        L, cst = store_unrolled(j)
        s.add(L, cst)
        if j == 0:
            L, cst = a_mv()
            s.add(L, cst)
            fl = [f for f in sch[0] if f is not a_mv]
        else:
            fl = sch[j]
        s.add(kern(64 * (j + 1), Q), KERN)
        for f in fl:
            L, cst = f()
            s.add(L, cst)
        if j == E - 1 and REC in ("timing", "words"):
            s.add(f"    addi tp, a0, {4 * E}", 1)     # record pointer of the loop (production: a nop, same cycles)
        elif j == E - 1:
            s.add("    nop", 1)
        out.append(f"/* {c} slot {j} */\n" + s.finish())
    # ---- the loop: slots E .. S-3
    ll = f".Lloop{c}{P}"
    s = Slot(P, f"{c}L", pads)
    s.lines.append(f"{ll}:")
    if REC == "timing":
        s.add(["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", "    sw   ra, 0(tp)", "    addi tp, tp, 4"], 4)
    elif REC == "words":
        s.add(["    sw   t3, 0(a6)", "    mv   ra, t3", "    sw   ra, 0(tp)", "    addi tp, tp, 4"], 4)
    else:
        s.add(["    sw   t3, 0(a6)", "    nop", "    nop", "    nop"], 4)
    s.add(["    lw   t0, 0(s1)", "    lw   t1, 0(s2)", "    addi s1, s1, 64", "    addi s2, s2, 64", "    lw   t2, 0(s3)", "    lw   t4, 0(s4)",
           "    addi s3, s3, 64", "    addi s4, s4, 64", "    add  t0, t0, t1", "    add  t0, t0, t2", "    add  t0, t0, t4", "    xor  t3, t0, a5"], 12)
    s.cost += 3                                                     # taken branch
    out.append(f"/* {c} loop */\n" + s.finish() + f"\n    beq  s1, s9, {ll}x\n    j    {ll}\n{ll}x:")     # both paths cost 3 cycles
    # ---- D: measure
    s = Slot(P, f"{c}D", pads)
    st, sc = store_group(0, "tp")
    s.add(st, sc)
    s.add(kern(0, Q), KERN)
    hand, back = f".Lh{c}{P}", f".Lb{c}{P}"
    s.add(["    csrr t5, 0x7e2", "    sub  t5, a1, t5", f"    sltiu t6, t5, {SLED + 1}", f"    beqz t6, {hand}", f"{back}:", "    slli t5, t5, 2", "    sub  t6, s10, t5"], 6)
    if REC == "timing":
        s.add(f"    sw   t5, SLEDREC(s0)", 1)                   # production: a nop
    else:
        s.add("    nop", 1)
    out.append(f"/* {c} slot D (measure) */\n" + s.finish())
    stubs.append(f"""{hand}:
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
    # ---- E: jump into the next copy's sled
    s = Slot(P, f"{c}E", pads)
    st, sc = store_group(4, "tp")
    s.add(st, sc)
    s.add(kern(0, NQ), KERN)
    s.add("    addi a0, tp, 8" if REC != "none" else "    nop", 1)
    s.add("    jalr x0, 0(t6)", 3)
    out.append(f"/* {c} slot E (jump into the next sled; its length is the sync) */\n" + "\n".join(s.lines))      # no pad: the sled is the pad
    EFFECTIVE.setdefault(str(P), {})[f"{c}E_cost"] = s.cost
    out.append(f".Lx{c}{P}:\n    j    lutg{P}_exit")
    stubs.append(f".Ls{c}{P}:\n    li   t5, 2\n    sw   t5, EXITC(s0)\n    j    lutg{P}_exit")
    out += stubs
    return "\n".join(out), sled_end


def function(P):
    pads = PADS_ALL.get(str(P), {})
    global PADS, CUR_P
    PADS = pads
    CUR_P = P
    f = [f"""
    .global lutg_run_p{P}
    .type lutg_run_p{P}, @function
    .align 4
lutg_run_p{P}:
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
    lw   s5, T0(s0)             /* row pointers of the first symbol (history 0) */
    lw   s6, T1(s0)
    lw   s7, T2(s0)
    lw   s8, T3(s0)
    lw   a0, REC(s0)
    lw   t0, 0(s5)
    lw   t1, 0(s6)
    lw   t2, 0(s7)
    lw   t4, 0(s8)
    add  t0, t0, t1
    add  t0, t0, t2
    add  t0, t0, t4
    xor  t3, t0, a5             /* word of sample 0 */
    la   t5, .LsEN{P}
    sw   t5, NS_N(s0)
    la   t5, .LsEA{P}
    sw   t5, NS_A(s0)
    la   t5, .LsEB{P}
    sw   t5, NS_B(s0)
    la   t5, .LsEC{P}
    sw   t5, NS_C(s0)
    lw   a1, TN(s0)             /* start of the first symbol; the target of the first measure is one symbol later (added in slot 0) */
    addi a1, a1, -{pads.get('KD', 8)}
.Lent{P}:                       /* wait for the start (spin until at most SLED cycles are left, then the sled) */
    csrr t5, 0x7e2
    lw   t6, TN(s0)
    sub  t5, t6, t5
    li   t6, {SLED}
    blt  t6, t5, .Lent{P}
    bgez t5, .Lentok{P}
    li   t5, 0
.Lentok{P}:
    slli t5, t5, 2
    la   t6, .LsEN{P}
    sub  t6, t6, t5
    jalr x0, 0(t6)
"""]
    ends = {}
    for c in COPIES:
        code, end = copy_code(P, c, pads)
        f.append(code)
    f.append(f"""lutg{P}_exit:
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
    .size lutg_run_p{P}, . - lutg_run_p{P}
""")
    return "\n".join(f)


emit("/* Generated by gen_lutg.py - do not edit by hand (rec mode: " + REC + ").\n"
     " * Generic cycle-deterministic QPSK / RRC modulator for the ESP32-C3: any samples per symbol S >= %d (run-time value), one DAC word every\n"
     " * P = 20 or 24 CPU cycles (8 or 6.67 MS/s at 160 MHz), tables of 4 groups x 16 rows x S words (RRC span 8). Context: struct lutg_ctx_t in main.c. */" % MIN_S)
for k, v in CTX.items():
    emit(f"    .equ {k}, {v}")
emit('    .section .iram1.lutg, "ax"\n    .option norvc')
for P in (20, 24):
    emit(function(P))
print("\n".join(o))
json.dump(EFFECTIVE, open(os.path.join(HERE, "lutg_pads_used.json"), "w"), indent=0)
