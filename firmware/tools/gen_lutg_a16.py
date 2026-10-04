#!/usr/bin/env python3
"""Generates main/lutg_a16.S: the cycle-deterministic 16APSK / RRC modulator for any number of samples per symbol S >= 16 (S is a run-time value),
one DAC store every 20 CPU cycles within a symbol. Fractional symbol deadlines add an optional cycle at the symbol boundary
(333000 Bd: 480/481 cycles per symbol, 7.992 MS/s average at 160 MHz). Function lutg_a16_run_p20. Derived from gen_lutg.py (see there for the structure of a
symbol: unrolled event slots, a 1-slot loop, the measuring slot D, the jump slot E and the sync sled).

What is different from the QPSK / 8PSK loops:
  * a symbol is a 4 bit point index v = 0..15 (one nibble of the ring, low nibble first); the history a2 holds 4 bits per symbol
  * the RRC filter is truncated to 6 symbols = 3 groups of 2 symbols (the tables of 256 rows per group would take 4 KB * S with 4 groups, which the
    RAM does not have): one sample = 3 loads, 2 adds, 1 xor, 1 store
  * the table of a group is stored row by row: T[(g * 256 + idx) * S + j], idx = (v_newer | v_older << 4); the row pointer is base + idx * 4 * S (a multiply,
    s4 = 4 * S) and the next sample of a row is 4 bytes further, so the immediates of the unrolled slots stay small
  * the ring is 8 KB (mask 13 bits): the RAM below the RF dump bank has to hold it next to this code
  * two USB bytes are read per symbol (the USB OUT FIFO is read byte by byte by the CPU and the host can send the next 64 byte packet only when
    the previous one is empty: at one read per 2 us the link carries 262 kB/s, 500 kBd needs 250)

usage (from firmware/):  python3 tools/gen_lutg_a16.py > main/lutg_a16.S
  --rec timing | words   recording builds (main.c: #define LUTG_REC 1 | 2), see gen_lutg.py
  --raw                  no padding at all
Per-slot padding lives in lutg_a16_pads.json ({"20": {"N5": 1, ...}}; N A B C = the code copies, 0..12 the unrolled slots, L the loop, D the
measuring slot; <key>p = nops right after the store).
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
PADFILE = os.path.join(HERE, "lutg_a16_pads.json")
PADS_ALL = json.load(open(PADFILE)) if os.path.exists(PADFILE) else {}

P = 20
E = 13               # unrolled event slots per symbol (S >= E + 3)
MIN_S = E + 3
SLED = 24
RING_SHIFT = 19      # 32 - 13: an 8 KB ring
COPIES = ("N", "A", "B", "C")
CTX = dict(T0=0, T1=4, T2=8, T3=12, RING=16, QW=20, QR=24, TN=28, NSYM=32, UNDER=36, MPER=40, MCNT=44, TRX=48, QWSEEN=52, LIM=56, LIM1=60,
           EXITC=64, LATE=68, S64M=72, SPER=76, RLIM=80, NS_N=84, NS_A=88, NS_B=92, NS_C=96, SLEDREC=100, REC=104, RECP=108,
           PHASE=112, REM=116, DIV=120)
KERN = 7
o = []
emit = o.append
EFFECTIVE = {}
PADS = {}
uid = [0]


def lab(name):
    uid[0] += 1
    return f".L{name}{uid[0]}"


def nops(n):
    """n one-cycle nops in few bytes: 2 byte c.nops in even numbers (the next instruction stays 4 byte aligned), an odd one is a 4 byte nop (IRAM is scarce)."""
    if n <= 0:
        return ""
    t = "    nop\n" if n % 2 else ""
    if n // 2:
        t += f"    .option push\n    .option rvc\n    .rept {2 * (n // 2)}\n    c.nop\n    .endr\n    .option pop\n"
    return t.rstrip("\n")


class Slot:
    """Lines of one slot and their estimated cycles; finish() adds the nops that make it exactly P."""

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
        grp = 3 if REC != "none" else 2
        pre = 0 if RAW else self.pads.get(self.key + "p", 0)
        if pre and src[0].startswith("    sw   t3, 0(a6)"):
            EFFECTIVE[self.key + "p"] = pre
            src = src[:grp] + [nops(pre)] + src[grp:]
        lines = schedule(src) if SCHED else src
        if n:
            lines = lines + [nops(n)]
        return "\n".join(lines)


def store_group(off, ptr="a0"):
    if REC == "timing":
        return ["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", f"    sw   ra, {off}({ptr})"], 3
    if REC == "words":
        return ["    sw   t3, 0(a6)", "    mv   ra, t3", f"    sw   ra, {off}({ptr})"], 3
    return ["    sw   t3, 0(a6)", nops(2)], 3


def store_unrolled(j):
    return store_group(4 * j)


def kern(imm, regs):
    r = regs
    return [f"    lw   t0, {imm}({r[0]})", f"    lw   t1, {imm}({r[1]})", f"    lw   t2, {imm}({r[2]})",
            "    add  t0, t0, t1", "    add  t0, t0, t2", "    xor  t3, t0, a5"]


Q = ("s1", "s2", "s3")        # current row pointers (entries of sample j + 1)
NQ = ("s5", "s6", "s7")       # next symbol's row pointers (sample 0)


def koff(j):
    return 4 * (j + 1)         # the row pointers point at sample 0 of the symbol (see a_mv)


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


# ------------------------------------------------------------------ events (lines, cycles); registers: t5 t6 a4 gp tp are free, t0-t2 belong to the kern (t3 = word)
def a_mv():
    return [f"    mv   {q}, {n}" for q, n in zip(Q, NQ)], 3


def a_tgt():
    return ["    lw   tp, SPER(s0)", "    add  a1, a1, tp"], 2


def a_phase0():
    # Previous next-row pointers have been copied to Q; s5..s7 are scratch until slot 4.
    return ["    lw   s5, PHASE(s0)", "    lw   s6, REM(s0)", "    add  s5, s5, s6",
            "    lw   s6, DIV(s0)", "    sltu s7, s5, s6"], 5


def a_phase2():
    return ["    addi s7, s7, -1", "    and  s6, s6, s7", "    sub  a1, a1, s7"], 3


def a_phase3():
    return ["    sub  s5, s5, s6", "    sw   s5, PHASE(s0)"], 2


def a_qend():                         # end value of s1 for the loop (s1 still at the row base)
    return ["    lw   t5, S64M(s0)", "    add  s9, s1, t5"], 2


def a_fill():                         # gp = ring has a symbol (not an underrun), a4 = ring has room for a USB packet
    return ["    lw   t6, RLIM(s0)", "    slli t5, s11, 1", "    sub  t5, t5, a7", "    slt  gp, zero, t5", "    slt  a4, t5, t6"], 5


def a_nsym(c, r="s5"):
    return [f"    lw   {r}, NSYM(s0)", f"    addi {r}, {r}, -1", f"    sw   {r}, NSYM(s0)", f"    beqz {r}, .Lx{c}"], 4


def a_xa():                           # t6 = ring byte holding the next symbol
    return ["    lw   t6, RING(s0)", "    srli t5, a7, 1", f"    slli t5, t5, {RING_SHIFT}", f"    srli t5, t5, {RING_SHIFT}", "    add  t5, t5, t6", "    lbu  t6, 0(t5)"], 6


def a_xb():                           # nibbles: two symbols per byte, 4 bit symbols in the history
    return ["    andi t5, a7, 1", "    slli t5, t5, 2", "    srl  t6, t6, t5", "    andi t6, t6, 15", "    slli a2, a2, 4", "    or   a2, a2, t6", "    add  a7, a7, gp"], 7


def a_under():
    return ["    lw   t5, UNDER(s0)", "    xori t6, gp, 1", "    add  t5, t5, t6", "    sw   t5, UNDER(s0)"], 4


def a_nptr(g):
    """next symbol's row pointer of group g: T_g + 4 * S * ((C >> 8g) & 255); s4 = 4 * S"""
    dest = NQ[g]
    L = [f"    lw   {dest}, T{g}(s0)"]
    if g == 0:
        L.append("    andi t5, a2, 255")
    else:
        L += [f"    srli t5, a2, {8 * g}", "    andi t5, t5, 255"]
    L += ["    mul  t5, t5, s4", f"    add  {dest}, {dest}, t5"]
    return L, 5 if g == 0 else 6


def a_mcn_a():
    return ["    lw   tp, MCNT(s0)", "    addi tp, tp, -1", "    sw   tp, MCNT(s0)", "    seqz tp, tp"], 4


def a_mcn_b():
    return ["    slli tp, tp, 2", "    add  tp, tp, s0", "    lw   s10, NS_N(tp)"], 3


def a_av():                           # USB: is a byte waiting (consumed in the next slot, an APB read costs ~6 cycles)
    return ["    lw   t5, 4(a3)", "    srli t5, t5, 2"], 7


def a_avc():
    return ["    andi t5, t5, 1", "    and  a4, a4, t5"], 2


def a_rd(stubs):                      # read the byte if there is one and the ring has room (balanced with the skip path)
    nd, back = lab("nd"), lab("rb")
    stubs.append(f"{nd}:\n{nops(PADS.get('RDNOPS', 2))}\n    j    {back}")
    return [f"    beqz a4, {nd}", "    lw   gp, 0(a3)", f"{back}:"], 7


def a_sb():                           # byte into the ring (harmless garbage when there was none: qw does not move)
    return ["    lw   t6, RING(s0)", f"    slli t5, s11, {RING_SHIFT}", f"    srli t5, t5, {RING_SHIFT}", "    add  t5, t5, t6", "    sb   gp, 0(t5)", "    add  s11, s11, a4"], 6


def a_qadj():                         # row pointers for the loop: sample E + 1
    return [f"    addi {q}, {q}, {4 * (E + 1)}" for q in Q], 3


# maintenance
def a_sil1():
    return ["    lw   t5, QWSEEN(s0)", "    sub  t5, s11, t5", "    snez t5, t5", "    neg  t5, t5", "    csrr t6, 0x7e2"], 5


def a_sil2():
    return ["    lw   gp, TRX(s0)", "    xor  tp, gp, t6", "    and  tp, tp, t5", "    xor  gp, gp, tp", "    sw   gp, TRX(s0)", "    sw   s11, QWSEEN(s0)",
            "    sub  t6, t6, gp"], 7


def a_sil3a(c):
    return ["    lw   t5, LIM(s0)", f"    bltu t5, t6, .Ls{c}", "    snez t6, s11", "    neg  t6, t6"], 4


def a_sil3b():
    return ["    lw   gp, LIM1(s0)", "    xor  tp, t5, gp", "    and  tp, tp, t6", "    xor  t5, t5, tp", "    sw   t5, LIM(s0)"], 5


def a_next(nsname, reload=False):
    L = [f"    lw   s10, {nsname}(s0)"]
    if reload:
        L += ["    lw   t5, MPER(s0)", "    sw   t5, MCNT(s0)"]
    return L, 3 if reload else 1


def a_rep_calc():
    return ["    slli t5, s11, 1", "    sub  t5, t5, a7", "    srli t5, t5, 2", "    li   gp, 0xB7"], 4


def a_wr(prep):
    return prep + ["    sw   gp, 0(a3)"], 8.1 + len(prep)


def a_flush():
    return ["    li   gp, 1", "    sw   gp, 4(a3)"], 9.1


def a_rep_under():
    return ["    lw   gp, UNDER(s0)", "    andi gp, gp, 255"], 2


# ------------------------------------------------------------------ one code copy
def copy_code(c, pads):
    """Returns the text of copy c: sled, E unrolled slots, the loop, D and E slots, out-of-line stubs."""
    stubs = []
    out = []
    sled_end = f".LsE{c}"
    out.append(f"    .align 4\nlutg_a16_copy_{c}:\n{nops(SLED)}\n{sled_end}:")
    sch = {j: [] for j in range(E)}
    sch[0] = [a_mv, a_phase0]           # reuse next-row registers before slots 4..6
    sch[1] = [a_fill, a_qend]
    sch[2] = [a_xa]
    sch[3] = [a_xb]
    sch[4] = [a_under, lambda: a_nptr(0)]
    sch[5] = [lambda: a_nptr(1)]
    sch[6] = [lambda: a_nptr(2)]
    if c == "N":
        sch[7] = [a_av]
        sch[8] = [a_avc]
        sch[9] = [lambda: a_rd(stubs)]
        sch[10] = [a_sb]
    elif c == "A":                       # silence check
        sch[6].append(lambda: a_next("NS_B"))
        sch[7] = [a_sil1]
        sch[8] = [a_sil2]
        sch[9] = [lambda: a_sil3a(c)]
        sch[10] = [a_sil3b]
    elif c == "B":                       # fill report: marker, fill lo, fill hi
        sch[6].append(lambda: a_next("NS_C"))
        sch[7] = [a_rep_calc]
        sch[8] = [lambda: a_wr([])]
        sch[9] = [lambda: a_wr(["    andi gp, t5, 255"])]
        sch[10] = [lambda: a_wr(["    srli gp, t5, 8"])]
    else:                                # C: underruns, flush
        sch[6].append(lambda: a_next("NS_N", reload=True))
        sch[7] = [a_rep_under]
        sch[8] = [lambda: a_wr([])]
        sch[9] = [a_flush]
    sch[11] = [lambda: a_nsym(c, "tp")]
    sch[12] = [a_qadj]
    if c == "N":                         # which copy comes next: the maintenance countdown (tp = 1 when the next pass is maintenance), then the sled end of that copy
        sch[11].append(a_mcn_a)
        sch[12].append(a_mcn_b)
    sch[2].append(a_phase2)
    sch[3].append(a_phase3)
    sch[12].append(a_tgt)
    for j in range(E):
        s = Slot(f"{c}{j}", pads)
        L, cst = store_unrolled(j)
        s.add(L, cst)
        if j == 0:
            L, cst = a_mv()
            s.add(L, cst)
            for event in sch[0][1:]:
                L, cst = event()
                s.add(L, cst)
            fl = []                 # hide the DIV load latency behind the kernel loads
        else:
            fl = sch[j]
        s.add(kern(koff(j), Q), KERN)
        for f in fl:
            L, cst = f()
            s.add(L, cst)
        if j == E - 1 and REC in ("timing", "words"):
            s.add(f"    addi tp, a0, {4 * E}", 1)     # record pointer of the loop (production: a nop, same cycles)
        elif j == E - 1:
            s.add("    nop", 1)
        out.append(f"/* {c} slot {j} */\n" + s.finish())
    # ---- the loop: slots E .. S-3
    ll = f".Lloop{c}"
    s = Slot(f"{c}L", pads)
    s.lines.append(f"{ll}:")
    if REC == "timing":
        s.add(["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", "    sw   ra, 0(tp)", "    addi tp, tp, 4"], 4)
    elif REC == "words":
        s.add(["    sw   t3, 0(a6)", "    mv   ra, t3", "    sw   ra, 0(tp)", "    addi tp, tp, 4"], 4)
    else:
        s.add(["    sw   t3, 0(a6)", nops(3)], 4)
    s.add(["    lw   t0, 0(s1)", "    lw   t1, 0(s2)", "    addi s1, s1, 4", "    addi s2, s2, 4", "    lw   t2, 0(s3)", "    addi s3, s3, 4",
           "    add  t0, t0, t1", "    add  t0, t0, t2", "    xor  t3, t0, a5"], 9)
    s.cost += 3                                                     # taken branch
    out.append(f"/* {c} loop */\n" + s.finish() + f"\n    beq  s1, s9, {ll}x\n    j    {ll}\n{ll}x:")     # both paths cost 3 cycles
    # ---- D: measure
    s = Slot(f"{c}D", pads)
    st, sc = store_group(0, "tp")
    s.add(st, sc)
    s.add(kern(0, Q), KERN)
    hand, back = f".Lh{c}", f".Lb{c}"
    s.add(["    csrr t5, 0x7e2", "    sub  t5, a1, t5", f"    sltiu t6, t5, {SLED + 1}", f"    beqz t6, {hand}", f"{back}:", "    slli t5, t5, 1", "    sub  t6, s10, t5"], 6)
    if REC == "timing":
        s.add("    sw   t5, SLEDREC(s0)", 1)
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
    s = Slot(f"{c}E", pads)
    st, sc = store_group(4, "tp")
    s.add(st, sc)
    s.add(kern(0, NQ), KERN)
    s.add("    addi a0, tp, 8" if REC != "none" else "    nop", 1)
    s.add("    jalr x0, 0(t6)", 3)
    out.append(f"/* {c} slot E (jump into the next sled; its length is the sync) */\n" + "\n".join(s.lines))
    EFFECTIVE[f"{c}E_cost"] = s.cost
    out.append(f".Lx{c}:\n    j    lutg_a16_exit")
    stubs.append(f".Ls{c}:\n    li   t5, 2\n    sw   t5, EXITC(s0)\n    j    lutg_a16_exit")
    out += stubs
    return "\n".join(out)


def function():
    pads = PADS_ALL.get(str(P), {})
    global PADS
    PADS = pads
    f = [f"""
    .global lutg_a16_run_p20
    .type lutg_a16_run_p20, @function
    .align 4
lutg_a16_run_p20:
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
    lw   s4, S64M(s0)           /* 4 * S: the byte size of a table row */
    addi s4, s4, 4
    lw   s5, T0(s0)             /* row pointers of the first symbol (history 0) */
    lw   s6, T1(s0)
    lw   s7, T2(s0)
    lw   a0, REC(s0)
    lw   t0, 0(s5)
    lw   t1, 0(s6)
    lw   t2, 0(s7)
    add  t0, t0, t1
    add  t0, t0, t2
    xor  t3, t0, a5             /* word of sample 0 */
    la   t5, .LsEN
    sw   t5, NS_N(s0)
    la   t5, .LsEA
    sw   t5, NS_A(s0)
    la   t5, .LsEB
    sw   t5, NS_B(s0)
    la   t5, .LsEC
    sw   t5, NS_C(s0)
    lw   a1, TN(s0)             /* start of the first symbol; the target of the first measure is one symbol later (base interval added in slot 12) */
    addi a1, a1, {-pads.get('KD', 8)}
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
    la   t6, .LsEN
    sub  t6, t6, t5
    jalr x0, 0(t6)
"""]
    for c in COPIES:
        f.append(copy_code(c, pads))
    f.append("""lutg_a16_exit:
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
    .size lutg_a16_run_p20, . - lutg_a16_run_p20
""")
    return "\n".join(f)


emit("/* Generated by gen_lutg_a16.py - do not edit by hand (rec mode: " + REC + ").\n"
     " * 16APSK / RRC modulator for the ESP32-C3: any samples per symbol S >= %d (run-time value), 20 cycles between samples, with an optional fractional cycle at the symbol boundary,\n"
     " * 3 groups of 2 symbols x 256 rows x S words (RRC span 6), 4 bit symbols in the USB stream. Context: struct lutg_ctx_t in main.c. */" % MIN_S)
for k, v in CTX.items():
    emit(f"    .equ {k}, {v}")
emit('    .section .iram1.lutg_a16, "ax"\n    .option norvc')
emit(function())
print("\n".join(o))
json.dump({str(P): EFFECTIVE}, open(os.path.join(HERE, "lutg_a16_pads_used.json"), "w"), indent=0)
