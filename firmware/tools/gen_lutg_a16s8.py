#!/usr/bin/env python3
"""Generate the fixed 8-SPS 16APSK RRC loop (1 MSym/s, DAC 8 MS/s).

Three row-major tables retain all six RRC symbols. A pass consumes four bytes
(eight nibble symbols). Normal pairs read eight USB bytes in four-byte groups;
the host supplies aligned writes into a 16 KB ring. Two copies cover normal
and maintenance work. Pair deadlines are exactly 320 CPU cycles apart; sample
slots are scheduled near 20 cycles, with USB clock quantization at some stores.
Generate from firmware/: python3 tools/gen_lutg_a16s8.py [--rec timing|words].
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
PADFILE = os.path.join(HERE, "lutg_a16s8_pads.json")
PADS_ALL = json.load(open(PADFILE)) if os.path.exists(PADFILE) else {}

P = 20
S = 8
NSYM = 8                 # symbols per pass
SLED = 32
SYNC = 2 * S * P         # cycles between two sync points (a pair of symbols)
COPIES = ("N", "M")
CTX = dict(T0=0, T1=4, T2=8, T3=12, RING=16, QW=20, QR=24, TN=28, NSYM=32, UNDER=36, MPER=40, MCNT=44, TRX=48, QWSEEN=52, LIM=56, LIM1=60,
           EXITC=64, LATE=68, S64M=72, SPER=76, RLIM=80, NS_N=84, NS_A=88, NS_B=92, NS_C=96, SLEDREC=100, REC=104, RECP=108)
KERN = 6
o = []
emit = o.append
EFFECTIVE = {}
PADS = {}
uid = [0]


def lab(name):
    uid[0] += 1
    return f".L{name}{uid[0]}"


def nops(n):
    """Compressed one-cycle padding; sync sled offsets use two bytes per cycle."""
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
        grp = 3
        if pre and src[0].startswith("    sw   t3, 0(a6)"):
            EFFECTIVE[self.key + "p"] = pre
            src = src[:grp] + [nops(pre)] + src[grp:]
        lines = schedule(src) if SCHED else src
        if n:
            lines = lines + [nops(n)]
        return "\n".join(lines)


def store_group(n):
    off = 4 * (n - NSYM*S if n >= 58 else n)
    if REC == "timing":
        return ["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", f"    sw   ra, {off}(a0)"], 3
    if REC == "words":
        return ["    sw   t3, 0(a6)", "    .option push\n    .option norvc\n    mv   ra, t3\n    .option pop", f"    sw   ra, {off}(a0)"], 3
    # Match the recording build's instruction widths, dependencies and timing.
    # Offset 60 is unused scratch in this function's 64-byte stack frame.
    return ["    sw   t3, 0(a6)", "    csrr ra, 0x7e2", "    .option push\n    .option norvc\n    sw   ra, 60(sp)\n    .option pop"], 3


def kern(imm, regs):
    return [f"    lw   t0, {imm}({regs[0]})", f"    lw   t1, {imm}({regs[1]})", f"    lw   t2, {imm}({regs[2]})",
            "    add  t0, t0, t1", "    add  t0, t0, t2", "    xor  t3, t0, a5"]


SETS = (("s1", "s2", "s3"), ("s5", "s6", "s7"))


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


# ------------------------------------------------------------------ events: (lines, cycles). Registers: t0-t3 form the sample kern, s9 is the nibble window.
# a2: history, a7/s11: consumed/written bytes, a4: USB increment (0 or 4),
# tp: even USB word / next sample-zero value, gp: odd USB word, t4-t6: scratch.
def ev_ext():
    return ["    andi t6, s9, 15", "    srli s9, s9, 4", "    slli a2, a2, 4", "    or   a2, a2, t6"], 4


def ev_nptr(k, g):
    dest = nxt(k)[g]
    lines = [f"    lw   {dest}, T{g}(s0)", f"    slli t6, a2, {24 - 8*g}", "    srli t6, t6, 19"]
    if g:
        lines.append("    andi t6, t6, -32")
    lines.append(f"    add  {dest}, {dest}, t6")
    return lines, len(lines)


def ev_av(stubs):
    cached, back = lab("cached"), lab("avback")
    stubs.append(f"{cached}:\n{nops(PADS.get('AVNOPS', 6))}\n    j    {back}")
    return [f"    bnez t5, {cached}", "    lw   t5, 4(a3)", "    andi t5, t5, 4", "    slli t5, t5, 1", f"{back}:"], 11


def ev_avc():
    return ["    snez t6, t5", "    and  a4, a4, t6", "    slli a4, a4, 2"], 3


def ev_rd(stubs, byte, word="tp"):
    nd, back = lab("nd"), lab("rb")
    skip = (1, 4 if word == "gp" else 3, 4, 4)[byte]
    stubs.append(f"{nd}:\n{nops(PADS.get(f'RDNOPS{byte}', skip))}\n    j    {back}")
    dest = word if byte == 0 else "t4"
    lines = [f"    beqz a4, {nd}", f"    lw   {dest}, 0(a3)", f"{back}:"]
    if byte:
        lines += [f"    slli t4, t4, {8 * byte}", f"    or   {word}, {word}, t4"]
    return lines, (7, 11 if word == "gp" else 10, 11, 11)[byte]


def ev_sb(word="tp", offset=0):
    return [f"    sw   {word}, {offset}(s8)", "    add  s11, s11, a4"], 2


def ev_packet_end():
    return ["    srli t4, a4, 2", "    sub  t5, t5, t4", "    sw   t5, NS_B(s0)"], 3


def ev_write_address():
    return ["    slli s8, s11, 18", "    srli s8, s8, 18", "    add  s8, s8, s4"], 3


def ev_adv():
    return ["    slli t5, gp, 2", "    add  a7, a7, t5"], 2


def ev_tgt():
    return [f"    addi a1, a1, {SYNC}"], 1


def ev_sil1():
    return ["    lw   t5, QWSEEN(s0)", "    sub  t5, s11, t5", "    snez t5, t5", "    neg  t5, t5", "    csrr t6, 0x7e2"], 5


def ev_sil2():
    return ["    lw   gp, TRX(s0)", "    xor  tp, gp, t6", "    and  tp, tp, t5", "    xor  gp, gp, tp", "    sw   gp, TRX(s0)", "    sw   s11, QWSEEN(s0)",
            "    sub  t6, t6, gp"], 7


def ev_sil3a(stubs):
    x = lab("s")
    stubs.append(f"{x}:\n    li   t5, 2\n    sw   t5, EXITC(s0)\n    j    lutg_a16s8_exit")
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
def build_pass(c, stubs):
    pl = {n: [] for n in range(NSYM*S)}
    def at(k,j,*events):
        pl[8*k+j] += list(events)
    for k in range(NSYM):
        at(k,0,ev_ext(),ev_nptr(k,0))
        at(k,1,ev_nptr(k,1))
        at(k,2 if k%2 else 7,ev_nptr(k,2))
        if not k%2:
            at(k,1,ev_write_address())
            if c == "N":
                sp=stubs[k//2]
                at(k,0,(["    lw   t5, NS_B(s0)", "    sub  a4, s11, a7"],2))
                at(k,1,(["    srli a4, a4, 6", "    sltiu a4, a4, 255"],2))
                at(k,2,ev_av(sp))
                at(k,3,ev_avc(),ev_rd(sp,0))
                for byte in range(1,4):
                    at(k,byte+3,ev_rd(sp,byte))
                at(k,7,ev_sb(),ev_packet_end())
        else:
            at(k,2,([f"    lw   tp, 0({nxt(k)[0]})", f"    lw   t4, 0({nxt(k)[1]})", "    add  tp, tp, t4"],3))
            at(k,3,([f"    lw   t4, 0({nxt(k)[2]})", "    add  tp, tp, t4", "    xor  tp, tp, a5"],3))
            at(k,0,ev_tgt())
            if c == "N":
                sp=stubs[k//2]
                for byte,j in enumerate((3,4,5,7)):
                    at(k,j,ev_rd(sp,byte,"gp"))
                at(k,7,ev_sb("gp",4))
    exit_label=lab("x")
    stubs[0].append(f"{exit_label}:\n    j    lutg_a16s8_exit")
    # Production counts one 257-pass maintenance period; recording counts
    # symbols. Both use the same instruction lengths and cycle schedule.
    step = -8 if REC != "none" else -1 if c == "M" else 0
    at(1,0,(["    lw   tp, NSYM(s0)", f"    addi tp, tp, {step}"],2))
    at(1,1,(["    sw   tp, NSYM(s0)", f"    beqz tp, {exit_label}"],2))
    at(5,0,(["    sub  gp, s11, a7", "    addi gp, gp, -3"],2))
    at(5,1,(["    slt  gp, zero, gp", "    sw   gp, NS_C(s0)", "    lw   t4, UNDER(s0)", "    xori t6, gp, 1", "    add  t4, t4, t6"],5))
    pl[42].insert(0,(["    sw   t4, UNDER(s0)"],1))
    at(7,0,(["    lw   gp, NS_C(s0)"],1))
    at(7,1,(["    slli t4, a7, 18", "    srli t4, t4, 18", "    add  t4, t4, s4"],3),ev_adv())
    at(7,1,([f"    addi a0, a0, {4 * NSYM*S}" if REC != "none" else "    .4byte 0x00000013"],1))
    # Read the window before the sample-zero preload reuses t4.
    pl[58].insert(0,(["    lw   s9, 0(t4)"],1))
    if c == "N":
        at(1,1,(["    lw   s10, MCNT(s0)"],1))
        at(1,2,(["    addi s10, s10, -1", "    sw   s10, MCNT(s0)"],2))
        at(3,0,(["    seqz s10, s10", "    slli s10, s10, 2"],2))
        at(3,1,(["    add  s10, s10, s0"],1))
        at(3,2,(["    lw   s10, NS_N(s10)"],1))
    else:
        at(2,3,ev_sil1())
        at(2,4,ev_sil2())
        at(2,5,ev_sil3a(stubs[1]))
        at(2,6,ev_sil3b())
        at(4,2,ev_rep_calc())
        at(4,3,ev_wr("gp"))
        at(4,4,ev_wr("tp"))
        at(4,5,ev_wr("a4"))
        at(6,2,ev_rep_under())
        at(6,3,ev_wr("tp"))
        at(6,4,(["    sw   a4, 4(a3)"],8.1))
        at(4,7,ev_next("NS_N",reload=True))
    return pl


def copy_code(c, pads):
    stubs = [[] for _ in range(NSYM // 2)]
    pl = build_pass(c, stubs)
    out = []
    for m in range(NSYM // 2):
        sled_end = f".LsE{c}{m}"
        out.append(f"    .align 4\nlutg_a16s8_copy_{c}{m}:\n{nops(SLED)}\n{sled_end}:")
        for jj in range(2 * S):
            n = 16 * m + jj
            k, j = n // S, n % S
            s = Slot(f"{c}{n}", pads)
            st, sc = store_group(n)
            s.add(st, sc)
            if jj == 10:
                s.add("    csrr t5, 0x7e2\n", 1)
            if jj == 14:                                   # D: deadline from the fixed CSR in slot 10
                s.add(kern(28, cur(k)), KERN)
                s.add(["    sub  t5, a1, t5", "    addi t5, t5, -100", f"    sltiu t6, t5, {SLED+1}"],3)
                hand, back = f".Lh{c}{m}", f".Lb{c}{m}"
                last = m == NSYM // 2 - 1
                tgt = ["    sub  t6, s10, t5"] if last else [f"    la   t6, .LsE{c}{m + 1}", "    sub  t6, t6, t5"]
                s.add([f"    beqz t6, {hand}", f"{back}:", "    slli t5, t5, 1"] + tgt,
                      2 + (1 if last else 3))
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
    li   t5, {SLED}
    j    {back}""")
                continue
            if jj == 15:                                   # E: jump into the sled
                for L, cst in pl[n]:
                    s.add(L, cst)
                s.add("    mv   t3, tp", 1)
                if c == "M":
                    s.add(nops(13),13)
                s.add("    jalr x0, 0(t6)", 3)
                out.append(f"/* {c} slot {n} (E, jump into the sled) */\n" + "\n".join(s.lines))
                EFFECTIVE[f"{c}{n}_cost"] = s.cost
                out += stubs[m]
                continue
            imm = 4*(j+1) if j < S-1 else 0
            events = pl[n]
            if j == S-1:
                lines, cost = events[0]
                s.add(lines, cost)
                events = events[1:]
            s.add(kern(imm, cur(k) if j < S - 1 else cur(k + 1)), KERN)
            for L, cst in events:
                s.add(L, cst)
            out.append(f"/* {c} slot {n} */\n" + s.finish())
    return "\n".join(out)


def function():
    pads = PADS_ALL.get(str(P), {})
    global PADS
    PADS = pads
    f = [f"""
    .global lutg_a16s8_run_p20
    .type lutg_a16s8_run_p20, @function
    .align 4
lutg_a16s8_run_p20:
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
    sw   zero, NS_B(s0)         /* complete 64-byte USB packet: eight-byte pairs left */
    li   t5, 1
    sw   t5, NS_C(s0)           /* initial record availability for underrun accounting */
    li   a2, 0                  /* symbol history */
    lw   s1, T0(s0)
    lw   s2, T1(s0)
    lw   s3, T2(s0)
    lw   s4, RING(s0)           /* ring base; s8 is the aligned USB write address */
    lw   a0, REC(s0)
    lw   t0, 0(s1)
    lw   t1, 0(s2)
    lw   t2, 0(s3)
    add  t0, t0, t1
    add  t0, t0, t2
    xor  t3, t0, a5
    la   t5, .LsEN0
    sw   t5, NS_N(s0)
    la   t5, .LsEM0
    sw   t5, NS_A(s0)
    sub  gp, s11, a7
    addi gp, gp, -3
    slt  gp, zero, gp
    slli t5, a7, 18
    srli t5, t5, 18
    add  t5, t5, s4
    lw   s9, 0(t5)
    slli t5, gp, 2
    add  a7, a7, t5
    slli s8, s11, 18
    srli s8, s8, 18
    add  s8, s8, s4
    lw   t6, RLIM(s0)
    sub  t5, s11, a7
    slt  a4, t5, t6
    lw   a1, TN(s0)             /* first pair deadline; each odd symbol advances it by 320 cycles */
    addi a1, a1, {-pads.get('KD', -16)}
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
    f.append("""lutg_a16s8_exit:
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
    .size lutg_a16s8_run_p20, . - lutg_a16s8_run_p20
""")
    return "\n".join(f)


emit("/* Generated by gen_lutg_a16s8.py - do not edit by hand (rec mode: " + REC + ").\n"
     " * 1 MBd 16APSK: 8 samples/symbol, DAC 8 MS/s (320-cycle pair deadlines), nibble symbols in the USB stream.\n"
     " * Tables: 3 groups x 256 rows x 8 words (RRC span 6). Context: struct lutg_ctx_t in main.c. */")
for k, v in CTX.items():
    emit(f"    .equ {k}, {v}")
emit('    .section .iram1.lutg_a16s8, "ax"\n    .option rvc')
emit(function())
print("\n".join(o).rstrip())
json.dump({str(P): EFFECTIVE}, open(os.path.join(HERE, "lutg_a16s8_pads_used.json"), "w"), indent=0)
