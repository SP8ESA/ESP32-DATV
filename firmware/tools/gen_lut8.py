#!/usr/bin/env python3
"""Generates main/lut8.S: a cycle-deterministic QPSK/RRC modulator, 8 samples per symbol, one DAC store every 20 CPU cycles.

Layout: a superpass = 8 symbols = 64 slots. Two copies: N (normal) and M (maintenance: fill report, silence check), each
starting with a SYNC. Every slot k: STORE (DAC word for this slot), KERN (word for the next slot), events, nop padding to
exactly 20 cycles (the PAD table is tuned from timing measurements). Slot 7 of every symbol ends with SYNC (nop sled to the
exact symbol start), so only slots 0..6 have to be exactly 20 cycles.

usage (from firmware/):  python3 tools/gen_lut8.py > main/lut8.S
Timing build: python3 tools/gen_lut8.py --timing > main/lut8.S and #define LUT8_TIMING in main.c: instead of driving the DAC
every slot records the cycle counter, and the end of a run prints the spacing of all stores (target: 20 cycles). Adjust the
per-slot padding in lut8_pads.json and regenerate.
"""
import json
import os
import sys

TIMING = "--timing" in sys.argv
HERE = os.path.dirname(os.path.abspath(__file__))
padfile = os.path.join(HERE, "lut8_pads.json")
PADS = json.load(open(padfile)) if os.path.exists(padfile) else {}

SLED = 24           # nops in every sync sled
SYNCK = 10          # cycles from the csrr to the store, outside the sled (only shifts the phase of all stores)
READ_NOPS = 9       # no-data path of the USB read, balanced against the data path
o = []
emit = o.append
EFFECTIVE = {}

# cost model (cycles) used for the first padding guess; the PAD table refines it
COST = {"STORE": 2, "KERN": 5}


def pad(copy, s, j, used):
    key = f"{copy}{s}{j}"
    n = PADS.get(key, 20 - used)
    if n < 0:
        n = 0
    EFFECTIVE[key] = n
    emit(f"    .rept {n}\n    nop\n    .endr")


def store(copy, s, j):
    k = 8 * s + j + (64 if copy == "M" else 0)
    if TIMING:
        emit(f"    csrr ra, 0x7e2\n    sw   ra, {64 + 4 * k}(s0)")
    else:
        emit("    sw   t3, 0(a6)\n    nop")            # same size and cost as the timing build


def kern(jn, p):
    emit(f"    lw   t0, {4 * jn}({p[0]})\n    lw   t1, {4 * jn}({p[1]})\n    add  t0, t0, t1\n    xor  t3, t0, a5")


uid = [0]


def lab(name):
    uid[0] += 1
    return f".L{name}{uid[0]}"


OUTLINE = []            # out-of-line early/late handlers of the sync points, emitted after the copies


def sync(adj=0):
    """Normal path: no taken branch after the sled. Early (spin) and late (count) cases jump to an out-of-line handler."""
    w, x, h = lab("sw"), lab("sx"), lab("sh")
    emit(f"""    addi s6, s6, {160 + adj}
{w}:
    csrr t4, 0x7e2
    sub  t5, s6, t4
    bgeu t5, s3, {h}
    slli t5, t5, 2
    auipc t6, 0
    sub  t6, t6, t5
    jalr x0, ({SLED} * 4 + 12)(t6)
    .rept {SLED}
    nop
    .endr
{x}:""")
    OUTLINE.append(f"""{h}:
    blt  zero, t5, {w}
    j    {x}""")                        # late: store at once; this path is shorter than the period, so the loop catches up


# ------------------------------------------------------------------ events (cycles in comments are estimates)
def ev_decode():                        # 6: next symbol from the bit buffer into the history
    emit("""    andi t4, s8, 3
    srli s8, s8, 2
    slli s7, s7, 2
    or   s7, s7, t4
    slli s7, s7, 16
    srli s7, s7, 16""")
    return 6


def ev_space():                         # 4: t5 = ring has room for one more USB packet
    emit("""    sub  t5, s11, a7
    addi t5, t5, 64
    srli t5, t5, 14
    seqz t5, t5""")
    return 4


def ev_nptr(n):                         # 6: table rows of the next symbol
    emit(f"""    andi t4, s7, 255
    slli t4, t4, 5
    add  {n[0]}, s1, t4
    srli t4, s7, 8
    slli t4, t4, 5
    add  {n[1]}, s2, t4""")
    return 6


def ev_raddr():                         # 3: t6 = ring address for the next USB byte
    emit("""    slli t6, s11, 18
    srli t6, t6, 18
    add  t6, t6, s10""")
    return 3


def ev_avail():                         # ~9: t4 = USB byte waiting and room in the ring
    emit("""    lw   t4, 4(a3)
    srli t4, t4, 2
    andi t4, t4, 1
    and  t4, t4, t5""")
    return 9


def ev_read():                          # ~12 both paths: read the byte into the ring
    nr, dn = lab("nr"), lab("rd")
    emit(f"""    beqz t4, {nr}
    lw   t4, 0(a3)
    sb   t4, 0(t6)
    addi s11, s11, 1
    j    {dn}
{nr}:
    .rept {PADS.get('READ_NOPS', READ_NOPS)}
    nop
    .endr
{dn}:""")
    return 12


def ev_refill_a():                      # ~8: next 8 symbols (16 bits) from the ring; t4 = underrun
    emit("""    sub  t4, s11, a7
    sltiu t4, t4, 2
    slli t5, a7, 18
    srli t5, t5, 18
    add  t5, t5, s10
    lhu  s8, 0(t5)""")
    return 8


def ev_refill_b():                      # ~8: advance the read counter unless underrun, count underruns
    emit("""    xori t5, t4, 1
    slli t5, t5, 1
    add  a7, a7, t5
    lw   t5, 28(s0)
    add  t5, t5, t4
    sw   t5, 28(s0)""")
    return 8


def ev_end_a():                         # ~8: superpass counter (exit when done), maintenance flag in t2
    ok = lab("ok")
    emit(f"""    addi a4, a4, -1
    bnez a4, {ok}
    j    lut8_exit
{ok}:
    lw   t2, 36(s0)
    addi t2, t2, -1
    sw   t2, 36(s0)
    seqz t2, t2""")
    return 9


def ev_end_b():                         # branch to the next copy (before its SYNC, so the sled absorbs the difference)
    m = lab("gm")
    emit(f"""    bnez t2, {m}
    j    lut8_copy_N
{m}:
    j    lut8_copy_M""")


# maintenance events (copy M): one APB access per slot (an APB write costs ~9 cycles), state in a2 / s9
def ev_rep_a():                         # a2 = IN FIFO free, s9 = fill in pairs
    emit("""    lw   a2, 4(a3)
    srli a2, a2, 1
    andi a2, a2, 1
    sub  s9, s11, a7
    srli s9, s9, 1""")
    return 11


def ev_rep_w(name, prep, nprep, reg_off=0):
    sk, dn = lab("rs"), lab("rw")
    emit(f"""    beqz a2, {sk}
{prep}
    sw   t6, {reg_off}(a3)
    j    {dn}
{sk}:
    .rept {PADS.get(name, 9 + nprep)}
    nop
    .endr
{dn}:""")
    return 12 + nprep


def ev_rep_b7():
    return ev_rep_w("W_B7", "    li   t6, 0xB7", 1)


def ev_rep_lo():
    return ev_rep_w("W_LO", "    andi t6, s9, 255", 1)


def ev_rep_hi():
    return ev_rep_w("W_HI", "    srli t6, s9, 8", 1)


def ev_rep_un():
    return ev_rep_w("W_UN", "    lw   t6, 28(s0)\n    andi t6, t6, 255", 3)


def ev_rep_fl():
    return ev_rep_w("W_FL", "    li   t6, 1", 1, 4)


def ev_sil_a():                         # reload the maintenance counter; t5 = USB bytes arrived since the last check
    emit("""    lw   t2, 32(s0)
    sw   t2, 36(s0)
    lw   t5, 44(s0)
    sub  t5, s11, t5""")
    return 6


def ev_sil_b():
    sm, dn = lab("ss"), lab("sd")
    emit(f"""    beqz t5, {sm}
    sw   s11, 44(s0)
    csrr t6, 0x7e2
    sw   t6, 40(s0)
    j    {dn}
{sm}:
    .rept {PADS.get('SILB_NOPS', 3)}
    nop
    .endr
{dn}:""")
    return 8


def ev_sil_c():                         # silent for longer than the limit: switch the transmitter off
    ok = lab("so")
    emit(f"""    csrr t6, 0x7e2
    lw   t4, 40(s0)
    sub  t6, t6, t4
    lw   t4, 48(s0)
    bgeu t4, t6, {ok}
    li   t4, 2
    sw   t4, 56(s0)
    j    lut8_exit
{ok}:""")
    return 8


def ev_lim():                           # after the first byte the silence limit drops from lim0 to lim1
    k, dn = lab("lk"), lab("ld")
    emit(f"""    beqz s11, {k}
    lw   t4, 52(s0)
    sw   t4, 48(s0)
    j    {dn}
{k}:
    .rept {PADS.get('LIM_NOPS', 3)}
    nop
    .endr
{dn}:""")
    return 8


def ev_stop():                          # a stop request from C (flag in ctx) ends the run
    ok = lab("st")
    emit(f"""    lw   t4, 56(s0)
    beqz t4, {ok}
    j    lut8_exit
{ok}:""")
    return 4


# ------------------------------------------------------------------ program
emit("""/* Generated by gen_lut8.py - do not edit by hand.
 * Cycle-deterministic QPSK / RRC modulator for the ESP32-C3: 8 samples per symbol, one DAC word every 20 CPU cycles (8 MS/s
 * at 160 MHz, 1 MBd), tables of 2 groups x 256 rows x 8 words (RRC span 8 symbols). See main.c (lut8_ctx_t) for the context. */
    .section .iram1.lut8, "ax"
    .option norvc
    .global lut8_run
    .type lut8_run, @function
    .align 4
lut8_run:
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
    mv   s0, a0
    lw   s1, 0(s0)              /* T0 */
    lw   s2, 4(s0)              /* T1 */
    lw   s10, 8(s0)             /* ring */
    lw   s11, 12(s0)            /* qw */
    lw   a7, 16(s0)             /* qr */
    lw   s6, 20(s0)             /* first symbol start (cycles) */
    addi s6, s6, -160
    lw   a4, 24(s0)             /* superpasses to run */
    li   s3, """ + str(SLED + 1) + """         /* sled range, for the unsigned compare */
    li   a6, 0x3fcb0000         /* held DAC word */
    li   a5, 0x80200            /* offset binary -> two's complement, both fields */
    li   a3, 0x60043000         /* USB Serial/JTAG */
    li   s7, 0                  /* symbol history */
    li   s8, 0                  /* symbol bits */
    mv   s4, s1                 /* rows of the first symbol (history 0) */
    mv   s5, s2
    mv   a0, s1
    mv   a1, s2
    lw   t0, 0(s4)
    lw   t1, 0(s5)
    add  t0, t0, t1
    xor  t3, t0, a5
    li   t2, 0
    j    lut8_copy_N
""")

for copy in ("N", "M"):
    # The copy starts with slot 7 of symbol 7 of the previous superpass (store, word for symbol 0, sync), so the jump between
    # superpasses sits at the end of slot 6 of symbol 7, where its cost is fixed and padded, never before a sync.
    emit(f"    .align 4\nlut8_copy_{copy}:")
    emit(f"/* {copy} symbol 7 slot 7 (previous superpass) */")
    store(copy, 7, 7)
    kern(0, ("s4", "s5"))
    sync(PADS.get("TOP_ADJ", -1))      # this sync sits at a different code alignment: its fixed delay differs
    for s in range(8):
        P = ("s4", "s5") if s % 2 == 0 else ("a0", "a1")
        Nn = ("a0", "a1") if s % 2 == 0 else ("s4", "s5")
        for j in range(8):
            if s == 7 and j == 7:
                break
            emit(f"/* {copy} symbol {s} slot {j} */")
            store(copy, s, j)
            used = COST["STORE"]
            if j < 7:
                kern(j + 1, P)
            else:
                kern(0, Nn)
            used += COST["KERN"]
            maint = copy == "M"
            if j == 0:
                used += ev_decode() + ev_space()
            elif j == 1:
                used += ev_nptr(Nn) + ev_raddr()
            elif j == 2:
                used += ev_avail()
            elif j == 3:
                used += ev_read()
            elif j == 4:
                if s == 7:
                    used += ev_refill_a()
                elif maint:
                    used += {1: ev_rep_a, 2: ev_rep_hi, 3: ev_sil_a, 4: ev_lim}.get(s, lambda: 0)()
            elif j == 5:
                if s == 7:
                    used += ev_refill_b()
                elif maint:
                    used += {1: ev_rep_b7, 2: ev_rep_un, 3: ev_sil_b}.get(s, lambda: 0)()
            elif j == 6:
                if s == 7:
                    used += ev_end_a() + 3
                elif maint:
                    used += {1: ev_rep_lo, 2: ev_rep_fl, 3: ev_sil_c}.get(s, lambda: 0)()
            if j < 7:
                pad(copy, s, j, used)
                if s == 7 and j == 6:
                    ev_end_b()
            else:
                sync(-PADS.get("TOP_ADJ", -1) if s == 0 else 0)

emit("\n".join(OUTLINE))
emit("""lut8_exit:
    sw   s11, 12(s0)
    sw   a7, 16(s0)
    sw   a4, 24(s0)
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
    addi sp, sp, 64
    ret
    .size lut8_run, . - lut8_run
""")
print("\n".join(o))
for k_ in ("READ_NOPS", "W_B7", "W_LO", "W_HI", "W_UN", "W_FL", "SILB_NOPS", "LIM_NOPS"):
    if k_ in PADS:
        EFFECTIVE[k_] = PADS[k_]
json.dump(EFFECTIVE, open(os.path.join(HERE, "lut8_pads_used.json"), "w"), indent=0)
