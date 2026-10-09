#!/usr/bin/env python3
"""Unrolled six-tap 32APSK at 8 MS/s: 32 samples per symbol (250 kS/s).

Two alternating row-pointer sets avoid moving six pointers at a symbol boundary.
Slow USB accesses use a sample whose first three RRC terms were prefetched in
the previous slot. Symbol-boundary deadline sleds absorb USB clock quantization.
The host supplies full 64-byte packets of byte symbols 0..35.
Generate from firmware/: python3 tools/gen_lutg_a32.py [--rec words|timing].
"""
import argparse
import json
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--rec", choices=("none", "words", "timing"), default="none")
ap.add_argument("--only", choices=("32",), default="32")
a = ap.parse_args()
here = Path(__file__).resolve().parent
padfile = here / "lutg_a32_pads.json"
pads = json.loads(padfile.read_text()) if padfile.exists() else {}
used = {}
P, SLED, ROWS = 20, 32, 36
C = dict(RING=24, QW=28, QR=32, TN=36, COUNT=40, MPER=44, MCNT=48,
         UNDER=52, LATE=56, TRX=60, QSEEN=64, LIM0=68, LIM1=72,
         LIMIT=76, PACKET=80, NEXT=84, N0=88, M0=92, N1=96, M1=100,
         REC=104, EXIT=108, RLIM=112, FILL=116, HI=120, ACTIVE=124, ROWPTR=128)
sets = [("s1", "s2", "s3", "s4", "s5", "s6"),
        ("s7", "s8", "s9", "s10", "s11", "gp")]
lines = []
emit = lines.append


def nops(n):
    return ["    c.nop"] * max(0, n)


def row_offset(g):
    return C["ROWPTR"] + 4 * ROWS * g


def kernel(ptrs, offset, partial=False):
    if partial:
        return [f"    lw t{i}, {offset}({ptrs[i + 3]})" for i in range(3)] + [
            "    add t0, t0, t1", "    add t0, t0, t2", "    add t0, t0, t5", "    xor t6, t0, a5"]
    return [f"    lw t{i}, {offset}({ptrs[i]})" for i in range(6)] + [
        "    add t0, t0, t1", "    add t0, t0, t2", "    add t0, t0, t3",
        "    add t0, t0, t4", "    add t0, t0, t5", "    xor t6, t0, a5"]


def prefetch(ptrs, offset):
    return [f"    lw t5, {offset}({ptrs[0]})", f"    lw t4, {offset}({ptrs[1]})",
            f"    lw t3, {offset}({ptrs[2]})", "    add t5, t5, t4", "    add t5, t5, t3"]


def store(j):
    # Three instructions at every DAC store; the production scratch store is compressed.
    if a.rec == "words":
        second = "    .option push\n    .option norvc\n    mv ra, t6\n    .option pop"
    else:
        second = "    csrr ra, 0x7e2"
    third = f"    sw ra, {4 * j}(a2)" if a.rec != "none" else "    c.swsp ra, 68(sp)"
    return ["    sw t6, 0(a6)", second, third]


def function(S):
    name = f"lutg_a32_s{S}_run_p20"
    cfg = pads.get(str(S), {})
    ring_bits = 13
    mask_shift = 32 - ring_bits
    paths = ["N0", "N1", "M0", "M1"]
    exit_label = f".L{name}_exit"
    emit(f"\n    .global {name}\n    .type {name}, @function\n    .align 4\n{name}:")
    emit("    addi sp, sp, -80")
    saved = ["ra", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "gp", "tp"]
    for i, reg in enumerate(saved):
        emit(f"    sw {reg}, {4 * i}(sp)")
    for instruction in ["mv s0, a0", "li a4, 0x60043000", "li a5, 0x80200", "li a6, 0x3fcb0000",
                        f"lw a3, {C['RING']}(s0)", f"lw a7, {C['QR']}(s0)", f"lw a2, {C['REC']}(s0)",
                        "li a1, 0", f"lw tp, {C['TN']}(s0)", f"addi tp, tp, {cfg.get('KD', 0)}"]:
        emit("    " + instruction)
    for g, reg in enumerate(sets[0]):
        emit(f"    lw {reg}, {row_offset(g)}(s0)")
    for path in paths:
        emit(f"    la ra, .L{name}_{path}_end")
        emit(f"    sw ra, {C[path]}(s0)")
    emit("\n".join(kernel(sets[0], 0)))
    emit(f".L{name}_startwait:\n    csrr t0, 0x7e2\n    lw ra, {C['TN']}(s0)\n    sub t0, ra, t0\n    bgtz t0, .L{name}_startwait\n    j .L{name}_N0_end")

    for path in paths:
        orient = int(path[1])
        cur, nxt = sets[orient], sets[1 - orient]
        q, flag, packet = nxt[:3]
        e = {}
        stubs = []

        def event(j, code, cost=None):
            if isinstance(code, str):
                code = [code]
            e[j] = (code, len(code) if cost is None else cost)

        event(0, [f"lw {q}, {C['QW']}(s0)", f"lw {packet}, {C['PACKET']}(s0)",
                  "slli a0, a1, 2", "srli a0, a0, 24", "andi a0, a0, 252"])
        event(1, ["add a0, a0, s0", f"lw {nxt[5]}, {row_offset(5)}(a0)", f"sub {flag}, {q}, a7", f"snez {flag}, {flag}",
                  f"slli a0, a7, {mask_shift}"])
        event(2, [f"srli a0, a0, {mask_shift}", "add a0, a0, a3", "lbu a0, 0(a0)", f"neg ra, {flag}", "and a0, a0, ra"], 5)
        event(3, ["andi a0, a0, 63", "sltiu ra, a0, 36", "slli ra, ra, 5", "ori ra, ra, 31", "and a0, a0, ra"])
        event(4, ["slli a1, a1, 6", "or a1, a1, a0", f"add a7, a7, {flag}", f"xori a0, {flag}, 1", f"lw {nxt[4]}, {C['RLIM']}(s0)"])
        under = [f"lw ra, {C['UNDER']}(s0)", "nop", "add ra, ra, a0", f"sw ra, {C['UNDER']}(s0)"]
        if path == "M1":
            under += [f"lbu a0, {C['HI']}(s0)"]
        event(5, under)

        heavy = {7, 10, 13}
        if path in ("N0", "N1"):
            cached, back = f".L{name}_{path}_cached", f".L{name}_{path}_avback"
            event(7, [f"bnez {packet}, {cached}", f"lw {flag}, 4(a4)", f"{back}:", f"andi {flag}, {flag}, 4"], 10)
            stubs += [f"{cached}:", f"    li {flag}, 0"] + nops(cfg.get("AVNOP", 2)) + [f"    j {back}"]
            event(8, [f"slli {flag}, {flag}, 4", f"add {packet}, {packet}, {flag}", f"sub {nxt[3]}, {q}, a7",
                      f"sltu {flag}, {nxt[3]}, {nxt[4]}", f"snez {nxt[3]}, {packet}"])
            for j in (10, 13):
                skip, rb = f".L{name}_{path}_skip{j}", f".L{name}_{path}_readback{j}"
                code = ([f"and {flag}, {flag}, {nxt[3]}"] if j == 10 else []) + [f"beqz {flag}, {skip}", "lw a0, 0(a4)", f"{rb}:"]
                event(j, code, 10 if j == 10 else 9)
                stubs += [f"{skip}:", "    li a0, 0"] + nops(cfg.get("RDNOP", 2)) + [f"    j {rb}"]
                event(j + 1, [f"slli {nxt[4]}, {q}, {mask_shift}", f"srli {nxt[4]}, {nxt[4]}, {mask_shift}",
                              f"add {nxt[4]}, {nxt[4]}, a3", f"sb a0, 0({nxt[4]})", f"add {q}, {q}, {flag}"])
            event(15, [f"slli ra, {flag}, 1", f"sub {packet}, {packet}, ra", f"sw {packet}, {C['PACKET']}(s0)", f"sw {q}, {C['QW']}(s0)"])
        elif path == "M1":
            event(7, ["sw a0, 0(a4)"], 8)
            event(8, [f"lbu a0, {C['UNDER']}(s0)", f"snez {flag}, {q}", f"slli {flag}, {flag}, 2", f"add {flag}, {flag}, s0"])
            event(10, ["sw a0, 0(a4)"], 8)
            event(11, ["li a0, 1", f"lw ra, {C['LIM0']}({flag})", "nop", f"sw ra, {C['LIMIT']}(s0)"])
            event(13, ["sw a0, 4(a4)"], 8)
        if (a.rec != "none" and path in ("M0", "M1")) or path == "M1":
            event(14, [f"lw ra, {C['COUNT']}(s0)", f"sw {q}, {C['QW']}(s0)", "addi ra, ra, -1",
                       f"sw ra, {C['COUNT']}(s0)"])
        if a.rec != "none" and path in ("N0", "N1"):
            event(24, [f"lw ra, {C['COUNT']}(s0)", "nop", "addi ra, ra, -1", f"sw ra, {C['COUNT']}(s0)"])
        for g in range(5):
            event((16 if path in ("N0", "N1") else 15) + g, [f"slli a0, a1, {26 - 6 * g}", "srli a0, a0, 24", "andi a0, a0, 252", "add a0, a0, s0", f"lw {nxt[g]}, {row_offset(g)}(a0)"])

        if path == "N0":
            event(21, [f"lw a0, {C['N1']}(s0)", "nop", f"sw a0, {C['NEXT']}(s0)"])
        elif path == "N1":
            event(21, [f"lw a0, {C['MCNT']}(s0)", "nop", "addi a0, a0, -1", f"sw a0, {C['MCNT']}(s0)"])
            event(22, ["seqz a0, a0", "slli a0, a0, 2", "add a0, s0, a0", f"lw a0, {C['N0']}(a0)"])
            event(23, [f"sw a0, {C['NEXT']}(s0)"])
        elif path == "M0":
            event(20, [f"lw a0, {C['QW']}(s0)", "nop", "sub a0, a0, a7", "srli a0, a0, 1", f"sw a0, {C['FILL']}(s0)"])
            heavy |= {22, 25}
            event(22, ["li a0, 0xB7", "sw a0, 0(a4)"], 9)
            event(23, [f"lw a0, {C['FILL']}(s0)", "nop", "srli a0, a0, 8", f"sw a0, {C['HI']}(s0)"])
            event(25, [f"lw a0, {C['FILL']}(s0)", "sw a0, 0(a4)"], 10)
        else:
            event(20, [f"lw ra, {C['MPER']}(s0)", f"lw a0, {C['N0']}(s0)", f"sw ra, {C['MCNT']}(s0)", f"sw a0, {C['NEXT']}(s0)"])
            event(21, [f"lw ra, {C['QW']}(s0)", f"lw a0, {C['QSEEN']}(s0)", "nop", "sub a0, ra, a0", f"sw ra, {C['QSEEN']}(s0)"])
            event(22, ["snez a0, a0", "neg a0, a0", f"sw a0, {C['ACTIVE']}(s0)", "csrr a0, 0x7e2"])
            event(23, [f"lw t0, {C['TRX']}(s0)", f"lw ra, {C['ACTIVE']}(s0)", "xor a0, a0, t0", "and a0, a0, ra"])
            event(24, [f"lw ra, {C['TRX']}(s0)", "nop", "xor a0, a0, ra", f"sw a0, {C['TRX']}(s0)"])
            event(25, ["csrr ra, 0x7e2", "sub a0, ra, a0", f"lw ra, {C['LIMIT']}(s0)", "nop", f"bltu ra, a0, {exit_label}"])

        # Prefetch three contributions in the preceding slot of every APB access.
        for j in heavy:
            if j - 1 in e and e[j - 1][0]:
                raise RuntimeError(f"prefetch overlaps events: S={S} {path} j={j - 1}")
            event(j - 1, [x.strip() for x in prefetch(cur, 4 * (j + 1))])

        stamp = S - 6
        clock = [f"addi tp, tp, {S * P}"]
        if path == "M0":
            clock += [f"lw a0, {C['M1']}(s0)", "nop", f"sw a0, {C['NEXT']}(s0)"]
        else:
            clock += ["nop"] * 3
        clock += ["csrr a0, 0x7e2"]
        event(stamp, clock)
        event(stamp + 1, ["sub a0, tp, a0", f"addi a0, a0, -{cfg.get('SYNCFIX', 80)}"])
        clamp, cb = f".L{name}_{path}_clamp", f".L{name}_{path}_clampback"
        event(stamp + 2, [f"sltiu ra, a0, {SLED + 1}", f"beqz ra, {clamp}", f"{cb}:", "slli a0, a0, 1"], 3)
        stubs += [f"{clamp}:", f"    bgtz a0, {clamp}high", f"    lw ra, {C['LATE']}(s0)", "    addi ra, ra, 1",
                  f"    sw ra, {C['LATE']}(s0)", "    li a0, 0", f"    j {cb}", f"{clamp}high:", f"    li a0, {SLED}", f"    j {cb}"]
        event(S - 3, [f"lw {cur[g]}, {4 * (S - 1)}({cur[g]})" for g in range(3)])
        event(S - 2, [f"lw ra, {C['NEXT']}(s0)", "nop", "sub a0, ra, a0"] + [x.strip() for x in prefetch(nxt, 0)], 8)

        emit(f"\n    .align 4\n.L{name}_{path}_sled:")
        emit("\n".join(nops(SLED)))
        emit(f".L{name}_{path}_end:")
        for j in range(S):
            key = f"{path}_{j}"
            emit(f"/* S={S} {path} sample {j} */")
            emit("\n".join(store(j)))
            if j == S - 2:
                # Keep last three terms in t3..t5 while t0 accumulates the preloaded first terms.
                body = [f"    lw t{g + 3}, {4 * (S - 1)}({cur[g + 3]})" for g in range(3)] + [
                    f"    add t0, {cur[0]}, {cur[1]}", f"    add t0, t0, {cur[2]}",
                    "    add t0, t0, t3", "    add t0, t0, t4", "    add t0, t0, t5", "    xor t6, t0, a5"]
                cost = 9
            else:
                ptr = nxt if j == S - 1 else cur
                offset = 0 if j == S - 1 else 4 * (j + 1)
                body = kernel(ptr, offset, j in heavy or j == S - 1)
                cost = 7 if j in heavy or j == S - 1 else 12
            if j == S - 1:
                emit(f"    lw ra, {C['COUNT']}(s0)")
            emit("\n".join(body))
            code, ecost = e.get(j, ([], 0))
            for instruction in code:
                emit(instruction if instruction.endswith(":") else "    " + instruction)
            if j == S - 1:
                emit(f"    addi a2, a2, {4 * S}" if a.rec != "none" else "    .4byte 0x00000013")
                emit(f"    beqz ra, {exit_label}\n    nop")
                emit("    jalr x0, 0(a0)")
                used[f"{S}_{key}"] = 0
            else:
                padding = cfg.get(key, round(P - (3 + cost + ecost)))
                if padding < 0:
                    raise RuntimeError(f"slot too long: {S} {key} {3 + cost + ecost}")
                emit("\n".join(nops(padding)))
                used[f"{S}_{key}"] = padding
        emit("\n".join(stubs))
    emit(f"{exit_label}:\n    sw a7, {C['QR']}(s0)")
    for i, reg in enumerate(saved):
        emit(f"    lw {reg}, {4 * i}(sp)")
    emit(f"    addi sp, sp, 80\n    li a0, 0\n    ret\n    .size {name}, . - {name}")


emit("/* Generated by tools/gen_lutg_a32.py; do not edit by hand. */\n    .section .iram1.lutg_a32, \"ax\"\n    .option rvc")
for S in ([int(a.only)] if a.only else [32, 64]):
    function(S)
print("\n".join(lines).rstrip())
(here / "lutg_a32_pads_used.json").write_text(json.dumps(used, indent=2) + "\n")
