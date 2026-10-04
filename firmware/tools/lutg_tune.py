#!/usr/bin/env python3
"""Tunes the nop pads of the generated lutg loops with the timing build (gen_lutg.py --rec timing, #define LUTG_REC 1 in main.c).

  python3 tools/lutg_tune.py --psk8 [--sps 24 --baud 333000] [--junk 0] [--rounds 8]      (from firmware/, IDF exported, ESP on the USB port)
  python3 tools/lutg_tune.py --a16 [--sps 24 --baud 333333]                                  (the 16APSK loop, gen_lutg_a16.py, tools/lutg_a16_pads.json)

One round = generate lutg*.S with the current pads, build, flash, run PSK8T / QPSKT, read "T<k> <copy>: slot:spacing ... | E x" (only the
slots whose spacing differs from the period are listed), and move the pad of the slot by (period - spacing). The pads of the unrolled slots
(<copy><j>), the loop (<copy>L) and slot D (<copy>D) are written to tools/lutg_psk8_pads.json (lutg_pads.json); the start values are the
estimates the generator wrote to *_pads_used.json. Stops when no slot is off. Never edits main.c: set LUTG_REC 1 there yourself.
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
HOST = os.path.join(os.path.dirname(FW), "host")
LINE = re.compile(r"\bT(\d+) ([NABC]):((?: \d+:-?\d+)*) \| E (-?\d+)")


def run(cmd, **kw):
    return subprocess.run(cmd, cwd=FW, capture_output=True, text=True, **kw)


def gen(psk8, a16=False):
    out = os.path.join(FW, "main", "lutg_a16.S" if a16 else "lutg_psk8.S" if psk8 else "lutg.S")
    r = run(["python3", "tools/gen_lutg_a16.py" if a16 else "tools/gen_lutg.py", "--rec", "timing"] + (["--psk8"] if psk8 and not a16 else []))
    if r.returncode:
        sys.exit(r.stderr)
    open(out, "w").write(r.stdout)
    return r.stderr


def flash(port):
    env = "unset PYTHONPATH; . ~/esp/esp-idf/export.sh >/dev/null 2>&1; idf.py build && idf.py -p %s flash" % port
    r = subprocess.run(["bash", "-c", env], cwd=FW, capture_output=True, text=True)
    if r.returncode:
        sys.exit(r.stdout[-2000:] + r.stderr[-2000:])


def measure(cmd, junk, port):
    r = subprocess.run(["python3", "lutg_record.py", cmd, "--junk", str(junk), "--port", port], cwd=HOST, capture_output=True, text=True)
    return r.stdout


def key(c, j, sps):
    if j <= 12:
        return f"{c}{j}"
    return f"{c}L" if j <= sps - 3 else f"{c}D"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--psk8", action="store_true")
    ap.add_argument("--a16", action="store_true")
    ap.add_argument("--sps", type=int, default=24)
    ap.add_argument("--baud", type=int, default=333000)
    ap.add_argument("--junk", type=int, default=0)
    ap.add_argument("--rounds", type=int, default=8)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--period", type=int, default=20)
    a = ap.parse_args()
    padfile = os.path.join(HERE, "lutg_a16_pads.json" if a.a16 else "lutg_psk8_pads.json" if a.psk8 else "lutg_pads.json")
    usedfile = os.path.join(HERE, "lutg_a16_pads_used.json" if a.a16 else "lutg_psk8_pads_used.json" if a.psk8 else "lutg_pads_used.json")
    P = str(a.period)
    allp = json.load(open(padfile)) if os.path.exists(padfile) else {}
    pads = allp.get(P) or {k: v for k, v in json.load(open(usedfile))[P].items() if not k.endswith("_cost")}
    cmd = f"A16T 2370.000 {a.baud} {a.sps} 300 5 0 0 0 0 10000 0 315" if a.a16 else f"{'PSK8T' if a.psk8 else 'QPSKT'} 2370.000 {a.baud} {a.sps} 300 5 0 0"
    seen, targets, accept = {}, {}, {}
    for rnd in range(a.rounds):
        allp[P] = pads
        json.dump(allp, open(padfile, "w"), indent=0, sort_keys=True)
        warn = gen(a.psk8, a.a16)
        flash(a.port)
        text = measure(cmd, a.junk, a.port)
        off, es = {}, set()
        rows = [m for m in map(LINE.search, text.splitlines()) if m]
        for m in rows[:-1]:                          # the last symbol is cut short by the exit of the loop
            for tok in m.group(3).split():
                j, d = (int(x) for x in tok.split(":"))
                off.setdefault(key(m.group(2), j, a.sps), set()).add(d)
            es.add((m.group(2), int(m.group(4))))
        print(f"round {rnd}: {sum(len(v) for v in off.values())} deviating (slot, spacing) pairs; E: {sorted(es)}" + (f"\n{warn}" if warn else ""))
        if not rows:
            sys.exit("no timing lines:\n" + text[-1500:])
        # USB accesses are quantised to the 48 MHz clock, so a slot that is off moves the phase of every later slot: fix only the earliest
        # deviating slot of each copy, then measure again. A slot that can not be 20 (its spacing jumps from below to above 20 when one
        # nop is added) is accepted at the nearest value and the next slot gets the opposite target, so the stream is back on the grid after it.
        order_keys = lambda c: [f"{c}{j}" for j in range(13)] + [f"{c}L", f"{c}D"]
        done_all = True
        for c in "NABC":
            keys = order_keys(c)
            for i, k in enumerate(keys):
                tgt = targets.get(k, a.period)
                obs = off.get(k, {a.period})
                if obs == {tgt}:
                    continue
                done_all = False
                d = sorted(obs)[0]
                post, pre = pads.get(k, 0), pads.get(k + "p", 0)
                seen.setdefault(k, {})[(pre, post)] = d
                tried = seen[k]
                below = {pp: dd for pp, dd in tried.items() if dd < tgt}
                above = {pp: dd for pp, dd in tried.items() if dd > tgt}
                plateau = [(pb, pa) for pb in below for pa in above if sum(pa) == sum(pb) + 1 and above[pa] - below[pb] >= 2 and tgt == a.period]
                if plateau and i + 1 < len(keys):
                    pb, pa = plateau[0]
                    pads[k], pads[k + "p"] = pa[1], pa[0]
                    targets[k] = above[pa]
                    targets[keys[i + 1]] = 2 * a.period - above[pa]
                    accept[k] = above[pa]
                    print(f"  {k}: no pad gives {a.period}: accept {above[pa]} (pre {pa[0]} pad {pa[1]}), next slot {keys[i + 1]} target {targets[keys[i + 1]]}")
                    break
                want = pre + post + tgt - d                       # the total number of nops that would be right if one nop were one cycle
                cands = sorted((abs(post_ + pre_ - want), pre_, post_) for pre_ in range(0, 6) for post_ in range(0, 10)
                               if (pre_, post_) not in tried and abs(post_ + pre_ - want) <= 1)
                if (pre, want - pre) not in tried and want - pre >= 0:
                    pre_n, post_n = pre, want - pre
                elif cands:
                    _, pre_n, post_n = cands[0]
                else:
                    print(f"  {k}: no untried (pre, post) left, seen {tried}")
                    break
                print(f"  {k}: spacing {sorted(obs)} (target {tgt}) pre {pre} pad {post} -> pre {pre_n} pad {post_n}" + ("   (the slot costs more than the period)" if want < 0 else ""))
                pads[k], pads[k + "p"] = post_n, pre_n
                break
        if done_all:
            print("all slots on target")
            return

if __name__ == "__main__":
    main()
