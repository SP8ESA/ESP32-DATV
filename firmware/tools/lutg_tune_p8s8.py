#!/usr/bin/env python3
"""Tunes the nop pads of the 1 MBd 8PSK loop (gen_lutg_p8s8.py) with its timing build (gen_lutg_p8s8.py --rec timing, #define LUTG_REC 1 in main.c).

  python3 tools/lutg_tune_p8s8.py [--junk 0] [--rounds 30]      (from firmware/, IDF exported, ESP on the USB port)

A pass = 8 symbols = 64 slots, a sync (D, E) every two symbols. The sync puts the next pair back on the absolute schedule, so the USB phase
at the start of every pair is the same whatever happened before: the 4 pairs of the 4 code copies are tuned independently and in parallel.
In every (copy, pair) the earliest slot whose store-to-store spacing is not 20 gets its pad corrected each round (a USB access completes on an
edge of the 48 MHz clock: a slot that is off moves the phase of the later ones). A slot that can not be 20 (the spacing jumps from 19 to 21 when one nop is added) is
kept at 19: a cycle too short costs nothing at the DAC (and gives the sync room), a cycle too long takes room from it. The E slots of the odd symbols (slots 15, 31, 47, 63)
are the sync: their spacing is not tuned. Pads go to tools/lutg_p8s8_pads.json; the start values are the generator's estimates.
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
LINE = re.compile(r"\bT(\d+) ([NM]):((?: \d+:-?\d+)*) \| E (-?\d+)")
PADFILE = os.path.join(HERE, "lutg_p8s8_pads.json")
USEDFILE = os.path.join(HERE, "lutg_p8s8_pads_used.json")


def run(cmd, **kw):
    return subprocess.run(cmd, cwd=FW, capture_output=True, text=True, **kw)


def build_flash(port):
    r = run(["python3", "tools/gen_lutg_p8s8.py", "--rec", "timing"])
    if r.returncode:
        sys.exit(r.stderr)
    open(os.path.join(FW, "main", "lutg_p8s8.S"), "w").write(r.stdout)
    env = "unset PYTHONPATH; . ~/esp/esp-idf/export.sh >/dev/null 2>&1; idf.py build && idf.py -p %s flash" % port
    b = subprocess.run(["bash", "-c", env], cwd=FW, capture_output=True, text=True)
    if b.returncode:
        sys.exit(b.stdout[-2000:] + b.stderr[-2000:])
    return r.stderr


def measure(junk, port, secs):
    r = subprocess.run(["python3", "lutg_record.py", f"PSK8T 2370.000 1000000 8 300 {secs} 0 0", "--junk", str(junk), "--port", port], cwd=HOST,
                       capture_output=True, text=True)
    return r.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--junk", type=int, default=0)
    ap.add_argument("--rounds", type=int, default=30)
    ap.add_argument("--secs", type=int, default=10, help="start phase in cycles, mod 10 (the 'seconds' argument of the recording build; 10 = phase 0, the production phase LUTG_P8S8_PHASE)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    a = ap.parse_args()
    allp = json.load(open(PADFILE)) if os.path.exists(PADFILE) else {}
    pads = allp.get("20", {})
    run(["python3", "tools/gen_lutg_p8s8.py"])          # the estimates of the generator (EFFECTIVE) for every slot that has no saved pad yet
    used = json.load(open(USEDFILE))["20"]
    pads = {**{k: v for k, v in used.items() if not k.endswith("_cost")}, **pads}
    pads = {k: v for k, v in pads.items() if k == "KD" or k[0] in "NM"}
    seen, accept = {}, {}
    for rnd in range(a.rounds):
        allp["20"] = pads
        json.dump(allp, open(PADFILE, "w"), indent=0, sort_keys=True)
        warn = build_flash(a.port)
        text = measure(a.junk, a.port, a.secs)
        rows = [m for m in map(LINE.search, text.splitlines()) if m]
        if not rows:
            sys.exit("no timing lines:\n" + text[-1500:])
        lm = re.search(r"SLED (\d+) late (\d+)", text)
        if lm and int(lm.group(2)) > 0:
            sys.exit(f"the sync was late {lm.group(2)} times: the timing rows are meaningless (a slot is longer than its pad allows, or KD is too big); "
                     "fix that first (tools/lutg_p8s8_pads.json is as of the last round)")
        off, sync = {}, {}
        for m in rows:
            k, c = int(m.group(1)), m.group(2)
            if k >= 56 or k < 8:                      # pass 0 is the start-up (its sync is the first one), the 7th pass is cut short by the exit of the loop
                continue
            kk = k % 8
            dev = {int(t.split(":")[0]): int(t.split(":")[1]) for t in m.group(3).split()}
            if any(not 0 < v < 80 for v in dev.values()) or not -1 <= int(m.group(4)) < 80:
                sys.exit("implausible timing row (the build is probably broken): " + m.group(0))
            for j in range(7):
                off.setdefault((c, 8 * kk + j), set()).add(dev.get(j, 20))
            e = int(m.group(4))
            if kk % 2 == 1:
                sync.setdefault(c, set()).add(e)
            else:
                off.setdefault((c, 8 * kk + 7), set()).add(e)
        bad = sum(1 for v in off.values() if v != {20})
        print(f"round {rnd}: {bad} slots with a spacing other than 20 (copy x slot); sync E spacings {dict((c, sorted(v)) for c, v in sync.items())}" + (f"\n{warn}" if warn else ""))
        done_all = True
        for c in "NM":
            for m in range(4):
                slots = [n for n in range(16 * m, 16 * m + 16) if n % 16 != 15]
                for n in slots:
                    k = f"{c}{n}"
                    tgt = accept.get(k, 20)
                    obs = off.get((c, n), {20})
                    if obs == {tgt} or obs == {20}:
                        accept.pop(k, None) if obs == {20} else None
                        continue
                    d = max(obs, key=lambda x: abs(x - tgt))
                    post, pre = pads.get(k, 0), pads.get(k + "p", 0)
                    seen.setdefault(k, {})[(pre, post)] = d
                    tried = seen[k]
                    # a slot that jumps from 19 to 21 when one nop is added can not be 20: keep the 19 side (it gives the sync more room, a 21 takes it away)
                    low = [pp for pp, dd in tried.items() if dd == 19]
                    high = [pp for pp, dd in tried.items() if dd >= 21 and any(sum(pp) == sum(q) + 1 for q in low)]
                    if low and high:
                        q = [q for q in low if any(sum(pp) == sum(q) + 1 for pp in high)][0]
                        if (pre, post) == q:
                            accept[k] = 19
                            print(f"  {k}: no pad gives 20 (19 with {sum(q)} nops, 21 with one more): keeping 19")
                            continue
                        pads[k], pads[k + "p"] = q[1], q[0]
                        accept[k] = 19
                        done_all = False
                        print(f"  {k}: no pad gives 20: back to the 19 side (pre {q[0]} pad {q[1]})")
                        break
                    done_all = False
                    want = pre + post + tgt - d                       # the total number of nops that would be right if one nop were one cycle
                    cands = sorted((abs(post_ + pre_ - want), pre_, post_) for pre_ in range(0, 6) for post_ in range(0, 10)
                                   if (pre_, post_) not in tried and abs(post_ + pre_ - want) <= 1)
                    if (pre, want - pre) not in tried and want - pre >= 0:
                        pre_n, post_n = pre, want - pre
                    elif cands:
                        _, pre_n, post_n = cands[0]
                    else:
                        print(f"  {k}: no untried (pre, post) left, seen {tried}" + ("   (the slot costs more than the period: move events)" if pre + post == 0 else ""))
                        break
                    print(f"  {k}: spacing {sorted(obs)} (target {tgt}) pre {pre} pad {post} -> pre {pre_n} pad {post_n}" + ("   (the slot costs more than the period)" if want < 0 else ""))
                    pads[k], pads[k + "p"] = post_n, pre_n
                    break
        if done_all:
            print("all slots on target")
            return


if __name__ == "__main__":
    main()
