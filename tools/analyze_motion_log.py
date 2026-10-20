#!/usr/bin/env python3
"""Summarise logs captured with the `Motion Indicator Debug Logging` switch.

Usage: tools/analyze_motion_log.py capture1.txt capture2.txt ...

Per file it prints the configuration, how often the ULD motion block really
changes ("updates"), the lengths of the sawtooth runs between resets, the share
of updates in which the mean value rises, and how far global_indicator_1/2
(divided by 65536) are from the mean of motion[0..15] / motion[16..31].
Only the Python standard library is needed.
"""
import re
import sys

HDR = re.compile(
    r"seq=(\d+) res=(\S+) mode=(\S+) freq=(\d+)Hz int=(\d+)ms"
    r"(?: sharp=(\d+)% order=(\S+) vhv=(\d+)s)? range=(\S+)mm \| "
    r"status=(\d+) aggregates=(\d+) detected=(\d+) global1=(\d+) global2=(\d+)"
)
ARR = re.compile(r"seq=(\d+) motion\[(\d+)\.\.(\d+)\].*: ([\d,]+)")


def load(path):
    hdr, arr = {}, {}
    with open(path, errors="replace") as fh:
        for line in fh:
            m = HDR.search(line)
            if m:
                hdr[int(m[1])] = m
            m = ARR.search(line)
            if m:
                arr.setdefault(int(m[1]), {})[int(m[2])] = [int(x) for x in m[4].split(",")]
    return hdr, arr


def analyse(path):
    hdr, arr = load(path)
    seqs = [s for s in sorted(hdr) if s in arr and 0 in arr[s] and 16 in arr[s]]
    if not seqs:
        print(f"{path}: no complete frames found")
        return
    h = hdr[seqs[0]]
    extra = f" sharp={h[6]}% order={h[7]} vhv={h[8]}s" if h[6] else ""
    print(f"\n{path}: {h[2]} {h[3]} {h[4]}Hz int={h[5]}ms{extra} range={h[9]}mm, {len(seqs)} frames")

    updates, seen = [], None
    for s in seqs:
        key = tuple(arr[s][0] + arr[s][16])
        if key != seen:
            seen = key
            lo, hi = arr[s][0], arr[s][16]
            updates.append((s, sum(lo) / 16, min(lo), max(lo), sum(hi) / 16,
                            int(hdr[s][13]) / 65536, int(hdr[s][14]) / 65536))
    print(f"  distinct motion blocks: {len(updates)} (one per {len(seqs) / len(updates):.1f} frames)")
    means = [u[1] for u in updates]
    print("  mean of motion[0..15] per update:", [round(x) for x in means[:24]], "..." if len(means) > 24 else "")

    runs, cur, rises, steps = [], 1, 0, 0
    for a, b in zip(means, means[1:]):
        steps += 1
        if b < 0.6 * a:  # reset of the sawtooth
            runs.append(cur)
            cur = 1
        else:
            cur += 1
            rises += b - a > 100
    runs.append(cur)
    print(f"  run lengths between resets (updates): {runs}; rising updates: {rises}/{steps}")

    r1 = [u[5] / u[1] for u in updates if u[1]]
    r2 = [u[6] / u[4] for u in updates if u[4]]
    out = lambda r: sum(1 for x in r if x < 0.8 or x > 1.25) / len(r)
    print(f"  global1/mean(m0-15): outside 0.8-1.25 in {out(r1):.0%} of updates (min {min(r1):.2f}, max {max(r1):.2f})")
    print(f"  global2/mean(m16-31): outside 0.8-1.25 in {out(r2):.0%} of updates (min {min(r2):.2f}, max {max(r2):.2f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        analyse(p)
