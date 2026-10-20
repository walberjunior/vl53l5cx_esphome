#!/usr/bin/env python3
"""Merge several "Calibrate Empty Environment" reports into one environment.conf.

Usage:
    tools/merge_calibrations.py [--min-delta 5] [--out environment.conf] log1.txt log2.txt ...

Each log may contain ESPHome timestamps/levels; only the "#!" lines and the
"zone>distance/delta,..." lines are read. All runs must use the same
resolution and rotation. Per zone the bands of all runs are united: bands that
overlap or touch become one band, every band is widened to at least
+/- min-delta, and at most 4 bands are kept (the closest pair is merged until
it fits; such zones get a comment). The result is checked with the same parser
that validates the file at compile time. Pure Python (standard library).
"""
import argparse
import re
import sys

MAX_REFS = 4
ZONE = re.compile(r"\]:\s*(\d+)>([\d/,\s]*)$|^\s*(\d+)>([\d/,\s]*)$")
DIRECTIVE = re.compile(r"#!\s*([A-Za-z_]+)\s*:\s*(\S+)")


def parse_log(text):
    """-> (directives dict, {zone: [(ref, delta), ...]})"""
    directives, zones = {}, {}
    for line in text.splitlines():
        m = DIRECTIVE.search(line)
        if m:
            directives[m.group(1).lower()] = m.group(2)
            continue
        m = ZONE.search(line.rstrip())
        if m:
            zone = int(m.group(1) or m.group(3))
            body = (m.group(2) if m.group(1) is not None else m.group(4)).strip()
            zones[zone] = [tuple(int(x) for x in p.split("/")) for p in body.split(",") if p.strip()]
    return directives, zones


def to_band(ref, delta):
    return [ref - delta, ref + delta]


def union_bands(bands):
    """Merge bands that overlap or touch (|ref1-ref2| <= d1+d2 once converted back)."""
    bands = sorted(bands)
    out = []
    for lo, hi in bands:
        if out and lo <= out[-1][1] + 1:  # overlapping or adjacent
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return out


def band_to_ref(lo, hi, min_delta):
    ref = (lo + hi) // 2
    delta = max(-(-(hi - lo) // 2), min_delta)
    return max(1, min(400, ref)), min(100, delta)


def separate(refs):
    """Merge references until none of them overlaps or touches another (parser rule)."""
    changed = True
    while changed:
        changed = False
        refs.sort()
        for i in range(len(refs) - 1):
            (r1, d1), (r2, d2) = refs[i], refs[i + 1]
            if r2 - r1 <= d1 + d2:
                lo, hi = min(r1 - d1, r2 - d2), max(r1 + d1, r2 + d2)
                refs[i : i + 2] = [band_to_ref(lo, hi, 0)]
                changed = True
                break
    return refs


def merge(runs, min_delta=5):
    """runs: list of (directives, zones). Returns (directives, {zone: refs}, notes)."""
    first = runs[0][0]
    for key in ("resolution", "rotation"):
        for d, _ in runs:
            if d.get(key) != first.get(key):
                raise ValueError(f"runs disagree on '{key}': {first.get(key)} vs {d.get(key)}")
    maxes = {d.get("max_distance_cm") for d, _ in runs}
    notes = []
    if len(maxes) > 1:
        notes.append(f"runs used different max_distance_cm values: {sorted(str(m) for m in maxes)}")
    merged, zone_notes = {}, {}
    all_zones = sorted({z for _, zs in runs for z in zs})
    for z in all_zones:
        bands = [to_band(r, max(d, min_delta)) for _, zs in runs for r, d in zs.get(z, [])]
        united = union_bands(bands)
        refs = separate([band_to_ref(lo, hi, min_delta) for lo, hi in united])
        dropped = 0
        while len(refs) > MAX_REFS:  # merge the closest pair until it fits
            refs.sort()
            gaps = [(refs[i + 1][0] - refs[i][0], i) for i in range(len(refs) - 1)]
            _, i = min(gaps)
            (r1, d1), (r2, d2) = refs[i], refs[i + 1]
            refs[i : i + 2] = [band_to_ref(min(r1 - d1, r2 - d2), max(r1 + d1, r2 + d2), 0)]
            dropped += 1
        if dropped:
            zone_notes[z] = f"{dropped} band(s) merged to respect the limit of {MAX_REFS}; review"
        merged[z] = refs
    directives = {k: first[k] for k in ("resolution", "rotation") if k in first}
    if len(maxes) == 1 and None not in maxes:
        directives["max_distance_cm"] = next(iter(maxes))
    return directives, merged, notes, zone_notes


def render(directives, merged, notes, zone_notes, sources):
    lines = [f"# Merged from {len(sources)} calibration(s): " + ", ".join(sources)]
    lines += [f"# WARNING: {n}" for n in notes]
    lines += [f"#! {k}: {v}" for k, v in directives.items()]
    for z in sorted(merged):
        if z in zone_notes:
            lines.append(f"# zone {z}: {zone_notes[z]}")
        lines.append(f"{z}>" + ",".join(f"{r}/{d}" for r, d in merged[z]))
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--min-delta", type=int, default=5)
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    runs = []
    for path in args.logs:
        with open(path, errors="replace") as fh:
            d, z = parse_log(fh.read())
        if not z:
            sys.exit(f"{path}: no zone lines found")
        runs.append((d, z))
    directives, merged, notes, zone_notes = merge(runs, args.min_delta)
    text = render(directives, merged, notes, zone_notes, args.logs)
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
