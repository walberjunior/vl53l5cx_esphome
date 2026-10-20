"""Parser/validator of the empty-environment file (environment.conf).

Pure Python (no ESPHome imports at module level other than config_validation)
so it can be unit tested outside ESPHome. Errors are raised as cv.Invalid with
the file name and line number, which makes ESPHome stop before generating code.

File format (plain text, '#' starts a comment):

    # comment
    #! resolution: 8x8          <- directives (also start with '#')
    #! rotation: 90
    #! max_distance_cm: 400
    0>150/3,210/5                <- zone>reference/delta,reference/delta,...
    5>                           <- explicit "no references" for zone 5

A file without any zone line (empty, or only comments/directives) is valid and
ignored (the environment filter then has nothing to apply).
"""

import re

import esphome.config_validation as cv

ZONES = 64  # 8x8, the only supported resolution in this version
MAX_REFS = 4
MIN_REF_CM = 1
MAX_REF_CM = 400
MAX_DELTA_CM = 100
ROTATIONS = {0: 0, 90: 1, 180: 2, 270: 3}
DIRECTIVES = {"resolution", "rotation", "max_distance_cm", "frames"}

_DIRECTIVE = re.compile(r"^#!\s*([A-Za-z_]+)\s*:\s*(.*?)\s*$")
_ZONE_LINE = re.compile(r"^(\d+)\s*>\s*(.*)$")
_PAIR = re.compile(r"^(\d+)\s*/\s*(\d+)$")

# Flat table layout shared with the C++ side: per zone (as numbered in the file),
# [count, ref0..ref3, delta0..delta3] -> 9 uint16 values.
TABLE_STRIDE = 1 + 2 * MAX_REFS


def parse_environment_text(text, name="environment.conf"):
    """Parse the file content. Returns a dict (see below) or raises cv.Invalid."""

    def err(line_no, message):
        return cv.Invalid(f"{name}:{line_no}: {message}")

    directives = {}
    zones = {}  # zone -> (line_no, [(ref, delta), ...])
    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = _DIRECTIVE.match(line)
            if m:
                key, value = m.group(1).lower(), m.group(2)
                if key not in DIRECTIVES:
                    raise err(
                        line_no,
                        f"unknown directive '{key}' (allowed: {', '.join(sorted(DIRECTIVES))})",
                    )
                if key in directives:
                    raise err(line_no, f"directive '{key}' appears twice")
                directives[key] = (line_no, value)
            continue
        if "#" in line:  # trailing comment
            line = line.split("#", 1)[0].strip()
        m = _ZONE_LINE.match(line)
        if not m:
            raise err(line_no, f"invalid line '{raw.strip()}', expected 'zone>reference/delta,...'")
        zone = int(m.group(1))
        if zone >= ZONES:
            raise err(line_no, f"zone {zone} does not exist (valid zones: 0-{ZONES - 1} for 8x8)")
        if zone in zones:
            raise err(line_no, f"zone {zone} is already defined on line {zones[zone][0]}")
        pairs = []
        body = m.group(2).strip()
        if body:
            for item in body.split(","):
                pm = _PAIR.match(item.strip())
                if not pm:
                    raise err(line_no, f"zone {zone}: invalid reference '{item.strip()}', expected 'distance/delta' in cm")
                ref, delta = int(pm.group(1)), int(pm.group(2))
                if not MIN_REF_CM <= ref <= MAX_REF_CM:
                    raise err(line_no, f"zone {zone}: distance {ref} cm out of range ({MIN_REF_CM}-{MAX_REF_CM} cm)")
                if delta > MAX_DELTA_CM:
                    raise err(line_no, f"zone {zone}: delta {delta} cm out of range (0-{MAX_DELTA_CM} cm)")
                pairs.append((ref, delta))
        if len(pairs) > MAX_REFS:
            raise err(line_no, f"zone {zone} has {len(pairs)} references, at most {MAX_REFS} are allowed")
        for i in range(len(pairs)):
            for j in range(i + 1, len(pairs)):
                (r1, d1), (r2, d2) = pairs[i], pairs[j]
                if r1 == r2:
                    raise err(line_no, f"zone {zone}: duplicate reference {r1} cm")
                if abs(r1 - r2) <= d1 + d2:
                    raise err(
                        line_no,
                        f"zone {zone}: ranges {r1}/{d1} and {r2}/{d2} overlap or touch; merge them into one reference",
                    )
        zones[zone] = (line_no, pairs)

    result = {
        "configured": False,
        "rotation": 0,
        "max_distance_cm": 0,
        "table": [0] * (ZONES * TABLE_STRIDE),
        "zones_with_refs": 0,
    }
    if not zones:
        return result  # empty or all commented: ignored

    first = min(line for line, _ in zones.values())

    def need(key):
        if key not in directives:
            raise cv.Invalid(
                f"{name}:{first}: the file defines zones but has no '#! {key}: ...' line; "
                "copy the '#!' lines printed by the calibration together with the zone lines"
            )
        return directives[key]

    line_no, value = need("resolution")
    if value.lower() != "8x8":
        raise err(line_no, f"resolution '{value}' is not supported; only 8x8 in this version")
    line_no, value = need("rotation")
    try:
        rotation = int(value.replace("°", "").strip())
    except ValueError:
        raise err(line_no, f"invalid rotation '{value}' (use 0, 90, 180 or 270)") from None
    if rotation not in ROTATIONS:
        raise err(line_no, f"invalid rotation '{value}' (use 0, 90, 180 or 270)")
    if "max_distance_cm" in directives:
        line_no, value = directives["max_distance_cm"]
        try:
            max_cm = int(value)
        except ValueError:
            raise err(line_no, f"invalid max_distance_cm '{value}'") from None
        if not 1 <= max_cm <= MAX_REF_CM:
            raise err(line_no, f"max_distance_cm {max_cm} out of range (1-{MAX_REF_CM})")
        result["max_distance_cm"] = max_cm

    table = result["table"]
    refs_zones = 0
    for zone, (_, pairs) in zones.items():
        base = zone * TABLE_STRIDE
        table[base] = len(pairs)
        for i, (ref, delta) in enumerate(pairs):
            table[base + 1 + i] = ref
            table[base + 1 + MAX_REFS + i] = delta
        refs_zones += 1 if pairs else 0
    result["rotation"] = ROTATIONS[rotation]
    result["zones_with_refs"] = refs_zones
    result["configured"] = refs_zones > 0
    return result


def load_environment_file(path):
    """Read and validate the file; raises cv.Invalid if it is missing or invalid."""
    try:
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
    except FileNotFoundError:
        raise cv.Invalid(
            f"environment file '{path}' not found. The file is mandatory: create it "
            "(it may be empty or contain only comments)"
        ) from None
    except OSError as exc:
        raise cv.Invalid(f"cannot read environment file '{path}': {exc}") from None
    return parse_environment_text(text, name=getattr(path, "name", str(path)))
