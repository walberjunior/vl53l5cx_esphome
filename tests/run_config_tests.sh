#!/usr/bin/env bash
# YAML schema tests using the real ESPHome (needs `pip install esphome==2026.9.0`).
# Usage: tests/run_config_tests.sh [path/to/esphome]
set -uo pipefail
ESPHOME="${1:-esphome}"
cd "$(dirname "$0")/.."
printf 'wifi_ssid: "x"\nwifi_password: "xxxxxxxxxxx"\n' > secrets.yaml
TMPD="$(mktemp -d)"
trap 'rm -rf secrets.yaml _t.yaml "$TMPD"' EXIT
fail=0
variant() {  # variant <description> <sed-expression> <expected message fragment | OK>
  sed -e "$2" example.yaml > _t.yaml
  out="$($ESPHOME config _t.yaml 2>&1)"
  if [[ "$3" == "OK" ]]; then
    grep -q "Configuration is valid" <<<"$out" && echo "PASS  $1" || { echo "FAIL  $1"; fail=1; }
  else
    grep -q "$3" <<<"$out" && echo "PASS  $1" || { echo "FAIL  $1 (expected: $3)"; fail=1; }
  fi
}
UNC='s/^\( *\)#M# /\1/'   # enables the motion indicator lines of example.yaml
variant "example.yaml is valid (motion indicator disabled)"  's/^$//'                                         OK
variant "motion indicator enabled with all its entities"      "$UNC"                                           OK
variant "targets_per_zone 1 with 2 target entries"            's/targets_per_zone: 4/targets_per_zone: 1/'     "lists 2 targets"
variant "targets_per_zone 5 rejected"                         's/targets_per_zone: 4/targets_per_zone: 5/'     "at most 4"
variant "motion span > 1500 mm"                               "$UNC; s/motion_distance_max: 1500/motion_distance_max: 2500/" "cannot exceed 1500"
variant "motion max <= min"                                   "$UNC; s/motion_distance_max: 1500/motion_distance_max: 400/"  "must be greater"
variant "motion entities need the hub option"                 "$UNC; s/^  motion_indicator: true\$/  motion_indicator: false/" "requires 'motion_indicator: true'"
variant "motion switches need the hub option"                 '/#M# *motion_indicator_.*logging:/{s/#M# //;n;s/#M# //}' "motion indicator switches require"
variant "i2c_chunk_size too small"                            's/i2c_chunk_size: 64/i2c_chunk_size: 8/'        "at least 16"
printf '#! resolution: 8x8\n#! rotation: 0\n0>150/3,210/5\n1>95/4\n' > "$TMPD/good.conf"
printf '#! resolution: 8x8\n#! rotation: 0\n0>150/3\n99>10/1\n' > "$TMPD/badzone.conf"
printf '#! resolution: 8x8\n#! rotation: 0\n0>50/1,100/1,150/1,200/1,250/1\n' > "$TMPD/five.conf"
printf '0>150/3\n' > "$TMPD/nodirective.conf"
printf '# only comments\n' > "$TMPD/comments.conf"
: > "$TMPD/empty.conf"
ENVSED() { echo "s#^  environment_file: .*#  environment_file: $TMPD/$1#"; }
variant "environment file with valid references"             "$(ENVSED good.conf)"        OK
variant "environment file that is empty is ignored"          "$(ENVSED empty.conf)"       OK
variant "environment file with only comments is ignored"     "$(ENVSED comments.conf)"    OK
variant "environment file with a nonexistent zone"           "$(ENVSED badzone.conf)"     "environment.conf\|badzone.conf:4: zone 99"
variant "environment file with more than 4 references"       "$(ENVSED five.conf)"        "at most 4"
variant "environment file without the required directives"   "$(ENVSED nodirective.conf)" "no '#! resolution"
variant "environment file that does not exist"               "$(ENVSED missing.conf)"     "mandatory"
variant "old text sensor key gives a rename hint"            's/^    ambient_per_spad_raw:/    ambient_per_spad:/'  "was renamed to 'ambient_per_spad_raw'"
variant "old logging switch key gives a rename hint"         's/^    target_status_raw_logging:/    target_status_logging:/' "was renamed to 'target_status_raw_logging'"
variant "old sensor key gives a rename hint"                 's/^    temperature_raw:/    temperature:/'  "was renamed to 'temperature_raw'"
exit $fail
