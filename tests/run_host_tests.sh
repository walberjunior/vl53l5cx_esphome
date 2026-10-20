#!/usr/bin/env bash
# Host-side tests: logic/unit tests + ULD build/size report for every targets_per_zone.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
COMP="$HERE/../components/vl53l5cx"
OUT="$(mktemp -d)"
for N in 1 2 3 4; do
  FLAGS="-DVL53L5CX_NB_TARGET_PER_ZONE=${N}U"
  gcc -std=gnu11 -Wall -Wextra -Wno-unused-parameter $FLAGS -c "$COMP/vl53l5cx_api.c" -o "$OUT/api_$N.o"
  gcc -std=gnu11 -Wall -Wextra $FLAGS -c "$COMP/vl53l5cx_plugin_motion_indicator.c" -o "$OUT/motion_$N.o"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" \
      "$HERE/sizeof_report.cpp" "$HERE/host_platform_stub.cpp" "$OUT/api_$N.o" "$OUT/motion_$N.o" -o "$OUT/sizeof_$N"
  "$OUT/sizeof_$N"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" "$HERE/test_logic.cpp" -o "$OUT/test_logic_$N"
  "$OUT/test_logic_$N"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" "$HERE/test_environment.cpp" -o "$OUT/test_env_$N"
  "$OUT/test_env_$N"
done
# Motion indicator compiled out (the component default): ULD, plugin and logic must still build and pass.
for N in 1 4; do
  FLAGS="-DVL53L5CX_NB_TARGET_PER_ZONE=${N}U -DVL53L5CX_DISABLE_MOTION_INDICATOR"
  gcc -std=gnu11 -Wall -Wextra -Wno-unused-parameter $FLAGS -I"$COMP" -c "$COMP/vl53l5cx_api.c" -o "$OUT/api_m$N.o"
  gcc -std=gnu11 -Wall -Wextra $FLAGS -I"$COMP" -c "$COMP/vl53l5cx_plugin_motion_indicator.c" -o "$OUT/motion_m$N.o"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" "$HERE/sizeof_report.cpp" "$HERE/host_platform_stub.cpp" "$OUT/api_m$N.o" "$OUT/motion_m$N.o" -o "$OUT/sizeof_m$N"
  echo -n "[motion disabled] "; "$OUT/sizeof_m$N"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" "$HERE/test_logic.cpp" -o "$OUT/test_logic_m$N"
  echo -n "[motion disabled] "; "$OUT/test_logic_m$N"
  g++ -std=gnu++17 -Wall -Wextra $FLAGS -I"$COMP" "$HERE/test_environment.cpp" -o "$OUT/test_env_m$N"
  echo -n "[motion disabled] "; "$OUT/test_env_m$N"
done
