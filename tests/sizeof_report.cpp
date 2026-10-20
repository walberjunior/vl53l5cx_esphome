// Prints the RAM cost of the ULD structures for the compiled targets-per-zone.
#include <cstdio>

#include "vl53l5cx_uld.h"

int main() {
  std::printf("targets_per_zone=%u  sizeof(Configuration)=%zu  sizeof(ResultsData)=%zu  "
              "sizeof(MotionConfiguration)=%zu  MAX_RESULTS_SIZE=%u  TEMP_BUFFER=%u\n",
              (unsigned) VL53L5CX_NB_TARGET_PER_ZONE, sizeof(VL53L5CX_Configuration), sizeof(VL53L5CX_ResultsData),
              sizeof(VL53L5CX_Motion_Configuration), (unsigned) VL53L5CX_MAX_RESULTS_SIZE,
              (unsigned) VL53L5CX_TEMPORARY_BUFFER_SIZE);
  return 0;
}
