// Host tests for the processed distance, the environment filter, the processed
// target count and the calibration algorithm.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "vl53l5cx_cells.h"

using namespace esphome::vl53l5cx;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failed++;                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)
#define CHECK_STR(actual, expected)                                                                        \
  do {                                                                                                     \
    g_checks++;                                                                                            \
    if (std::string(actual) != std::string(expected)) {                                                    \
      g_failed++;                                                                                          \
      std::printf("FAIL %s:%d:\n  got      %s\n  expected %s\n", __FILE__, __LINE__, std::string(actual).c_str(), \
                  std::string(expected).c_str());                                                          \
    }                                                                                                      \
  } while (0)

static ProcessingContext ctx(FilterProfile f, uint16_t max_cm, const EnvRuntime *env = nullptr) {
  ProcessingContext c;
  c.filter = f;
  c.max_distance_cm = max_cm;
  c.env = env;
  return c;
}

static void set_target(VL53L5CX_ResultsData &r, uint8_t zone, uint8_t t, int mm, uint8_t status) {
  r.distance_mm[zone * TARGETS_PER_ZONE + t] = static_cast<int16_t>(mm);
  r.target_status[zone * TARGETS_PER_ZONE + t] = status;
}

static void test_limit_table_from_the_specification() {
  // Number = 200 cm: NaN -> 200, 250 -> 200, 210 -> 200, 200 -> 200, 180 -> 180, 150 -> 150
  CHECK(limit_distance_cm(false, 0, 200) == 200);
  CHECK(limit_distance_cm(true, 250, 200) == 200);
  CHECK(limit_distance_cm(true, 210, 200) == 200);
  CHECK(limit_distance_cm(true, 200, 200) == 200);
  CHECK(limit_distance_cm(true, 180, 200) == 180);
  CHECK(limit_distance_cm(true, 150, 200) == 150);
  CHECK(limit_distance_cm(true, 150, 400) == 150);
}

static void test_processed_vs_raw() {
  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  // zone 0: valid target at 150 cm; zone 1: valid at 250 cm; zone 2: status 12 at 120 cm; zone 3: no target
  r.nb_target_detected[0] = 1; set_target(r, 0, 0, 1500, 5);
  r.nb_target_detected[1] = 1; set_target(r, 1, 0, 2500, 5);
  r.nb_target_detected[2] = 1; set_target(r, 2, 0, 1200, 12);
  r.nb_target_detected[3] = 0; set_target(r, 3, 0, 777, 255);
  const ProcessingContext c = ctx(FilterProfile::RANGE_VALID, 200);
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, c).value == 150);
  CHECK(get_cell(MatrixId::DISTANCE, r, 1, 0, c).value == 200);  // above the limit
  CHECK(get_cell(MatrixId::DISTANCE, r, 2, 0, c).value == 200);  // rejected by the status filter
  CHECK(get_cell(MatrixId::DISTANCE, r, 3, 0, c).value == 200);  // no target -> limit
  for (uint8_t z = 0; z < 4; z++)
    CHECK(get_cell(MatrixId::DISTANCE, r, z, 0, c).valid);       // processed distance is never nan
  // RAW is untouched by limit and filter
  CHECK(get_cell(MatrixId::DISTANCE_RAW, r, 1, 0, c).value == 250);
  CHECK(get_cell(MatrixId::DISTANCE_RAW, r, 2, 0, c).value == 120);
  CHECK(!get_cell(MatrixId::DISTANCE_RAW, r, 3, 0, c).valid);
  CHECK(get_cell(MatrixId::TARGET_STATUS, r, 2, 0, c).value == 12);
  CHECK(r.distance_mm[1 * TARGETS_PER_ZONE] == 2500);  // the ULD frame itself is unchanged
}

static void test_environment_bands() {
  // zone 0 of the file at rotation 0: 150/3 and 210/5 -> 147..153 and 205..215, independent
  std::vector<uint16_t> flat(ENV_ZONES * ENV_TABLE_STRIDE, 0);
  flat[0] = 2; flat[1] = 150; flat[2] = 210; flat[1 + ENV_MAX_REFS] = 3; flat[2 + ENV_MAX_REFS] = 5;
  EnvRuntime env;
  env_build_runtime(env, flat.data(), Rotation::DEG_0);
  CHECK(env.any);
  for (int cm : {147, 150, 153, 205, 210, 215})
    CHECK(env.match(0, cm));
  for (int cm : {146, 154, 180, 204, 216, 0, 400})
    CHECK(!env.match(0, cm));  // no continuous interval between the two bands
  CHECK(!env.match(1, 150));   // other zones are not affected

  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  r.nb_target_detected[0] = 1;
  set_target(r, 0, 0, 1520, 5);  // 152 cm: environment
  const ProcessingContext c = ctx(FilterProfile::RANGE_VALID, 400, &env);
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, c).value == 400);                 // environment -> limit
  CHECK(get_cell(MatrixId::DISTANCE_RAW, r, 0, 0, c).value == 152);             // RAW untouched
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 0, 0, c).value == 0); // processed count ignores it
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED_RAW, r, 0, 0, c).value == 1);
  set_target(r, 0, 0, 1800, 5);  // 180 cm: deviation from every band
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, c).value == 180);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 0, 0, c).value == 1);
  // the same frame without the filter keeps the distance
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, ctx(FilterProfile::RANGE_VALID, 400)).value == 180);
}

static void test_environment_follows_the_physical_zone() {
  // The file numbers zones as displayed with rotation 90. Shown index 7 is physical zone 0.
  std::vector<uint16_t> flat(ENV_ZONES * ENV_TABLE_STRIDE, 0);
  flat[7 * ENV_TABLE_STRIDE + 0] = 1;
  flat[7 * ENV_TABLE_STRIDE + 1] = 200;
  flat[7 * ENV_TABLE_STRIDE + 1 + ENV_MAX_REFS] = 4;
  EnvRuntime env;
  env_build_runtime(env, flat.data(), Rotation::DEG_90);
  CHECK(env.zone[0].count == 1 && env.zone[0].ref[0] == 200 && env.zone[0].delta[0] == 4);
  for (uint8_t z = 1; z < ENV_ZONES; z++)
    CHECK(env.zone[z].count == 0);
  // The same file read as rotation 180 attaches it to another physical zone (63).
  EnvRuntime env180;
  env_build_runtime(env180, flat.data(), Rotation::DEG_180);
  RotationMap m;
  m.build(8, Rotation::DEG_180);
  CHECK(env180.zone[m.src[7]].count == 1);
}

static void test_processed_count_with_several_targets() {
  if (TARGETS_PER_ZONE < 2)
    return;
  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  // zone 5: T1 at 90 cm (person), T2 at 210 cm (floor, in the environment); zone has 2 targets
  r.nb_target_detected[5] = 2;
  set_target(r, 5, 0, 900, 5);
  set_target(r, 5, 1, 2100, 5);
  std::vector<uint16_t> flat(ENV_ZONES * ENV_TABLE_STRIDE, 0);
  flat[5 * ENV_TABLE_STRIDE] = 1; flat[5 * ENV_TABLE_STRIDE + 1] = 210; flat[5 * ENV_TABLE_STRIDE + 1 + ENV_MAX_REFS] = 5;
  EnvRuntime env;
  env_build_runtime(env, flat.data(), Rotation::DEG_0);
  const ProcessingContext c = ctx(FilterProfile::RANGE_VALID, 400, &env);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED_RAW, r, 5, 0, c).value == 2);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 5, 0, c).value == 1);  // only the person
  CHECK(get_cell(MatrixId::DISTANCE, r, 5, 0, c).value == 90);
  CHECK(get_cell(MatrixId::DISTANCE, r, 5, 1, c).value == 400);                  // floor target -> limit
  // a target at the limit is not counted either
  set_target(r, 5, 1, 4000, 5);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 5, 0, c).value == 1);
}

static CalResult calibrate(const std::vector<int> &samples) {
  CalZone z;
  for (int s : samples)
    z.add(static_cast<uint16_t>(s));
  CalResult res;
  cal_finalize(z, res);
  return res;
}

static std::string line_of(const CalResult &r, uint8_t zone = 0) {
  char buf[128];
  cal_format_line(buf, sizeof(buf), zone, r);
  return buf;
}

static void test_calibration_two_surfaces() {
  std::vector<int> s;
  for (int i = 0; i < 30; i++) s.push_back(150 + (i % 3) - 1);   // 149..151
  for (int i = 0; i < 30; i++) s.push_back(210 + (i % 5) - 2);   // 208..212
  CalResult r = calibrate(s);
  CHECK(r.count == 2);
  CHECK(r.ref[0] == 150 && r.ref[1] == 210);
  CHECK(r.delta[0] >= CAL_MIN_DELTA_CM && r.delta[0] <= 7);
  CHECK(r.delta[1] >= CAL_MIN_DELTA_CM && r.delta[1] <= 8);
  CHECK_STR(line_of(r, 7), "7>150/" + std::to_string(r.delta[0]) + ",210/" + std::to_string(r.delta[1]));
  // the suggested bands contain every sample of their cluster and never overlap
  for (int v : s) {
    bool in = false;
    for (uint8_t k = 0; k < r.count; k++)
      in = in || (v >= r.ref[k] - r.delta[k] && v <= r.ref[k] + r.delta[k]);
    CHECK(in);
  }
  CHECK(r.ref[1] - r.delta[1] > r.ref[0] + r.delta[0]);
}

static void test_minimum_distance() {
  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  r.nb_target_detected[0] = 1; set_target(r, 0, 0, 0, 5);     // 0 cm glitch with a "valid" status
  r.nb_target_detected[1] = 1; set_target(r, 1, 0, 90, 5);    // 9 cm
  r.nb_target_detected[2] = 1; set_target(r, 2, 0, 100, 5);   // exactly 10 cm
  ProcessingContext c = ctx(FilterProfile::RANGE_VALID, 400);
  c.min_distance_cm = 10;
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, c).value == 400);   // not a target -> limit
  CHECK(get_cell(MatrixId::DISTANCE, r, 1, 0, c).value == 400);
  CHECK(get_cell(MatrixId::DISTANCE, r, 2, 0, c).value == 10);    // at the minimum: kept
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 0, 0, c).value == 0);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED, r, 2, 0, c).value == 1);
  // RAW keeps the glitch for diagnosis
  CHECK(get_cell(MatrixId::DISTANCE_RAW, r, 0, 0, c).valid && get_cell(MatrixId::DISTANCE_RAW, r, 0, 0, c).value == 0);
  CHECK(get_cell(MatrixId::NUMBER_OF_TARGETS_DETECTED_RAW, r, 0, 0, c).value == 1);
  c.min_distance_cm = 0;  // filter off
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, c).value == 0);
}

static void test_calibration_wide_band_flag() {
  // a surface spread continuously over 120..200 cm becomes one wide band that is flagged
  std::vector<int> s;
  for (int v = 120; v <= 200; v += 4)
    for (int k = 0; k < 3; k++) s.push_back(v);
  CalResult r = calibrate(s);
  CHECK(r.count == 1);
  CHECK(r.wide);
  CHECK(r.delta[0] >= CAL_WIDE_BAND_DELTA_CM);
  CHECK(r.wide_low_cm >= 100 && r.wide_low_cm <= 125);
  // a narrow surface is not flagged
  r = calibrate(std::vector<int>(40, 150));
  CHECK(!r.wide);
}

static void test_calibration_limit_noise_and_empty() {
  // zone that always reads "limit"
  CalResult r = calibrate(std::vector<int>(50, 400));
  CHECK(r.count == 1 && r.ref[0] == 400 && r.delta[0] == CAL_MIN_DELTA_CM);
  CHECK_STR(line_of(r, 3), "3>400/" + std::to_string(CAL_MIN_DELTA_CM));
  // 3 stray samples out of 100 (3%) are kept, 2 of 100 (2%) are noise but listed
  std::vector<int> s(98, 210);
  s.push_back(120); s.push_back(121);
  r = calibrate(s);
  CHECK(r.count == 1 && r.ref[0] == 210);
  bool listed_noise = false;
  for (uint8_t i = 0; i < r.cand_count; i++)
    listed_noise = listed_noise || (r.cand[i].noise && r.cand[i].ref >= 120 && r.cand[i].ref <= 121);
  CHECK(listed_noise);
  // no samples: explicit empty line, nothing invented
  r = calibrate({});
  CHECK(r.count == 0);
  CHECK_STR(line_of(r, 5), "5>");
}

static void test_calibration_gap_and_chain_merge() {
  // 150 and 155 are within the 6 cm gap: one reference; 170 is 15 cm away: another one
  std::vector<int> s;
  for (int i = 0; i < 20; i++) { s.push_back(150); s.push_back(155); s.push_back(170); }
  CalResult r = calibrate(s);
  CHECK(r.count == 2);
  CHECK(r.ref[0] >= 151 && r.ref[0] <= 154);
  CHECK(r.ref[1] == 170);
}

static void test_calibration_more_than_four_surfaces() {
  std::vector<int> s;
  const int centers[] = {60, 100, 140, 180, 220, 260};
  const int weights[] = {10, 40, 30, 25, 20, 15};
  for (int k = 0; k < 6; k++)
    for (int i = 0; i < weights[k]; i++) s.push_back(centers[k]);
  CalResult r = calibrate(s);
  CHECK(r.count == 4);                        // at most 4 references per zone
  CHECK(r.cand_count == 6);                   // but everything observed is available for the logs
  for (uint8_t k = 0; k + 1 < r.count; k++)
    CHECK(r.ref[k] < r.ref[k + 1]);           // sorted by distance
  CHECK(r.ref[0] == 100 && r.ref[1] == 140);  // the 4 most frequent: 100, 140, 180, 220
  CHECK(r.ref[2] == 180 && r.ref[3] == 220);
}

static void test_calibration_cluster_overflow() {
  // more distinct values than clusters: the closest clusters are merged, nothing crashes
  CalZone z;
  for (int rep = 0; rep < 3; rep++)
    for (int v = 20; v <= 400; v += 8)
      z.add(static_cast<uint16_t>(v));
  CHECK(z.n <= CAL_MAX_CLUSTERS);
  CalResult r;
  cal_finalize(z, r);
  CHECK(r.count <= ENV_MAX_REFS);
}

int main() {
  test_limit_table_from_the_specification();
  test_processed_vs_raw();
  test_environment_bands();
  test_environment_follows_the_physical_zone();
  test_processed_count_with_several_targets();
  test_calibration_two_surfaces();
  test_minimum_distance();
  test_calibration_wide_band_flag();
  test_calibration_limit_noise_and_empty();
  test_calibration_gap_and_chain_merge();
  test_calibration_more_than_four_surfaces();
  test_calibration_cluster_overflow();
  std::printf("environment tests, targets_per_zone=%u: %d checks, %d failed\n", (unsigned) TARGETS_PER_ZONE, g_checks,
              g_failed);
  return g_failed == 0 ? 0 : 1;
}
