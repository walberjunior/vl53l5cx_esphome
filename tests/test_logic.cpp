// Host-side unit tests for the pure logic of the VL53L5CX component. They run
// against the real ULD result structure (compile with -DVL53L5CX_NB_TARGET_PER_ZONE=N).
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "vl53l5cx_cells.h"

using namespace esphome::vl53l5cx;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    g_checks++;                                                          \
    if (!(cond)) {                                                       \
      g_failed++;                                                        \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                    \
  } while (0)
#define CHECK_STR(actual, expected)                                                                  \
  do {                                                                                               \
    g_checks++;                                                                                      \
    if (std::string(actual) != std::string(expected)) {                                              \
      g_failed++;                                                                                    \
      std::printf("FAIL %s:%d:\n  got      %s\n  expected %s\n", __FILE__, __LINE__, std::string(actual).c_str(), \
                  std::string(expected).c_str());                                                    \
    }                                                                                                \
  } while (0)

static ProcessingContext ctx_of(FilterProfile f, uint16_t max_cm = 400, const EnvRuntime *env = nullptr) {
  ProcessingContext c;
  c.filter = f;
  c.max_distance_cm = max_cm;
  c.env = env;
  return c;
}

static const Rotation ROTS[4] = {Rotation::DEG_0, Rotation::DEG_90, Rotation::DEG_180, Rotation::DEG_270};

static std::string fmt_identity(uint8_t n, Rotation rot) {
  RotationMap m;
  m.build(n, rot);
  char buf[MATRIX_TEXT_BUFFER_SIZE];
  format_matrix(buf, sizeof(buf), n, m, [](uint8_t z) { return Cell{true, z}; });
  return buf;
}

static void test_rotation_known_4x4() {
  CHECK_STR(fmt_identity(4, Rotation::DEG_0), "0,1,2,3|4,5,6,7|8,9,10,11|12,13,14,15");
  CHECK_STR(fmt_identity(4, Rotation::DEG_90), "12,8,4,0|13,9,5,1|14,10,6,2|15,11,7,3");
  CHECK_STR(fmt_identity(4, Rotation::DEG_180), "15,14,13,12|11,10,9,8|7,6,5,4|3,2,1,0");
  CHECK_STR(fmt_identity(4, Rotation::DEG_270), "3,7,11,15|2,6,10,14|1,5,9,13|0,4,8,12");
}

static void test_rotation_properties() {
  for (uint8_t n : {4, 8}) {
    for (Rotation rot : ROTS) {
      RotationMap m;
      m.build(n, rot);
      // bijection: no zone lost or duplicated
      std::vector<int> seen(n * n, 0);
      for (int i = 0; i < n * n; i++) {
        CHECK(m.src[i] < n * n);
        seen[m.src[i]]++;
      }
      for (int i = 0; i < n * n; i++)
        CHECK(seen[i] == 1);
      // forward formulas from the specification agree with the gather table
      for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
          uint8_t dr, dc;
          rotate_forward(n, rot, r, c, dr, dc);
          CHECK(m.src[dr * n + dc] == r * n + c);
        }
      }
    }
    // spec formulas, spelled out literally
    uint8_t dr, dc;
    rotate_forward(n, Rotation::DEG_90, 1, 2, dr, dc);
    CHECK(dr == 2 && dc == n - 1 - 1);
    rotate_forward(n, Rotation::DEG_180, 1, 2, dr, dc);
    CHECK(dr == n - 1 - 1 && dc == n - 1 - 2);
    rotate_forward(n, Rotation::DEG_270, 1, 2, dr, dc);
    CHECK(dr == n - 1 - 2 && dc == 1);
    // four quarter turns are the identity; 270 undoes 90
    for (uint8_t z = 0; z < n * n; z++) {
      uint8_t r = z / n, c = z % n;
      for (int k = 0; k < 4; k++) {
        uint8_t nr, nc;
        rotate_forward(n, Rotation::DEG_90, r, c, nr, nc);
        r = nr;
        c = nc;
      }
      CHECK(r == z / n && c == z % n);
      uint8_t ar, ac, br, bc;
      rotate_forward(n, Rotation::DEG_90, z / n, z % n, ar, ac);
      rotate_forward(n, Rotation::DEG_270, ar, ac, br, bc);
      CHECK(br == z / n && bc == z % n);
    }
  }
  // 8x8 corner spot checks: source zone 0 (row 0, col 0)
  RotationMap m;
  m.build(8, Rotation::DEG_90);
  CHECK(m.src[7] == 0);   // lands at (0, 7)
  m.build(8, Rotation::DEG_180);
  CHECK(m.src[63] == 0);  // lands at (7, 7)
  m.build(8, Rotation::DEG_270);
  CHECK(m.src[56] == 0);  // lands at (7, 0)
}

static void fill_frame(VL53L5CX_ResultsData &r, uint8_t nb) {
  std::memset(&r, 0, sizeof(r));
  for (int z = 0; z < 64; z++) {
    r.ambient_per_spad[z] = z;
    r.nb_spads_enabled[z] = z;
    r.nb_target_detected[z] = nb;
    for (int t = 0; t < TARGETS_PER_ZONE; t++) {
      const int i = z * TARGETS_PER_ZONE + t;
      r.signal_per_spad[i] = z;
      r.range_sigma_mm[i] = z;
      r.target_status[i] = z;  // distinct raw codes (0..63) for association checks
      r.reflectance[i] = z;
      r.distance_mm[i] = z * 10;  // -> z cm
    }
  }
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  for (int a = 0; a < 32; a++)
    r.motion_indicator.motion[a] = a;
#endif
}

static std::string fmt_matrix(MatrixId id, const VL53L5CX_ResultsData &r, uint8_t n, Rotation rot,
                              FilterProfile f = FilterProfile::DISABLED, uint8_t target = 0) {
  RotationMap m;
  m.build(n, rot);
  char buf[MATRIX_TEXT_BUFFER_SIZE];
  format_matrix(buf, sizeof(buf), n, m, [&](uint8_t z) { return get_cell(id, r, z, target, ctx_of(f)); });
  return buf;
}

static void test_all_matrices_share_the_same_transform() {
  static VL53L5CX_ResultsData frame;
  fill_frame(frame, 1);
  const MatrixId same[] = {MatrixId::AMBIENT_PER_SPAD, MatrixId::NUMBER_OF_SPADS_ENABLED, MatrixId::SIGNAL_PER_SPAD,
                           MatrixId::RANGE_SIGMA,      MatrixId::TARGET_STATUS,           MatrixId::REFLECTANCE,
                           MatrixId::DISTANCE_RAW,     MatrixId::DISTANCE};
  const uint8_t before_hash_len = 8;
  (void) before_hash_len;
  for (uint8_t n : {4, 8}) {
    for (Rotation rot : ROTS) {
      const std::string expect = fmt_identity(n, rot);
      for (MatrixId id : same)
        CHECK_STR(fmt_matrix(id, frame, n, rot), expect);  // incl. Distance RAW: rotated like everything else
    }
  }
  // Motion indicator is always a 4x4 aggregate grid, also in 8x8 mode.
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  for (Rotation rot : ROTS)
    CHECK_STR(fmt_matrix(MatrixId::MOTION_INDICATOR, frame, 4, rot), fmt_identity(4, rot));
#else
  CHECK(!get_cell(MatrixId::MOTION_INDICATOR, frame, 0, 0, ctx_of(FilterProfile::DISABLED)).valid);  // compiled out -> nan
#endif
}

static void test_rotation_does_not_alter_measurements() {
  static VL53L5CX_ResultsData frame, copy;
  fill_frame(frame, 1);
  std::memcpy(&copy, &frame, sizeof(frame));
  for (Rotation rot : ROTS) {
    for (uint8_t m = 0; m < MATRIX_COUNT; m++)
      (void) fmt_matrix(static_cast<MatrixId>(m), frame, 8, rot, FilterProfile::RANGE_VALID);
  }
  CHECK(std::memcmp(&frame, &copy, sizeof(frame)) == 0);
}

static void test_status_filter_tables() {
  struct Case {
    FilterProfile p;
    std::vector<uint8_t> accepted;  // for the rest: rejected
  } cases[] = {
      {FilterProfile::RANGE_VALID, {5, 9}},
      {FilterProfile::HIGH_CONFIDENCE, {5}},
      {FilterProfile::CONFIDENCE_BASED, {5, 6, 9}},
  };
  for (const auto &c : cases) {
    for (int s = 0; s < 256; s++) {
      bool expect = false;
      for (uint8_t a : c.accepted)
        expect = expect || (a == s);
      CHECK(status_accepted(c.p, static_cast<uint8_t>(s)) == expect);
    }
  }
  for (int s = 0; s < 256; s++) {
    CHECK(status_accepted(FilterProfile::PERMISSIVE, static_cast<uint8_t>(s)) == (s != 0 && s != 255));
    CHECK(status_accepted(FilterProfile::DISABLED, static_cast<uint8_t>(s)));
  }
}

static void test_distance_filter_and_raw() {
  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  // 4x4 frame; zone z carries status codes chosen from the table, 1 target each
  const uint8_t codes[16] = {5, 9, 6, 0, 255, 12, 13, 4, 3, 2, 7, 8, 10, 11, 1, 5};
  for (int z = 0; z < 16; z++) {
    r.nb_target_detected[z] = 1;
    r.target_status[z * TARGETS_PER_ZONE] = codes[z];
    r.distance_mm[z * TARGETS_PER_ZONE] = 1000 + z;  // 100.0 .. 101.5 cm -> rounds to 100/101/102
  }
  std::string raw = fmt_matrix(MatrixId::DISTANCE_RAW, r, 4, Rotation::DEG_0);
  CHECK_STR(raw, "100,100,100,100|100,101,101,101|101,101,101,101|101,101,101,102");
  // RAW is identical for every filter profile
  for (int p = 0; p <= 4; p++) {
    CHECK_STR(fmt_matrix(MatrixId::DISTANCE_RAW, r, 4, Rotation::DEG_0, static_cast<FilterProfile>(p)), raw);
    // Target Status always shows the raw ULD codes, regardless of the filter
    CHECK_STR(fmt_matrix(MatrixId::TARGET_STATUS, r, 4, Rotation::DEG_0, static_cast<FilterProfile>(p)),
              "5,9,6,0|255,12,13,4|3,2,7,8|10,11,1,5");
  }
  CHECK_STR(fmt_matrix(MatrixId::DISTANCE, r, 4, Rotation::DEG_0, FilterProfile::DISABLED), raw);
  CHECK_STR(fmt_matrix(MatrixId::DISTANCE, r, 4, Rotation::DEG_0, FilterProfile::RANGE_VALID),
            "100,100,400,400|400,400,400,400|400,400,400,400|400,400,400,102");
  CHECK_STR(fmt_matrix(MatrixId::DISTANCE, r, 4, Rotation::DEG_0, FilterProfile::HIGH_CONFIDENCE),
            "100,400,400,400|400,400,400,400|400,400,400,400|400,400,400,102");
  CHECK_STR(fmt_matrix(MatrixId::DISTANCE, r, 4, Rotation::DEG_0, FilterProfile::CONFIDENCE_BASED),
            "100,100,100,400|400,400,400,400|400,400,400,400|400,400,400,102");
  CHECK_STR(fmt_matrix(MatrixId::DISTANCE, r, 4, Rotation::DEG_0, FilterProfile::PERMISSIVE),
            "100,100,100,400|400,101,101,101|101,101,101,101|101,101,101,102");
  // Distance and Target Status keep the same zone association after rotation
  for (Rotation rot : ROTS) {
    RotationMap m;
    m.build(4, rot);
    for (int i = 0; i < 16; i++) {
      const uint8_t z = m.src[i];
      const Cell d = get_cell(MatrixId::DISTANCE, r, z, 0, ctx_of(FilterProfile::HIGH_CONFIDENCE));
      const Cell s = get_cell(MatrixId::TARGET_STATUS, r, z, 0, ctx_of(FilterProfile::HIGH_CONFIDENCE));
      CHECK((d.value != 400) == (s.value == 5));  // processed: rejected -> limit
    }
  }
}

static void test_conversion_and_unavailable_slots() {
  CHECK(mm_to_cm(0) == 0);
  CHECK(mm_to_cm(4) == 0);
  CHECK(mm_to_cm(5) == 1);
  CHECK(mm_to_cm(14) == 1);
  CHECK(mm_to_cm(15) == 2);
  CHECK(mm_to_cm(4000) == 400);
  CHECK(mm_to_cm(-3) == 0);
  static VL53L5CX_ResultsData r;
  std::memset(&r, 0, sizeof(r));
  r.nb_target_detected[0] = 0;
  r.target_status[0] = 255;
  r.distance_mm[0] = 1234;
  CHECK(!get_cell(MatrixId::DISTANCE_RAW, r, 0, 0, ctx_of(FilterProfile::DISABLED)).valid);  // no target in the zone
  CHECK(get_cell(MatrixId::DISTANCE, r, 0, 0, ctx_of(FilterProfile::DISABLED)).value == 400);  // processed: no target -> limit
  CHECK(get_cell(MatrixId::TARGET_STATUS, r, 0, 0, ctx_of(FilterProfile::DISABLED)).value == 255);  // raw code kept
  // Range sigma stays in the ULD unit (mm)
  r.nb_target_detected[1] = 1;
  r.range_sigma_mm[1 * TARGETS_PER_ZONE] = 37;
  CHECK(get_cell(MatrixId::RANGE_SIGMA, r, 1, 0, ctx_of(FilterProfile::DISABLED)).value == 37);
  if (TARGETS_PER_ZONE >= 2) {
    r.distance_mm[1 * TARGETS_PER_ZONE + 1] = 2000;
    CHECK(!get_cell(MatrixId::DISTANCE_RAW, r, 1, 1, ctx_of(FilterProfile::DISABLED)).valid);  // slot beyond nb_target_detected
    r.nb_target_detected[1] = 2;
    CHECK(get_cell(MatrixId::DISTANCE_RAW, r, 1, 1, ctx_of(FilterProfile::DISABLED)).value == 200);
  }
}

static void test_format_limits() {
  char buf[MATRIX_TEXT_BUFFER_SIZE];
  RotationMap m;
  m.build(8, Rotation::DEG_0);
  size_t len = format_matrix(buf, sizeof(buf), 8, m, [](uint8_t) { return Cell{true, 999}; });
  CHECK(len == 255);  // 64 x 3 digits + 63 separators
  CHECK(std::string(buf).find('|') == 31);  // 8 values x 3 digits + 7 commas
  size_t bars = 0;
  for (size_t i = 0; i < len; i++)
    bars += buf[i] == '|';
  CHECK(bars == 7);
  len = format_matrix(buf, sizeof(buf), 8, m, [](uint8_t) { return Cell{true, 4000000000u}; });
  CHECK(len == 255);  // clamped to 999
  CHECK(std::string(buf).substr(0, 7) == "999,999");
  len = format_matrix(buf, sizeof(buf), 8, m, [](uint8_t) { return Cell{false, 0}; });
  CHECK(len == 255);  // "nan" has 3 characters as well
  m.build(4, Rotation::DEG_0);
  len = format_matrix(buf, sizeof(buf), 4, m, [](uint8_t) { return Cell{true, 4000000000u}; });
  CHECK(len == 16 * 10 + 15);  // 4x4: no clamping of 32-bit values
  CHECK(len <= MATRIX_TEXT_MAX_CHARS);
  CHECK(max_value_for_zones(64) == 999);
  CHECK(max_value_for_zones(16) == 0xFFFFFFFFu);
}

static void test_scheduler() {
  // interval 0: one publication per fresh frame at 15 Hz, never duplicated
  {
    PublishScheduler s;
    s.interval_ms = 0;
    uint32_t seq = 0, pubs = 0;
    for (uint32_t t = 0; t < 10000; t += 5) {  // loop runs every 5 ms
      if (t % 66 == 0)
        seq++;                                  // new frame about every 66 ms
      if (s.due(t, seq)) {
        pubs++;
        s.mark(t, seq);
      }
      CHECK(!s.due(t, seq));  // same frame is never published twice
    }
    CHECK(pubs == seq);
  }
  // interval 1000 ms with 15 Hz frames: about one publication per second, latest frame
  {
    PublishScheduler s;
    s.interval_ms = 1000;
    uint32_t seq = 0, pubs = 0, last_pub_seq = 0;
    for (uint32_t t = 0; t < 10000; t++) {
      if (t % 67 == 0)
        seq++;
      if (s.due(t, seq)) {
        pubs++;
        last_pub_seq = seq;
        s.mark(t, seq);
      }
    }
    CHECK(pubs >= 9 && pubs <= 11);
    CHECK(last_pub_seq + 15 >= seq);  // always the latest data, no backlog
  }
  // interval shorter than the frame period: still once per frame
  {
    PublishScheduler s;
    s.interval_ms = 10;
    uint32_t seq = 0, pubs = 0;
    for (uint32_t t = 0; t < 2000; t++) {
      if (t % 100 == 0)
        seq++;
      if (s.due(t, seq)) {
        pubs++;
        s.mark(t, seq);
      }
    }
    CHECK(pubs == seq);
  }
  // runtime change of the interval takes effect immediately
  {
    PublishScheduler s;
    s.interval_ms = 5000;
    s.mark(0, 1);
    CHECK(!s.due(1000, 2));
    s.interval_ms = 500;
    CHECK(s.due(1000, 2));
    s.interval_ms = 0;
    CHECK(s.due(1001, 3));
  }
}

static void test_validation_helpers() {
  CHECK(max_frequency_hz(RES_4X4) == 60);
  CHECK(max_frequency_hz(RES_8X8) == 15);
  CHECK(max_integration_ms(RES_4X4, 1) == 998);
  CHECK(max_integration_ms(RES_4X4, 60) == 15);
  CHECK(max_integration_ms(RES_8X8, 15) == 16);
  CHECK(max_integration_ms(RES_8X8, 1) == 249);
  for (uint8_t res : {RES_4X4, RES_8X8}) {
    for (uint32_t f = 1; f <= max_frequency_hz(res); f++) {
      const uint32_t t = max_integration_ms(res, f);
      const float period = 1000.0f / f;
      const float slots = integration_slots(res);
      CHECK(slots * t + 1 < period);            // allowed value satisfies the constraint
      CHECK(slots * (t + 1) + 1 >= period);     // and it is the largest one
      CHECK(t >= 2);                            // the default of 5 ms is always allowed
    }
  }
  CHECK(vhv_repeat_frames(0, 15) == 0);
  CHECK(vhv_repeat_frames(300, 15) == 4500);
  CHECK(vhv_repeat_frames(300, 60) == 18000);
  CHECK(vhv_repeat_frames(1, 1) == 1);
  CHECK(matrix_size(RES_4X4) == 4 && matrix_size(RES_8X8) == 8);
}

int main() {
  test_rotation_known_4x4();
  test_rotation_properties();
  test_all_matrices_share_the_same_transform();
  test_rotation_does_not_alter_measurements();
  test_status_filter_tables();
  test_distance_filter_and_raw();
  test_conversion_and_unavailable_slots();
  test_format_limits();
  test_scheduler();
  test_validation_helpers();
  std::printf("targets_per_zone=%u: %d checks, %d failed\n", (unsigned) TARGETS_PER_ZONE, g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
