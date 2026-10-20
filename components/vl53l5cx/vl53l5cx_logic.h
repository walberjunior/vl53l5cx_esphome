// Pure, dependency-free logic of the VL53L5CX component: virtual rotation,
// target-status filtering, matrix text formatting and the publication
// scheduler. Kept free of ESPHome/ULD headers so it can be unit-tested on a
// host compiler (see tests/).
#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace vl53l5cx {

// ---------------------------------------------------------------------------
// Virtual rotation (spec 15.1)
//
// Convention: zone index = row * N + col, exactly as delivered by the ULD
// (row 0 = first N zones). Rotation is clockwise and defined on that grid:
//   0   : (r, c) -> (r, c)
//   90  : (r, c) -> (c, N-1-r)
//   180 : (r, c) -> (N-1-r, N-1-c)
//   270 : (r, c) -> (N-1-c, r)
// ---------------------------------------------------------------------------
enum class Rotation : uint8_t { DEG_0 = 0, DEG_90 = 1, DEG_180 = 2, DEG_270 = 3 };

inline void rotate_forward(uint8_t n, Rotation rot, uint8_t row, uint8_t col, uint8_t &drow, uint8_t &dcol) {
  switch (rot) {
    case Rotation::DEG_90:
      drow = col;
      dcol = static_cast<uint8_t>(n - 1 - row);
      break;
    case Rotation::DEG_180:
      drow = static_cast<uint8_t>(n - 1 - row);
      dcol = static_cast<uint8_t>(n - 1 - col);
      break;
    case Rotation::DEG_270:
      drow = static_cast<uint8_t>(n - 1 - col);
      dcol = row;
      break;
    case Rotation::DEG_0:
    default:
      drow = row;
      dcol = col;
      break;
  }
}

// Inverse mapping: for a destination cell, which source cell lands there.
inline void rotate_inverse(uint8_t n, Rotation rot, uint8_t drow, uint8_t dcol, uint8_t &row, uint8_t &col) {
  switch (rot) {
    case Rotation::DEG_90:
      row = static_cast<uint8_t>(n - 1 - dcol);
      col = drow;
      break;
    case Rotation::DEG_180:
      row = static_cast<uint8_t>(n - 1 - drow);
      col = static_cast<uint8_t>(n - 1 - dcol);
      break;
    case Rotation::DEG_270:
      row = dcol;
      col = static_cast<uint8_t>(n - 1 - drow);
      break;
    case Rotation::DEG_0:
    default:
      row = drow;
      col = dcol;
      break;
  }
}

// Gather table: src[destination_index] = source zone index. One table is
// shared by every matrix, so all matrices use the same transformation.
struct RotationMap {
  uint8_t n{4};
  uint8_t src[64]{};

  void build(uint8_t size, Rotation rot) {
    n = size;
    for (uint8_t dr = 0; dr < size; dr++) {
      for (uint8_t dc = 0; dc < size; dc++) {
        uint8_t r = 0, c = 0;
        rotate_inverse(size, rot, dr, dc, r, c);
        src[dr * size + dc] = static_cast<uint8_t>(r * size + c);
      }
    }
  }
};

// ---------------------------------------------------------------------------
// Target status filter (UM2884 5.5, Table 4)
// ---------------------------------------------------------------------------
enum class FilterProfile : uint8_t {
  DISABLED = 0,
  RANGE_VALID = 1,       // 5, 9
  PERMISSIVE = 2,        // everything except 0 and 255
  HIGH_CONFIDENCE = 3,   // 5
  CONFIDENCE_BASED = 4,  // 5, 6, 9
};

inline bool status_accepted(FilterProfile profile, uint8_t status) {
  switch (profile) {
    case FilterProfile::RANGE_VALID:
      return status == 5 || status == 9;
    case FilterProfile::PERMISSIVE:
      return status != 0 && status != 255;
    case FilterProfile::HIGH_CONFIDENCE:
      return status == 5;
    case FilterProfile::CONFIDENCE_BASED:
      return status == 5 || status == 6 || status == 9;
    case FilterProfile::DISABLED:
    default:
      return true;
  }
}

// ---------------------------------------------------------------------------
// Matrix text format
//
// Values are comma separated; the last value of every row is followed by '|'
// instead of ','. There is no trailing separator. An unavailable value is
// written as "nan". The complete text never exceeds 255 characters (the Home
// Assistant state limit): the number of digits per value is derived from that
// budget (8x8: 3 digits, values above 999 are clamped to 999; 4x4: 10 digits,
// i.e. no clamping of 32-bit values).
// ---------------------------------------------------------------------------
constexpr size_t MATRIX_TEXT_MAX_CHARS = 255;
constexpr size_t MATRIX_TEXT_BUFFER_SIZE = 256;  // 255 characters + NUL

struct Cell {
  bool valid;
  uint32_t value;
};

// Digits available per value so that the whole text stays within 255 chars.
inline uint8_t digits_for_zones(uint8_t zones) {
  const uint32_t digits = static_cast<uint32_t>((MATRIX_TEXT_MAX_CHARS - (zones - 1)) / zones);
  return static_cast<uint8_t>(digits > 10 ? 10 : digits);
}

inline uint32_t max_value_for_zones(uint8_t zones) {
  const uint8_t digits = digits_for_zones(zones);
  if (digits >= 10)
    return 0xFFFFFFFFu;
  uint32_t v = 1;
  for (uint8_t i = 0; i < digits; i++)
    v *= 10;
  return v - 1;
}

// cell_at(source_zone_index) -> Cell. Returns the text length.
template<typename F> size_t format_matrix(char *buf, size_t cap, uint8_t n, const RotationMap &map, F &&cell_at) {
  const uint8_t zones = static_cast<uint8_t>(n * n);
  const uint32_t max_value = max_value_for_zones(zones);
  const size_t digits = digits_for_zones(zones);  // also covers the 3 characters of "nan"
  size_t pos = 0;
  for (uint8_t i = 0; i < zones; i++) {
    if (pos + digits + 1 > cap)
      break;  // room for the value and the terminating NUL; cannot happen with cap >= MATRIX_TEXT_BUFFER_SIZE
    const Cell cell = cell_at(map.src[i]);
    if (!cell.valid) {
      buf[pos++] = 'n';
      buf[pos++] = 'a';
      buf[pos++] = 'n';
    } else {
      uint32_t v = cell.value > max_value ? max_value : cell.value;
      char tmp[10];
      uint8_t k = 0;
      do {
        tmp[k++] = static_cast<char>('0' + (v % 10));
        v /= 10;
      } while (v != 0);
      while (k > 0)
        buf[pos++] = tmp[--k];
    }
    if (i + 1 < zones)
      buf[pos++] = ((i + 1) % n == 0) ? '|' : ',';
  }
  buf[pos] = '\0';
  return pos;
}

// ---------------------------------------------------------------------------
// Publication scheduler (spec 4)
//
// Only decides *when* matrices are published. It never touches the ranging
// configuration. A frame is published at most once (sequence numbers); with
// interval 0 every fresh frame is published; with interval > 0 at most one
// publication per interval, always using the latest frame.
// ---------------------------------------------------------------------------
struct PublishScheduler {
  uint32_t interval_ms{1000};
  uint32_t last_ms{0};
  uint32_t last_seq{0};  // frame sequence numbers start at 1
  bool published_once{false};

  bool due(uint32_t now_ms, uint32_t frame_seq) const {
    if (frame_seq == last_seq)
      return false;  // nothing new since the last publication
    if (interval_ms == 0 || !published_once)
      return true;
    return static_cast<uint32_t>(now_ms - last_ms) >= interval_ms;
  }

  void mark(uint32_t now_ms, uint32_t frame_seq) {
    last_ms = now_ms;
    last_seq = frame_seq;
    published_once = true;
  }
};

// ---------------------------------------------------------------------------
// Configuration validation helpers (UM2884 4.4, 4.6)
// ---------------------------------------------------------------------------
constexpr uint8_t RES_4X4 = 16;  // VL53L5CX_RESOLUTION_4X4
constexpr uint8_t RES_8X8 = 64;  // VL53L5CX_RESOLUTION_8X8

inline uint8_t matrix_size(uint8_t resolution) { return resolution == RES_8X8 ? 8 : 4; }
inline uint8_t max_frequency_hz(uint8_t resolution) { return resolution == RES_8X8 ? 15 : 60; }
inline uint8_t integration_slots(uint8_t resolution) { return resolution == RES_8X8 ? 4 : 1; }

// Largest integration time (ms) such that slots * t + 1 ms < period.
inline uint32_t max_integration_ms(uint8_t resolution, uint32_t freq_hz) {
  if (freq_hz == 0)
    return 0;
  const float period_ms = 1000.0f / static_cast<float>(freq_hz);
  const float slots = static_cast<float>(integration_slots(resolution));
  const float limit = (period_ms - 1.0f) / slots;  // t must be strictly below this
  uint32_t t = static_cast<uint32_t>(limit);
  if (static_cast<float>(t) >= limit)
    t = t > 0 ? t - 1 : 0;
  return t;
}

// Number of frames between two periodic temperature compensations.
inline uint32_t vhv_repeat_frames(uint32_t seconds, uint32_t freq_hz) {
  if (seconds == 0)
    return 0;
  const uint64_t frames = static_cast<uint64_t>(seconds) * freq_hz;
  if (frames == 0)
    return 1;
  return frames > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<uint32_t>(frames);
}

}  // namespace vl53l5cx
}  // namespace esphome
