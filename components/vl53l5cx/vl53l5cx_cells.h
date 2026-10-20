// Per-zone value extraction from a ULD result frame. Header-only so that it
// can be unit-tested on a host compiler against the real ULD result struct.
#pragma once

#include "vl53l5cx_environment.h"
#include "vl53l5cx_logic.h"
#include "vl53l5cx_uld.h"

namespace esphome {
namespace vl53l5cx {

// Order matches the switch order in the specification and the Python side.
enum class MatrixId : uint8_t {
  AMBIENT_PER_SPAD = 0,
  NUMBER_OF_TARGETS_DETECTED = 1,  // processed count (status filter + maximum distance + environment filter)
  NUMBER_OF_SPADS_ENABLED = 2,
  SIGNAL_PER_SPAD = 3,
  RANGE_SIGMA = 4,
  TARGET_STATUS = 5,
  REFLECTANCE = 6,
  MOTION_INDICATOR = 7,
  DISTANCE_RAW = 8,
  DISTANCE = 9,
  NUMBER_OF_TARGETS_DETECTED_RAW = 10,
};
constexpr uint8_t MATRIX_COUNT = 11;
constexpr uint8_t TARGETS_PER_ZONE = VL53L5CX_NB_TARGET_PER_ZONE;

// Matrices that exist once per target index (T1..T<targets_per_zone>).
inline bool is_per_target(MatrixId id) {
  switch (id) {
    case MatrixId::SIGNAL_PER_SPAD:
    case MatrixId::RANGE_SIGMA:
    case MatrixId::TARGET_STATUS:
    case MatrixId::REFLECTANCE:
    case MatrixId::DISTANCE_RAW:
    case MatrixId::DISTANCE:
      return true;
    default:
      return false;
  }
}

inline uint32_t mm_to_cm(int16_t mm) {
  const int32_t v = mm < 0 ? 0 : mm;  // ULD already clamps negatives to 0
  return static_cast<uint32_t>((v + 5) / 10);
}

// Settings of the processed (non-RAW) matrices: Distance and the processed
// Number of Targets Detected. RAW matrices never use them.
struct ProcessingContext {
  FilterProfile filter{FilterProfile::RANGE_VALID};  // Target Status Filter
  uint16_t max_distance_cm{ENV_MAX_DISTANCE_CM};     // Maximum Distance
  uint16_t min_distance_cm{0};                       // Minimum Distance: closer readings are invalid
  const EnvRuntime *env{nullptr};                    // nullptr = environment filter inactive
};

// Processed distance (cm) of one target slot. NaN-like cases (no target in the
// slot, status rejected by the filter, closer than Minimum Distance) and distances above the limit become
// the limit; a distance inside an environment band also becomes the limit.
// `counts` is set when the slot is a real target: valid status, below the
// limit and not part of the known environment.
inline uint16_t processed_distance_cm(const VL53L5CX_ResultsData &r, uint8_t zone, uint8_t target,
                                      const ProcessingContext &ctx, bool *counts = nullptr) {
  const uint32_t idx = static_cast<uint32_t>(zone) * TARGETS_PER_ZONE + target;
  const uint32_t raw_cm = mm_to_cm(r.distance_mm[idx]);
  const bool valid = target < r.nb_target_detected[zone] && status_accepted(ctx.filter, r.target_status[idx]) &&
                     raw_cm >= ctx.min_distance_cm;  // e.g. a 0 cm glitch is not a target
  uint16_t cm = limit_distance_cm(valid, raw_cm, ctx.max_distance_cm);
  const bool environment = ctx.env != nullptr && ctx.env->match(zone, cm);
  if (counts != nullptr)
    *counts = valid && cm < ctx.max_distance_cm && !environment;
  return environment ? ctx.max_distance_cm : cm;
}

// `zone` is the source (unrotated) zone index; for MOTION_INDICATOR it is the
// aggregate index (a 4x4 grid in both resolutions). `target` is 0-based.
//
// RAW matrices and the other per-zone matrices come straight from the ULD. A
// target slot at or beyond nb_target_detected has no measurement and is
// reported as unavailable ("nan") for every RAW per-target matrix except
// TARGET_STATUS, which always preserves the raw ULD code.
inline Cell get_cell(MatrixId id, const VL53L5CX_ResultsData &r, uint8_t zone, uint8_t target,
                     const ProcessingContext &ctx) {
  const uint32_t idx = static_cast<uint32_t>(zone) * TARGETS_PER_ZONE + target;
  const bool has_target = target < r.nb_target_detected[zone];
  switch (id) {
    case MatrixId::AMBIENT_PER_SPAD:
      return {true, r.ambient_per_spad[zone]};
    case MatrixId::NUMBER_OF_TARGETS_DETECTED_RAW:
      return {true, r.nb_target_detected[zone]};
    case MatrixId::NUMBER_OF_TARGETS_DETECTED: {
      uint32_t n = 0;
      for (uint8_t t = 0; t < TARGETS_PER_ZONE; t++) {
        bool counts = false;
        (void) processed_distance_cm(r, zone, t, ctx, &counts);
        n += counts ? 1 : 0;
      }
      return {true, n};
    }
    case MatrixId::NUMBER_OF_SPADS_ENABLED:
      return {true, r.nb_spads_enabled[zone]};
    case MatrixId::SIGNAL_PER_SPAD:
      return {has_target, r.signal_per_spad[idx]};
    case MatrixId::RANGE_SIGMA:
      return {has_target, r.range_sigma_mm[idx]};
    case MatrixId::TARGET_STATUS:
      return {true, r.target_status[idx]};
    case MatrixId::REFLECTANCE:
      return {has_target, r.reflectance[idx]};
    case MatrixId::MOTION_INDICATOR:
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
      return {true, r.motion_indicator.motion[zone]};
#else
      return {false, 0};  // motion indicator compiled out
#endif
    case MatrixId::DISTANCE_RAW:
      return {has_target, mm_to_cm(r.distance_mm[idx])};
    case MatrixId::DISTANCE:
      return {true, processed_distance_cm(r, zone, target, ctx)};
    default:
      return {false, 0};
  }
}

}  // namespace vl53l5cx
}  // namespace esphome
