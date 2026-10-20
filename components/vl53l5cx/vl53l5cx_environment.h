// Empty-environment filter and calibration algorithm. Pure, dependency-free
// (no ESPHome, no ULD) so it can be unit tested on a host compiler.
//
// All distances are integer centimetres of the *processed* distance matrix
// (status filter + maximum-distance limit already applied). Zone numbers used
// here are PHYSICAL zones (ULD zone index, unrotated); the rotation only exists
// when converting from/to the zone numbers written in the environment file.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "vl53l5cx_logic.h"

namespace esphome {
namespace vl53l5cx {

constexpr uint8_t ENV_ZONES = 64;                // 8x8 only in this version
constexpr uint8_t ENV_MAX_REFS = 4;              // references per zone
constexpr uint8_t ENV_TABLE_STRIDE = 1 + 2 * ENV_MAX_REFS;  // per zone: count, refs[4], deltas[4]
constexpr uint16_t ENV_MAX_DISTANCE_CM = 400;
constexpr uint16_t ENV_MAX_DELTA_CM = 100;

// Calibration tuning constants (first values, expected to be tuned in the field).
constexpr uint8_t CAL_MAX_CLUSTERS = 8;
constexpr uint16_t CAL_CLUSTER_GAP_CM = 6;       // samples closer than this join one cluster
constexpr uint16_t CAL_MIN_DELTA_CM = 5;         // typical zone jitter is about +/-5 cm
constexpr uint16_t CAL_WIDE_BAND_DELTA_CM = 30;  // bands this wide are flagged for review in the report
constexpr uint32_t CAL_NOISE_PERCENT = 3;        // clusters below this share of the samples are noise

// ---------------------------------------------------------------------------
// Distance processing (spec 4.3): NaN / above-limit -> limit
// ---------------------------------------------------------------------------
inline uint16_t limit_distance_cm(bool valid, uint32_t cm, uint16_t max_cm) {
  if (!valid || cm > max_cm)
    return max_cm;
  return static_cast<uint16_t>(cm);
}

// ---------------------------------------------------------------------------
// Runtime reference table, indexed by physical zone
// ---------------------------------------------------------------------------
struct EnvZoneRefs {
  uint8_t count{0};
  uint16_t ref[ENV_MAX_REFS]{};
  uint16_t delta[ENV_MAX_REFS]{};
};

struct EnvRuntime {
  EnvZoneRefs zone[ENV_ZONES];
  bool any{false};  // at least one zone has references

  // Independent bands [ref - delta, ref + delta], inclusive.
  bool match(uint8_t z, uint16_t cm) const {
    const EnvZoneRefs &r = zone[z];
    for (uint8_t i = 0; i < r.count; i++) {
      const int32_t d = static_cast<int32_t>(cm) - static_cast<int32_t>(r.ref[i]);
      if (d >= -static_cast<int32_t>(r.delta[i]) && d <= static_cast<int32_t>(r.delta[i]))
        return true;
    }
    return false;
  }
};

// `flat` is the compile-time table: ENV_ZONES x ENV_TABLE_STRIDE values, zones
// numbered as displayed with rotation `file_rotation` (index = row*8 + col of
// the rotated matrix). They are converted to physical zones here, so the
// references stay attached to the same physical zone if Matrix Rotation changes.
inline void env_build_runtime(EnvRuntime &out, const uint16_t *flat, Rotation file_rotation) {
  RotationMap map;
  map.build(8, file_rotation);
  out = EnvRuntime();
  for (uint8_t shown = 0; shown < ENV_ZONES; shown++) {
    const uint16_t *row = flat + static_cast<size_t>(shown) * ENV_TABLE_STRIDE;
    uint8_t count = static_cast<uint8_t>(row[0] > ENV_MAX_REFS ? ENV_MAX_REFS : row[0]);
    EnvZoneRefs &dst = out.zone[map.src[shown]];
    dst.count = count;
    for (uint8_t i = 0; i < count; i++) {
      dst.ref[i] = row[1 + i];
      dst.delta[i] = row[1 + ENV_MAX_REFS + i];
    }
    if (count > 0)
      out.any = true;
  }
}

// ---------------------------------------------------------------------------
// Calibration: online 1-D clustering of the samples of one zone
// ---------------------------------------------------------------------------
struct CalCluster {
  uint32_t count{0};
  uint64_t sum{0};
  uint64_t sumsq{0};
  uint16_t min{0};
  uint16_t max{0};

  void add(uint16_t v) {
    if (count == 0) {
      min = max = v;
    } else {
      if (v < min)
        min = v;
      if (v > max)
        max = v;
    }
    count++;
    sum += v;
    sumsq += static_cast<uint64_t>(v) * v;
  }
  void merge(const CalCluster &o) {
    if (o.count == 0)
      return;
    if (count == 0) {
      *this = o;
      return;
    }
    if (o.min < min)
      min = o.min;
    if (o.max > max)
      max = o.max;
    count += o.count;
    sum += o.sum;
    sumsq += o.sumsq;
  }
  double mean() const { return count ? static_cast<double>(sum) / count : 0.0; }
  double sigma() const {
    if (count < 2)
      return 0.0;
    const double m = mean();
    const double var = static_cast<double>(sumsq) / count - m * m;
    return var > 0 ? std::sqrt(var) : 0.0;
  }
  uint16_t ref() const {  // 1..ENV_MAX_DISTANCE_CM, the range accepted by the environment file
    long r = std::lround(mean());
    if (r < 1)
      r = 1;
    if (r > ENV_MAX_DISTANCE_CM)
      r = ENV_MAX_DISTANCE_CM;
    return static_cast<uint16_t>(r);
  }
  // Tolerance: largest observed deviation + 1 cm. For a Gaussian-looking cluster 3 sigma adds a margin, but the margin
  // never exceeds CAL_MIN_DELTA_CM beyond the observed extremes: a wide, flat distribution must not be extended into
  // distances that were never seen (3 sigma of a flat 120-200 cm spread would blind 90-230 cm).
  uint16_t delta() const {
    const double m = mean();
    const double dev = std::fmax(m - min, max - m);
    double d = std::ceil(dev) + 1.0;
    const double s3 = std::ceil(3.0 * sigma());
    if (s3 > d)
      d = std::fmin(s3, d + CAL_MIN_DELTA_CM);
    if (d < CAL_MIN_DELTA_CM)
      d = CAL_MIN_DELTA_CM;
    if (d > ENV_MAX_DELTA_CM)
      d = ENV_MAX_DELTA_CM;
    return static_cast<uint16_t>(d);
  }
};

struct CalZone {
  CalCluster c[CAL_MAX_CLUSTERS];
  uint8_t n{0};
  uint32_t total{0};

  static uint32_t gap(const CalCluster &a, const CalCluster &b) {  // distance between ranges (0 if they overlap)
    if (a.max < b.min)
      return b.min - a.max;
    if (b.max < a.min)
      return a.min - b.max;
    return 0;
  }

  void add(uint16_t v) {
    total++;
    int best = -1;
    uint32_t best_gap = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < n; i++) {
      uint32_t g = 0;
      if (v < c[i].min)
        g = c[i].min - v;
      else if (v > c[i].max)
        g = v - c[i].max;
      if (g <= CAL_CLUSTER_GAP_CM && g < best_gap) {
        best = i;
        best_gap = g;
      }
    }
    if (best >= 0) {
      c[best].add(v);
      return;
    }
    if (n == CAL_MAX_CLUSTERS) {  // no free slot: merge the two closest clusters
      uint8_t bi = 0, bj = 1;
      uint32_t bg = 0xFFFFFFFFu;
      for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
          const uint32_t g = gap(c[i], c[j]);
          if (g < bg) {
            bg = g;
            bi = i;
            bj = j;
          }
        }
      }
      c[bi].merge(c[bj]);
      c[bj] = c[n - 1];
      n--;
    }
    c[n] = CalCluster();
    c[n].add(v);
    n++;
  }
};

struct CalCandidate {
  uint16_t ref;
  uint32_t count;
  bool noise;      // below the noise share
  bool selected;   // part of the suggested references
};

struct CalResult {
  uint8_t count{0};  // suggested references (0-4)
  uint16_t ref[ENV_MAX_REFS]{};
  uint16_t delta[ENV_MAX_REFS]{};
  CalCandidate cand[CAL_MAX_CLUSTERS]{};  // everything that was observed, for the logs
  uint8_t cand_count{0};
  uint32_t samples{0};
  bool wide{false};          // some suggested band has delta >= CAL_WIDE_BAND_DELTA_CM
  uint16_t wide_low_cm{0};   // lowest distance covered by a wide band
};

inline bool bands_touch(const CalCluster &a, const CalCluster &b) {
  const int32_t d = static_cast<int32_t>(a.ref()) - static_cast<int32_t>(b.ref());
  const int32_t ad = d < 0 ? -d : d;
  return ad <= static_cast<int32_t>(a.delta()) + static_cast<int32_t>(b.delta());
}

// Turns the clusters of a zone into at most ENV_MAX_REFS references.
inline void cal_finalize(const CalZone &zone, CalResult &out) {
  out = CalResult();
  out.samples = zone.total;
  CalCluster cl[CAL_MAX_CLUSTERS];
  uint8_t n = zone.n;
  for (uint8_t i = 0; i < n; i++)
    cl[i] = zone.c[i];
  // sort by minimum (insertion sort, n <= 8)
  for (uint8_t i = 1; i < n; i++) {
    CalCluster key = cl[i];
    int j = i - 1;
    while (j >= 0 && cl[j].min > key.min) {
      cl[j + 1] = cl[j];
      j--;
    }
    cl[j + 1] = key;
  }
  // merge clusters whose ranges are closer than the cluster gap
  for (bool again = true; again;) {
    again = false;
    for (uint8_t i = 0; i + 1 < n; i++) {
      if (CalZone::gap(cl[i], cl[i + 1]) <= CAL_CLUSTER_GAP_CM) {
        cl[i].merge(cl[i + 1]);
        for (uint8_t k = i + 1; k + 1 < n; k++)
          cl[k] = cl[k + 1];
        n--;
        again = true;
        break;
      }
    }
  }
  // noise: clusters holding less than CAL_NOISE_PERCENT of the samples
  bool keep[CAL_MAX_CLUSTERS];
  uint8_t kept = 0;
  for (uint8_t i = 0; i < n; i++) {
    keep[i] = static_cast<uint64_t>(cl[i].count) * 100 >= static_cast<uint64_t>(zone.total) * CAL_NOISE_PERCENT;
    kept += keep[i] ? 1 : 0;
  }
  // gather survivors, merge those whose tolerance bands overlap
  CalCluster sv[CAL_MAX_CLUSTERS];
  uint8_t sn = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (keep[i])
      sv[sn++] = cl[i];
  }
  for (bool again = true; again;) {
    again = false;
    for (uint8_t i = 0; i < sn && !again; i++) {
      for (uint8_t j = i + 1; j < sn; j++) {
        if (bands_touch(sv[i], sv[j])) {
          sv[i].merge(sv[j]);
          for (uint8_t k = j; k + 1 < sn; k++)
            sv[k] = sv[k + 1];
          sn--;
          again = true;
          break;
        }
      }
    }
  }
  // select the four most frequent survivors
  bool chosen[CAL_MAX_CLUSTERS] = {};
  uint8_t nsel = sn < ENV_MAX_REFS ? sn : ENV_MAX_REFS;
  for (uint8_t s = 0; s < nsel; s++) {
    int best = -1;
    for (uint8_t i = 0; i < sn; i++) {
      if (!chosen[i] && (best < 0 || sv[i].count > sv[best].count))
        best = i;
    }
    chosen[best] = true;
  }
  // references sorted by distance
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < sn; i++) {
    if (!chosen[i])
      continue;
    uint8_t pos = cnt++;
    while (pos > 0 && out.ref[pos - 1] > sv[i].ref()) {
      out.ref[pos] = out.ref[pos - 1];
      out.delta[pos] = out.delta[pos - 1];
      pos--;
    }
    out.ref[pos] = sv[i].ref();
    out.delta[pos] = sv[i].delta();
  }
  out.count = cnt;
  for (uint8_t k = 0; k < out.count; k++) {
    if (out.delta[k] >= CAL_WIDE_BAND_DELTA_CM) {
      const uint16_t low = out.ref[k] > out.delta[k] ? out.ref[k] - out.delta[k] : 0;
      if (!out.wide || low < out.wide_low_cm)
        out.wide_low_cm = low;
      out.wide = true;
    }
  }
  // candidates for the logs: every observed cluster after the gap merge
  for (uint8_t i = 0; i < n && out.cand_count < CAL_MAX_CLUSTERS; i++) {
    CalCandidate &c = out.cand[out.cand_count++];
    c.ref = cl[i].ref();
    c.count = cl[i].count;
    c.noise = !keep[i];
    c.selected = false;
    for (uint8_t k = 0; k < out.count; k++) {
      if (cl[i].ref() >= out.ref[k] - (out.delta[k] < out.ref[k] ? out.delta[k] : out.ref[k]) &&
          cl[i].ref() <= out.ref[k] + out.delta[k])
        c.selected = true;
    }
  }
}

// "zone>ref/delta,ref/delta" (no trailing separator); "zone>" when there is none.
inline size_t cal_format_line(char *buf, size_t cap, uint8_t shown_zone, const CalResult &r) {
  size_t pos = 0;
  auto put_uint = [&](uint32_t v) {
    char tmp[10];
    uint8_t k = 0;
    do {
      tmp[k++] = static_cast<char>('0' + v % 10);
      v /= 10;
    } while (v);
    while (k && pos + 1 < cap)
      buf[pos++] = tmp[--k];
  };
  put_uint(shown_zone);
  if (pos + 1 < cap)
    buf[pos++] = '>';
  for (uint8_t i = 0; i < r.count; i++) {
    if (i > 0 && pos + 1 < cap)
      buf[pos++] = ',';
    put_uint(r.ref[i]);
    if (pos + 1 < cap)
      buf[pos++] = '/';
    put_uint(r.delta[i]);
  }
  buf[pos] = '\0';
  return pos;
}

}  // namespace vl53l5cx
}  // namespace esphome
