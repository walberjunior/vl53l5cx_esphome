#include "vl53l5cx.h"

#include <cmath>
#include <cstring>
#include <new>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vl53l5cx {

static const char *const TAG = "vl53l5cx";

static constexpr uint32_t FAULT_RETRY_MS = 15000;        // time before a full re-initialisation is attempted
static constexpr uint32_t APPLY_DEBOUNCE_MS = 200;       // coalesces rapid UI changes into one restart
static constexpr uint32_t TEMPERATURE_REFRESH_MS = 60000;  // temperature is re-published at least this often
static constexpr uint32_t MAX_VHV_SECONDS = 86400;
static constexpr uint32_t PREF_KEY_ENV_BASE = 0x4C354E56U;  // + I2C address
static constexpr uint32_t PREF_KEY_ENV2_BASE = 0x4C354E57U;  // + I2C address (settings version 2)
static constexpr uint8_t PREF_ENV_VERSION = 2;
static constexpr uint32_t CAL_MIN_FRAMES = 5;
static constexpr uint8_t CAL_REPORT_LINES_PER_LOOP = 4;
static constexpr uint32_t PREF_KEY_BASE = 0x4C353543U;  // + I2C address
static constexpr uint8_t PREF_VERSION = 1;

static const char *const MATRIX_LABELS[MATRIX_COUNT] = {
    "Ambient Per SPAD RAW [Kcps/SPAD]",
    "Number of Targets Detected [targets, processed]",
    "Number of SPADs Enabled RAW [SPADs]",
    "Signal Per SPAD RAW [Kcps/SPAD]",
    "Range Sigma RAW [mm]",
    "Target Status RAW [ULD code]",
    "Reflectance RAW [%]",
    "Motion Indicator RAW [ULD units]",
    "Distance RAW [cm]",
    "Distance [cm, processed]",
    "Number of Targets Detected RAW [targets]",
};
static const char *const ROTATION_LABELS[4] = {"0", "90", "180", "270"};
static const char *const FILTER_LABELS[5] = {"Disabled", "Range Valid", "Permissive", "High Confidence",
                                             "Confidence Based"};
static const char *mode_label(uint8_t mode) {
  return mode == VL53L5CX_RANGING_MODE_CONTINUOUS ? "Continuous" : "Autonomous";
}
static const char *order_label(uint8_t order) {
  return order == VL53L5CX_TARGET_ORDER_CLOSEST ? "Closest" : "Strongest";
}

static bool same_config(const SensorConfig &a, const SensorConfig &b) {
  return a.resolution == b.resolution && a.frequency_hz == b.frequency_hz && a.mode == b.mode &&
         a.integration_ms == b.integration_ms && a.sharpener == b.sharpener && a.target_order == b.target_order &&
         a.vhv_seconds == b.vhv_seconds;
}

// ---------------------------------------------------------------------------
// Setup / lifecycle
// ---------------------------------------------------------------------------
void VL53L5CXComponent::setup() {
  this->tx_buffer_ = new (std::nothrow) uint8_t[this->chunk_size_ + 2];
  if (this->tx_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate the I2C transfer buffer");
    this->mark_failed();
    return;
  }
  this->dev_.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
  this->dev_.platform.i2c_device = static_cast<i2c::I2CDevice *>(this);
  this->dev_.platform.tx_buffer = this->tx_buffer_;
  this->dev_.platform.chunk_size = this->chunk_size_;

  if (this->lpn_pin_ != nullptr)
    this->lpn_pin_->setup();
  if (this->i2c_reset_pin_ != nullptr) {
    this->i2c_reset_pin_->setup();
    this->i2c_reset_pin_->digital_write(false);  // I2C_RST must be low (UM2884 4.1)
  }
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    this->store_.pin = this->interrupt_pin_->to_isr();
    this->interrupt_pin_->attach_interrupt(&VL53L5CXStore::gpio_intr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);
  }

  this->pref_ = global_preferences->make_preference<PersistedSettings>(PREF_KEY_BASE + this->address_);
  this->restore_settings_();
  this->env_pref_ = global_preferences->make_preference<PersistedEnvSettings>(PREF_KEY_ENV2_BASE + this->address_);
  this->env_pref_v1_ = global_preferences->make_preference<PersistedEnvSettingsV1>(PREF_KEY_ENV_BASE + this->address_);
  this->restore_env_settings_();
  this->build_environment_();
  this->rebuild_rotation_maps_();
  this->publish_config_states_();
  for (uint8_t i = 0; i < MATRIX_COUNT; i++) {
    if (this->logging_switch_[i] != nullptr)
      this->logging_switch_[i]->publish_state(false);  // logging is OFF after boot
  }
  if (this->motion_debug_switch_ != nullptr)
    this->motion_debug_switch_->publish_state(false);

  if (!this->bring_up_()) {
    // fault_() has already logged and scheduled a retry.
    return;
  }
}

void VL53L5CXComponent::power_cycle_() {
  if (this->lpn_pin_ != nullptr) {
    this->lpn_pin_->digital_write(false);
    VL53L5CX_WaitMs(&this->dev_.platform, 10);
    this->lpn_pin_->digital_write(true);
    VL53L5CX_WaitMs(&this->dev_.platform, 10);
  }
}

bool VL53L5CXComponent::bring_up_() {
  this->state_ = State::INIT;
  this->frame_valid_ = false;
  this->dirty_ = false;
  this->power_cycle_();

  uint8_t alive = 0;
  uint8_t status = vl53l5cx_is_alive(&this->dev_, &alive);
  if (status != 0 || alive == 0) {
    this->fault_("Sensor not detected on the I2C bus", status != 0 ? status : VL53L5CX_STATUS_ERROR);
    return false;
  }
  ESP_LOGD(TAG, "Sensor detected, loading firmware (blocking, several seconds at low I2C speeds)...");
  status = vl53l5cx_init(&this->dev_);
  if (status != 0) {
    this->fault_("vl53l5cx_init failed", status);
    return false;
  }
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  if (this->motion_enabled_) {
    if (!this->check_(vl53l5cx_motion_indicator_init(&this->dev_, &this->motion_cfg_, this->desired_.resolution),
                      "motion_indicator_init") ||
        !this->check_(vl53l5cx_motion_indicator_set_distance_motion(&this->dev_, &this->motion_cfg_,
                                                                    this->motion_min_mm_, this->motion_max_mm_),
                      "motion_indicator_set_distance_motion")) {
      this->fault_("Motion indicator initialisation failed", this->last_error_);
      return false;
    }
  }
#endif
  this->state_ = State::READY;
  if (!this->apply_config_(this->desired_, true)) {
    this->fault_("Initial configuration failed", this->last_error_);
    return false;
  }
  this->status_clear_error();
  ESP_LOGI(TAG, "Ranging started: %ux%u, %u Hz, %s mode", matrix_size(this->applied_.resolution),
           matrix_size(this->applied_.resolution), (unsigned) this->applied_.frequency_hz,
           this->applied_.mode == VL53L5CX_RANGING_MODE_CONTINUOUS ? "continuous" : "autonomous");
  return true;
}

bool VL53L5CXComponent::check_(uint8_t status, const char *what) {
  if (status == 0)
    return true;
  ESP_LOGE(TAG, "%s failed (ULD status %u)", what, (unsigned) status);
  this->set_driver_error_(status);
  return false;
}

void VL53L5CXComponent::set_driver_error_(uint8_t status) {
  if (status != this->last_error_)
    ESP_LOGW(TAG, "Driver error code changed: %u -> %u", (unsigned) this->last_error_, (unsigned) status);
  if ((status != this->last_error_ || !this->error_published_) && this->driver_error_sensor_ != nullptr) {
    this->driver_error_sensor_->publish_state(status);
    this->error_published_ = true;
  }
  this->last_error_ = status;
}

void VL53L5CXComponent::fault_(const char *what, uint8_t status) {
  if (this->calib_active_ || this->calib_pending_)
    this->calib_abort_("sensor fault");
  ESP_LOGE(TAG, "%s (ULD status %u). Retrying full initialisation in %u s", what, (unsigned) status,
           (unsigned) (FAULT_RETRY_MS / 1000));
  this->set_driver_error_(status != 0 ? status : VL53L5CX_STATUS_ERROR);
  this->state_ = State::FAULT;
  this->fault_at_ = millis();
  this->frame_valid_ = false;
  this->high_freq_.stop();
  this->status_set_error();
}

// Stop -> (resolution) -> frequency -> mode -> integration -> sharpener ->
// target order -> VHV -> start. Resolution first because the valid frequency
// range depends on it (UM2884 4.3).
bool VL53L5CXComponent::apply_config_(const SensorConfig &c, bool with_resolution) {
  if (this->calib_active_)
    this->calib_abort_("the sensor configuration is being changed");
  if (this->state_ == State::RANGING) {
    if (!this->check_(vl53l5cx_stop_ranging(&this->dev_), "stop_ranging"))
      return false;
    this->state_ = State::READY;
  }
  this->frame_valid_ = false;
  this->high_freq_.stop();

  bool ok = true;
  if (with_resolution) {
    ok = this->check_(vl53l5cx_set_resolution(&this->dev_, c.resolution), "set_resolution");
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    if (ok && this->motion_enabled_) {
      ok = this->check_(vl53l5cx_motion_indicator_set_resolution(&this->dev_, &this->motion_cfg_, c.resolution),
                        "motion_indicator_set_resolution");
    }
#endif
  }
  ok = ok && this->check_(vl53l5cx_set_ranging_frequency_hz(&this->dev_, c.frequency_hz), "set_ranging_frequency_hz");
  ok = ok && this->check_(vl53l5cx_set_ranging_mode(&this->dev_, c.mode), "set_ranging_mode");
  ok = ok && this->check_(vl53l5cx_set_integration_time_ms(&this->dev_, c.integration_ms), "set_integration_time_ms");
  ok = ok && this->check_(vl53l5cx_set_sharpener_percent(&this->dev_, c.sharpener), "set_sharpener_percent");
  ok = ok && this->check_(vl53l5cx_set_target_order(&this->dev_, c.target_order), "set_target_order");
  ok = ok && this->check_(vl53l5cx_set_VHV_repeat_count(&this->dev_, vhv_repeat_frames(c.vhv_seconds, c.frequency_hz)),
                          "set_VHV_repeat_count");
  ok = ok && this->check_(vl53l5cx_start_ranging(&this->dev_), "start_ranging");
  if (!ok)
    return false;

  this->applied_ = c;
  this->state_ = State::RANGING;
  this->store_.data_ready = false;
  this->last_frame_ms_ = millis();
  this->last_poll_ms_ = this->last_frame_ms_;
  this->int_watchdog_warned_ = false;
  if (c.frequency_hz >= 10)
    this->high_freq_.start();
  this->set_driver_error_(0);
  return true;
}

void VL53L5CXComponent::apply_pending_() {
  this->dirty_ = false;
  const SensorConfig previous = this->applied_;
  const bool resolution_changed = this->desired_.resolution != previous.resolution;
  ESP_LOGD(TAG, "Applying configuration (ranging restart)");
  if (this->apply_config_(this->desired_, resolution_changed))
    return;

  ESP_LOGE(TAG, "Configuration rejected by the sensor, restoring the previous configuration");
  this->desired_ = previous;
  this->publish_config_states_();
  this->save_settings_();  // never keep a configuration the sensor rejected
  if (!this->apply_config_(previous, true))
    this->fault_("Could not restore the previous configuration", this->last_error_);
}

void VL53L5CXComponent::loop() {
  const uint32_t now = millis();
  if (this->report_pos_ < this->report_.size())
    this->calib_report_step_();
  if (this->calib_active_) {
    if (this->state_ != State::RANGING) {
      this->calib_abort_("ranging stopped");
    } else if (static_cast<int32_t>(now - this->calib_end_ms_) >= 0) {
      this->calib_finish_();
    } else if (static_cast<int32_t>(now - this->calib_next_progress_ms_) >= 0) {
      ESP_LOGI(TAG, "Calibration: %u/%u s, %u frames collected. Keep the passage completely free.",
               (unsigned) ((now - this->calib_start_ms_) / 1000), (unsigned) this->calib_duration_s_,
               (unsigned) this->calib_frames_);
      this->calib_next_progress_ms_ = now + 10000;
    }
  }
  if (this->state_ == State::FAULT) {
    if (now - this->fault_at_ >= FAULT_RETRY_MS)
      this->bring_up_();
    return;
  }
  if (this->state_ != State::RANGING)
    return;
  if (this->dirty_ && static_cast<int32_t>(now - this->apply_at_) >= 0) {
    this->apply_pending_();
    if (this->state_ != State::RANGING)
      return;
  }
  if (this->calib_pending_ && !this->dirty_) {  // the switch to 1 Hz has been applied (or failed)
    this->calib_pending_ = false;
    if (this->applied_.frequency_hz == 1) {
      this->calib_begin_();
    } else {
      ESP_LOGW(TAG, "Calibration cancelled: the sensor could not switch to 1 Hz");
      this->calib_restore_frequency_();
    }
  }
  this->service_ranging_(now);
}

// ---------------------------------------------------------------------------
// Ranging service: acquisition, per-frame processing, publication
// ---------------------------------------------------------------------------
void VL53L5CXComponent::service_ranging_(uint32_t now) {
  const uint32_t period = this->period_ms_();
  bool ready = false;

  auto poll = [this]() -> bool {
    uint8_t is_ready = 0;
    const uint8_t st = vl53l5cx_check_data_ready(&this->dev_, &is_ready);
    if (st != 0) {
      this->set_driver_error_(st);
      return false;
    }
    return is_ready != 0;
  };

  if (this->interrupt_pin_ != nullptr) {
    if (this->store_.data_ready) {
      this->store_.data_ready = false;
      ready = true;
    } else if (now - this->last_frame_ms_ > 3 * period + 100 && now - this->last_poll_ms_ >= period) {
      // Safety net if the INT line is not wired correctly: poll once per period.
      this->last_poll_ms_ = now;
      ready = poll();
      if (ready && !this->int_watchdog_warned_) {
        ESP_LOGW(TAG, "Data was ready but no interrupt was seen on the INT pin; check wiring/pull-up");
        this->int_watchdog_warned_ = true;
      }
    }
  } else {
    uint32_t poll_interval = period / 8;
    if (poll_interval < 2)
      poll_interval = 2;
    if (poll_interval > 50)
      poll_interval = 50;
    if (now - this->last_poll_ms_ >= poll_interval) {
      this->last_poll_ms_ = now;
      ready = poll();
    }
  }

  if (ready)
    this->read_frame_(now);

  if (this->frame_valid_ && this->scheduler_.due(now, this->frame_seq_)) {
    this->publish_matrices_();
    this->scheduler_.mark(now, this->frame_seq_);
  }
}

void VL53L5CXComponent::read_frame_(uint32_t now) {
  const uint8_t status = vl53l5cx_get_ranging_data(&this->dev_, &this->results_);
  if (status != 0) {
    // Includes corrupted frames (2) and I2C failures (255): the frame is discarded.
    this->set_driver_error_(status);
    return;
  }
  this->set_driver_error_(0);
  this->frame_n_ = matrix_size(this->applied_.resolution);
  if (this->frame_n_ != 8 && this->env_.any && this->env_enabled_ && !this->env_4x4_warned_) {
    ESP_LOGW(TAG, "The environment filter only works in 8x8; it is bypassed while running 4x4");
    this->env_4x4_warned_ = true;
  }
  this->frame_seq_++;
  this->frame_valid_ = true;
  this->last_frame_ms_ = now;

  this->publish_temperature_(now);
  if (this->calib_active_)
    this->calib_accumulate_();

  // Per-frame logging at the hardware rate, independent of publication. A
  // matrix is only formatted when its switch is ON.
  for (uint8_t i = 0; i < MATRIX_COUNT; i++) {
    if (this->log_enabled_[i])
      this->log_matrix_(static_cast<MatrixId>(i));
  }
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  if (this->debug_motion_)
    this->log_motion_debug_();
#endif
}

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
// Debug aid (switch `motion_indicator_raw_debug_logging`): dumps the complete ULD motion
// structure exactly as delivered. The ULD has already divided motion[] by 65535
// (raw format is not enabled); global_indicator_1/2 are not divided by the ULD.
void VL53L5CXComponent::log_motion_debug_() {
  const auto &m = this->results_.motion_indicator;
  ESP_LOGI(TAG,
           "motion-debug seq=%u res=%ux%u mode=%s freq=%uHz int=%ums sharp=%u%% order=%s vhv=%us range=%u-%umm | "
           "status=%u aggregates=%u detected=%u global1=%u global2=%u",
           (unsigned) this->frame_seq_, (unsigned) this->frame_n_, (unsigned) this->frame_n_,
           mode_label(this->applied_.mode), (unsigned) this->applied_.frequency_hz,
           (unsigned) this->applied_.integration_ms, (unsigned) this->applied_.sharpener,
           order_label(this->applied_.target_order), (unsigned) this->applied_.vhv_seconds,
           (unsigned) this->motion_min_mm_, (unsigned) this->motion_max_mm_, (unsigned) m.status, (unsigned) m.nb_of_aggregates,
           (unsigned) m.nb_of_detected_aggregates, (unsigned) m.global_indicator_1, (unsigned) m.global_indicator_2);
  for (uint8_t half = 0; half < 2; half++) {
    char line[16 * 11 + 1];
    size_t pos = 0;
    for (uint8_t i = 0; i < 16; i++) {
      const int n = snprintf(line + pos, sizeof(line) - pos, i == 0 ? "%u" : ",%u", (unsigned) m.motion[half * 16 + i]);
      if (n < 0 || static_cast<size_t>(n) >= sizeof(line) - pos)
        break;
      pos += static_cast<size_t>(n);
    }
    ESP_LOGI(TAG, "motion-debug seq=%u motion[%u..%u] (as delivered, unrotated): %s", (unsigned) this->frame_seq_,
             (unsigned) (half * 16), (unsigned) (half * 16 + 15), line);
  }
}
#endif  // VL53L5CX_DISABLE_MOTION_INDICATOR

void VL53L5CXComponent::publish_temperature_(uint32_t now) {
  if (this->temperature_sensor_ == nullptr)
    return;
  const int16_t t = this->results_.silicon_temp_degc;
  if (t != this->last_temperature_ || now - this->last_temperature_ms_ >= TEMPERATURE_REFRESH_MS) {
    this->temperature_sensor_->publish_state(static_cast<float>(t));
    this->last_temperature_ = t;
    this->last_temperature_ms_ = now;
  }
}

size_t VL53L5CXComponent::format_(MatrixId id, uint8_t target, char *buf, size_t cap) const {
  const uint8_t n = (id == MatrixId::MOTION_INDICATOR) ? 4 : this->frame_n_;  // motion is always a 4x4 grid
  const RotationMap &map = (n == 4) ? this->map4_ : this->map8_;
  const VL53L5CX_ResultsData &r = this->results_;
  const ProcessingContext ctx = this->make_context_();
  return format_matrix(buf, cap, n, map, [&](uint8_t zone) { return get_cell(id, r, zone, target, ctx); });
}

void VL53L5CXComponent::publish_matrices_() {
  for (uint8_t m = 0; m < MATRIX_COUNT; m++) {
    const MatrixId id = static_cast<MatrixId>(m);
    const uint8_t targets = is_per_target(id) ? TARGETS_PER_ZONE : 1;
    for (uint8_t t = 0; t < targets; t++) {
      text_sensor::TextSensor *ts = this->matrix_ts_[m][t];
      if (ts == nullptr)
        continue;  // nothing is formatted for matrices without an entity
      const size_t len = this->format_(id, t, this->text_buf_, MATRIX_TEXT_BUFFER_SIZE);
      ts->publish_state(this->text_buf_, len);
    }
  }
}

void VL53L5CXComponent::log_matrix_(MatrixId id) {
  const uint8_t m = static_cast<uint8_t>(id);
  const uint8_t targets = is_per_target(id) ? TARGETS_PER_ZONE : 1;
  const uint8_t n = (id == MatrixId::MOTION_INDICATOR) ? 4 : this->frame_n_;
  for (uint8_t t = 0; t < targets; t++) {
    this->format_(id, t, this->text_buf_, MATRIX_TEXT_BUFFER_SIZE);
    if (is_per_target(id)) {
      ESP_LOGI(TAG, "seq=%u t=%ums %ux%u rot=%s %s T%u: %s", (unsigned) this->frame_seq_, (unsigned) millis(),
               (unsigned) n, (unsigned) n, ROTATION_LABELS[static_cast<uint8_t>(this->rotation_)], MATRIX_LABELS[m],
               (unsigned) (t + 1), this->text_buf_);
    } else {
      ESP_LOGI(TAG, "seq=%u t=%ums %ux%u rot=%s %s: %s", (unsigned) this->frame_seq_, (unsigned) millis(),
               (unsigned) n, (unsigned) n, ROTATION_LABELS[static_cast<uint8_t>(this->rotation_)], MATRIX_LABELS[m],
               this->text_buf_);
    }
  }
}

void VL53L5CXComponent::rebuild_rotation_maps_() {
  this->map4_.build(4, this->rotation_);
  this->map8_.build(8, this->rotation_);
}

// ---------------------------------------------------------------------------
// Requests from entities
// ---------------------------------------------------------------------------
void VL53L5CXComponent::commit_(const SensorConfig &cfg, bool persist) {
  if (same_config(cfg, this->desired_)) {
    this->publish_config_states_();
    return;
  }
  const SensorConfig &o = this->desired_;
  if (cfg.resolution != o.resolution)
    ESP_LOGI(TAG, "Resolution: %ux%u -> %ux%u", matrix_size(o.resolution), matrix_size(o.resolution),
             matrix_size(cfg.resolution), matrix_size(cfg.resolution));
  if (cfg.mode != o.mode)
    ESP_LOGI(TAG, "Ranging Mode: %s -> %s", mode_label(o.mode), mode_label(cfg.mode));
  if (cfg.target_order != o.target_order)
    ESP_LOGI(TAG, "Target Order: %s -> %s", order_label(o.target_order), order_label(cfg.target_order));
  if (cfg.frequency_hz != o.frequency_hz)
    ESP_LOGI(TAG, "Ranging Frequency: %u Hz -> %u Hz", (unsigned) o.frequency_hz, (unsigned) cfg.frequency_hz);
  if (cfg.integration_ms != o.integration_ms)
    ESP_LOGI(TAG, "Integration Time: %u ms -> %u ms", (unsigned) o.integration_ms, (unsigned) cfg.integration_ms);
  if (cfg.sharpener != o.sharpener)
    ESP_LOGI(TAG, "Sharpener: %u %% -> %u %%", (unsigned) o.sharpener, (unsigned) cfg.sharpener);
  if (cfg.vhv_seconds != o.vhv_seconds)
    ESP_LOGI(TAG, "Periodic Temperature Compensation: %u s -> %u s", (unsigned) o.vhv_seconds,
             (unsigned) cfg.vhv_seconds);
  this->desired_ = cfg;
  this->dirty_ = true;
  this->apply_at_ = millis() + APPLY_DEBOUNCE_MS;
  this->publish_config_states_();
  if (persist)
    this->save_settings_();
}

void VL53L5CXComponent::publish_config_states_() {
  if (this->resolution_select_ != nullptr)
    this->resolution_select_->publish_state(static_cast<size_t>(this->desired_.resolution == RES_8X8 ? 1 : 0));
  if (this->mode_select_ != nullptr)
    this->mode_select_->publish_state(
        static_cast<size_t>(this->desired_.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS ? 1 : 0));
  if (this->order_select_ != nullptr)
    this->order_select_->publish_state(
        static_cast<size_t>(this->desired_.target_order == VL53L5CX_TARGET_ORDER_STRONGEST ? 1 : 0));
  if (this->filter_select_ != nullptr)
    this->filter_select_->publish_state(static_cast<size_t>(this->filter_));
  if (this->rotation_select_ != nullptr)
    this->rotation_select_->publish_state(static_cast<size_t>(this->rotation_));
  if (this->frequency_number_ != nullptr)
    this->frequency_number_->publish_state(this->desired_.frequency_hz);
  if (this->integration_number_ != nullptr)
    this->integration_number_->publish_state(this->desired_.integration_ms);
  if (this->sharpener_number_ != nullptr)
    this->sharpener_number_->publish_state(this->desired_.sharpener);
  if (this->interval_number_ != nullptr)
    this->interval_number_->publish_state(this->scheduler_.interval_ms);
  if (this->vhv_number_ != nullptr)
    this->vhv_number_->publish_state(this->desired_.vhv_seconds);
  if (this->max_distance_number_ != nullptr)
    this->max_distance_number_->publish_state(this->max_distance_cm_);
  if (this->min_distance_number_ != nullptr)
    this->min_distance_number_->publish_state(this->min_distance_cm_);
  if (this->calib_duration_number_ != nullptr)
    this->calib_duration_number_->publish_state(this->calib_duration_s_);
  if (this->env_switch_ != nullptr)
    this->env_switch_->publish_state(this->env_enabled_);
}

void VL53L5CXComponent::request_resolution(size_t index) {
  if (this->calib_blocks_("Resolution"))
    return;
  SensorConfig c = this->desired_;
  c.resolution = index == 0 ? RES_4X4 : RES_8X8;
  // The ranging frequency range depends on the resolution (UM2884 Table 2).
  if (c.frequency_hz > max_frequency_hz(c.resolution)) {
    ESP_LOGW(TAG, "Ranging Frequency reduced from %u Hz to %u Hz (maximum for %ux%u)", (unsigned) c.frequency_hz,
             (unsigned) max_frequency_hz(c.resolution), matrix_size(c.resolution), matrix_size(c.resolution));
    c.frequency_hz = max_frequency_hz(c.resolution);
  }
  if (c.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS) {
    const uint32_t max_int = max_integration_ms(c.resolution, c.frequency_hz);
    if (c.integration_ms > max_int) {
      ESP_LOGW(TAG, "Integration Time reduced from %u ms to %u ms (limit for this resolution/frequency)",
               (unsigned) c.integration_ms, (unsigned) max_int);
      c.integration_ms = static_cast<uint16_t>(max_int);
    }
  }
  this->commit_(c);
}

void VL53L5CXComponent::request_ranging_mode(size_t index) {
  if (this->calib_blocks_("Ranging Mode"))
    return;
  SensorConfig c = this->desired_;
  c.mode = index == 0 ? VL53L5CX_RANGING_MODE_CONTINUOUS : VL53L5CX_RANGING_MODE_AUTONOMOUS;
  if (c.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS &&
      c.integration_ms > max_integration_ms(c.resolution, c.frequency_hz)) {
    ESP_LOGW(TAG, "Autonomous mode rejected: Integration Time (%u ms) too long for %u Hz; lower it first",
             (unsigned) c.integration_ms, (unsigned) c.frequency_hz);
    this->publish_config_states_();
    return;
  }
  this->commit_(c);
}

void VL53L5CXComponent::request_target_order(size_t index) {
  if (this->calib_blocks_("Target Order"))
    return;
  SensorConfig c = this->desired_;
  c.target_order = index == 0 ? VL53L5CX_TARGET_ORDER_CLOSEST : VL53L5CX_TARGET_ORDER_STRONGEST;
  this->commit_(c);
}

void VL53L5CXComponent::request_frequency(float hz) {
  if (this->calib_blocks_("Ranging Frequency"))
    return;
  SensorConfig c = this->desired_;
  const long v = lroundf(hz);
  if (v < 1 || v > max_frequency_hz(c.resolution)) {
    ESP_LOGW(TAG, "Ranging Frequency %ld Hz rejected: valid range for %ux%u is 1-%u Hz", v, matrix_size(c.resolution),
             matrix_size(c.resolution), (unsigned) max_frequency_hz(c.resolution));
    this->publish_config_states_();
    return;
  }
  c.frequency_hz = static_cast<uint8_t>(v);
  if (c.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS && c.integration_ms > max_integration_ms(c.resolution, v)) {
    ESP_LOGW(TAG, "Ranging Frequency %ld Hz rejected: Integration Time (%u ms x %u + 1 ms) must stay below the period",
             v, (unsigned) c.integration_ms, (unsigned) integration_slots(c.resolution));
    this->publish_config_states_();
    return;
  }
  this->commit_(c);
}

void VL53L5CXComponent::request_integration_time(float ms) {
  if (this->calib_blocks_("Integration Time"))
    return;
  SensorConfig c = this->desired_;
  const long v = lroundf(ms);
  if (v < 2 || v > 1000) {
    ESP_LOGW(TAG, "Integration Time %ld ms rejected: valid range is 2-1000 ms", v);
    this->publish_config_states_();
    return;
  }
  if (c.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS && v > static_cast<long>(max_integration_ms(c.resolution, c.frequency_hz))) {
    ESP_LOGW(TAG, "Integration Time %ld ms rejected: maximum is %u ms at %u Hz (%ux%u); sum of integration times + 1 ms "
                  "must stay below the period",
             v, (unsigned) max_integration_ms(c.resolution, c.frequency_hz), (unsigned) c.frequency_hz,
             matrix_size(c.resolution), matrix_size(c.resolution));
    this->publish_config_states_();
    return;
  }
  if (c.mode == VL53L5CX_RANGING_MODE_CONTINUOUS)
    ESP_LOGI(TAG, "Integration Time stored, but it only takes effect in Autonomous mode");
  c.integration_ms = static_cast<uint16_t>(v);
  this->commit_(c);
}

void VL53L5CXComponent::request_sharpener(float percent) {
  if (this->calib_blocks_("Sharpener"))
    return;
  SensorConfig c = this->desired_;
  const long v = lroundf(percent);
  if (v < 0 || v > 99) {
    ESP_LOGW(TAG, "Sharpener %ld %% rejected: valid range is 0-99 %%", v);
    this->publish_config_states_();
    return;
  }
  c.sharpener = static_cast<uint8_t>(v);
  this->commit_(c);
}

void VL53L5CXComponent::request_temperature_compensation(float seconds) {
  if (this->calib_blocks_("Periodic Temperature Compensation"))
    return;
  SensorConfig c = this->desired_;
  const long v = lroundf(seconds);
  if (v < 0 || v > static_cast<long>(MAX_VHV_SECONDS)) {
    ESP_LOGW(TAG, "Periodic Temperature Compensation %ld s rejected: valid range is 0-%u s (0 = disabled)", v,
             (unsigned) MAX_VHV_SECONDS);
    this->publish_config_states_();
    return;
  }
  c.vhv_seconds = static_cast<uint32_t>(v);
  this->commit_(c);
}

void VL53L5CXComponent::request_target_status_filter(size_t index) {
  if (index > static_cast<size_t>(FilterProfile::CONFIDENCE_BASED)) {
    this->publish_config_states_();
    return;
  }
  if (static_cast<size_t>(this->filter_) != index && this->calib_active_)
    this->calib_abort_("Target Status Filter changed");
  if (static_cast<size_t>(this->filter_) != index)
    ESP_LOGI(TAG, "Target Status Filter: %s -> %s", FILTER_LABELS[static_cast<size_t>(this->filter_)],
             FILTER_LABELS[index]);
  this->filter_ = static_cast<FilterProfile>(index);  // software only, no restart
  this->publish_config_states_();
  this->save_settings_();
}

void VL53L5CXComponent::request_matrix_rotation(size_t index) {
  if (index > static_cast<size_t>(Rotation::DEG_270)) {
    this->publish_config_states_();
    return;
  }
  if (static_cast<size_t>(this->rotation_) != index)
    ESP_LOGI(TAG, "Matrix Rotation: %s° -> %s°", ROTATION_LABELS[static_cast<size_t>(this->rotation_)],
             ROTATION_LABELS[index]);
  this->rotation_ = static_cast<Rotation>(index);  // software only, no restart
  this->rebuild_rotation_maps_();
  this->publish_config_states_();
  this->save_settings_();
}

void VL53L5CXComponent::request_matrix_update_interval(float ms) {
  const long v = lroundf(ms);
  if (v < 0 || v > 60000) {
    ESP_LOGW(TAG, "Matrix Update Interval %ld ms rejected: valid range is 0-60000 ms", v);
    this->publish_config_states_();
    return;
  }
  if (this->scheduler_.interval_ms != static_cast<uint32_t>(v))
    ESP_LOGI(TAG, "Matrix Update Interval: %u ms -> %u ms", (unsigned) this->scheduler_.interval_ms, (unsigned) v);
  this->scheduler_.interval_ms = static_cast<uint32_t>(v);  // publication only, ranging is untouched
  this->publish_config_states_();
  this->save_settings_();
}

void VL53L5CXComponent::request_logging(uint8_t matrix, bool enabled) {
  if (matrix >= MATRIX_COUNT)
    return;
  this->log_enabled_[matrix] = enabled;
  if (this->logging_switch_[matrix] != nullptr)
    this->logging_switch_[matrix]->publish_state(enabled);
}


// ---------------------------------------------------------------------------
// Persistence: every control except the logging switches survives a reboot.
// ---------------------------------------------------------------------------
void VL53L5CXComponent::save_settings_() {
  PersistedSettings s{};
  s.version = PREF_VERSION;
  s.resolution = this->desired_.resolution;
  // The 1 Hz used while calibrating is temporary and must never reach the flash.
  s.frequency_hz = this->calib_freq_overridden_ ? this->calib_saved_freq_ : this->desired_.frequency_hz;
  s.mode = this->desired_.mode;
  s.sharpener = this->desired_.sharpener;
  s.target_order = this->desired_.target_order;
  s.filter = static_cast<uint8_t>(this->filter_);
  s.rotation = static_cast<uint8_t>(this->rotation_);
  s.integration_ms = this->desired_.integration_ms;
  s.vhv_seconds = this->desired_.vhv_seconds;
  s.interval_ms = this->scheduler_.interval_ms;
  this->pref_.save(&s);  // written to flash by ESPHome's periodic preferences sync
}

void VL53L5CXComponent::restore_settings_() {
  PersistedSettings s{};
  if (!this->pref_.load(&s)) {
    ESP_LOGD(TAG, "No stored settings, using defaults");
    return;
  }
  const bool mode_ok = s.mode == VL53L5CX_RANGING_MODE_CONTINUOUS || s.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS;
  bool ok = s.version == PREF_VERSION && (s.resolution == RES_4X4 || s.resolution == RES_8X8) && mode_ok &&
            s.frequency_hz >= 1 && s.frequency_hz <= max_frequency_hz(s.resolution) && s.integration_ms >= 2 &&
            s.integration_ms <= 1000 && s.sharpener <= 99 &&
            (s.target_order == VL53L5CX_TARGET_ORDER_CLOSEST || s.target_order == VL53L5CX_TARGET_ORDER_STRONGEST) &&
            s.filter <= static_cast<uint8_t>(FilterProfile::CONFIDENCE_BASED) &&
            s.rotation <= static_cast<uint8_t>(Rotation::DEG_270) && s.vhv_seconds <= MAX_VHV_SECONDS &&
            s.interval_ms <= 60000;
  if (ok && s.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS &&
      s.integration_ms > max_integration_ms(s.resolution, s.frequency_hz))
    ok = false;
  if (!ok) {
    ESP_LOGW(TAG, "Stored settings are invalid or from another version, using defaults");
    return;
  }
  this->desired_.resolution = s.resolution;
  this->desired_.frequency_hz = s.frequency_hz;
  this->desired_.mode = s.mode;
  this->desired_.integration_ms = s.integration_ms;
  this->desired_.sharpener = s.sharpener;
  this->desired_.target_order = s.target_order;
  this->desired_.vhv_seconds = s.vhv_seconds;
  this->applied_ = this->desired_;
  this->filter_ = static_cast<FilterProfile>(s.filter);
  this->rotation_ = static_cast<Rotation>(s.rotation);
  this->scheduler_.interval_ms = s.interval_ms;
  ESP_LOGI(TAG, "Restored settings: %ux%u, %u Hz, %s, filter %u, rotation %s", matrix_size(s.resolution),
           matrix_size(s.resolution), (unsigned) s.frequency_hz,
           s.mode == VL53L5CX_RANGING_MODE_CONTINUOUS ? "continuous" : "autonomous", (unsigned) s.filter,
           ROTATION_LABELS[s.rotation]);
}

void VL53L5CXComponent::request_motion_debug_logging(bool enabled) {
  this->debug_motion_ = enabled;
  if (this->motion_debug_switch_ != nullptr)
    this->motion_debug_switch_->publish_state(enabled);
}

// ---------------------------------------------------------------------------
// Processed matrices, environment filter and calibration
// ---------------------------------------------------------------------------
ProcessingContext VL53L5CXComponent::make_context_() const {
  ProcessingContext ctx;
  ctx.filter = this->filter_;
  ctx.max_distance_cm = this->max_distance_cm_;
  ctx.min_distance_cm = this->min_distance_cm_;
  // The environment filter only exists for 8x8 in this version.
  ctx.env = (this->env_enabled_ && this->env_.any && this->frame_n_ == 8) ? &this->env_ : nullptr;
  return ctx;
}

void VL53L5CXComponent::build_environment_() {
  if (!this->env_configured_ || this->env_table_ == nullptr) {
    ESP_LOGI(TAG, "Environment file has no zone lines: the environment filter has nothing to apply");
    return;
  }
  env_build_runtime(this->env_, this->env_table_, static_cast<Rotation>(this->env_file_rotation_ & 3));
  unsigned zones = 0;
  for (uint8_t z = 0; z < ENV_ZONES; z++)
    zones += this->env_.zone[z].count > 0 ? 1 : 0;
  ESP_LOGI(TAG, "Environment file loaded: %u zones with references (zone numbers taken with rotation %s)", zones,
           ROTATION_LABELS[this->env_file_rotation_ & 3]);
  if (this->env_file_max_cm_ != 0 && this->env_file_max_cm_ != this->max_distance_cm_) {
    ESP_LOGW(TAG, "The environment file was calibrated with Maximum Distance %u cm but the current value is %u cm",
             (unsigned) this->env_file_max_cm_, (unsigned) this->max_distance_cm_);
  }
}

void VL53L5CXComponent::save_env_settings_() {
  PersistedEnvSettings s{};
  s.version = PREF_ENV_VERSION;
  s.environment_filter = this->env_enabled_ ? 1 : 0;
  s.max_distance_cm = this->max_distance_cm_;
  s.calibration_duration_s = this->calib_duration_s_;
  s.min_distance_cm = this->min_distance_cm_;
  this->env_pref_.save(&s);
}

void VL53L5CXComponent::restore_env_settings_() {
  PersistedEnvSettings s{};
  if (!this->env_pref_.load(&s)) {
    PersistedEnvSettingsV1 old{};  // settings stored by firmware without Minimum Distance
    if (!this->env_pref_v1_.load(&old))
      return;
    s.version = PREF_ENV_VERSION;
    s.environment_filter = old.environment_filter;
    s.max_distance_cm = old.max_distance_cm;
    s.calibration_duration_s = old.calibration_duration_s;
    s.min_distance_cm = this->min_distance_cm_;  // default
    ESP_LOGI(TAG, "Migrating the stored environment settings (Minimum Distance defaults to %u cm)",
             (unsigned) s.min_distance_cm);
  }
  if (s.version != PREF_ENV_VERSION || s.environment_filter > 1 || s.max_distance_cm < 1 ||
      s.max_distance_cm > ENV_MAX_DISTANCE_CM || s.calibration_duration_s < 10 || s.calibration_duration_s > 3600 ||
      s.min_distance_cm >= s.max_distance_cm || s.min_distance_cm > 100) {
    ESP_LOGW(TAG, "Stored environment settings are invalid, using defaults");
    return;
  }
  this->env_enabled_ = s.environment_filter != 0;
  this->max_distance_cm_ = s.max_distance_cm;
  this->min_distance_cm_ = s.min_distance_cm;
  this->calib_duration_s_ = s.calibration_duration_s;
  ESP_LOGI(TAG, "Restored: Minimum Distance %u cm, Maximum Distance %u cm, Environment Filter %s, Calibration Duration "
                "%u s",
           (unsigned) s.min_distance_cm, (unsigned) s.max_distance_cm, s.environment_filter ? "ON" : "OFF",
           (unsigned) s.calibration_duration_s);
}

void VL53L5CXComponent::request_maximum_distance(float cm) {
  const long v = lroundf(cm);
  if (v < 1 || v > ENV_MAX_DISTANCE_CM || v <= static_cast<long>(this->min_distance_cm_)) {
    ESP_LOGW(TAG, "Maximum Distance %ld cm rejected: valid range is %u-%u cm (it must stay above Minimum Distance)", v,
             (unsigned) this->min_distance_cm_ + 1, (unsigned) ENV_MAX_DISTANCE_CM);
    this->publish_config_states_();
    return;
  }
  if (static_cast<uint16_t>(v) != this->max_distance_cm_) {
    if (this->calib_active_)
      this->calib_abort_("Maximum Distance changed");
    ESP_LOGI(TAG, "Maximum Distance: %u cm -> %ld cm", (unsigned) this->max_distance_cm_, v);
    this->max_distance_cm_ = static_cast<uint16_t>(v);
    if (this->env_.any && this->env_file_max_cm_ != 0 && this->env_file_max_cm_ != this->max_distance_cm_) {
      ESP_LOGW(TAG, "The environment file was calibrated with Maximum Distance %u cm but the current value is %u cm",
               (unsigned) this->env_file_max_cm_, (unsigned) this->max_distance_cm_);
    }
  }
  this->publish_config_states_();
  this->save_env_settings_();
}

void VL53L5CXComponent::request_minimum_distance(float cm) {
  const long v = lroundf(cm);
  if (v < 0 || v > 100 || v >= static_cast<long>(this->max_distance_cm_)) {
    ESP_LOGW(TAG, "Minimum Distance %ld cm rejected: valid range is 0-%u cm (below Maximum Distance)", v,
             (unsigned) (this->max_distance_cm_ < 100 ? this->max_distance_cm_ - 1 : 100));
    this->publish_config_states_();
    return;
  }
  if (static_cast<uint16_t>(v) != this->min_distance_cm_) {
    if (this->calib_active_ || this->calib_pending_)
      this->calib_abort_("Minimum Distance changed");
    ESP_LOGI(TAG, "Minimum Distance: %u cm -> %ld cm", (unsigned) this->min_distance_cm_, v);
    this->min_distance_cm_ = static_cast<uint16_t>(v);
  }
  this->publish_config_states_();
  this->save_env_settings_();
}

void VL53L5CXComponent::request_calibration_duration(float seconds) {
  const long v = lroundf(seconds);
  if (v < 10 || v > 3600) {
    ESP_LOGW(TAG, "Calibration Duration %ld s rejected: valid range is 10-3600 s", v);
    this->publish_config_states_();
    return;
  }
  if (static_cast<uint16_t>(v) != this->calib_duration_s_)
    ESP_LOGI(TAG, "Calibration Duration: %u s -> %ld s", (unsigned) this->calib_duration_s_, v);
  this->calib_duration_s_ = static_cast<uint16_t>(v);
  this->publish_config_states_();
  this->save_env_settings_();
}

void VL53L5CXComponent::request_environment_filter(bool enabled) {
  if (enabled != this->env_enabled_)
    ESP_LOGI(TAG, "Environment Filter: %s -> %s", this->env_enabled_ ? "ON" : "OFF", enabled ? "ON" : "OFF");
  this->env_enabled_ = enabled;
  if (enabled && !this->env_.any)
    ESP_LOGI(TAG, "Environment Filter is ON but the environment file has no references; nothing is filtered");
  this->publish_config_states_();
  this->save_env_settings_();
}

bool VL53L5CXComponent::calib_blocks_(const char *what) {
  if (!this->calib_active_ && !this->calib_pending_)
    return false;
  ESP_LOGW(TAG, "%s change rejected: the empty-environment calibration is running (press Calibrate Empty Environment "
                "again to cancel it)", what);
  this->publish_config_states_();
  return true;
}

void VL53L5CXComponent::request_calibration() {
  if (this->calib_active_ || this->calib_pending_) {  // pressing the button again cancels
    this->calib_abort_("cancelled by the user");
    return;
  }
  if (this->report_pos_ < this->report_.size()) {
    ESP_LOGW(TAG, "The previous calibration report is still being printed");
    return;
  }
  if (this->state_ != State::RANGING || this->dirty_) {
    ESP_LOGW(TAG, "Calibration refused: the sensor is not ranging or a configuration change is pending");
    return;
  }
  if (this->applied_.resolution != RES_8X8) {
    ESP_LOGW(TAG, "Calibration refused: the empty-environment calibration only supports 8x8 (set Resolution to 8x8)");
    return;
  }
  if (this->desired_.frequency_hz != 1) {
    // Save the current frequency, calibrate at 1 Hz, restore it at the end. The 1 Hz is never persisted.
    this->calib_saved_freq_ = this->desired_.frequency_hz;
    this->calib_freq_overridden_ = true;
    this->calib_pending_ = true;
    ESP_LOGI(TAG, "Calibration: switching Ranging Frequency from %u Hz to 1 Hz (temporary; %u Hz is restored when the "
                  "calibration ends)",
             (unsigned) this->calib_saved_freq_, (unsigned) this->calib_saved_freq_);
    SensorConfig c = this->desired_;
    c.frequency_hz = 1;
    this->commit_(c, false);
    return;
  }
  this->calib_begin_();
}

void VL53L5CXComponent::calib_begin_() {
  this->calib_zones_ = new (std::nothrow) CalZone[ENV_ZONES];
  if (this->calib_zones_ == nullptr) {
    ESP_LOGE(TAG, "Calibration refused: not enough memory");
    this->calib_restore_frequency_();
    return;
  }
  const uint32_t now = millis();
  this->calib_active_ = true;
  this->calib_frames_ = 0;
  this->calib_start_ms_ = now;
  this->calib_end_ms_ = now + static_cast<uint32_t>(this->calib_duration_s_) * 1000u;
  this->calib_next_progress_ms_ = now + 10000;
  ESP_LOGI(TAG, "Environment calibration started: keep the passage COMPLETELY FREE for %u s (8x8, 1 Hz, Minimum "
                "Distance %u cm, Maximum Distance %u cm, filter %s)",
           (unsigned) this->calib_duration_s_, (unsigned) this->min_distance_cm_, (unsigned) this->max_distance_cm_,
           FILTER_LABELS[static_cast<size_t>(this->filter_)]);
}

void VL53L5CXComponent::calib_restore_frequency_() {
  if (!this->calib_freq_overridden_)
    return;
  this->calib_freq_overridden_ = false;
  SensorConfig c = this->desired_;
  if (c.frequency_hz == this->calib_saved_freq_)
    return;
  if (this->calib_saved_freq_ > max_frequency_hz(c.resolution) ||
      (c.mode == VL53L5CX_RANGING_MODE_AUTONOMOUS &&
       c.integration_ms > max_integration_ms(c.resolution, this->calib_saved_freq_))) {
    ESP_LOGW(TAG, "Could not restore Ranging Frequency %u Hz with the current settings; staying at %u Hz",
             (unsigned) this->calib_saved_freq_, (unsigned) c.frequency_hz);
    return;
  }
  ESP_LOGI(TAG, "Calibration ended: restoring Ranging Frequency to %u Hz", (unsigned) this->calib_saved_freq_);
  c.frequency_hz = this->calib_saved_freq_;
  this->commit_(c);  // the restored value equals the stored one
}

void VL53L5CXComponent::calib_accumulate_() {
  const ProcessingContext ctx{this->filter_, this->max_distance_cm_, this->min_distance_cm_,
                              nullptr};  // calibrate without the environment filter
  for (uint8_t z = 0; z < ENV_ZONES; z++) {
    const uint8_t nb = this->results_.nb_target_detected[z];
    bool any = false;
    for (uint8_t t = 0; t < TARGETS_PER_ZONE && t < nb; t++) {
      const uint32_t idx = static_cast<uint32_t>(z) * TARGETS_PER_ZONE + t;
      if (!status_accepted(ctx.filter, this->results_.target_status[idx]) ||
          mm_to_cm(this->results_.distance_mm[idx]) < ctx.min_distance_cm)
        continue;
      any = true;
      this->calib_zones_[z].add(limit_distance_cm(true, mm_to_cm(this->results_.distance_mm[idx]), ctx.max_distance_cm));
    }
    if (!any)
      this->calib_zones_[z].add(ctx.max_distance_cm);  // no valid target: the zone reads "limit"
  }
  this->calib_frames_++;
}

void VL53L5CXComponent::calib_abort_(const char *why) {
  if (!this->calib_active_ && !this->calib_pending_)
    return;
  ESP_LOGW(TAG, "Environment calibration aborted: %s", why);
  this->calib_active_ = false;
  this->calib_pending_ = false;
  delete[] this->calib_zones_;
  this->calib_zones_ = nullptr;
  this->calib_restore_frequency_();
}

void VL53L5CXComponent::calib_finish_() {
  this->calib_active_ = false;
  if (this->calib_frames_ < CAL_MIN_FRAMES) {
    ESP_LOGW(TAG, "Environment calibration failed: only %u frames were collected (use a longer duration)",
             (unsigned) this->calib_frames_);
    delete[] this->calib_zones_;
    this->calib_zones_ = nullptr;
    this->calib_restore_frequency_();
    return;
  }
  char line[192];
  this->report_.clear();
  this->report_pos_ = 0;
  snprintf(line, sizeof(line), "Environment calibration finished: %u frames in %u s at 1 Hz (8x8).",
           (unsigned) this->calib_frames_, (unsigned) this->calib_duration_s_);
  this->report_.emplace_back(line);
  this->report_.emplace_back("Copy the lines below (without the timestamp/level added by ESPHome) into "
                             "components/vl53l5cx/environment.conf, review them and recompile:");
  this->report_.emplace_back("#! resolution: 8x8");
  snprintf(line, sizeof(line), "#! rotation: %s", ROTATION_LABELS[static_cast<uint8_t>(this->rotation_)]);
  this->report_.emplace_back(line);
  snprintf(line, sizeof(line), "#! max_distance_cm: %u", (unsigned) this->max_distance_cm_);
  this->report_.emplace_back(line);
  snprintf(line, sizeof(line), "#! frames: %u", (unsigned) this->calib_frames_);
  this->report_.emplace_back(line);
  // Zones are numbered as displayed with the current Matrix Rotation (the same numbering as the matrices).
  for (uint8_t shown = 0; shown < ENV_ZONES; shown++) {
    const uint8_t physical = this->map8_.src[shown];
    CalResult res;
    cal_finalize(this->calib_zones_[physical], res);
    bool extra = res.cand_count > res.count;
    if (extra) {
      std::string c = "# zone " + std::to_string(shown) + ": observed";
      for (uint8_t i = 0; i < res.cand_count; i++) {
        snprintf(line, sizeof(line), "%s %u cm (n=%u%s)", i ? "," : "", (unsigned) res.cand[i].ref,
                 (unsigned) res.cand[i].count, res.cand[i].noise ? ", noise" : "");
        c += line;
      }
      this->report_.push_back(c);
    }
    if (res.wide) {
      snprintf(line, sizeof(line),
               "# zone %u: wide band, distances from %u cm up are ignored; check that no person can appear there",
               (unsigned) shown, (unsigned) res.wide_low_cm);
      this->report_.emplace_back(line);
    }
    cal_format_line(line, sizeof(line), shown, res);
    this->report_.emplace_back(line);
  }
  delete[] this->calib_zones_;
  this->calib_zones_ = nullptr;
  this->calib_restore_frequency_();
}

void VL53L5CXComponent::calib_report_step_() {
  for (uint8_t i = 0; i < CAL_REPORT_LINES_PER_LOOP && this->report_pos_ < this->report_.size(); i++)
    ESP_LOGI(TAG, "%s", this->report_[this->report_pos_++].c_str());
  if (this->report_pos_ >= this->report_.size()) {
    this->report_.clear();
    this->report_.shrink_to_fit();
    this->report_pos_ = 0;
  }
}

void VL53L5CXComponent::dump_config() {
#ifdef VL53L5CX_DISABLE_MOTION_INDICATOR
  const char *motion_note = " (compiled out)";
#else
  const char *motion_note = "";
#endif
  ESP_LOGCONFIG(TAG, "VL53L5CX:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  LPn Pin: ", this->lpn_pin_);
  LOG_PIN("  I2C_RST Pin: ", this->i2c_reset_pin_);
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  ESP_LOGCONFIG(TAG,
                "  ULD: %s\n"
                "  Targets per zone (compile time): %u\n"
                "  I2C chunk size: %u bytes\n"
                "  Motion indicator: %s (%u-%u mm)%s\n"
                "  Data-ready: %s",
                VL53L5CX_API_REVISION, (unsigned) TARGETS_PER_ZONE, (unsigned) this->chunk_size_,
                this->motion_enabled_ ? "enabled" : "disabled", (unsigned) this->motion_min_mm_,
                (unsigned) this->motion_max_mm_, motion_note, this->interrupt_pin_ != nullptr ? "INT pin" : "polling");
  unsigned zones_with_refs = 0;
  for (uint8_t z = 0; z < ENV_ZONES; z++)
    zones_with_refs += this->env_.zone[z].count > 0 ? 1 : 0;
  ESP_LOGCONFIG(TAG,
                "  Environment file: %s (zones with references: %u, zone numbers taken with rotation %s)\n"
                "  Environment filter: %s, Minimum Distance: %u cm, Maximum Distance: %u cm",
                this->env_configured_ ? "configured" : "empty (ignored)", zones_with_refs,
                ROTATION_LABELS[this->env_file_rotation_ & 3], this->env_enabled_ ? "ON" : "OFF",
                (unsigned) this->min_distance_cm_, (unsigned) this->max_distance_cm_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed");
}

}  // namespace vl53l5cx
}  // namespace esphome
