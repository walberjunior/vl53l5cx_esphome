#pragma once

#include "esphome/components/button/button.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <string>
#include <vector>

#include "vl53l5cx_cells.h"
#include "vl53l5cx_logic.h"
#include "vl53l5cx_uld.h"

namespace esphome {
namespace vl53l5cx {

/// Interrupt flag set by the A3/INT pin (falling edge).
struct VL53L5CXStore {
  volatile bool data_ready{false};
  ISRInternalGPIOPin pin;
  static void IRAM_ATTR gpio_intr(VL53L5CXStore *arg) { arg->data_ready = true; }
};

/// Settings that are written to the sensor (all require ranging to be stopped).
struct SensorConfig {
  uint8_t resolution{RES_4X4};
  uint8_t frequency_hz{1};
  uint8_t mode{VL53L5CX_RANGING_MODE_AUTONOMOUS};
  uint16_t integration_ms{5};
  uint8_t sharpener{14};
  uint8_t target_order{VL53L5CX_TARGET_ORDER_STRONGEST};
  uint32_t vhv_seconds{300};
};

/// Everything the user can change except the logging switches. Stored as one
/// preference blob and restored at boot.
struct PersistedSettings {
  uint8_t version;
  uint8_t resolution;
  uint8_t frequency_hz;
  uint8_t mode;
  uint8_t sharpener;
  uint8_t target_order;
  uint8_t filter;
  uint8_t rotation;
  uint16_t integration_ms;
  uint32_t vhv_seconds;
  uint32_t interval_ms;
};

/// Settings of the environment features, stored in their own preference so the
/// older settings block stays valid.
struct PersistedEnvSettings {
  uint8_t version;
  uint8_t environment_filter;
  uint16_t max_distance_cm;
  uint16_t calibration_duration_s;
  uint16_t min_distance_cm;  // added in version 2
};

/// Layout of version 1 (kept only to migrate settings stored by older firmware).
struct PersistedEnvSettingsV1 {
  uint8_t version;
  uint8_t environment_filter;
  uint16_t max_distance_cm;
  uint16_t calibration_duration_s;
};

class VL53L5CXComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- configuration (Python codegen) -------------------------------------
  void set_lpn_pin(GPIOPin *pin) { this->lpn_pin_ = pin; }
  void set_i2c_reset_pin(GPIOPin *pin) { this->i2c_reset_pin_ = pin; }
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }
  void set_i2c_chunk_size(uint16_t size) { this->chunk_size_ = size; }
  void set_motion_indicator(bool enabled, uint16_t min_mm, uint16_t max_mm) {
    this->motion_enabled_ = enabled;
    this->motion_min_mm_ = min_mm;
    this->motion_max_mm_ = max_mm;
  }

  /// Compile-time environment table (see environment.conf). `table` has
  /// ENV_ZONES x ENV_TABLE_STRIDE values, `rotation` is the Matrix Rotation the zone
  /// numbers of the file refer to.
  void set_environment(const uint16_t *table, uint8_t rotation, uint16_t max_distance_cm, bool configured) {
    this->env_table_ = table;
    this->env_file_rotation_ = rotation;
    this->env_file_max_cm_ = max_distance_cm;
    this->env_configured_ = configured;
  }

  // ---- entity registration -------------------------------------------------
  void set_driver_error_sensor(sensor::Sensor *s) { this->driver_error_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_sensor_ = s; }
  void set_matrix_text_sensor(uint8_t matrix, uint8_t target, text_sensor::TextSensor *ts) {
    if (matrix < MATRIX_COUNT && target < TARGETS_PER_ZONE)
      this->matrix_ts_[matrix][target] = ts;
  }
  void set_resolution_select(select::Select *s) { this->resolution_select_ = s; }
  void set_ranging_mode_select(select::Select *s) { this->mode_select_ = s; }
  void set_target_order_select(select::Select *s) { this->order_select_ = s; }
  void set_target_status_filter_select(select::Select *s) { this->filter_select_ = s; }
  void set_matrix_rotation_select(select::Select *s) { this->rotation_select_ = s; }
  void set_ranging_frequency_number(number::Number *n) { this->frequency_number_ = n; }
  void set_integration_time_number(number::Number *n) { this->integration_number_ = n; }
  void set_sharpener_number(number::Number *n) { this->sharpener_number_ = n; }
  void set_matrix_update_interval_number(number::Number *n) { this->interval_number_ = n; }
  void set_temperature_compensation_number(number::Number *n) { this->vhv_number_ = n; }
  void set_motion_debug_switch(switch_::Switch *s) { this->motion_debug_switch_ = s; }
  void set_maximum_distance_number(number::Number *n) { this->max_distance_number_ = n; }
  void set_minimum_distance_number(number::Number *n) { this->min_distance_number_ = n; }
  void set_calibration_duration_number(number::Number *n) { this->calib_duration_number_ = n; }
  void set_environment_filter_switch(switch_::Switch *s) { this->env_switch_ = s; }
  void set_calibrate_button(button::Button *b) { this->calibrate_button_ = b; }
  void set_logging_switch(uint8_t matrix, switch_::Switch *s) {
    if (matrix < MATRIX_COUNT)
      this->logging_switch_[matrix] = s;
  }

  // ---- requests from entities ----------------------------------------------
  // Hardware settings: validated, then applied with stop -> configure -> start.
  void request_resolution(size_t index);
  void request_ranging_mode(size_t index);
  void request_target_order(size_t index);
  void request_frequency(float hz);
  void request_integration_time(float ms);
  void request_sharpener(float percent);
  void request_temperature_compensation(float seconds);
  // Software-only settings: applied immediately, ranging is never touched.
  void request_target_status_filter(size_t index);
  void request_matrix_rotation(size_t index);
  void request_matrix_update_interval(float ms);
  void request_logging(uint8_t matrix, bool enabled);
  void request_maximum_distance(float cm);
  void request_minimum_distance(float cm);
  void request_calibration_duration(float seconds);
  void request_environment_filter(bool enabled);
  void request_calibration();
  void request_motion_debug_logging(bool enabled);

 protected:
  enum class State : uint8_t { INIT, READY, RANGING, FAULT };

  // lifecycle
  bool bring_up_();
  void power_cycle_();
  bool apply_config_(const SensorConfig &cfg, bool with_resolution);
  void apply_pending_();
  void fault_(const char *what, uint8_t status);
  bool check_(uint8_t status, const char *what);
  void set_driver_error_(uint8_t status);
  void commit_(const SensorConfig &cfg, bool persist = true);
  void publish_config_states_();
  void restore_settings_();
  void save_settings_();

  // ranging
  void service_ranging_(uint32_t now);
  void read_frame_(uint32_t now);
  void publish_temperature_(uint32_t now);
  void publish_matrices_();
  void log_matrix_(MatrixId id);
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  void log_motion_debug_();
#endif
  size_t format_(MatrixId id, uint8_t target, char *buf, size_t cap) const;
  void rebuild_rotation_maps_();
  ProcessingContext make_context_() const;
  void restore_env_settings_();
  void save_env_settings_();
  void build_environment_();
  void calib_accumulate_();
  void calib_begin_();
  void calib_restore_frequency_();
  bool calib_blocks_(const char *what);
  void calib_abort_(const char *why);
  void calib_finish_();
  void calib_report_step_();
  uint32_t period_ms_() const { return 1000u / (this->applied_.frequency_hz ? this->applied_.frequency_hz : 1); }

  // hardware
  GPIOPin *lpn_pin_{nullptr};
  GPIOPin *i2c_reset_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};
  VL53L5CXStore store_;
  uint16_t chunk_size_{64};
  bool motion_enabled_{false};  // motion indicator is disabled by default
  bool debug_motion_{false};
  uint16_t motion_min_mm_{400};
  uint16_t motion_max_mm_{1500};

  // ULD objects (large; live inside the heap-allocated component)
  VL53L5CX_Configuration dev_{};
  VL53L5CX_ResultsData results_{};
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
  VL53L5CX_Motion_Configuration motion_cfg_{};
#endif
  uint8_t *tx_buffer_{nullptr};

  // state
  State state_{State::INIT};
  SensorConfig desired_;  // what the user asked for
  SensorConfig applied_;  // what the sensor currently runs
  ESPPreferenceObject pref_;
  ESPPreferenceObject env_pref_;
  ESPPreferenceObject env_pref_v1_;
  bool dirty_{false};
  uint32_t apply_at_{0};
  uint32_t fault_at_{0};
  uint8_t last_error_{0};
  bool error_published_{false};
  HighFrequencyLoopRequester high_freq_;

  // frame bookkeeping
  bool frame_valid_{false};
  uint8_t frame_n_{4};
  uint32_t frame_seq_{0};
  uint32_t last_frame_ms_{0};
  uint32_t last_poll_ms_{0};
  bool int_watchdog_warned_{false};
  int16_t last_temperature_{INT16_MIN};
  uint32_t last_temperature_ms_{0};

  // environment filter / processed matrices (software only, no ranging restart)
  const uint16_t *env_table_{nullptr};
  uint8_t env_file_rotation_{0};
  uint16_t env_file_max_cm_{0};
  bool env_configured_{false};
  EnvRuntime env_;
  bool env_enabled_{true};
  uint16_t max_distance_cm_{ENV_MAX_DISTANCE_CM};
  uint16_t min_distance_cm_{10};
  uint16_t calib_duration_s_{60};
  bool env_4x4_warned_{false};
  uint16_t env_mismatch_warned_for_{0};

  // calibration of the empty environment
  bool calib_pending_{false};          // waiting for the switch to 1 Hz
  bool calib_freq_overridden_{false};  // the 1 Hz is temporary; the saved value is restored at the end
  uint8_t calib_saved_freq_{1};
  bool calib_active_{false};
  uint32_t calib_start_ms_{0};
  uint32_t calib_end_ms_{0};
  uint32_t calib_next_progress_ms_{0};
  uint32_t calib_frames_{0};
  CalZone *calib_zones_{nullptr};  // ENV_ZONES entries, allocated only while calibrating
  std::vector<std::string> report_;
  size_t report_pos_{0};

  // software settings
  FilterProfile filter_{FilterProfile::RANGE_VALID};
  Rotation rotation_{Rotation::DEG_0};
  RotationMap map4_;
  RotationMap map8_;
  PublishScheduler scheduler_;
  bool log_enabled_[MATRIX_COUNT]{};
  char text_buf_[MATRIX_TEXT_BUFFER_SIZE + 4]{};

  // entities
  sensor::Sensor *driver_error_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  text_sensor::TextSensor *matrix_ts_[MATRIX_COUNT][TARGETS_PER_ZONE]{};
  select::Select *resolution_select_{nullptr};
  select::Select *mode_select_{nullptr};
  select::Select *order_select_{nullptr};
  select::Select *filter_select_{nullptr};
  select::Select *rotation_select_{nullptr};
  number::Number *frequency_number_{nullptr};
  number::Number *integration_number_{nullptr};
  number::Number *sharpener_number_{nullptr};
  number::Number *interval_number_{nullptr};
  number::Number *vhv_number_{nullptr};
  switch_::Switch *logging_switch_[MATRIX_COUNT]{};
  switch_::Switch *motion_debug_switch_{nullptr};
  number::Number *max_distance_number_{nullptr};
  number::Number *min_distance_number_{nullptr};
  number::Number *calib_duration_number_{nullptr};
  switch_::Switch *env_switch_{nullptr};
  button::Button *calibrate_button_{nullptr};
};

}  // namespace vl53l5cx
}  // namespace esphome
