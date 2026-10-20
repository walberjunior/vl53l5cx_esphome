#pragma once

#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/helpers.h"

#include "vl53l5cx.h"

namespace esphome {
namespace vl53l5cx {

// Selects. Option order is fixed by the Python side (select.py).
class ResolutionSelect final : public select::Select, public Parented<VL53L5CXComponent> {
 protected:
  void control(size_t index) override { this->parent_->request_resolution(index); }
};
class RangingModeSelect final : public select::Select, public Parented<VL53L5CXComponent> {
 protected:
  void control(size_t index) override { this->parent_->request_ranging_mode(index); }
};
class TargetOrderSelect final : public select::Select, public Parented<VL53L5CXComponent> {
 protected:
  void control(size_t index) override { this->parent_->request_target_order(index); }
};
class TargetStatusFilterSelect final : public select::Select, public Parented<VL53L5CXComponent> {
 protected:
  void control(size_t index) override { this->parent_->request_target_status_filter(index); }
};
class MatrixRotationSelect final : public select::Select, public Parented<VL53L5CXComponent> {
 protected:
  void control(size_t index) override { this->parent_->request_matrix_rotation(index); }
};

// Numbers
class RangingFrequencyNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_frequency(value); }
};
class IntegrationTimeNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_integration_time(value); }
};
class SharpenerNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_sharpener(value); }
};
class MatrixUpdateIntervalNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_matrix_update_interval(value); }
};
class TemperatureCompensationNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_temperature_compensation(value); }
};

// Logging switches (one per matrix)
class MatrixLoggingSwitch final : public switch_::Switch, public Parented<VL53L5CXComponent> {
 public:
  explicit MatrixLoggingSwitch(uint8_t matrix) : matrix_(matrix) {}

 protected:
  void write_state(bool state) override { this->parent_->request_logging(this->matrix_, state); }
  uint8_t matrix_;
};

class MaximumDistanceNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_maximum_distance(value); }
};
class MinimumDistanceNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_minimum_distance(value); }
};
class CalibrationDurationNumber final : public number::Number, public Parented<VL53L5CXComponent> {
 protected:
  void control(float value) override { this->parent_->request_calibration_duration(value); }
};
class EnvironmentFilterSwitch final : public switch_::Switch, public Parented<VL53L5CXComponent> {
 protected:
  void write_state(bool state) override { this->parent_->request_environment_filter(state); }
};
class CalibrateEnvironmentButton final : public button::Button, public Parented<VL53L5CXComponent> {
 protected:
  void press_action() override { this->parent_->request_calibration(); }
};

// Debug switch: logs the complete ULD motion structure for every frame.
class MotionDebugLoggingSwitch final : public switch_::Switch, public Parented<VL53L5CXComponent> {
 protected:
  void write_state(bool state) override { this->parent_->request_motion_debug_logging(state); }
};

}  // namespace vl53l5cx
}  // namespace esphome
