import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CENTIMETER,
    UNIT_HERTZ,
    UNIT_MILLISECOND,
    UNIT_PERCENT,
    UNIT_SECOND,
)

from . import CONF_VL53L5CX_ID, VL53L5CXComponent, vl53l5cx_ns

DEPENDENCIES = ["vl53l5cx"]

RangingFrequencyNumber = vl53l5cx_ns.class_("RangingFrequencyNumber", number.Number)
IntegrationTimeNumber = vl53l5cx_ns.class_("IntegrationTimeNumber", number.Number)
SharpenerNumber = vl53l5cx_ns.class_("SharpenerNumber", number.Number)
MatrixUpdateIntervalNumber = vl53l5cx_ns.class_("MatrixUpdateIntervalNumber", number.Number)
MaximumDistanceNumber = vl53l5cx_ns.class_("MaximumDistanceNumber", number.Number)
MinimumDistanceNumber = vl53l5cx_ns.class_("MinimumDistanceNumber", number.Number)
CalibrationDurationNumber = vl53l5cx_ns.class_("CalibrationDurationNumber", number.Number)
TemperatureCompensationNumber = vl53l5cx_ns.class_(
    "TemperatureCompensationNumber", number.Number
)

# key: (class, unit, min, max, step, setter, icon)
NUMBERS = {
    # 1-60 Hz at 4x4, 1-15 Hz at 8x8 (validated at runtime against the resolution).
    "ranging_frequency": (RangingFrequencyNumber, UNIT_HERTZ, 1, 60, 1, "set_ranging_frequency_number", "mdi:speedometer"),
    # 2-1000 ms; only effective in Autonomous mode.
    "integration_time": (IntegrationTimeNumber, UNIT_MILLISECOND, 2, 1000, 1, "set_integration_time_number", "mdi:timer-outline"),
    "sharpener": (SharpenerNumber, UNIT_PERCENT, 0, 99, 1, "set_sharpener_number", "mdi:blur"),
    # Publication interval only; never changes the ranging frequency.
    "matrix_update_interval": (MatrixUpdateIntervalNumber, UNIT_MILLISECOND, 0, 60000, 10, "set_matrix_update_interval_number", "mdi:update"),
    # Processed Distance: NaN and anything above this value become this value (RAW untouched).
    "maximum_distance": (MaximumDistanceNumber, UNIT_CENTIMETER, 1, 400, 1, "set_maximum_distance_number", "mdi:ruler"),
    # Processed Distance: readings closer than this (e.g. a 0 cm glitch) are not targets.
    "minimum_distance": (MinimumDistanceNumber, UNIT_CENTIMETER, 0, 100, 1, "set_minimum_distance_number", "mdi:ruler"),
    # Duration of the empty-environment calibration.
    "calibration_duration": (CalibrationDurationNumber, UNIT_SECOND, 10, 3600, 1, "set_calibration_duration_number", "mdi:timer-sand"),
    # Seconds between periodic temperature compensations (0 = disabled).
    "periodic_temperature_compensation": (TemperatureCompensationNumber, UNIT_SECOND, 0, 86400, 1, "set_temperature_compensation_number", "mdi:thermometer-auto"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        **{
            cv.Optional(key): number.number_schema(
                cls,
                unit_of_measurement=unit,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon=icon,
            )
            for key, (cls, unit, _lo, _hi, _step, _setter, icon) in NUMBERS.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    for key, (_cls, _unit, lo, hi, step, setter, _icon) in NUMBERS.items():
        if conf := config.get(key):
            num = await number.new_number(conf, min_value=lo, max_value=hi, step=step)
            await cg.register_parented(num, config[CONF_VL53L5CX_ID])
            cg.add(getattr(hub, setter)(num))
