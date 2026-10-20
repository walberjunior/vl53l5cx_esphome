import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import CONF_VL53L5CX_ID, VL53L5CXComponent, renamed_option

DEPENDENCIES = ["vl53l5cx"]

CONF_DRIVER_ERROR = "driver_error_raw"
CONF_TEMPERATURE_RAW = "temperature_raw"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        # ULD status of the most recent operation (UM2884 5.6): 0 = no error,
        # 127 = incorrect setting, 255 = major error (usually I2C timeout),
        # other values = OR-combination of ULD error bits.
        cv.Optional(CONF_DRIVER_ERROR): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:alert-circle-outline",
        ),
        # Silicon temperature reported with every frame (int8, 1 degC steps).
        cv.Optional(CONF_TEMPERATURE_RAW): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("driver_error"): renamed_option("driver_error", CONF_DRIVER_ERROR),
        cv.Optional("temperature"): renamed_option("temperature", CONF_TEMPERATURE_RAW),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    if conf := config.get(CONF_DRIVER_ERROR):
        sens = await sensor.new_sensor(conf)
        cg.add(hub.set_driver_error_sensor(sens))
    if conf := config.get(CONF_TEMPERATURE_RAW):
        sens = await sensor.new_sensor(conf)
        cg.add(hub.set_temperature_sensor(sens))
