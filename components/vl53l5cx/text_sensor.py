import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.final_validate as fv

from . import (
    CONF_MOTION_INDICATOR,
    CONF_TARGETS_PER_ZONE,
    CONF_VL53L5CX_ID,
    MATRIX_IDS,
    RENAMED_KEYS,
    VL53L5CXComponent,
    renamed_option,
)

DEPENDENCIES = ["vl53l5cx"]

# Matrices that exist once per zone.
ZONE_MATRICES = (
    "ambient_per_spad_raw",
    "number_of_targets_detected",  # processed
    "number_of_targets_detected_raw",
    "number_of_spads_enabled_raw",
    "motion_indicator_raw",
)
# Matrices that exist once per target index. Each key takes a list: entry 1 is
# target 1 (first target in the configured Target Order), entry 2 is target 2...
TARGET_MATRICES = (
    "signal_per_spad_raw",
    "range_sigma_raw",
    "target_status_raw",
    "reflectance_raw",
    "distance_raw",
    "distance",  # processed
)

_schema = text_sensor.text_sensor_schema(icon="mdi:grid")
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        **{cv.Optional(key): _schema for key in ZONE_MATRICES},
        **{cv.Optional(key): cv.ensure_list(_schema) for key in TARGET_MATRICES},
        **{cv.Optional(old): renamed_option(old, new) for old, new in RENAMED_KEYS.items()},
    }
)


def _final_validate(config):
    full_config = fv.full_config.get()
    hub_path = full_config.get_path_for_id(config[CONF_VL53L5CX_ID])[:-1]
    hub = full_config.get_config_for_path(hub_path)
    targets = hub[CONF_TARGETS_PER_ZONE]
    for key in TARGET_MATRICES:
        if key in config and len(config[key]) > targets:
            raise cv.Invalid(
                f"'{key}' lists {len(config[key])} targets but the hub is "
                f"compiled with {CONF_TARGETS_PER_ZONE}: {targets}"
            )
    if "motion_indicator_raw" in config and not hub[CONF_MOTION_INDICATOR]:
        raise cv.Invalid(
            "'motion_indicator_raw' requires 'motion_indicator: true' on the vl53l5cx hub "
            "(the motion indicator is disabled by default)"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    for key in ZONE_MATRICES:
        if conf := config.get(key):
            ts = await text_sensor.new_text_sensor(conf)
            cg.add(hub.set_matrix_text_sensor(MATRIX_IDS[key], 0, ts))
    for key in TARGET_MATRICES:
        for index, conf in enumerate(config.get(key, [])):
            ts = await text_sensor.new_text_sensor(conf)
            cg.add(hub.set_matrix_text_sensor(MATRIX_IDS[key], index, ts))
