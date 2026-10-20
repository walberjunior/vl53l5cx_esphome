import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from . import (
    CONF_MOTION_INDICATOR,
    CONF_VL53L5CX_ID,
    MATRIX_IDS,
    RENAMED_KEYS,
    VL53L5CXComponent,
    vl53l5cx_ns,
)

DEPENDENCIES = ["vl53l5cx"]

MatrixLoggingSwitch = vl53l5cx_ns.class_("MatrixLoggingSwitch", switch.Switch)
MotionDebugLoggingSwitch = vl53l5cx_ns.class_(
    "MotionDebugLoggingSwitch", switch.Switch
)
CONF_MOTION_DEBUG = "motion_indicator_raw_debug_logging"
EnvironmentFilterSwitch = vl53l5cx_ns.class_("EnvironmentFilterSwitch", switch.Switch)
CONF_ENVIRONMENT_FILTER = "environment_filter"

# One independent switch per matrix; key = "<matrix>_logging".
LOGGING_KEYS = {f"{matrix}_logging": idx for matrix, idx in MATRIX_IDS.items()}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        **{
            cv.Optional(key): switch.switch_schema(
                MatrixLoggingSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:text-box-search-outline",
                # Logging is always OFF after boot.
                default_restore_mode="ALWAYS_OFF",
            )
            for key in LOGGING_KEYS
        },
        **{
            cv.Optional(f"{old}_logging"): cv.invalid(
                f"'{old}_logging' was renamed to '{new}_logging' (RAW = original ULD value)"
            )
            for old, new in RENAMED_KEYS.items()
        },
        cv.Optional("motion_indicator_debug_logging"): cv.invalid(
            "'motion_indicator_debug_logging' was renamed to 'motion_indicator_raw_debug_logging'"
        ),
        # Runtime on/off of the empty-environment filter (restored at boot).
        cv.Optional(CONF_ENVIRONMENT_FILTER): switch.switch_schema(
            EnvironmentFilterSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:filter-variant",
        ),
        # Debug aid: logs the complete ULD motion structure for every frame.
        cv.Optional(CONF_MOTION_DEBUG): switch.switch_schema(
            MotionDebugLoggingSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:bug-outline",
            default_restore_mode="ALWAYS_OFF",
        ),
    }
)


def _final_validate(config):
    if CONF_MOTION_DEBUG in config or "motion_indicator_raw_logging" in config:
        full_config = fv.full_config.get()
        hub_path = full_config.get_path_for_id(config[CONF_VL53L5CX_ID])[:-1]
        hub = full_config.get_config_for_path(hub_path)
        if not hub[CONF_MOTION_INDICATOR]:
            raise cv.Invalid(
                "the motion indicator switches require 'motion_indicator: true' on the "
                "vl53l5cx hub (the motion indicator is disabled by default)"
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    for key, idx in LOGGING_KEYS.items():
        if conf := config.get(key):
            sw = await switch.new_switch(conf, idx)
            await cg.register_parented(sw, config[CONF_VL53L5CX_ID])
            cg.add(hub.set_logging_switch(idx, sw))
    if conf := config.get(CONF_MOTION_DEBUG):
        sw = await switch.new_switch(conf)
        await cg.register_parented(sw, config[CONF_VL53L5CX_ID])
        cg.add(hub.set_motion_debug_switch(sw))
    if conf := config.get(CONF_ENVIRONMENT_FILTER):
        sw = await switch.new_switch(conf)
        await cg.register_parented(sw, config[CONF_VL53L5CX_ID])
        cg.add(hub.set_environment_filter_switch(sw))
