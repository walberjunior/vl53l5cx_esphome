"""ESPHome external component for the STMicroelectronics VL53L5CX multizone ToF sensor."""

from pathlib import Path

import esphome.codegen as cg
from esphome import pins
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_INTERRUPT_PIN
from esphome.core import CORE

from .environment import load_environment_file

CODEOWNERS = []
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "text_sensor", "select", "number", "switch", "button"]
MULTI_CONF = True

vl53l5cx_ns = cg.esphome_ns.namespace("vl53l5cx")
VL53L5CXComponent = vl53l5cx_ns.class_(
    "VL53L5CXComponent", cg.Component, i2c.I2CDevice
)

CONF_VL53L5CX_ID = "vl53l5cx_id"
CONF_LPN_PIN = "lpn_pin"
CONF_I2C_RESET_PIN = "i2c_reset_pin"
CONF_TARGETS_PER_ZONE = "targets_per_zone"
CONF_I2C_CHUNK_SIZE = "i2c_chunk_size"
CONF_ENVIRONMENT_FILE = "environment_file"
CONF_ENVIRONMENT_TABLE_ID = "environment_table_id"
CONF_MOTION_INDICATOR = "motion_indicator"
CONF_MOTION_DISTANCE_MIN = "motion_distance_min"
CONF_MOTION_DISTANCE_MAX = "motion_distance_max"

# Matrix identifiers, shared with C++ (enum class MatrixId).
MATRIX_IDS = {
    # RAW = exactly what the ULD returns (only Matrix Rotation is applied)
    "ambient_per_spad_raw": 0,
    # processed count: Target Status Filter + Maximum Distance + environment filter
    "number_of_targets_detected": 1,
    "number_of_spads_enabled_raw": 2,
    "signal_per_spad_raw": 3,
    "range_sigma_raw": 4,
    "target_status_raw": 5,
    "reflectance_raw": 6,
    "motion_indicator_raw": 7,
    "distance_raw": 8,
    # processed distance: Target Status Filter + Maximum Distance + environment filter
    "distance": 9,
    "number_of_targets_detected_raw": 10,
}

# Keys renamed when the RAW suffix was introduced -> new key. The old keys give a clear error.
RENAMED_KEYS = {
    "ambient_per_spad": "ambient_per_spad_raw",
    "number_of_spads_enabled": "number_of_spads_enabled_raw",
    "signal_per_spad": "signal_per_spad_raw",
    "range_sigma": "range_sigma_raw",
    "target_status": "target_status_raw",
    "reflectance": "reflectance_raw",
    "motion_indicator": "motion_indicator_raw",
}


def renamed_option(old, new):
    return cv.invalid(f"'{old}' was renamed to '{new}' (RAW = original ULD value)")


def _environment_file(value):
    """Validates the environment file (mandatory; may be empty or only comments).

    A relative name is looked up in this component's folder. Returns the
    absolute path (to_code parses it again), so the printed config stays small.
    """
    name = cv.string_strict(value)
    path = Path(name)
    if not path.is_absolute():
        path = Path(__file__).parent / path
    load_environment_file(path)  # raises cv.Invalid with file:line
    return str(path)


def _validate(config):
    if CORE.is_esp8266:
        raise cv.Invalid(
            "The VL53L5CX ULD stores an 84 KB firmware image that does not fit "
            "into ESP8266 RAM; ESP8266 is not supported"
        )
    lo = config[CONF_MOTION_DISTANCE_MIN]
    hi = config[CONF_MOTION_DISTANCE_MAX]
    if hi <= lo:
        raise cv.Invalid(
            f"{CONF_MOTION_DISTANCE_MAX} must be greater than {CONF_MOTION_DISTANCE_MIN}"
        )
    if hi - lo > 1500:
        raise cv.Invalid(
            f"The span between {CONF_MOTION_DISTANCE_MIN} and "
            f"{CONF_MOTION_DISTANCE_MAX} cannot exceed 1500 mm (UM2884 4.13)"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VL53L5CXComponent),
            cv.GenerateID(CONF_ENVIRONMENT_TABLE_ID): cv.declare_id(cg.uint16),
            # Mandatory file with the empty-environment references. It lives in the
            # component folder, may be empty or fully commented (then ignored).
            cv.Optional(CONF_ENVIRONMENT_FILE, default="environment.conf"): _environment_file,
            cv.Optional(CONF_LPN_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_I2C_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
            # Compile-time ULD setting VL53L5CX_NB_TARGET_PER_ZONE (UM2884 4.10).
            cv.Optional(CONF_TARGETS_PER_ZONE, default=4): cv.int_range(min=1, max=4),
            cv.Optional(CONF_I2C_CHUNK_SIZE, default=64): cv.int_range(
                min=16, max=1024
            ),
            # Disabled by default. When false the motion plugin, its result block
            # (144 bytes per frame on I2C) and all motion entities are compiled out.
            cv.Optional(CONF_MOTION_INDICATOR, default=False): cv.boolean,
            cv.Optional(CONF_MOTION_DISTANCE_MIN, default=400): cv.int_range(
                min=400, max=4000
            ),
            cv.Optional(CONF_MOTION_DISTANCE_MAX, default=1500): cv.int_range(
                min=400, max=4000
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x29)),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # The ULD C sources and the C++ side both read this macro.
    cg.add_build_flag(
        f"-DVL53L5CX_NB_TARGET_PER_ZONE={config[CONF_TARGETS_PER_ZONE]}U"
    )
    env = load_environment_file(Path(config[CONF_ENVIRONMENT_FILE]))
    table = cg.progmem_array(config[CONF_ENVIRONMENT_TABLE_ID], env["table"])
    cg.add(
        var.set_environment(
            table, env["rotation"], env["max_distance_cm"], env["configured"]
        )
    )
    if not config[CONF_MOTION_INDICATOR]:
        # ST's own switch: removes the motion block from the ULD result struct.
        cg.add_build_flag("-DVL53L5CX_DISABLE_MOTION_INDICATOR")
    cg.add(var.set_i2c_chunk_size(config[CONF_I2C_CHUNK_SIZE]))
    cg.add(
        var.set_motion_indicator(
            config[CONF_MOTION_INDICATOR],
            config[CONF_MOTION_DISTANCE_MIN],
            config[CONF_MOTION_DISTANCE_MAX],
        )
    )
    if CONF_LPN_PIN in config:
        cg.add(var.set_lpn_pin(await cg.gpio_pin_expression(config[CONF_LPN_PIN])))
    if CONF_I2C_RESET_PIN in config:
        cg.add(
            var.set_i2c_reset_pin(
                await cg.gpio_pin_expression(config[CONF_I2C_RESET_PIN])
            )
        )
    if CONF_INTERRUPT_PIN in config:
        cg.add(
            var.set_interrupt_pin(
                await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
            )
        )
