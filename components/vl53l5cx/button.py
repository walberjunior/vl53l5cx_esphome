import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_VL53L5CX_ID, VL53L5CXComponent, vl53l5cx_ns

DEPENDENCIES = ["vl53l5cx"]

CalibrateEnvironmentButton = vl53l5cx_ns.class_(
    "CalibrateEnvironmentButton", button.Button
)
CONF_CALIBRATE_ENVIRONMENT = "calibrate_environment"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        # Collects the empty passage for "Calibration Duration" seconds (8x8 only)
        # and prints suggested references in the format of environment.conf.
        cv.Optional(CONF_CALIBRATE_ENVIRONMENT): button.button_schema(
            CalibrateEnvironmentButton,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:tune-variant",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    if conf := config.get(CONF_CALIBRATE_ENVIRONMENT):
        btn = await button.new_button(conf)
        await cg.register_parented(btn, config[CONF_VL53L5CX_ID])
        cg.add(hub.set_calibrate_button(btn))
