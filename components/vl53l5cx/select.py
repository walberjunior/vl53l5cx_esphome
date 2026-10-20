import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from . import CONF_VL53L5CX_ID, VL53L5CXComponent, vl53l5cx_ns

DEPENDENCIES = ["vl53l5cx"]

ResolutionSelect = vl53l5cx_ns.class_("ResolutionSelect", select.Select)
RangingModeSelect = vl53l5cx_ns.class_("RangingModeSelect", select.Select)
TargetOrderSelect = vl53l5cx_ns.class_("TargetOrderSelect", select.Select)
TargetStatusFilterSelect = vl53l5cx_ns.class_("TargetStatusFilterSelect", select.Select)
MatrixRotationSelect = vl53l5cx_ns.class_("MatrixRotationSelect", select.Select)

# (yaml key, class, options, setter). The option ORDER is part of the contract
# with the C++ side (index based).
SELECTS = {
    "resolution": (ResolutionSelect, ["4x4", "8x8"], "set_resolution_select", "mdi:grid"),
    "ranging_mode": (
        RangingModeSelect,
        ["Continuous", "Autonomous"],
        "set_ranging_mode_select",
        "mdi:sine-wave",
    ),
    "target_order": (
        TargetOrderSelect,
        ["Closest", "Strongest"],
        "set_target_order_select",
        "mdi:sort",
    ),
    "target_status_filter": (
        TargetStatusFilterSelect,
        ["Disabled", "Range Valid", "Permissive", "High Confidence", "Confidence Based"],
        "set_target_status_filter_select",
        "mdi:filter",
    ),
    "matrix_rotation": (
        MatrixRotationSelect,
        ["0°", "90°", "180°", "270°"],
        "set_matrix_rotation_select",
        "mdi:rotate-right",
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_VL53L5CX_ID): cv.use_id(VL53L5CXComponent),
        **{
            cv.Optional(key): select.select_schema(
                cls, entity_category=ENTITY_CATEGORY_CONFIG, icon=icon
            )
            for key, (cls, _options, _setter, icon) in SELECTS.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_VL53L5CX_ID])
    for key, (_cls, options, setter, _icon) in SELECTS.items():
        if conf := config.get(key):
            sel = await select.new_select(conf, options=options)
            await cg.register_parented(sel, config[CONF_VL53L5CX_ID])
            cg.add(getattr(hub, setter)(sel))
