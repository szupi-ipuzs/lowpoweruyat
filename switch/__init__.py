from esphome.components import switch
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_SWITCH_DATAPOINT
from .. import lpuyat_ns, CONF_LPUYAT_ID, LPUyat

DEPENDENCIES = ["lpuyat"]
CODEOWNERS = ["@jesserockz"]

LPUyatSwitch = lpuyat_ns.class_("LPUyatSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.switch_schema(LPUyatSwitch)
    .extend(
        {
            cv.GenerateID(CONF_LPUYAT_ID): cv.use_id(LPUyat),
            cv.Required(CONF_SWITCH_DATAPOINT): cv.uint8_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_LPUYAT_ID])
    cg.add(var.set_lpuyat_parent(paren))

    cg.add(var.set_switch_id(config[CONF_SWITCH_DATAPOINT]))
