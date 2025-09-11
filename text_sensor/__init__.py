from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_SENSOR_DATAPOINT
from .. import lpuyat_ns, CONF_LPUYAT_ID, LPUyat

DEPENDENCIES = ["lpuyat"]
CODEOWNERS = ["@dentra"]

LPUyatTextSensor = lpuyat_ns.class_("LPUyatTextSensor", text_sensor.TextSensor, cg.Component)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(LPUyatTextSensor),
            cv.GenerateID(CONF_LPUYAT_ID): cv.use_id(LPUyat),
            cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_LPUYAT_ID])
    cg.add(var.set_lpuyat_parent(paren))

    cg.add(var.set_sensor_id(config[CONF_SENSOR_DATAPOINT]))
