from esphome.components import binary_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_SENSOR_DATAPOINT

from .. import lpuyat_ns, CONF_LPUYAT_ID, LPUyat

DEPENDENCIES = ["lpuyat"]
CODEOWNERS = ["@szupi_ipuzs"]

LPUyatBinarySensor = lpuyat_ns.class_(
    "LPUyatBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(LPUyatBinarySensor)
    .extend(
        {
            cv.GenerateID(CONF_LPUYAT_ID): cv.use_id(LPUyat),
            cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_LPUYAT_ID])
    cg.add(var.set_lpuyat_parent(paren))

    cg.add(var.set_sensor_id(config[CONF_SENSOR_DATAPOINT]))
