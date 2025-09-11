from esphome.components import sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, CONF_SENSOR_DATAPOINT
from .. import lpuyat_ns, CONF_LPUYAT_ID, LPUyat

DEPENDENCIES = ["lpuyat"]
CODEOWNERS = ["@jesserockz"]

LPUyatSensor = lpuyat_ns.class_("LPUyatSensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = (
    sensor.sensor_schema(LPUyatSensor)
    .extend(
        {
            cv.GenerateID(CONF_LPUYAT_ID): cv.use_id(LPUyat),
            cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    paren = await cg.get_variable(config[CONF_LPUYAT_ID])
    cg.add(var.set_lpuyat_parent(paren))

    cg.add(var.set_sensor_id(config[CONF_SENSOR_DATAPOINT]))
