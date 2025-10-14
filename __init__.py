from esphome.components import time
from esphome import automation
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TIME_ID, CONF_TRIGGER_ID, CONF_SENSOR_DATAPOINT

DEPENDENCIES = ["uart"]

CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS = "ignore_mcu_update_on_datapoints"

CONF_ON_DATAPOINT_UPDATE = "on_datapoint_update"
CONF_DATAPOINT_TYPE = "datapoint_type"
CONF_DP_ACK_DELAY = "dp_ack_delay"
CONF_PAIR_ACK_DELAY = "pair_ack_delay"

lpuyat_ns = cg.esphome_ns.namespace("lpuyat")
LPUyatDatapointType = lpuyat_ns.enum("LPUyatDatapointType")
LPUyat = lpuyat_ns.class_("LPUyat", cg.Component, uart.UARTDevice)

DPTYPE_ANY = "any"
DPTYPE_RAW = "raw"
DPTYPE_BOOL = "bool"
DPTYPE_INT = "int"
DPTYPE_UINT = "uint"
DPTYPE_STRING = "string"
DPTYPE_ENUM = "enum"
DPTYPE_BITMASK = "bitmask"

DATAPOINT_TYPES = {
    DPTYPE_ANY: lpuyat_ns.struct("LPUyatDatapoint"),
    DPTYPE_RAW: cg.std_vector.template(cg.uint8),
    DPTYPE_BOOL: cg.bool_,
    DPTYPE_INT: cg.int_,
    DPTYPE_UINT: cg.uint32,
    DPTYPE_STRING: cg.std_string,
    DPTYPE_ENUM: cg.uint8,
    DPTYPE_BITMASK: cg.uint32,
}

DATAPOINT_TRIGGERS = {
    DPTYPE_ANY: lpuyat_ns.class_(
        "LPUyatDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_ANY]),
    ),
    DPTYPE_RAW: lpuyat_ns.class_(
        "LPUyatRawDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_RAW]),
    ),
    DPTYPE_BOOL: lpuyat_ns.class_(
        "LPUyatBoolDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_BOOL]),
    ),
    DPTYPE_INT: lpuyat_ns.class_(
        "LPUyatIntDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_INT]),
    ),
    DPTYPE_UINT: lpuyat_ns.class_(
        "LPUyatUIntDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_UINT]),
    ),
    DPTYPE_STRING: lpuyat_ns.class_(
        "LPUyatStringDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_STRING]),
    ),
    DPTYPE_ENUM: lpuyat_ns.class_(
        "LPUyatEnumDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_ENUM]),
    ),
    DPTYPE_BITMASK: lpuyat_ns.class_(
        "LPUyatBitmaskDatapointUpdateTrigger",
        automation.Trigger.template(DATAPOINT_TYPES[DPTYPE_BITMASK]),
    ),
}


def assign_declare_id(value):
    value = value.copy()
    value[CONF_TRIGGER_ID] = cv.declare_id(
        DATAPOINT_TRIGGERS[value[CONF_DATAPOINT_TYPE]]
    )(value[CONF_TRIGGER_ID].id)
    return value


CONF_LPUYAT_ID = "lpuyat_id"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LPUyat),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS): cv.ensure_list(
                cv.uint8_t
            ),
            cv.Optional(CONF_ON_DATAPOINT_UPDATE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        DATAPOINT_TRIGGERS[DPTYPE_ANY]
                    ),
                    cv.Required(CONF_SENSOR_DATAPOINT): cv.uint8_t,
                    cv.Optional(CONF_DATAPOINT_TYPE, default=DPTYPE_ANY): cv.one_of(
                        *DATAPOINT_TRIGGERS, lower=True
                    ),
                },
                extra_validators=assign_declare_id,
            ),
            cv.Optional(CONF_DP_ACK_DELAY, default="1000ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PAIR_ACK_DELAY, default="30s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_id(time_))
    if CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS in config:
        for dp in config[CONF_IGNORE_MCU_UPDATE_ON_DATAPOINTS]:
            cg.add(var.add_ignore_mcu_update_on_datapoints(dp))
    for conf in config.get(CONF_ON_DATAPOINT_UPDATE, []):
        trigger = cg.new_Pvariable(
            conf[CONF_TRIGGER_ID], var, conf[CONF_SENSOR_DATAPOINT]
        )
        await automation.build_automation(
            trigger, [(DATAPOINT_TYPES[conf[CONF_DATAPOINT_TYPE]], "x")], conf
        )
    if CONF_DP_ACK_DELAY in config:
        cg.add(var.set_dp_ack_delay(config[CONF_DP_ACK_DELAY]))
    if CONF_PAIR_ACK_DELAY in config:
        cg.add(var.set_pairing_delay(config[CONF_PAIR_ACK_DELAY]))
