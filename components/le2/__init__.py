import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_FLOW_CONTROL_PIN,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_PASSWORD,
)

CODEOWNERS = ["@latonita"]

MULTI_CONF = True

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_LE2_ID = "le2_id"

le2_ns = cg.esphome_ns.namespace("esphome::le2")
LE2Component = le2_ns.class_("LE2Component", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LE2Component),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0): cv.int_range (min=0x0, max=0xffffffff),
            cv.Optional(CONF_PASSWORD, default=0): cv.int_range (min=0x0, max=0xffffffff),
            cv.Optional(CONF_RECEIVE_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA),
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "le2", 
    require_rx=True,
    require_tx=True,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_receive_timeout(config[CONF_RECEIVE_TIMEOUT].total_milliseconds))
    cg.add(var.set_requested_meter_address(config[CONF_ADDRESS]))
    cg.add(var.set_password(config[CONF_PASSWORD]))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

