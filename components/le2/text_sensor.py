import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    CONF_DATE,
    CONF_TIME,
    CONF_DATETIME,
    CONF_NAME,
)

from . import LE2Component, CONF_LE2_ID

AUTO_LOAD = ["le2"]
CODEOWNERS = ["@latonita"]

CONF_ELECTRO_TARIFF = "electricity_tariff"

CONF_NETWORK_ADDRESS = "network_address"
CONF_SERIAL_NR = "serial_nr"
CONF_READING_STATE = "reading_state"
CONF_ERROR_CODE = "error_code"

TEXT_SENSORS = [
    CONF_ELECTRO_TARIFF,
    CONF_DATE,
    CONF_TIME,
    CONF_DATETIME,
    CONF_NETWORK_ADDRESS,
    CONF_SERIAL_NR,
    CONF_READING_STATE,
    CONF_ERROR_CODE,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LE2_ID): cv.use_id(LE2Component),
        cv.Optional(CONF_ELECTRO_TARIFF): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_DATE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_TIME): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_DATETIME): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_NETWORK_ADDRESS): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_SERIAL_NR): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_READING_STATE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_ERROR_CODE): cv.maybe_simple_value(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            key=CONF_NAME,
        ),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_LE2_ID])
    for key in TEXT_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
