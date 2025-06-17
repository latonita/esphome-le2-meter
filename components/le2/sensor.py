import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)
from . import LE2Component, CONF_LE2_ID

CODEOWNERS = ["@latonita"]

DEPENDENCIES = ["le2"]

CONF_ACTIVE_POWER = "active_power"
CONF_TARIFF = "current_tariff"
CONF_ENERGY_TOTAL = "energy_total"
CONF_ENERGY_T1 = "energy_t1"
CONF_ENERGY_T2 = "energy_t2"
CONF_ENERGY_T3 = "energy_t3"
CONF_ENERGY_T4 = "energy_t4"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LE2_ID): cv.use_id(LE2Component),
        cv.Optional(CONF_ACTIVE_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:home-lightning-bolt-outline",
        ),
        cv.Optional(CONF_ENERGY_TOTAL): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        cv.Optional(CONF_ENERGY_T1): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        cv.Optional(CONF_ENERGY_T2): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        cv.Optional(CONF_ENERGY_T3): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        cv.Optional(CONF_ENERGY_T4): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_LE2_ID])

    for key in [
        CONF_ACTIVE_POWER,
        CONF_ENERGY_TOTAL,
        CONF_ENERGY_T1,
        CONF_ENERGY_T2,
        CONF_ENERGY_T3,
        CONF_ENERGY_T4,
    ]:
        if key not in config:
            continue
        conf = config[key]
        sens = await sensor.new_sensor(conf)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))
