import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ACTIVE_POWER,
    CONF_CURRENT,
    CONF_APPARENT_POWER,
    CONF_EXPORT_ACTIVE_ENERGY,
    CONF_EXPORT_REACTIVE_ENERGY,
    CONF_FREQUENCY,    
    CONF_ID,
    CONF_IMPORT_ACTIVE_ENERGY,
    CONF_IMPORT_REACTIVE_ENERGY,
    CONF_NAME,
    CONF_POWER_FACTOR,
    CONF_REACTIVE_POWER,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_POWER,
    ICON_CURRENT_AC,
    ICON_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_VOLT_AMPS_REACTIVE_HOURS,
    UNIT_VOLT_AMPS_REACTIVE,
    UNIT_VOLT,
    UNIT_WATT_HOURS,
    UNIT_WATT,
    UNIT_VOLT_AMPS,
)
from . import LE2Component, CONF_LE2_ID

CODEOWNERS = ["@latonita"]

DEPENDENCIES = ["le2"]

ICON_VOLTAGE = "mdi:sine-wave"

CONF_TARIFF_1 = "tariff_1"
CONF_TARIFF_2 = "tariff_2"
CONF_TARIFF_3 = "tariff_3"
CONF_TARIFF_4 = "tariff_4"
CONF_TARIFF_5 = "tariff_5"
CONF_TARIFF_6 = "tariff_6"
CONF_TARIFF_7 = "tariff_7"
CONF_TARIFF_8 = "tariff_8"

TARIFF_CONSUMPTION_SENSORS = {
    CONF_IMPORT_ACTIVE_ENERGY: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-export",
        ),
        key=CONF_NAME,
    ),
    CONF_IMPORT_REACTIVE_ENERGY: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-export",
        ),
        key=CONF_NAME,
    ),
    CONF_EXPORT_ACTIVE_ENERGY: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        key=CONF_NAME,
    ),
    CONF_EXPORT_REACTIVE_ENERGY: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE_HOURS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:transmission-tower-import",
        ),
        key=CONF_NAME,
    ),
}

TARIFF_CONSUMPTION_SCHEMA = cv.Schema(
    {cv.Optional(sensor): schema for sensor, schema in TARIFF_CONSUMPTION_SENSORS.items()}
)

CONF_PHASE = "phase"
CONF_NEUTRAL = "neutral"

PHASE_SENSORS = {
    CONF_CURRENT: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        key=CONF_NAME,
    ),
    CONF_ACTIVE_POWER: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        key=CONF_NAME,
    ),
    CONF_REACTIVE_POWER: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS_REACTIVE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        key=CONF_NAME,
    ),
    CONF_POWER_FACTOR: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement="",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_POWER_FACTOR,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        key=CONF_NAME,
    ),
    CONF_APPARENT_POWER: cv.maybe_simple_value(
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=3,
            icon=ICON_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        key=CONF_NAME,
    ),
}


PHASE_SCHEMA = cv.Schema(
    {cv.Optional(sensor): schema for sensor, schema in PHASE_SENSORS.items()}
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LE2_ID): cv.use_id(LE2Component),
        cv.Optional(CONF_TARIFF_1): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_2): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_3): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_4): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_5): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_6): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_7): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_TARIFF_8): TARIFF_CONSUMPTION_SCHEMA,
        cv.Optional(CONF_FREQUENCY): cv.maybe_simple_value(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                icon=ICON_CURRENT_AC,
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_VOLTAGE): cv.maybe_simple_value(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                icon=ICON_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_PHASE): PHASE_SCHEMA,
        cv.Optional(CONF_NEUTRAL): PHASE_SCHEMA,

    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_LE2_ID])

    for key in [
        CONF_FREQUENCY,
        CONF_VOLTAGE,
    ]:
        if key not in config:
            continue
        sensor_conf = config[key]
        sens = await sensor.new_sensor(sensor_conf)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))

    for i, phase in enumerate([CONF_PHASE, CONF_NEUTRAL]):
        if phase not in config:
            continue

        conf = config[phase]
        for j, sensor_key in enumerate(PHASE_SENSORS.keys()):
            if sensor_key not in conf:
                continue
            sens = await sensor.new_sensor(conf[sensor_key])
            cg.add(getattr(hub, f"set_phase_measurements_sensor")(i, j, sens))

    for i, tariff_key  in enumerate([CONF_TARIFF_1, CONF_TARIFF_2, CONF_TARIFF_3,
                                     CONF_TARIFF_4, CONF_TARIFF_5, CONF_TARIFF_6,
                                     CONF_TARIFF_7, CONF_TARIFF_8], start=0):
        if tariff_key not in config:
            continue
        conf = config[tariff_key]
        for j, sensor_key in enumerate(TARIFF_CONSUMPTION_SENSORS.keys(), start=0):
            if sensor_key not in conf:
                continue
            sens = await sensor.new_sensor(conf[sensor_key])
            cg.add(getattr(hub, f"set_tariff_consumption_sensor")(j, i, sens))
