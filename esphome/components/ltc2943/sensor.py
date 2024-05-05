import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_BATTERY_VOLTAGE,
    # CONF_CURRENT,
    # CONF_BATTERY_LEVEL,
    CONF_TEMPERATURE,
    # CONF_MAX_CURRENT,
    # CONF_SHUNT_RESISTANCE,
    # CONF_SHUNT_VOLTAGE,
    DEVICE_CLASS_VOLTAGE,
    # DEVICE_CLASS_CURRENT,
    # DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    # UNIT_AMPERE,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

ltc2943_ns = cg.esphome_ns.namespace("ltc2943")
LTC2943Component = ltc2943_ns.class_(
    "LTC2943Component", cg.PollingComponent, i2c.I2CDevice
)

CcPrescaleValue = ltc2943_ns.enum("CcPrescaleValue")
CC_PRESCALE_VALUES = {
    1: CcPrescaleValue.PRESCALE_1,
    4: CcPrescaleValue.PRESCALE_4,
    16: CcPrescaleValue.PRESCALE_16,
    64: CcPrescaleValue.PRESCALE_64,
    256: CcPrescaleValue.PRESCALE_256,
    1024: CcPrescaleValue.PRESCALE_1024,
    4096: CcPrescaleValue.PRESCALE_4096,
    "DEFAULT": CcPrescaleValue.PRESCALE_DEFAULT,
}

AlccMode = ltc2943_ns.enum("AlccMode")
ALCC_MODES = {
    "DISABLED": AlccMode.ALCC_MODE_DISABLED,
    "CHARGE_COMPLETE": AlccMode.ALCC_MODE_CHARGE_COMPLETE,
    "ALERT": AlccMode.ALCC_MODE_ALERT,
}


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LTC2943Component),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x64))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Link parameters from user
    # cg.add

    # Link Sensors
    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
