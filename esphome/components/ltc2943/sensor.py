import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_SHUNT_RESISTANCE,
    CONF_BATTERY_LEVEL,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

CONF_ALCC_MODE = "alcc_mode"
CONF_PRESCALER = "prescaler"
CONF_BATTERY_CAPACITY = "battery_capacity"

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
            # Configuration Params
            cv.Optional(CONF_SHUNT_RESISTANCE, default=0.05): cv.All(
                cv.resistance, cv.Range(min=0.0)
            ),
            cv.Required(CONF_BATTERY_CAPACITY): cv.All(cv.Range(min=1.0)),
            cv.Optional(CONF_ALCC_MODE, default="DISABLED"): cv.enum(
                ALCC_MODES, string=True
            ),
            cv.Optional(CONF_PRESCALER, default=4096): cv.enum(
                CC_PRESCALE_VALUES, int=True
            ),
            # Alert threshold parameters
            # TODO: Add these...
            # Sensors
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
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
    .extend(cv.polling_component_schema("30s"))
    .extend(i2c.i2c_device_schema(0x64))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Link configuration parameters
    cg.add(var.set_shunt_resistance_ohm(config[CONF_SHUNT_RESISTANCE]))
    cg.add(var.set_battery_capacity_mah(config[CONF_BATTERY_CAPACITY]))
    cg.add(var.set_alcc_pin_mode(config[CONF_ALCC_MODE]))
    cg.add(var.set_clmb_cnt_prescale_value(config[CONF_PRESCALER]))

    # Link Alert threshold parameters

    # Link Sensors
    if CONF_BATTERY_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery_soc_sensor(sens))

    if CONF_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))

    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT])
        cg.add(var.set_current_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
