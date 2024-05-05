#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

#define BYTESWAP16(n) (((n & 0xFF00) >> 8) | ((n & 0x00FF) << 8))

namespace esphome {
namespace ltc2943 {

// +------------------------------------------------------------+
// |                    TABLE 1. Register Map                   |
// +---------+------+----------------------------+----+---------+
// | ADDRESS | NAME | DESC                       | RW | DEFAULT |
// +---------+------+----------------------------+----+---------+
// |  0x00   |  A   | Status                     | R  |   N/A   |
// |  0x01   |  B   | Control                    | RW |   0x3C  |
// |  0x02   |  C   | Accumulated Charge MSB     | RW |   0x7F  |
// |  0x03   |  D   | Accumulated Charge LSB     | RW |   0xFF  |
// |  0x04   |  E   | Charge Threshold High MSB  | RW |   0xFF  |
// |  0x05   |  F   | Charge Threshold High LSB  | RW |   0xFF  |
// |  0x06   |  G   | Charge Threshold Low MSB   | RW |   0x00  |
// |  0x07   |  H   | Charge Threshold Low LSB   | RW |   0x00  |
// |  0x08   |  I   | Voltage MSB                | R  |   0x00  |
// |  0x09   |  J   | Voltage LSB                | R  |   0x00  |
// |  0x0A   |  K   | Voltage Threshold High MSB | RW |   0xFF  |
// |  0x0B   |  L   | Voltage Threshold High LSB | RW |   0xFF  |
// |  0x0C   |  M   | Voltage Threshold Low MSB  | RW |   0x00  |
// |  0x0D   |  N   | Voltage Threshold Low LSB  | RW |   0x00  |
// |  0x0E   |  O   | Current MSB                | R  |   0x00  |
// |  0x0F   |  P   | Current LSB                | R  |   0x00  |
// |  0x10   |  Q   | Current Threshold High MSB | RW |   0xFF  |
// |  0x11   |  R   | Current Threshold High LSB | RW |   0xFF  |
// |  0x12   |  S   | Current Threshold Low MSB  | RW |   0x00  |
// |  0x13   |  T   | Current Threshold Low LSB  | RW |   0x00  |
// |  0x14   |  U   | Temperature MSB            | R  |   0x00  |
// |  0x15   |  V   | Temperature LSB            | R  |   0x00  |
// |  0x16   |  W   | Temperature Threshold Hi   | RW |   0xFF  |
// |  0x17   |  X   | Temperature Threshold Lo   | RW |   0x00  |
// +---------+------+----------------------------+----+---------+

static const uint8_t LTC2943_REGISTER_STATUS = 0x00;
static const uint8_t LTC2943_REGISTER_CONTROL = 0x01;
static const uint8_t LTC2943_REGISTER_ACCUM_CHG_MSB = 0x02;
static const uint8_t LTC2943_REGISTER_ACCUM_CHG_LSB = 0x03;
static const uint8_t LTC2943_REGISTER_CHG_THLD_HI_MSB = 0x04;
static const uint8_t LTC2943_REGISTER_CHG_THLD_HI_LSB = 0x05;
static const uint8_t LTC2943_REGISTER_CHG_THLD_LO_MSB = 0x06;
static const uint8_t LTC2943_REGISTER_CHG_THLD_LO_LSB = 0x07;
static const uint8_t LTC2943_REGISTER_VOLTAGE_MSB = 0x08;
static const uint8_t LTC2943_REGISTER_VOLTAGE_LSB = 0x09;
static const uint8_t LTC2943_REGISTER_VOLTAGE_THLD_HI_MSB = 0x0A;
static const uint8_t LTC2943_REGISTER_VOLTAGE_THLD_HI_LSB = 0x0B;
static const uint8_t LTC2943_REGISTER_VOLTAGE_THLD_LO_MSB = 0x0C;
static const uint8_t LTC2943_REGISTER_VOLTAGE_THLD_LO_LSB = 0x0D;
static const uint8_t LTC2943_REGISTER_CURRENT_MSB = 0x0E;
static const uint8_t LTC2943_REGISTER_CURRENT_LSB = 0x0F;
static const uint8_t LTC2943_REGISTER_CURRENT_THLD_HI_MSB = 0x10;
static const uint8_t LTC2943_REGISTER_CURRENT_THLD_HI_LSB = 0x11;
static const uint8_t LTC2943_REGISTER_CURRENT_THLD_LO_MSB = 0x12;
static const uint8_t LTC2943_REGISTER_CURRENT_THLD_LO_LSB = 0x13;
static const uint8_t LTC2943_REGISTER_TEMPERATURE_MSB = 0x14;
static const uint8_t LTC2943_REGISTER_TEMPERATURE_LSB = 0x15;
static const uint8_t LTC2943_REGISTER_TEMPERATURE_THLD_HI = 0x16;
static const uint8_t LTC2943_REGISTER_TEMPERATURE_THLD_LO = 0x17;

enum AlccMode : uint16_t { ALCC_MODE_DISABLED = 0, ALCC_MODE_CHARGE_COMPLETE = 1, ALCC_MODE_ALERT = 2 };

static const uint16_t LTC2943_PRESCALER_VAlUES[] = {1, 4, 16, 64, 256, 1024, 4096, 4096};

enum CcPrescaleValue : uint16_t {
  PRESCALE_1 = 0,
  PRESCALE_4 = 1,
  PRESCALE_16 = 2,
  PRESCALE_64 = 3,
  PRESCALE_256 = 4,
  PRESCALE_1024 = 5,
  PRESCALE_4096 = 6,
  PRESCALE_DEFAULT = 7
};

union StatusRegister {
  uint8_t raw;
  struct {
    uint8_t undervoltage_lockout_alert : 1;
    uint8_t voltage_alert : 1;
    uint8_t charge_alert_low : 1;
    uint8_t charge_alert_high : 1;
    uint8_t temperature_alert : 1;
    uint8_t accumulated_charge_over_underflow : 1;
    uint8_t current_alert : 1;
    uint8_t reserved : 1;
  } __attribute__((packed));
};

union ControlRegister {
  uint8_t raw;
  struct {
    uint8_t shutdown : 1;
    AlccMode alcc_config : 2;
    CcPrescaleValue prescaler : 3;
    uint8_t mode : 2;
  } __attribute__((packed));
};

union SplitShort {
  uint16_t value;
  struct {
    uint8_t high;
    uint8_t low;
  } __attribute__((packed));
};

union LTC2943Registers {
  uint8_t raw[24];
  struct {
    StatusRegister status;
    ControlRegister control;
    SplitShort raw_accumChg;
    SplitShort chg_thld_high;
    SplitShort chg_thld_low;
    SplitShort raw_voltage;
    SplitShort voltage_thld_high;
    SplitShort voltage_thld_low;
    SplitShort raw_current;
    SplitShort current_thld_high;
    SplitShort current_thld_low;
    SplitShort raw_temperature;
    uint8_t temperature_thld_high;
    uint8_t temperature_thld_low;
  } __attribute__((packed));
};

class LTC2943Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_shunt_resistance_ohm(float shunt_resistance_ohm) { shunt_resistance_ohm_ = shunt_resistance_ohm; }
  void set_battery_capacity_mah(float battery_capacity_mah) { battery_capacity_mah_ = battery_capacity_mah; }
  void set_alcc_pin_mode(AlccMode mode) { alcc_pin_mode_ = mode; }
  void set_clmb_cnt_prescale_value(CcPrescaleValue value) { clmb_cnt_prescale_value_ = value; }

  void set_battery_voltage_sensor(sensor::Sensor *sensor) { bat_voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  // void set_soc_override(number::Number *number) { soc_override_ = number; }

 protected:
  float shunt_resistance_ohm_;
  float battery_capacity_mah_;
  AlccMode alcc_pin_mode_{AlccMode::ALCC_MODE_ALERT};
  CcPrescaleValue clmb_cnt_prescale_value_{CcPrescaleValue::PRESCALE_DEFAULT};
  SplitShort accum_chg_;
  SplitShort chg_thld_hi_{0xFFFF};
  SplitShort chg_thld_lo_{0x0000};
  SplitShort voltage_;
  SplitShort voltage_thld_hi_{0xFFFF};
  SplitShort voltage_thld_lo_{0x0000};
  SplitShort current_;
  SplitShort current_thld_hi_{0xFFFF};
  SplitShort current_thld_lo_{0x0000};
  SplitShort temperature_;
  uint8_t temperatre_thld_hi{0xFF};
  uint8_t temperatre_thld_lo{0x00};
  sensor::Sensor *bat_voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
};

}  // namespace ltc2943
}  // namespace esphome
