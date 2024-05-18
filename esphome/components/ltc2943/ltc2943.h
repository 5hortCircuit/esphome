#pragma once

#include "float.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ltc2943 {

static const char *LTC2943_ALCC_MODES[3] = {"DISABLED", "CC", "AL"};

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

class LTC2943Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  // Configuration Params
  void set_shunt_resistance_ohm(float value) { shunt_resistance_ohm_ = value; }
  void set_battery_capacity_mah(float value) { battery_capacity_mah_ = value; }
  void set_alcc_pin_mode(AlccMode mode) { alcc_pin_mode_ = mode; }
  void set_clmb_cnt_prescale_value(CcPrescaleValue value) { clmb_cnt_prescale_value_ = value; }
  // Alert threshold parameters
  void set_chg_thld_hi(float value) { chg_thld_hi_ = value; }
  void set_chg_thld_lo(float value) { chg_thld_lo_ = value; }
  void set_voltage_thld_hi(float value) { voltage_thld_hi_ = value; }
  void set_voltage_thld_lo(float value) { voltage_thld_lo_ = value; }
  void set_current_thld_hi(float value) { current_thld_hi_ = value; }
  void set_current_thld_lo(float value) { current_thld_lo_ = value; }
  void set_temperature_thld_hi(float value) { temperatre_thld_hi_ = value; }
  void set_temperature_thld_lo(float value) { temperatre_thld_lo_ = value; }
  // Sensors
  void set_battery_soc_sensor(sensor::Sensor *sensor) { bat_soc_sensor_ = sensor; }
  void set_battery_voltage_sensor(sensor::Sensor *sensor) { bat_voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { bat_current_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  // void set_soc_override(number::Number *number) { soc_override_ = number; }

 protected:
  // Configuration params
  float shunt_resistance_ohm_;
  float battery_capacity_mah_;
  AlccMode alcc_pin_mode_;
  CcPrescaleValue clmb_cnt_prescale_value_;
  float q_lsb_{0.000340F};
  // Alert threshold parameters
  float chg_thld_hi_{100.0F};
  float chg_thld_lo_{0.0F};
  float voltage_thld_hi_{23.6F};
  float voltage_thld_lo_{0.0F};
  float current_thld_hi_{FLT_MAX};
  float current_thld_lo_{-1.0 * FLT_MAX};
  float temperatre_thld_hi_{FLT_MAX};
  float temperatre_thld_lo_{-1.0 * FLT_MAX};
  // Sensors
  sensor::Sensor *bat_soc_sensor_{nullptr};
  sensor::Sensor *bat_voltage_sensor_{nullptr};
  sensor::Sensor *bat_current_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  //------------------------------
  // Conversion Functions
  //------------------------------
  float adc_cnt_to_mah_consumed_(uint16_t adc_code);
  uint16_t soc_percent_to_mah_adc_cnt_(uint8_t soc, float capacity);
  float adc_cnt_to_voltage_(uint16_t adc_code);
  uint16_t voltage_to_adc_cnt_(float voltage);
  float adc_cnt_to_current_(uint16_t adc_code, float resistor);
  uint16_t current_to_adc_cnt_(float current, float resistor);
  float adc_cnt_to_celcius_temperature_(uint16_t adc_code);
  uint8_t celcius_temperature_to_thld_code_(float temperature);
};

}  // namespace ltc2943
}  // namespace esphome
