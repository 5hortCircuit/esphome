#include "ltc2943.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>

namespace esphome {
namespace ltc2943 {

static const char *const TAG = "ltc2943";

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

//------------------------------
// Constants
//------------------------------
static const float LTC2943_FULLSCALE_VOLTAGE = 23.6F;                                 // VDC
static const float LTC2943_VOLTAGE_LSB = LTC2943_FULLSCALE_VOLTAGE / 0xFFFF;          // VDC per bit
static const float LTC2943_FULLSCALE_TEMPERATURE = 510.0F;                            // *K
static const float LTC2943_TEMPERATURE_LSB = LTC2943_FULLSCALE_TEMPERATURE / 0xFFFF;  // *K per bit
static const float LTC2943_FULLSCALE_CURRENT = 60E-3;                                 // ADC

//------------------------------
// ESPHome Functions
//------------------------------
void LTC2943Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLC2943...");

  ControlRegister ctrlReg;
  ctrlReg.shutdown = 1;                                // B[0]   - Shutdown 0=Run 1=Off
  ctrlReg.alcc_config = this->alcc_pin_mode_;          // B[2:1] - ALCC Configure
  ctrlReg.prescaler = this->clmb_cnt_prescale_value_;  // B[5:3] - Prescaler M
  ctrlReg.mode = 0b11;                                 // B[7:6] - ADC Mode -> Automatic 0b11

  esphome::i2c::ErrorCode error;

  // Write the control register with the shutdown set so that Accumulated Charge register can be reset
  error = this->write_register(LTC2943_REGISTER_CONTROL, &ctrlReg.raw, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "Init error; I2C code: %d", error);
    this->mark_failed();
    return;
  }

  uint8_t value[2] = {0xFF, 0xFF};
  error = this->write_register(LTC2943_REGISTER_ACCUM_CHG_MSB, &value[0], 2, false);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "Accum Chg Init error; I2C code: %d", error);
    this->mark_failed();
    return;
  }

  // Turn back on
  ctrlReg.shutdown = 0;  // B[0]   - Shutdown 0=Run 1=Off
  error = this->write_register(LTC2943_REGISTER_CONTROL, &ctrlReg.raw, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "Init error; I2C code: %d", error);
    this->mark_failed();
    return;
  }

  if (this->alcc_pin_mode_ == AlccMode::ALCC_MODE_ALERT) {
    // TODO: Check and load the alert values
  }

  // q_lsb = 0.340mAh • (50mΩ/RSENSE) • (M/4096)
  this->q_lsb_ = 0.00034F * (0.050F / this->shunt_resistance_ohm_) *
                 (LTC2943_PRESCALER_VAlUES[this->clmb_cnt_prescale_value_ & 0b111] / 4096.0F);
  this->q_lsb_ *= 1000;  // Convert to mA
}

void LTC2943Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LTC2943:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with LTC2943 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  ESP_LOGCONFIG(TAG, "  Battery Capacity (mAh): %f", this->battery_capacity_mah_);
  ESP_LOGCONFIG(TAG, "  Shunt Resistor Value: %f", this->shunt_resistance_ohm_);
  ESP_LOGCONFIG(TAG, "  Prescaling factor M: %d (raw: %d)",
                LTC2943_PRESCALER_VAlUES[this->clmb_cnt_prescale_value_ & 0b111], this->clmb_cnt_prescale_value_);
  ESP_LOGCONFIG(TAG, "  ALCC pin configuration: %s", LTC2943_ALCC_MODES[this->alcc_pin_mode_]);
  ESP_LOGCONFIG(TAG, "  mQ_LSB: %f", this->q_lsb_);

  LOG_SENSOR("  ", "Battery SOC", this->bat_soc_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", this->bat_voltage_sensor_);
  LOG_SENSOR("  ", "Battery Current", this->bat_current_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

float LTC2943Component::get_setup_priority() const { return setup_priority::DATA; }

void LTC2943Component::update() {
  if (this->bat_soc_sensor_ != nullptr) {
    uint8_t data[2];
    if (this->read_register(LTC2943_REGISTER_ACCUM_CHG_MSB, &data[0], 2, false)) {
      this->status_set_warning();
      return;
    }
    uint16_t adc_cnt = ((uint16_t) (data[0] << 8) | data[1]);
    // Convert to consumed Ah
    float consumed_mah = adc_cnt_to_mah_consumed_(adc_cnt);
    float soc = ((this->battery_capacity_mah_ - consumed_mah) / this->battery_capacity_mah_) * 100.0;
    this->bat_soc_sensor_->publish_state(soc);
  }

  if (this->bat_voltage_sensor_ != nullptr) {
    uint8_t data[2];
    if (this->read_register(LTC2943_REGISTER_VOLTAGE_MSB, &data[0], 2, false)) {
      this->status_set_warning();
      return;
    }
    uint16_t adc_cnt = ((uint16_t) (data[0] << 8) | data[1]);
    // Convert to voltage
    float bat_voltage = adc_cnt_to_voltage_(adc_cnt);
    this->bat_voltage_sensor_->publish_state(bat_voltage);
  }

  if (this->bat_current_sensor_ != nullptr) {
    uint8_t data[2];
    if (this->read_register(LTC2943_REGISTER_CURRENT_MSB, &data[0], 2, false)) {
      this->status_set_warning();
      return;
    }
    uint16_t adc_cnt = ((uint16_t) (data[0] << 8) | data[1]);
    // Convert to current
    float bat_current = adc_cnt_to_current_(adc_cnt, this->shunt_resistance_ohm_);
    this->bat_current_sensor_->publish_state(bat_current);
  }

  if (this->temperature_sensor_ != nullptr) {
    uint8_t data[2];
    if (this->read_register(LTC2943_REGISTER_TEMPERATURE_MSB, &data[0], 2, false)) {
      this->status_set_warning();
      return;
    }
    uint16_t adc_cnt = ((uint16_t) (data[0] << 8) | data[1]);
    // Convert to temperature
    float temperature_c = adc_cnt_to_celcius_temperature_(adc_cnt);
    this->temperature_sensor_->publish_state(temperature_c);
  }

  this->status_clear_warning();
}

//------------------------------
// Conversion Functions
//------------------------------

// Convert the 16-bit raw ADC code to Ah
float LTC2943Component::adc_cnt_to_mah_consumed_(uint16_t adc_code) {
  float ah_consumed;
  adc_code = 0xFFFFU - adc_code;
  ah_consumed = (float) adc_code * this->q_lsb_;
  return ah_consumed;
}

// Convert the State Of Charge % to a 16-bit raw ADC code
uint16_t LTC2943Component::soc_percent_to_mah_adc_cnt_(uint8_t soc, float capacity) {
  uint16_t adc_code;
  float mah = (1 - (soc / 100.0F)) * capacity;
  adc_code = 0xFFFF - (uint16_t) (mah / this->q_lsb_);
  return adc_code;
}

// Convert the 16-bit raw ADC code to voltage
float LTC2943Component::adc_cnt_to_voltage_(uint16_t adc_code) { return adc_code * LTC2943_VOLTAGE_LSB; }

// Convert a voltage to a 16-bit raw ADC code
uint16_t LTC2943Component::voltage_to_adc_cnt_(float voltage) { return (uint16_t) (voltage / LTC2943_VOLTAGE_LSB); }

// Convert the 16-bit raw ADC code to amperes
float LTC2943Component::adc_cnt_to_current_(uint16_t adc_code, float resistor) {
  return (((float) adc_code - 0x7FFF) / 0x7FFF) * (LTC2943_FULLSCALE_CURRENT / resistor);
}

// Convert amperes to a 16-bit raw ADC code
uint16_t LTC2943Component::current_to_adc_cnt_(float current, float resistor) {
  return (uint16_t) (((current * resistor) / LTC2943_FULLSCALE_CURRENT) * 0x7FFF) + 0x7FFF;
}

// Convert the 16-bit raw ADC code to Celcius
float LTC2943Component::adc_cnt_to_celcius_temperature_(uint16_t adc_code) {
  return (adc_code * LTC2943_TEMPERATURE_LSB) - 273.15;
}

// Convert a Celcius temperature into a 8-bit raw ADC code for threshold
uint8_t LTC2943Component::celcius_temperature_to_thld_code_(float temperature) {
  return (uint8_t) ((temperature + 273.15) / 2);
}

}  // namespace ltc2943
}  // namespace esphome
