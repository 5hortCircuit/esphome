#include "ltc2943.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>

namespace esphome {
namespace ltc2943 {

static const char *const TAG = "ltc2943";

void LTC2943Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLC2943...");

  ControlRegister ctrlReg;
  // B[0] - Shutdown 0=Run 1=Off
  ctrlReg.shutdown = 0;
  // B[2:1] - ALCC Configure
  ctrlReg.alcc_config = this->alcc_pin_mode_;
  // B[5:3] - Prescaler M
  ctrlReg.prescaler = this->clmb_cnt_prescale_value_;
  // B[7:6] - ADC Mode
  ctrlReg.mode = 0b11;  // Automatic Mode: continuously performing voltage, current and temperature conversions
  // ctrlReg.mode = 0b10;  // Scan Mode: performing voltage, current and temperature conversion every 10s

  esphome::i2c::ErrorCode error = this->write_register(LTC2943_REGISTER_CONTROL, &ctrlReg.raw, 1);

  if (error != 0) {
    ESP_LOGCONFIG(TAG, "Init error; I2C code: %d", error);
    this->mark_failed();
    return;
  }

  //  add_on_state_callback() for number::Number
}

void LTC2943Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LTC2943:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with LTC2943 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  ESP_LOGCONFIG(TAG, "  Coulomb counter prescaling factor M: %d",
                LTC2943_PRESCALER_VAlUES[this->clmb_cnt_prescale_value_ & 0b111]);
  ESP_LOGCONFIG(TAG, "  ALCC pin configuration: %d", this->alcc_pin_mode_);

  LOG_SENSOR("  ", "Voltage", this->bat_voltage_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

float LTC2943Component::get_setup_priority() const { return setup_priority::DATA; }

void LTC2943Component::update() {
  esphome::i2c::ErrorCode i2c_error_code;

  // Dump all of the data
  uint8_t register_values[24];
  i2c_error_code = this->read_register(LTC2943_REGISTER_STATUS, &register_values[0], sizeof(register_values));
  if (i2c_error_code != i2c::ERROR_OK) {
    this->status_set_warning("ltc2943: Bad register readback.");
    return;
  }
  // Copy it to the structure
  LTC2943Registers raw_reg_data;
  memcpy(&raw_reg_data, &register_values, sizeof(register_values));

  if (this->temperature_sensor_ != nullptr) {
    /*
    SplitShort raw_temperature;
    if (!this->write(&LTC2943_REGISTER_TEMPERATURE_MSB, 1)) {
      this->status_set_warning();
      return;
    }
    if (!this->read(&raw_temperature.high, 2)) {
      this->status_set_warning();
      return;
    }
    ESP_LOGD(TAG, "TEMP RAW: %d HIGH: %d LOW: %d", raw_temperature.value, raw_temperature.high, raw_temperature.low);

    // i2c::I2CRegister tempReg = this->reg(LTC2943_REGISTER_TEMPERATURE_MSB);
    // tempReg.get();
    */
    /*
    if (!this->read_register(LTC2943_REGISTER_TEMPERATURE_MSB, &raw_temperature.low, 2)) {
      this->status_set_warning("Unable to read temperature from LTC2943.");
      return;
    }
    i2cErrorCode = this->read_register(LTC2943_REGISTER_TEMPERATURE_MSB, &raw_temperature.high, 2);
    if (i2cErrorCode != i2c::NO_ERROR) {
      ESP_LOGD(TAG, "ltc2943: i2c read error during temperature: %d", i2cErrorCode);
      this->status_set_warning();
      return;
    }
    // if (!this->read_register(LTC2943_REGISTER_TEMPERATURE_MSB, &raw_temperature.high, 1)) {
    //   this->status_set_warning();
    //   return;
    // }
    ESP_LOGD(TAG, "raw_temperature: %d", raw_temperature.value);
    */
    // Convert to temperature 510*(raw/65535) - 273.15
    raw_reg_data.raw_temperature.value = BYTESWAP16(raw_reg_data.raw_temperature.value);
    float temperature = (510.0f * (raw_reg_data.raw_temperature.value / 65535.0f)) - 273.15f;
    this->temperature_sensor_->publish_state(temperature);
  }

  if (this->bat_voltage_sensor_ != nullptr) {
    /*
    SplitShort raw_bat_voltage;
    if (!this->read_byte(LTC2943_REGISTER_VOLTAGE_LSB, &raw_bat_voltage.low)) {
      this->status_set_warning();
      return;
    }
    if (!this->read_byte(LTC2943_REGISTER_VOLTAGE_MSB, &raw_bat_voltage.high)) {
      this->status_set_warning();
      return;
    }
    */
    // Conver to voltage
    raw_reg_data.raw_voltage.value = BYTESWAP16(raw_reg_data.raw_voltage.value);
    float bat_voltage = 23.6f * (raw_reg_data.raw_voltage.value / 65535.0f);
    this->bat_voltage_sensor_->publish_state(bat_voltage);
  }

  this->status_clear_warning();
}

}  // namespace ltc2943
}  // namespace esphome
