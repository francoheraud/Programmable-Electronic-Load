#pragma once
#include <Arduino.h>

constexpr uint8_t ADS7128_I2C_ADDR          = 0x10;
constexpr uint8_t ADS7128_OP_SINGLE_WRITE   = 0x08;  // write one register
constexpr uint8_t ADS7128_OP_SINGLE_READ    = 0x10;  // read one register
constexpr uint8_t ADS7128_REG_GENERAL_CFG   = 0x01;
constexpr uint8_t ADS7128_REG_OSR_CFG       = 0x03;  // oversampling (averaging) ratio
constexpr uint8_t ADS7128_REG_PIN_CFG       = 0x05;  // analog-input vs GPIO per pin
constexpr uint8_t ADS7128_REG_CHANNEL_SEL   = 0x11;  // which channel manual-mode converts
constexpr uint8_t ADS7128_REG_RECENT_CH0_LSB = 0xA0; // RECENT_CHn = 0xA0 + 2*n (LSB), +1 (MSB)
constexpr uint8_t ADS7128_OSR_128 = 0x07;

class Ads7128 {
public:
  bool begin();
  bool readChannel(uint8_t channel, uint16_t &rawCode);
  bool readTemperatures(float tempsC[6]);
private:
  bool writeReg(uint8_t reg, uint8_t value);
};

extern Ads7128 ads7128;
