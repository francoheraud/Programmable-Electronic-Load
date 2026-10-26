#pragma once
#include <Arduino.h>

//  PGA +/-4.096V covers the full 3.3V thermistor divider.
//  Probes are optional: unavailable channels are NaN.
//  readTemperatures returns false for ADC communication failures only.

constexpr uint8_t ADS1015_I2C_ADDR = 0x48;
constexpr uint8_t ADS1015_REG_CONV   = 0x00;
constexpr uint8_t ADS1015_REG_CONFIG = 0x01;
constexpr uint16_t ADS1015_CFG_AIN0 = 0xC383;
constexpr uint16_t ADS1015_CFG_AIN1 = 0xD383;
constexpr uint16_t ADS1015_CFG_AIN2 = 0xE383;
constexpr uint16_t ADS1015_CFG_AIN3 = 0xF383;

class Ads1015 {
public:
  bool readTemperatures(float tempsC[4]);

private:
  bool startConversion(uint8_t channel);
  bool readConversion(uint8_t channel, int16_t &code);
};

extern Ads1015 ads1015;
