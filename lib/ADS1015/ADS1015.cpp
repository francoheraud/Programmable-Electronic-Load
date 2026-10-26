#include "ADS1015.h"
#include <Display.h>
#include <ExternalNtc.h>

Ads1015 ads1015;

static const uint16_t kChannelConfig[4] = {
  ADS1015_CFG_AIN0, ADS1015_CFG_AIN1, ADS1015_CFG_AIN2, ADS1015_CFG_AIN3
};

bool Ads1015::startConversion(uint8_t channel) {
  uint16_t cfg = kChannelConfig[channel];
  Wire2.beginTransmission(ADS1015_I2C_ADDR);
  Wire2.write(ADS1015_REG_CONFIG);
  Wire2.write((cfg >> 8) & 0xFF);   // MSB first
  Wire2.write(cfg & 0xFF);
  uint8_t status = Wire2.endTransmission();
  if (status != 0) {
    Serial.printf("ADS1015: start conv ch%u failed (I2C status %u)\n", channel, status);
    return false;
  }
  return true;
}

bool Ads1015::readConversion(uint8_t channel, int16_t &code) {
  Wire2.beginTransmission(ADS1015_I2C_ADDR);
  Wire2.write(ADS1015_REG_CONV);
  uint8_t status = Wire2.endTransmission(false);
  if (status != 0) {
    Serial.printf("ADS1015: conv read ch%u (addr phase) failed (I2C status %u)\n",
                  channel, status);
    return false;
  }
  uint8_t got = Wire2.requestFrom(ADS1015_I2C_ADDR, (uint8_t)2);
  if (got != 2) {
    Serial.printf("ADS1015: conv read ch%u got %u/2 bytes -- not responding\n",
                  channel, got);
    return false;
  }
  uint8_t msb = Wire2.read();
  uint8_t lsb = Wire2.read();

  int16_t raw16 = (int16_t)(((uint16_t)msb << 8) | lsb);
  code = raw16 >> 4;
  return true;
}

bool Ads1015::readTemperatures(float tempsC[4]) {
  bool allOk = true;
  // Missing/invalid probes are optional. NaN explicitly marks unavailable
  // samples; valid channels still update even when another probe is absent.
  float fresh[4] = {NAN, NAN, NAN, NAN};
  for (uint8_t ch = 0; ch < 4; ch++) {
    if (!startConversion(ch)) {
      allOk = false;
      continue;
    }

    delay(1);

    int16_t code;
    if (readConversion(ch, code)) {
      externalNtcCelsius(code, fresh[ch]);
    } else {
      allOk = false;
    }
  }
  for (uint8_t ch = 0; ch < 4; ++ch) tempsC[ch] = fresh[ch];
  return allOk;
}
