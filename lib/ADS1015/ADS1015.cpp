#include "ADS1015.h"
#include <Display.h>

Ads1015 ads1015;

// NTC / divider constants -- same external divider topology as the heatsink
// sensors. TODO: confirm BETA against the probe thermistor's datasheet.
static const float NTC_FIXED_R_OHMS = 10000.0f;  // R29/R36/R37/R45, on PCB
static const float NTC_R0_OHMS      = 10000.0f;  // NTC resistance at T0
static const float NTC_T0_KELVIN    = 298.15f;   // 25 C reference
static const float NTC_BETA         = 3950.0f;   // TODO: confirm from datasheet

// ADS1015 is 12-bit, left-justified in a 16-bit word (result in bits 15:4).
// Gain=2 => full-scale = +/-2.048V, so +full-scale (0x7FF0) = 2.048V.
// Single-ended readings are 0..+FS, i.e. raw code 0..2047 -> 0..2.048V.
static const float ADS1015_FS_VOLTS = 2.048f;
static const int   ADS1015_FS_CODES = 2047;      // 11 bits for single-ended +FS

// Config value for each channel, indexed 0..3.
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
  uint8_t status = Wire2.endTransmission(false);  // repeated START
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

static float ads1015CodeToCelsius(int16_t code) {
  if (code < 0) code = 0;  // single-ended shouldn't go negative; clamp

  // Voltage across the NTC from the ADC code.
  float vNtc = ((float)code / (float)ADS1015_FS_CODES) * ADS1015_FS_VOLTS;

  // Guard the divider math against the rails.
  const float vSupply = 3.3f;
  if (vNtc <= 0.0f)        vNtc = 1e-4f;
  if (vNtc >= vSupply)     vNtc = vSupply - 1e-4f;

  float rNtc = NTC_FIXED_R_OHMS * vNtc / (vSupply - vNtc);
  float invT = (1.0f / NTC_T0_KELVIN) + (1.0f / NTC_BETA) * logf(rNtc / NTC_R0_OHMS);
  return (1.0f / invT) - 273.15f;
}

bool Ads1015::readTemperatures(float tempsC[4]) {
  bool allOk = true;
  for (uint8_t ch = 0; ch < 4; ch++) {
    if (!startConversion(ch)) {
      allOk = false;
      continue;
    }

    delay(1);

    int16_t code;
    if (readConversion(ch, code)) {
      tempsC[ch] = ads1015CodeToCelsius(code);
    } else {
      allOk = false;
    }
  }
  return allOk;
}
