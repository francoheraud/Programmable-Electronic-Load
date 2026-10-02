#include "ADS7128.h"
#include <Wire.h>

Ads7128 ads7128;

// Write one ADS7128 register: [opcode 0x08][reg addr][data].
bool Ads7128::writeReg(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(ADS7128_I2C_ADDR);
  Wire1.write(ADS7128_OP_SINGLE_WRITE);
  Wire1.write(reg);
  Wire1.write(value);
  uint8_t status = Wire1.endTransmission();
  if (status != 0) {
    Serial.printf("ADS7128: write reg 0x%02X failed (I2C status %u)\n", reg, status);
    return false;
  }
  return true;
}

bool Ads7128::begin() {
  bool ok = true;

  // PIN_CFG = 0x00 -> every channel is an analog input (0 = analog, 1 = GPIO).
  // Also the reset default, but written explicitly so intent is clear and
  // we don't depend on prior state.
  ok &= writeReg(ADS7128_REG_PIN_CFG, 0x00);

  // OSR_CFG -> 128x oversampling for clean, slow temperature readings.
  ok &= writeReg(ADS7128_REG_OSR_CFG, ADS7128_OSR_128);

  if (!ok) {
    Serial.println("Ads7128::begin: configuration failed -- check wiring/address 0x10");
  }
  return ok;
}

bool Ads7128::readChannel(uint8_t channel, uint16_t &rawCode) {
  if (channel > 7) {
    Serial.printf("ADS7128: channel %u out of range (0-7)\n", channel);
    return false;
  }

  // 1. Select the channel to convert (manual mode).
  if (!writeReg(ADS7128_REG_CHANNEL_SEL, channel)) {
    return false;  // write already logged the failure
  }

  // 2. Read the RECENT_CHn result registers (LSB then MSB) for this channel.
  //    RECENT_CHn_LSB = 0xA0 + 2*n, MSB = that + 1.
  uint8_t lsbReg = ADS7128_REG_RECENT_CH0_LSB + (channel * 2);

  Wire1.beginTransmission(ADS7128_I2C_ADDR);
  Wire1.write(ADS7128_OP_SINGLE_READ);
  Wire1.write(lsbReg);
  uint8_t status = Wire1.endTransmission(false);  // repeated START
  if (status != 0) {
    Serial.printf("ADS7128: ch%u read (address phase) failed (I2C status %u)\n",
                  channel, status);
    return false;
  }

  uint8_t got = Wire1.requestFrom(ADS7128_I2C_ADDR, (uint8_t)2);
  if (got != 2) {
    Serial.printf("ADS7128: ch%u read got %u/2 bytes -- not responding\n", channel, got);
    return false;
  }

  uint8_t lsb = Wire1.read();
  uint8_t msb = Wire1.read();

  // Averaging is enabled (OSR_CFG = 128x), so the device returns a full
  // 16-bit averaged result (datasheet Figure 25, Frame B) -- NOT the
  // 12-bit left-justified format. Do not shift; use the full 16 bits.
  rawCode = ((uint16_t)msb << 8) | lsb;
  return true;
}

// TODO: confirm BETA against the thermistor's datasheet.
static const float NTC_FIXED_R_OHMS = 10000.0f;  // R51..R57 top resistor
static const float NTC_R0_OHMS      = 10000.0f;  // NTC resistance at T0
static const float NTC_T0_KELVIN    = 298.15f;   // 25 C reference
static const float NTC_BETA         = 3950.0f;   // TODO: confirm from datasheet
static const float ADC_FULL_SCALE   = 65535.0f;  // 16-bit averaged result

// Convert one raw 16-bit ADC code to temperature in Celsius.
static float ntcCodeToCelsius(uint16_t raw) {
  float ratio = (float)raw / ADC_FULL_SCALE;
  if (ratio <= 0.0f)  ratio = 1e-6f;
  if (ratio >= 1.0f)  ratio = 1.0f - 1e-6f;
  float rNtc = NTC_FIXED_R_OHMS * ratio / (1.0f - ratio);

  float invT = (1.0f / NTC_T0_KELVIN) + (1.0f / NTC_BETA) * logf(rNtc / NTC_R0_OHMS);
  float tKelvin = 1.0f / invT;
  return tKelvin - 273.15f;
}

bool Ads7128::readTemperatures(float tempsC[6]) {
  bool allOk = true;
  for (uint8_t ch = 0; ch < 6; ch++) {
    uint16_t raw;
    if (readChannel(ch, raw)) {
      tempsC[ch] = ntcCodeToCelsius(raw);
    } else {
      allOk = false;
    }
  }
  return allOk;
}
