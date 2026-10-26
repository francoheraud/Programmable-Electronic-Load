#include "ADS7128.h"
#include <Wire.h>

Ads7128 ads7128;
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

// TI ADS7128 sections 8.3.6 and 8.3.14: manual averaged conversions
// populate RECENT registers only with statistics enabled.
constexpr uint8_t REG_STATUS = 0x00;
constexpr uint8_t REG_OPMODE = 0x04;
constexpr uint8_t REG_SEQUENCE = 0x10;
constexpr uint8_t STATS_EN = 0x20;
constexpr uint8_t CNVST = 0x08;
constexpr uint8_t OSR_DONE = 0x08;
constexpr uint32_t CONVERSION_TIMEOUT_MS = 10;

bool Ads7128::readRegs(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire1.beginTransmission(ADS7128_I2C_ADDR);
  // Block read increments the address; single read repeats the same register.
  Wire1.write(count > 1 ? uint8_t(0x30) : ADS7128_OP_SINGLE_READ);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(ADS7128_I2C_ADDR, count) != count) {
    while (Wire1.available()) Wire1.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire1.read();
  return true;
}

bool Ads7128::begin() {
  return writeReg(ADS7128_REG_GENERAL_CFG, 0x00) &&
         writeReg(REG_SEQUENCE, 0x00) &&
         writeReg(REG_OPMODE, 0x00) &&
         writeReg(ADS7128_REG_PIN_CFG, 0x00) &&
         writeReg(ADS7128_REG_OSR_CFG, ADS7128_OSR_128);
}

bool Ads7128::readChannel(uint8_t channel, uint16_t &rawCode) {
  if (channel > 7) return false;
  if (!writeReg(ADS7128_REG_GENERAL_CFG, 0x00) ||
      !writeReg(ADS7128_REG_CHANNEL_SEL, channel) ||
      !writeReg(REG_STATUS, OSR_DONE) || // W1C: discard any old completion.
      !writeReg(ADS7128_REG_GENERAL_CFG, STATS_EN) ||
      !writeReg(ADS7128_REG_GENERAL_CFG, STATS_EN | CNVST)) return false;

  const uint32_t start = millis();
  bool ready = false;
  do {
    uint8_t status;
    if (!readRegs(REG_STATUS, &status, 1)) break;
    if (status & OSR_DONE) { ready = true; break; }
    delay(1); // Yield while the ADC performs its 128-sample average.
  } while (uint32_t(millis() - start) < CONVERSION_TIMEOUT_MS);

  // Freeze statistics before reading the adjacent LSB/MSB registers.
  if (!writeReg(ADS7128_REG_GENERAL_CFG, 0x00)) return false;
  if (!ready) {
    Serial.printf("ADS7128: ch%u conversion failed or timed out\n", channel);
    return false;
  }
  uint8_t data[2];
  if (!readRegs(ADS7128_REG_RECENT_CH0_LSB + 2 * channel, data, 2)) return false;
  rawCode = (uint16_t(data[1]) << 8) | data[0];
  return true;
}

static const float NTC_FIXED_R_OHMS = 10000.0f;
static const float NTC_R0_OHMS      = 10000.0f;
static const float NTC_T0_KELVIN    = 298.15f; 
static const float NTC_BETA         = 3950.0f;
static const float ADC_FULL_SCALE   = 65536.0f;

static bool ntcCodeToCelsius(uint16_t raw, float &temperature) {
  if (raw == 0 || raw == 0xFFFF) return false;
  float ratio = (float)raw / ADC_FULL_SCALE;
  float rNtc = NTC_FIXED_R_OHMS * ratio / (1.0f - ratio);

  float invT = (1.0f / NTC_T0_KELVIN) + (1.0f / NTC_BETA) * logf(rNtc / NTC_R0_OHMS);
  if (!isfinite(invT) || invT <= 0.0f) return false;
  temperature = 1.0f / invT - 273.15f;
  return isfinite(temperature);
}

bool Ads7128::readTemperatures(float tempsC[6]) {
  float readings[6];
  for (uint8_t ch = 0; ch < 6; ch++) {
    uint16_t raw;
    if (!readChannel(ch, raw)) return false;
    if (!ntcCodeToCelsius(raw, readings[ch])) {
      Serial.printf("ADS7128: ch%u invalid NTC code %u\n", ch, raw);
      return false;
    }
  }
  // Publish only a complete valid scan. Callers retain the existing fault path.
  for (uint8_t ch = 0; ch < 6; ch++) tempsC[ch] = readings[ch];
  return true;
}
