#include "INA.h"
#include <Wire.h>

Ina ina;

static const float kCurrentLsbAmps = 5.05f / 32768.0f;

// endTransmission() return codes (Wire.h): 0 = success, 1 = data too long,
// 2 = NACK on address, 3 = NACK on data, 4 = other error, 5 = timeout (the
// bus-level timeout set via Wire1.setTimeOut(), see initialiseSystem()).
bool Ina::writeAdcConfig(uint8_t i2cAddr, uint16_t configValue) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(INA_REG_ADC_CONFIG);
  Wire1.write((configValue >> 8) & 0xFF);   // MSB first
  Wire1.write(configValue & 0xFF);
  uint8_t status = Wire1.endTransmission();
  if (status != 0) {
    Serial.printf("INA 0x%02X: ADC config write failed (I2C status %u)\n", i2cAddr, status);
    return false;
  }
  return true;
}

bool Ina::writeShuntCal(uint8_t i2cAddr, uint16_t calValue) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(INA_REG_SHUNT_CAL);
  Wire1.write((calValue >> 8) & 0xFF);
  Wire1.write(calValue & 0xFF);
  uint8_t status = Wire1.endTransmission();
  if (status != 0) {
    Serial.printf("INA 0x%02X: shunt-cal write failed (I2C status %u)\n", i2cAddr, status);
    return false;
  }
  return true;
}

bool Ina::readBusVoltage(uint8_t i2cAddr, float &busVoltage) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(INA_REG_VBUS);
  uint8_t status = Wire1.endTransmission(false);
  if (status != 0) {
    Serial.printf("INA 0x%02X: VBUS read (address phase) failed (I2C status %u)\n", i2cAddr, status);
    return false;
  }
  uint8_t got = Wire1.requestFrom(i2cAddr, (uint8_t)2);
  if (got != 2) {
    Serial.printf("INA 0x%02X: VBUS read got %u/2 bytes -- device not responding\n", i2cAddr, got);
    return false;
  }
  uint16_t raw = (Wire1.read() << 8) | Wire1.read();
  busVoltage = raw * 3.125e-3f;
  return true;
}

bool Ina::readCurrent(uint8_t i2cAddr, float &current) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(INA_REG_CURRENT);
  uint8_t status = Wire1.endTransmission(false);
  if (status != 0) {
    Serial.printf("INA 0x%02X: current read (address phase) failed (I2C status %u)\n", i2cAddr, status);
    return false;
  }
  uint8_t got = Wire1.requestFrom(i2cAddr, (uint8_t)2);
  if (got != 2) {
    Serial.printf("INA 0x%02X: current read got %u/2 bytes -- device not responding\n", i2cAddr, got);
    return false;
  }
  int16_t raw = (Wire1.read() << 8) | Wire1.read();
  current = raw * kCurrentLsbAmps;
  return true;
}

bool Ina::begin(const uint8_t addrs[6], uint16_t shuntCalValue, uint8_t vbusLeg) {
  bool allOk = true;
  for (uint8_t i = 0; i < 6; i++) {
    if (!writeShuntCal(addrs[i], shuntCalValue)) {
      Serial.printf("Ina::begin: INA 0x%02X shunt-cal write failed\n", addrs[i]);
      allOk = false;
    }

    uint16_t adcCfg = (i == vbusLeg) ? INA_ADCCFG_SHUNT_AND_BUS
                                     : INA_ADCCFG_SHUNT_ONLY;
    if (!writeAdcConfig(addrs[i], adcCfg)) {
      Serial.printf("Ina::begin: INA 0x%02X ADC config write failed\n", addrs[i]);
      allOk = false;
    }
  }
  return allOk;
}
