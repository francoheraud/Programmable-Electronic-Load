#pragma once
#include <Arduino.h>

#define INA_REG_SHUNT_CAL   0x02
#define INA_REG_ADC_CONFIG  0x01
#define INA_REG_VBUS        0x05
#define INA_REG_CURRENT     0x07
#define INA_REG_POWER       0x08

#define INA_ADCCFG_SHUNT_ONLY     0xA902
#define INA_ADCCFG_SHUNT_AND_BUS  0xB902

#define I2C_TRANSACTION_TIMEOUT_MS 5

class Ina {
public:
  // Configures all 6 INAs (shunt-cal + ADC config, matching the per-leg
  // VBUS_AND_SHUNT/SHUNT_ONLY split). Assumes Wire1 is already begun.
  // addrs must point to exactly 6 I2C addresses. Returns false if any
  // individual chip write failed (each failure is still logged per-chip).
  bool begin(const uint8_t addrs[6], uint16_t shuntCalValue, uint8_t vbusLeg);

  // Returns true on success / false on I2C failure (NACK, bus timeout,
  // short read, etc). On false, the output parameter is left unchanged.
  bool readBusVoltage(uint8_t i2cAddr, float &busVoltage);

  bool readCurrent(uint8_t i2cAddr, float &current);

private:
  bool writeShuntCal(uint8_t i2cAddr, uint16_t calValue);
  bool writeAdcConfig(uint8_t i2cAddr, uint16_t configValue);
};

extern Ina ina;
