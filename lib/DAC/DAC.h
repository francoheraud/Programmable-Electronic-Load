#pragma once
#include <Arduino.h>
#include <SPI.h>

#define DAC_REG_SETPOINT_BASE  0x08 // SetpointN = base + (N-1)
#define DAC_REG_PROG_OC        0x0E // OUT6 -> /16 divider -> Prog_OC
#define DAC_REG_PROG_OV        0x0F // OUT7 -> Prog_OV (direct)
#define DAC_REG_CONFIG         0x03
#define DAC_REG_GAIN           0x04
#define DAC_NUM_SETPOINTS      6

class Dac80508 {
public:
  void begin(SPIClass& spi, int sckPin, int mosiPin, int csPin);
  bool writeAllSetpoints(const uint16_t codes[DAC_NUM_SETPOINTS]);
  bool writeOC(uint16_t code);
  bool writeOV(uint16_t code);

private:
  bool writeSetpoint(uint8_t legIndex, uint16_t code);
  bool writeChannel(uint8_t channelReg, uint16_t code);
  SPIClass* _spi = nullptr;
  int _csPin = -1;
};

extern Dac80508 dac;
