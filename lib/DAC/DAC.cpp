#include "DAC.h"

Dac80508 dac;

static const SPISettings kDacSpiSettings(1000000, MSBFIRST, SPI_MODE1);

void Dac80508::begin(SPIClass& spi, int sckPin, int mosiPin, int csPin) {
  _spi = &spi;
  _csPin = csPin;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin(sckPin, -1 /* MISO unused */, mosiPin, csPin);
  delay(1); //power up delay

  writeChannel(DAC_REG_GAIN, 0x01FF); // REFDIV-EN=1, BUFF0..7-GAIN=1 -> 2.5 V full scale
}

bool Dac80508::writeChannel(uint8_t channelReg, uint16_t code) {
  if (!_spi) {
    Serial.println("Dac80508: writeChannel() called before begin() -- ignoring");
    return false;
  }
  _spi->beginTransaction(kDacSpiSettings);
  digitalWrite(_csPin, LOW);
  _spi->transfer(channelReg);        
  _spi->transfer((code >> 8) & 0xFF);
  _spi->transfer(code & 0xFF);       
  digitalWrite(_csPin, HIGH);        
  _spi->endTransaction();
  return true;
}

bool Dac80508::writeAllSetpoints(const uint16_t codes[DAC_NUM_SETPOINTS]) {
  bool allOk = true;
  for (uint8_t i = 0; i < DAC_NUM_SETPOINTS; i++) {
    if (!writeChannel(DAC_REG_SETPOINT_BASE + i, codes[i])) {
      allOk = false;
    }
  }
  return allOk;
}

bool Dac80508::writeSetpoint(uint8_t legIndex, uint16_t code) {
  if (legIndex >= DAC_NUM_SETPOINTS) {
    Serial.printf("Dac80508: writeSetpoint() legIndex %u out of range\n", legIndex);
    return false;
  }
  return writeChannel(DAC_REG_SETPOINT_BASE + legIndex, code);
}

bool Dac80508::writeOC(uint16_t code) {
  return writeChannel(DAC_REG_PROG_OC, code);
}

bool Dac80508::writeOV(uint16_t code) {
  return writeChannel(DAC_REG_PROG_OV, code);
}
