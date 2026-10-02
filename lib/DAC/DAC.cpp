#include "DAC.h"

Dac80508 dac;

// DAC80508 uses 24-bit frames: 8 command/address bits + 16 data bits.
// SPI_MODE1 and MSB-first per the datasheet -- confirm against your exact
// part's timing diagram if the outputs misbehave.
static const SPISettings kDacSpiSettings(1000000, MSBFIRST, SPI_MODE1);

void Dac80508::begin(SPIClass& spi, int sckPin, int mosiPin, int csPin) {
  _spi = &spi;
  _csPin = csPin;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);              // idle high; CS is active-low
  _spi->begin(sckPin, -1 /* MISO unused */, mosiPin, csPin);
  delay(1); // power up delay

  // GAIN register: REFDIV-EN=0 (no divide) + all 8 channels BUFF-GAIN=1 (x2)
  // -> DIV=1, GAIN=2 -> output range 0 to 2*VREF = 0 to 5.0V (internal 2.5V ref)
  // Confirming this with Tane! TODO: update
  // This is the recommended config; 0x0000 (DIV=1, GAIN=1) is "Not recommended" per Table 1.
  writeChannel(DAC_REG_GAIN, 0x00FF);
}

bool Dac80508::writeChannel(uint8_t channelReg, uint16_t code) {
  if (!_spi) {
    Serial.println("Dac80508: writeChannel() called before begin() -- ignoring");
    return false;
  }
  _spi->beginTransaction(kDacSpiSettings);
  digitalWrite(_csPin, LOW);
  _spi->transfer(channelReg);              // 8 command/address bits
  _spi->transfer((code >> 8) & 0xFF);      // data high byte
  _spi->transfer(code & 0xFF);             // data low byte
  digitalWrite(_csPin, HIGH);              // rising edge latches the frame
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
  if (legIndex >= DAC_NUM_SETPOINTS) {         // guard against a bad index
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