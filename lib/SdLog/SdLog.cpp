#include "SdLog.h"

SdLog sdLog;

static const uint32_t SD_SPI_HZ = 4000000;

bool SdLog::begin(SPIClass& spi, int sckPin, int misoPin, int mosiPin, int csPin) {
  spi.begin(sckPin, misoPin, mosiPin, csPin);
  _ready = SD.begin(csPin, spi, SD_SPI_HZ);
  if (!_ready) {
    Serial.println("SdLog: microSD mount failed -- check card and SPI_1 wiring");
    return false;
  }
  Serial.printf("SdLog: card mounted (%llu MB)\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

bool SdLog::startTest() {
  if (!_ready) { return false; }
  endTest();

  char path[24];
  while (_nextIndex < 10000) {
    snprintf(path, sizeof(path), "/test_%04u.csv", _nextIndex);
    if (!SD.exists(path)) { break; }
    _nextIndex++;
  }
  if (_nextIndex >= 10000) {
    Serial.println("SdLog: no free test file index");
    return false;
  }

  _file = SD.open(path, FILE_WRITE);
  if (!_file) {
    Serial.printf("SdLog: could not open %s\n", path);
    return false;
  }
  _nextIndex++;

  _file.println("time_ms,state,V,I_total,P,"
                "I1,I2,I3,I4,I5,I6,"
                "Th1,Th2,Th3,Th4,Th5,Th6,"
                "Te1,Te2,Te3,Te4");
  _file.flush();
  Serial.printf("SdLog: logging to %s\n", path);
  return true;
}

bool SdLog::append(const LogRecord& r) {
  if (!_file) { return false; }

  _file.printf("%lu,%u,%.3f,%.3f,%.2f", (unsigned long)r.timestampMs,
               (unsigned)r.state, r.voltage, r.current, r.power);
  for (uint8_t i = 0; i < 6; i++) { _file.printf(",%.3f", r.legCurrent[i]); }
  for (uint8_t i = 0; i < 6; i++) { _file.printf(",%.1f", r.tempInt[i]); }
  for (uint8_t i = 0; i < 4; i++) { _file.printf(",%.1f", r.tempExt[i]); }
  _file.println();
  _file.flush();
  return true;
}

void SdLog::endTest() {
  if (_file) { _file.close(); }
}
