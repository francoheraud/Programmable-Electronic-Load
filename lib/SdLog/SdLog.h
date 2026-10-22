#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SystemState.h>

struct LogRecord {
  uint32_t timestampMs;     // ms since the test started (testTimeElapsed)
  LoadState state;
  float voltage;
  float current;            // total current
  float power;
  float legCurrent[6];
  float tempInt[6];         // heatsink NTCs
  float tempExt[4];         // external probes
};

class SdLog {
public:
  // Mounts the card on its own SPI bus. Returns false if no card / mount failed.
  bool begin(SPIClass& spi, int sckPin, int misoPin, int mosiPin, int csPin);

  // Opens the next numbered CSV (test_NNNN.csv) and writes the header.
  bool startTest();

  // Appends one row and flushes so a power loss costs at most one row.
  bool append(const LogRecord& rec);

  void endTest();

  bool ready() const { return _ready; }
  bool active() const { return _file; }

private:
  bool _ready = false;
  uint16_t _nextIndex = 1;
  File _file;
};

extern SdLog sdLog;
