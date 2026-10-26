#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SystemState.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

struct LogRecord {
  uint32_t timestampMs;
  LoadState state;
  float voltage, current, power;
  float legCurrent[6];
  float tempInt[6];
  float tempExt[4];
};

// Single producer (Arduino loop), single SD-owner worker. No SD calls from
// startTest/append/endTest; queue sends never wait. begin mounts only at startup.
class SdLog {
public:
  bool begin(SPIClass& spi, int sckPin, int misoPin, int mosiPin, int csPin);
  bool startTest();
  bool append(const LogRecord& rec);
  void endTest(); // Requests close after queued samples drain; never waits.
  bool ready() const { return _ready.load(); }
  bool active() const { return _producerActive.load(); }
  bool busy() const { return active() || _pending.load() != 0 || _fileOpen.load(); }
  uint32_t dropped() const { return _dropped.load(); }
  uint32_t errors() const { return _errors.load(); }
private:
  struct Sample { uint32_t run, sequence, dropped; LogRecord record; };
  static void taskEntry(void* self);
  void worker();
  bool openRun(uint32_t run);
  bool writeSample(const Sample& sample);
  void closeFile();
  void fail();
  QueueHandle_t _queue = nullptr;
  TaskHandle_t _task = nullptr;
  File _file; // Worker-owned after begin returns.
  uint16_t _nextIndex = 1;
  uint32_t _run = 0, _sequence = 0; // Producer-owned.
  uint32_t _fileRun = 0, _lastFlushMs = 0; // Worker-owned.
  std::atomic<bool> _ready{false}, _producerActive{false}, _fileOpen{false};
  std::atomic<uint32_t> _endedThrough{0}, _pending{0}, _dropped{0}, _errors{0};
};
extern SdLog sdLog;
