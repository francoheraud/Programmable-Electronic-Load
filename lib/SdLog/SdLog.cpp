#include "SdLog.h"
#include <stdio.h>

void diagnoseSdRaw(SPIClass &spi, int csPin, uint32_t frequency);

SdLog sdLog;
static const uint32_t SD_SPI_HZ = 1000000; // Reduced clock for PCB/card communication diagnosis.
static const unsigned QUEUE_DEPTH = 64; // ~32 seconds at 2 Hz, fixed RAM usage.
static const uint32_t FLUSH_PERIOD_MS = 2000;

// Startup-only storage demonstration; no load commands or sensor data are generated.
// Runs before the asynchronous writer starts, so SD access cannot race it.
static bool demonstrateSdWrite() {
  char path[32];
  bool found = false;
  for (unsigned i = 1; i <= 1000; ++i) {
    snprintf(path, sizeof(path), "/sdcheck_%04u.csv", i);
    if (!SD.exists(path)) { found = true; break; }
  }
  if (!found) {
    Serial.println("SDCHECK: FAIL: all demo filenames occupied; no files overwritten");
    return false;
  }
  static const char contents[] =
    "record_type,sample,message\n"
    "storage_test,1,synthetic test row - not a load measurement\n"
    "storage_test,2,synthetic test row - not a load measurement\n"
    "storage_test,3,synthetic test row - not a load measurement\n";
  const size_t expected = sizeof(contents) - 1;
  File out = SD.open(path, FILE_WRITE);
  if (!out) { Serial.println("SDCHECK: FAIL: cannot create CSV"); return false; }
  const size_t written = out.write(reinterpret_cast<const uint8_t *>(contents), expected);
  out.flush();
  out.close();
  if (written != expected) {
    Serial.printf("SDCHECK: FAIL: short write %u/%u bytes to %s\n", unsigned(written), unsigned(expected), path);
    return false;
  }
  File in = SD.open(path, FILE_READ);
  if (!in) { Serial.println("SDCHECK: FAIL: cannot reopen CSV"); return false; }
  bool matches = in.size() == expected;
  for (size_t i = 0; i < expected && matches; ++i) {
    matches = in.read() == static_cast<uint8_t>(contents[i]);
  }
  in.close();
  Serial.printf("SDCHECK: %s: %s (%u bytes, 3 synthetic rows)\n",
                matches ? "PASS write/close/reopen/readback" : "FAIL readback mismatch", path, unsigned(expected));
  return matches;
}

bool SdLog::begin(SPIClass& spi, int sckPin, int misoPin, int mosiPin, int csPin) {
  if (_task) return ready();
  spi.begin(sckPin, misoPin, mosiPin, csPin);
  if (!SD.begin(csPin, spi, SD_SPI_HZ)) {
    Serial.println("SdLog: card mount failed; operation continues without logging");
    diagnoseSdRaw(spi, csPin, SD_SPI_HZ);
    return false;
  }
  if (!demonstrateSdWrite()) { SD.end(); return false; }
  _queue = xQueueCreate(QUEUE_DEPTH, sizeof(Sample));
  if (!_queue) { SD.end(); return false; }
  _ready.store(true);
#if CONFIG_FREERTOS_UNICORE
  const BaseType_t core = 0;
#else
  const BaseType_t core = 1 - xPortGetCoreID();
#endif
  if (xTaskCreatePinnedToCore(taskEntry, "sd-writer", 6144, this, 1, &_task, core) != pdPASS) {
    _ready.store(false);
    vQueueDelete(_queue);
    _queue = nullptr;
    SD.end();
    return false;
  }
  Serial.println("SdLog: mounted; asynchronous writer ready (64 samples, 2 s flush)");
  return true;
}

bool SdLog::startTest() {
  if (!ready() || active()) return false;
  ++_run;
  _sequence = 0;
  _producerActive.store(true);
  return true;
}

bool SdLog::append(const LogRecord& record) {
  if (!active()) return false;
  Sample sample{_run, _sequence++, _dropped.load(), record};
  if (!ready()) { _dropped.fetch_add(1); return false; }
  // Increment before publishing so the worker cannot decrement from zero.
  _pending.fetch_add(1);
  if (xQueueSend(_queue, &sample, 0) != pdTRUE) {
    _pending.fetch_sub(1);
    _dropped.fetch_add(1); // Drop newest; preserve queued ordering/run boundaries.
    return false;
  }
  return true;
}

void SdLog::endTest() {
  if (!_producerActive.exchange(false)) return;
  // Out-of-band close watermark cannot be lost when the sample queue is full.
  _endedThrough.store(_run);
}

void SdLog::taskEntry(void* self) { static_cast<SdLog*>(self)->worker(); }

void SdLog::closeFile() {
  if (_file) { _file.flush(); _file.close(); }
  _fileOpen.store(false);
}

void SdLog::fail() {
  _errors.fetch_add(1);
  _ready.store(false); // No repeated retry loop on a removed/full/failing card.
  closeFile();
  Serial.println("SdLog: write/open failure; logging disabled until reboot");
}

bool SdLog::openRun(uint32_t run) {
  closeFile();
  char path[24];
  while (_nextIndex < 10000) {
    snprintf(path, sizeof(path), "/test_%04u.csv", _nextIndex++);
    if (!SD.exists(path)) {
      _file = SD.open(path, FILE_WRITE);
      if (!_file) return false;
      _fileOpen.store(true);
      _fileRun = run;
      const char header[] = "time_ms,state,V,I_total,P,I1,I2,I3,I4,I5,I6,Th1,Th2,Th3,Th4,Th5,Th6,Te1,Te2,Te3,Te4,sample_no,dropped_total\n";
      if (_file.write(reinterpret_cast<const uint8_t*>(header), sizeof(header)-1) != sizeof(header)-1) return false;
      _lastFlushMs = millis();
      Serial.printf("SdLog: logging to %s\n", path);
      return true;
    }
    vTaskDelay(1); // Yield even when scanning a card with many old test files.
  }
  return false;
}

bool SdLog::writeSample(const Sample& sample) {
  const auto& r = sample.record;
  char row[768];
  int n = snprintf(row, sizeof(row), "%lu,%u,%.3f,%.3f,%.2f",
      (unsigned long)r.timestampMs, (unsigned)r.state, r.voltage, r.current, r.power);
  if (n < 0 || n >= (int)sizeof(row)) return false;
  size_t used = n;
  auto addFloat = [&](float value, unsigned precision) {
    int count = snprintf(row + used, sizeof(row) - used, ",%.*f", precision, value);
    if (count < 0 || count >= (int)(sizeof(row) - used)) return false;
    used += count;
    return true;
  };
  for (float v : r.legCurrent) if (!addFloat(v, 3)) return false;
  for (float v : r.tempInt) if (!addFloat(v, 1)) return false;
  for (float v : r.tempExt) if (!addFloat(v, 1)) return false;
  n = snprintf(row + used, sizeof(row) - used, ",%lu,%lu\n",
      (unsigned long)sample.sequence, (unsigned long)sample.dropped);
  if (n < 0 || n >= (int)(sizeof(row) - used)) return false;
  used += n;
  return _file.write(reinterpret_cast<const uint8_t*>(row), used) == used && !_file.getWriteError();
}

void SdLog::worker() {
  for (;;) {
    Sample sample;
    if (xQueueReceive(_queue, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!ready()) {
        _dropped.fetch_add(1);
      } else if ((!_file || _fileRun != sample.run) && !openRun(sample.run)) {
        _dropped.fetch_add(1);
        fail();
      } else if (!writeSample(sample)) {
        _dropped.fetch_add(1);
        fail();
      }
      _pending.fetch_sub(1);
    }
    if (_file && _pending.load() == 0 && _fileRun <= _endedThrough.load()) {
      closeFile();
      Serial.printf("SdLog: run closed; dropped=%lu errors=%lu\n",
          (unsigned long)dropped(), (unsigned long)errors());
    } else if (_file && millis() - _lastFlushMs >= FLUSH_PERIOD_MS) {
      _file.flush();
      _lastFlushMs = millis();
      if (_file.getWriteError()) fail();
    }
    vTaskDelay(1); // Yield while draining backlog, allowing the idle task to run.
  }
}
