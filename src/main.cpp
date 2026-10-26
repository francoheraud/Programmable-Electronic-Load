#include <Arduino.h>
#include <Wire.h>
#include <SystemState.h>
#include <Display.h>
#include <INA.h>
#include <ADS1015.h>
#include <ADS7128.h>
#include <DAC.h>
#include <UI.h>
#include <SdLog.h>
#include <Pins.h>
#include <DeviceConfig.h>
#include <ProtectionDac.h>
#include <CvPid.h>
#include <LoadSetpoints.h>
#include <string.h>
#include <stdarg.h>

static const float heatsinkDifferentialAlert = 5.0f;
static const float heatsinkTemperatureAlert = 1000.0f;

static const float MAX_TOTAL_CURRENT_A = MAX_CURRENT_PER_LEG_A * DAC_NUM_SETPOINTS;
static CvPid cvPid; // Gains, filtering and slew limits are in include/CvPid.h.

#define FAN_PWM_FREQ_HZ   25000
#define FAN_PWM_RES_BITS  8
static const float   FAN_BAND_UPPER_C[4] = {25.0f, 30.0f, 45.0f, 70.0f};
static const uint8_t FAN_BAND_DUTY[5]    = {26, 64, 128, 191, 255}; // -> 10/25/50/75/100% of 8 bit
static const float   FAN_HYSTERESIS_C    = 2.0f;

// Self-test limits
static const float SELF_TEST_MAX_IDLE_CURRENT_A = 0.05f;
static const float SELF_TEST_MAX_BUS_V          = 5.0f;
static const float SELF_TEST_MIN_TEMP_C         = -10.0f;
static const float SELF_TEST_MAX_TEMP_C         = 60.0f;

// setpoints
float overVoltage = 30.0f;
float overCurrent = MAX_TOTAL_CURRENT_A;
float overPower = 500.0f;
float overTemperature = SELF_TEST_MAX_TEMP_C;
float setCurrent = 0.0f;
float setVoltage = 0.0f;
float setTime = 0.0f;
uint32_t testTimeElapsed = 0;
uint32_t testStartMs = 0;

// Live measurements
float measV_Bus = 0.0f;
float measCurrent[6] = {0, 0, 0, 0, 0, 0};
float totalCurrent = 0.0f;
float measTemp_Int[6] = {0, 0, 0, 0, 0, 0};
float measTemp_Ext[4] = {NAN, NAN, NAN, NAN};
float measPower = 0.0f;

static SPIClass sdSpi(HSPI);
static bool inaInitOk = false;
static bool adcInitOk = false;
static bool oledInitOk = false;
static bool selfTestFailed = false;
static bool protectionInitOk = false;

LoadState loadState = LoadState::INIT;
FaultReason faultReason = FaultReason::NONE;
bool faultActive = false;
bool sensorReadFailed = false;
static bool inaReadFailed = false;
static bool inaSampleValid = false;
static uint32_t inaSampleMs = 0, inaSampleSequence = 0;
static bool tempReadFailed = false;
static uint32_t inaFailSinceMs = 0;
static uint32_t tempFailSinceMs = 0;

// Loop control frequencies
static uint32_t lastControlMs = 0;
static uint32_t lastProtectionMs = 0;
static uint32_t lastUiMs = 0;
static uint32_t lastLoggingMs = 0;
static uint32_t lastTemperatureMs = 0;
static uint32_t lastSystemMs = 0;
static uint32_t lastResolveMs = 0;
static const uint32_t CONTROL_PERIOD_MS = 20;
static const uint32_t PROTECTION_PERIOD_MS = 20;
static const uint32_t UI_PERIOD_MS = 50;
static const uint32_t LOGGING_PERIOD_MS = 500;
static const uint32_t TEMPERATURE_PERIOD_MS = 500;
static const uint32_t SYSTEM_PERIOD_MS = 200;
static const uint32_t CV_SYSTEM_PERIOD_MS = 50; // Faster feedback only during CV; keep fault timeout unchanged.
static const uint32_t RESOLVE_PERIOD_MS = 50;
static const uint32_t MAX_STEP_DURATION_MS = 20; //timeout delay

// Timing remains measured, but reports are on demand instead of flooding UART.
static uint32_t slowStepCount = 0, worstStepMs = 0;
static const char *worstStepName = "none";
static inline void checkStepDuration(const char* stepName, uint32_t stepStartMs) {
  const uint32_t elapsed = millis() - stepStartMs;
  if (elapsed > MAX_STEP_DURATION_MS) ++slowStepCount;
  if (elapsed > worstStepMs) { worstStepMs = elapsed; worstStepName = stepName; }
}
static bool intTempValid = false, extTempValid = false;
static bool intTempEverValid = false, extTempEverValid = false;
static bool ntcMatch = false;
static uint32_t intTempLastGood = 0, extTempLastGood = 0;

static bool programProtectionThresholds() {
  static float programmedOC = -1.0f, programmedOV = -1.0f;
  if (overCurrent == programmedOC && overVoltage == programmedOV) return true;
  uint16_t oc, ov;
  if (!protectionDacCodes(overCurrent, overVoltage, oc, ov)) {
    Serial.println("PROTECTION: invalid OC/OV limits; load inhibited");
    return false;
  }
  if (!dac.writeOC(oc) || !dac.writeOV(ov)) return false;
  programmedOC = overCurrent; programmedOV = overVoltage;
  Serial.printf("PROTECTION: OC %.3f A code=%u Prog_OC=%.4f V; OV %.3f V code=%u Prog_OV=%.4f V (commanded, not read back)\n",
    overCurrent, oc, oc * 2.5f / 65536.0f / 16.0f,
    overVoltage, ov, ov * 2.5f / 65536.0f);
  return true;
}

void initialiseSystem() {
  digitalWrite(PIN_FAULT_SET, HIGH); // Hold disabled throughout initialization.
  pinMode(PIN_FAULT_SET, OUTPUT);
  pinMode(PIN_FAULT_RST, OUTPUT);
  digitalWrite(PIN_FAULT_RST, LOW);

  ledcSetup(0, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
  ledcAttachPin(PIN_PWM, 0);
  ledcWrite(0, FAN_BAND_DUTY[0]);

  pinMode(PIN_HARD_DISABLE, INPUT);
  pinMode(PIN_ENC_A,    INPUT_PULLUP);
  pinMode(PIN_ENC_B,    INPUT_PULLUP);
  pinMode(PIN_BACK_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ENC_BUTTON, INPUT_PULLDOWN);

  Wire1.begin(PIN_SDA_1, PIN_SCL_1);
  Wire1.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  Wire2.begin(PIN_SDA_2, PIN_SCL_2);
  Wire2.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);

  inaInitOk = ina.begin(inaAddrs, shuntCal, VBUS_LEG);
  if (!inaInitOk) {
    Serial.println("initialiseSystem: one or more INAs failed to configure");
  }

  adcInitOk = ads7128.begin();
  if (!adcInitOk) {
    Serial.println("initialiseSystem: ADS7128 failed to configure");
  }

  oledInitOk = display.begin();
  if (!oledInitOk) {
    Serial.println("initialiseSystem: OLED did not ACK on I2C_2");
  }

  if (!sdLog.begin(sdSpi, PIN_SCK_1, PIN_MISO_1, PIN_MOSI_1, PIN_CS_1)) {
    Serial.println("initialiseSystem: no microSD, test data will not be logged");
  }

  ui.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BUTTON, PIN_BACK_BUTTON);

  dac.begin(SPI, PIN_SCK_2, PIN_MOSI_2, PIN_CS_2);
  const uint16_t zeroCodes[DAC_NUM_SETPOINTS] = {};
  const bool zeroOk = dac.writeAllSetpoints(zeroCodes);
  protectionInitOk = zeroOk && programProtectionThresholds();
  delay(2); // Allow DAC outputs to settle before latch reset.


  Serial.println("initialised");
}

static void writeCurrentSetpoint(float totalCurrentA);
void readSystemData();

void runSelfTest() {
  bool ok = true;
  auto fail = [&](const char* msg) {
    Serial.printf("SELF-TEST FAIL: %s\n", msg);
    ok = false;
  };

  if (!protectionInitOk) fail("protection DAC initialization");
  if (protectionInitOk) {
    digitalWrite(PIN_FAULT_SET, LOW); // Setpoints zero; thresholds now programmed.
    digitalWrite(PIN_FAULT_RST, HIGH);
    delay(10);
    digitalWrite(PIN_FAULT_RST, LOW);
    delay(10);
  }
  if (digitalRead(PIN_HARD_DISABLE) == HIGH) { fail("hardware disable asserted"); }

  if (!inaInitOk)  { fail("INA configuration"); }
  if (!adcInitOk)  { fail("ADS7128 configuration"); }
  if (!oledInitOk) { fail("OLED not responding"); }

  writeCurrentSetpoint(0.0f);
  delay(CONTROL_PERIOD_MS);
  readSystemData();

  if (inaReadFailed)  { fail("INA read"); }
  if (!intTempValid) { fail("heatsink ADC read failed"); }
  else if (!ntcMatch) { fail("heatsink NTC mismatch"); }
  if (!extTempValid) { fail("external ADC communication failed"); }

  for (uint8_t i = 0; i < 6; i++) {
    if (fabsf(measCurrent[i]) > SELF_TEST_MAX_IDLE_CURRENT_A) {
      Serial.printf("  leg %u idle current %.3f A\n", i + 1, measCurrent[i]);
      fail("idle leg current too high");
    }
  }
  if (measV_Bus > SELF_TEST_MAX_BUS_V) {
    Serial.printf("  bus voltage %.2f V\n", measV_Bus);
    fail("bus voltage above low-input limit");
  }
  for (uint8_t i = 0; i < 6; i++) {
    if (measTemp_Int[i] < SELF_TEST_MIN_TEMP_C || measTemp_Int[i] > SELF_TEST_MAX_TEMP_C) {
      Serial.printf("  heatsink NTC%u %.1f C\n", i + 1, measTemp_Int[i]);
      fail("heatsink temperature out of range");
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (isfinite(measTemp_Ext[i]) && measTemp_Ext[i] > overTemperature) {
      Serial.printf("  external sensor %u %.1f C\n", i + 1, measTemp_Ext[i]);
      fail("external temperature out of range");
    }
  }

  if (ok) {
    Serial.println("startup test passed");
    loadState = LoadState::IDLE;
  } else {
    selfTestFailed = true;
    faultActive = true;
    faultReason = FaultReason::SELF_TEST;
    loadState = LoadState::FAULT;
    digitalWrite(PIN_FAULT_SET, HIGH);  // MCU_DISABLE
    display.showMessage("SELF-TEST", "FAILED - see serial");
    Serial.println("startup test FAILED -- load disabled until reset");
  }
}

void readSystemData() {
  totalCurrent = 0.0f;
  bool allOk = true;

  // only reads one INA for bus voltage
  if (!ina.readBusVoltage(inaAddrs[0], measV_Bus)) {
    allOk = false;
  }
  for (uint8_t i = 0; i < 6; i++)
  {
    if (!ina.readCurrent(inaAddrs[i], measCurrent[i])) {
      allOk = false;
    }
  }
  for (int i = 0; i < 6; i++) {
    totalCurrent += measCurrent[i];
  }
  measPower = measV_Bus * totalCurrent;

  if (!allOk) {
    if (!inaReadFailed) { inaFailSinceMs = millis(); }
  } else {
    inaFailSinceMs = 0;
  }
  if (allOk) {
    inaSampleMs = millis();
    ++inaSampleSequence;
    inaSampleValid = true;
  }
  inaReadFailed = !allOk;
  sensorReadFailed = inaReadFailed || tempReadFailed;
}

static bool heatsinkTempsConsistent(const float tempsC[6]) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 6; i++) {
    sum += tempsC[i];
  }
  float meanTemp = sum / 6.0f;

  bool consistent = true;
  for (uint8_t i = 0; i < 6; i++) {
    if (fabsf(tempsC[i] - meanTemp) > heatsinkDifferentialAlert * fabsf(meanTemp)) {
      Serial.printf("readTemperatureData: heatsink NTC%u (%.1fC) deviates >%.0f%% from mean (%.1fC)\n",
                    i + 1, tempsC[i], heatsinkDifferentialAlert * 100.0f, meanTemp);
      consistent = false;
    }
  }
  return consistent;
}

void readTemperatureData() {
  bool allOk = true;

  intTempValid = ads7128.readTemperatures(measTemp_Int);
  if (intTempValid) { intTempEverValid = true; intTempLastGood = millis(); }
  ntcMatch = false;
  if (!intTempValid) {
    allOk = false;
  } else if (!(ntcMatch = heatsinkTempsConsistent(measTemp_Int))) {
    allOk = false;
  }

  extTempValid = ads1015.readTemperatures(measTemp_Ext);
  if (extTempValid) { extTempEverValid = true; extTempLastGood = millis(); }
  if (!extTempValid) {
    allOk = false;
  }

  if (!allOk) {
    if (!tempReadFailed) { tempFailSinceMs = millis(); }
  } else {
    tempFailSinceMs = 0;
  }
  tempReadFailed = !allOk;
  sensorReadFailed = inaReadFailed || tempReadFailed;
}

void userInputs() {
  ui.tick();
  // Apply edited limits before the next control iteration. Invalid settings
  // latch inhibition until reset rather than allowing stale hardware thresholds.
  if (!selfTestFailed && !programProtectionThresholds()) {
    digitalWrite(PIN_FAULT_SET, HIGH);
    selfTestFailed = true;
    faultActive = true;
    faultReason = FaultReason::SELF_TEST;
    loadState = LoadState::FAULT;
  }
}

static uint16_t currentToDacCode(float currentPerLegA) {
  // DAC80508: 2.5 V full scale (GAIN=0x01FF), 65536 code steps.
  // Each output is divided by 16 (15k/1k) and drives a 25 milliohm shunt.
  // Vdac = Ileg * 0.025 * 16. 5.05 A uses 80.8% of the DAC range;
  // the current safety limit is NOT the current at full DAC scale (6.25 A).
  // Keep zero at code zero; idle current is not a calibrated DAC offset.
  if (!isfinite(currentPerLegA) || currentPerLegA <= 0.0f) { return 0; }
  if (currentPerLegA > MAX_CURRENT_PER_LEG_A) { currentPerLegA = MAX_CURRENT_PER_LEG_A; }
  const float dacVoltage = currentPerLegA * 0.025f * 16.0f;
  return (uint16_t)(dacVoltage * (65536.0f / 2.5f));
}

static void writeCurrentSetpoint(float totalCurrentA) {
  const bool fresh = inaSampleValid && !inaReadFailed && millis() - inaSampleMs <= 2 * SYSTEM_PERIOD_MS;
  const float ceiling = LoadSetpoints::currentCeiling(measV_Bus, overPower, fresh);
  if (!isfinite(totalCurrentA) || faultActive || selfTestFailed) totalCurrentA = 0;
  totalCurrentA = fminf(totalCurrentA, ceiling);
  if (totalCurrentA < 0.0f) { totalCurrentA = 0.0f; }
  if (totalCurrentA > LoadSetpoints::MAX_CURRENT_A) { totalCurrentA = LoadSetpoints::MAX_CURRENT_A; }

  uint16_t code = currentToDacCode(totalCurrentA / DAC_NUM_SETPOINTS);
  uint16_t codes[DAC_NUM_SETPOINTS];
  for (uint8_t i = 0; i < DAC_NUM_SETPOINTS; i++) {
    codes[i] = code;
  }
  dac.writeAllSetpoints(codes);
}

void loadControl() {

  float commandCurrent = 0.0f;

  if (loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV) {
    testTimeElapsed = millis() - testStartMs;
  }

  const bool cvEnabled = loadState == LoadState::RUNNING_CV && !faultActive && !selfTestFailed;
  const bool freshPowerSample = inaSampleValid && !inaReadFailed && millis() - inaSampleMs <= 2 * SYSTEM_PERIOD_MS;
  const float cvPowerLimit = LoadSetpoints::currentCeiling(fmaxf(measV_Bus, setVoltage), overPower, freshPowerSample);
  const float cvLimit = fminf(setCurrent, cvPowerLimit);
  commandCurrent = cvPid.update(cvEnabled, inaSampleValid && !inaReadFailed,
                               millis(), inaSampleMs, inaSampleSequence,
                               measV_Bus, setVoltage, cvLimit);

  if (loadState == LoadState::RUNNING_CC) {
    commandCurrent = setCurrent;
  }

  if (loadState == LoadState::IDLE || loadState == LoadState::FAULT) {
    commandCurrent = 0.0f;
  }

  writeCurrentSetpoint(commandCurrent);
}

void fanControl() {
  static uint8_t band = 4;
  if (tempReadFailed || loadState == LoadState::FAULT) {
    band = 4;
    ledcWrite(0, FAN_BAND_DUTY[band]);
    return;
  }
  float sum = 0.0f;
  for (uint8_t i = 0; i < 6; i++) {
    sum += measTemp_Int[i];
  }
  float avgTemp = sum / 6.0f;

  while (band < 4 && avgTemp >= FAN_BAND_UPPER_C[band]) { band++; }
  while (band > 0 && avgTemp < FAN_BAND_UPPER_C[band - 1] - FAN_HYSTERESIS_C) { band--; }

  ledcWrite(0, FAN_BAND_DUTY[band]);
}

bool confirmFault() {
  uint32_t now = millis();
  if (inaReadFailed && (now - inaFailSinceMs) > 2 * SYSTEM_PERIOD_MS) {
    faultReason = FaultReason::SENSOR_UNRESPONSIVE;
    Serial.println("MCU Fault: sensor unresponsive (INA read failed for >2 periods)");
    return true;
  }
  if (tempReadFailed && (now - tempFailSinceMs) > 2 * TEMPERATURE_PERIOD_MS) {
    faultReason = FaultReason::SENSOR_UNRESPONSIVE;
    Serial.println("MCU Fault: sensor unresponsive (temperature read failed for >2 periods)");
    return true;
  }
  if (measV_Bus > overVoltage)  {
    faultReason = FaultReason::VOLTAGE;
    Serial.println("MCU Over Voltage Tripped");
    return true;
  }
  if (!isfinite(measPower) || measPower >= LoadSetpoints::powerLimit(overPower)) {
    faultReason = FaultReason::POWER;
    Serial.println("MCU Over Power Tripped");
    return true;
  }
  if (totalCurrent > overCurrent) {
    faultReason = FaultReason::CURRENT;
    Serial.println("MCU Over Current Tripped");
    return true;
  }
  for (uint8_t i = 0; i < 6; i++) {
    if (measTemp_Int[i] > heatsinkTemperatureAlert) {
      faultReason = FaultReason::INT_TEMP;
      Serial.printf("MCU Over Temperature Tripped: heatsink NTC%u (%.1fC)\n", i + 1, measTemp_Int[i]);
      return true;
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    if (isfinite(measTemp_Ext[i]) && measTemp_Ext[i] > overTemperature) {
      faultReason = FaultReason::EXT_TEMP;
      Serial.printf("MCU Over Temperature Tripped: external sensor %u (%.1fC)\n", i + 1, measTemp_Ext[i]);
      return true;
    }
  }
  return false;
}

void checkFault() {
  if (digitalRead(PIN_HARD_DISABLE) == HIGH) {
    loadState = LoadState::FAULT;
    faultActive = true;
    Serial.println("hardware-trip fault");
    if (!confirmFault())    {
      Serial.println("FLAG: No fault cause found");
    }
  } else if (confirmFault())  {
    Serial.println("FLAG: Software only trip");
    loadState = LoadState::FAULT;
    faultActive = true;
  }
}

void resolveFault() {
  if (selfTestFailed) { return; }  // self-test faults only clear on reset
  if (!confirmFault()) {
    digitalWrite(PIN_FAULT_SET, LOW);   // release MCU_DISABLE
    digitalWrite(PIN_FAULT_RST, HIGH);  // pulse MCU_RESET
    delay(10);
    digitalWrite(PIN_FAULT_RST, LOW);

    if (digitalRead(PIN_HARD_DISABLE) == HIGH) {
      Serial.println("resolveFault: hardware fault still latched, reset aborted");
      return;
    }

    faultActive = false;
    faultReason = FaultReason::NONE;
    loadState = LoadState::IDLE;
    Serial.println("Fault cleared, returning to IDLE");
  }
}

static bool isRunningState(LoadState s) {
  return s == LoadState::RUNNING_CC || s == LoadState::RUNNING_CV;
}

void logData() {
  static bool wasRunning = false;
  const bool running = isRunningState(loadState);
  const uint32_t now = millis();
  const bool started = running && !wasRunning;
  if (started) sdLog.startTest();
  if (!running && wasRunning) sdLog.endTest();
  wasRunning = running;
  if (!running || !sdLog.active()) return;
  if (!started && now - lastLoggingMs < LOGGING_PERIOD_MS) return;
  lastLoggingMs = now;

  // Copy a snapshot into the queue without waiting. Only the SD task touches files.
  LogRecord rec = {};
  rec.timestampMs = now - testStartMs;
  rec.state = loadState;
  rec.voltage = measV_Bus;
  rec.current = totalCurrent;
  rec.power = measPower;
  memcpy(rec.legCurrent, measCurrent, sizeof(rec.legCurrent));
  memcpy(rec.tempInt, measTemp_Int, sizeof(rec.tempInt));
  memcpy(rec.tempExt, measTemp_Ext, sizeof(rec.tempExt));
  sdLog.append(rec); // Overflow is counted, never allowed to delay control.
}

// Read-only serial commands. Cached readings avoid extra ADC conversions;
// bounded receive work and availableForWrite prevent command output blocking.
static char serialReply[1024];
static size_t replyLength = 0, replySent = 0;
static void replyAdd(const char *format, ...) {
  if (replyLength >= sizeof(serialReply)-1) return;
  va_list args; va_start(args, format);
  const size_t space = sizeof(serialReply)-replyLength;
  int n = vsnprintf(serialReply+replyLength, space, format, args);
  va_end(args);
  if (n > 0) replyLength += size_t(n) < space ? size_t(n) : space-1;
}
static void serialCommand(const char *cmd) {
  replyLength = replySent = 0;
  if (strcmp(cmd,"temps")==0) {
    const uint32_t now = millis();
    replyAdd("TEMPS: cached snapshot, normal refresh 500 ms\r\n");
    if (intTempEverValid) replyAdd("Heatsink ADC: %s; last complete good scan %lu ms ago; matching=%s\r\n",
      intTempValid?"OK":"READ FAILED",(unsigned long)(now-intTempLastGood),ntcMatch?"YES":"NO");
    else replyAdd("Heatsink ADC: no valid scan yet\r\n");
    for (unsigned i=0;i<6;++i) {
      if (intTempEverValid) replyAdd("NTC%u: %.2f C%s\r\n",i+1,measTemp_Int[i],intTempValid?"":" (STALE)");
      else replyAdd("NTC%u: unavailable\r\n",i+1);
    }
    if (extTempEverValid) replyAdd("External ADC: %s; last complete good scan %lu ms ago\r\n",
      extTempValid?"OK":"READ FAILED",(unsigned long)(now-extTempLastGood));
    else replyAdd("External ADC: no valid scan yet\r\n");
    for (unsigned i=0;i<4;++i) {
      if (isfinite(measTemp_Ext[i])) replyAdd("EXT%u: %.2f C\r\n",i+1,measTemp_Ext[i]);
      else replyAdd("EXT%u: unavailable (optional probe absent/invalid or read failed)\r\n",i+1);
    }
    replyAdd("Temperature fault=%s; startup failed=%s; disable input=%s\r\n",
      tempReadFailed?"YES":"NO",selfTestFailed?"YES":"NO",digitalRead(PIN_HARD_DISABLE)==HIGH?"HIGH":"LOW");
  } else if (strcmp(cmd,"health")==0) {
    replyAdd("HEALTH: steps over %lu ms=%lu; slowest=%s (%lu ms); automatic timing messages OFF\r\n",
      (unsigned long)MAX_STEP_DURATION_MS,(unsigned long)slowStepCount,worstStepName,(unsigned long)worstStepMs);
  } else {
    replyAdd("Commands: temps (all 6 heatsink + 4 external NTCs), health, help. Press Enter.\r\n");
  }
}
static void serviceSerialCommands() {
  if (replySent < replyLength) {
    int room = Serial.availableForWrite();
    if (room > 0) {
      size_t n = replyLength-replySent;
      if (n > 32) n=32;
      if (n > size_t(room)) n=size_t(room);
      replySent += Serial.write(reinterpret_cast<const uint8_t *>(serialReply+replySent),n);
    }
    return;
  }
  static char command[32];
  static size_t used=0;
  static bool overflow=false;
  for (unsigned count=0;count<32 && Serial.available();++count) {
    int ch=Serial.read();
    if (ch=='\r' || ch=='\n') {
      if (used || overflow) {
        command[used]='\0';
        serialCommand(overflow?"help":command);
        used=0; overflow=false; return;
      }
    } else if (!overflow) {
      if (ch>= 'A' && ch<= 'Z') ch += 'a'-'A';
      if (used < sizeof(command)-1) command[used++]=char(ch);
      else overflow=true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ELEC5552 Electronic Load - startup sequence");

  initialiseSystem();
  Serial.println("CV tuning v3 TRIAL: Kp=0.002 Ki=0.008 Kd=0 (PI); sample=50 ms; rise=0.015 fall=0.030 A/s");
  delay(100);  // let the INA/ADC finish their first conversions
  readSystemData();
  readTemperatureData();
  fanControl();
  runSelfTest();
  Serial.println("Serial commands: temps, health, help (press Enter)");

  uint32_t now = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t stepStart;

  const uint32_t systemPeriodMs = loadState == LoadState::RUNNING_CV
                                   ? CV_SYSTEM_PERIOD_MS : SYSTEM_PERIOD_MS;
  if (now - lastSystemMs >= systemPeriodMs) {
    lastSystemMs = now;
    stepStart = millis();
    readSystemData();
    checkStepDuration("readSystemData", stepStart);
    ui.recordStatsSample();
  }
  if (now - lastTemperatureMs >= TEMPERATURE_PERIOD_MS) {
    lastTemperatureMs = now;
    stepStart = millis();
    readTemperatureData();
    checkStepDuration("readTemperatureData", stepStart);
    stepStart = millis();
    fanControl();
    checkStepDuration("fanControl", stepStart);
  }
  if (!faultActive && now - lastProtectionMs >= PROTECTION_PERIOD_MS) {
    lastProtectionMs = now;
    stepStart = millis();
    checkFault();
    checkStepDuration("checkFault", stepStart);
  }
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    if (loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV) {
      if (testTimeElapsed > (uint32_t)(setTime * 1000.0f)) {loadState = LoadState::IDLE;}
    }
    stepStart = millis();
    loadControl();
    checkStepDuration("loadControl", stepStart);
    lastControlMs = now;
  }
  if (faultActive && now - lastResolveMs >= RESOLVE_PERIOD_MS) {
    lastResolveMs = now;
    stepStart = millis();
    resolveFault();
    checkStepDuration("resolveFault", stepStart);
  }
  if (now - lastUiMs >= UI_PERIOD_MS) {
    lastUiMs = now;
    stepStart = millis();
    userInputs();
    checkStepDuration("userInputs", stepStart);
  }
  // Observe run transitions every loop; logData limits samples to 2 Hz.
  logData();
  serviceSerialCommands();
}
