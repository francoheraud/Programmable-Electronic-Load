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

static const float heatsinkDifferentialAlert = 0.05f;
static const float heatsinkTemperatureAlert = 100.0f;

static const float MAX_TOTAL_CURRENT_A = MAX_CURRENT_PER_LEG_A * DAC_NUM_SETPOINTS;
static const float CV_CURRENT_GAIN = 0.5f; // A of current demand per V of error

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
float measTemp_Ext[4] = {0, 0, 0, 0};
float measPower = 0.0f;

static SPIClass sdSpi(HSPI);
static bool inaInitOk = false;
static bool adcInitOk = false;
static bool oledInitOk = false;
static bool selfTestFailed = false;

LoadState loadState = LoadState::INIT;
FaultReason faultReason = FaultReason::NONE;
bool faultActive = false;
bool sensorReadFailed = false;
static bool inaReadFailed = false;
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
static const uint32_t RESOLVE_PERIOD_MS = 50;
static const uint32_t MAX_STEP_DURATION_MS = 20; //timeout delay

static inline void checkStepDuration(const char* stepName, uint32_t stepStartMs) {
  uint32_t elapsed = millis() - stepStartMs;
  if (elapsed > MAX_STEP_DURATION_MS) {
    Serial.printf("LOOP HEALTH: step '%s' took %lu ms (limit %lu ms) -- "
                  "check for a stalled I2C/SPI call\n",
                  stepName, (unsigned long)elapsed, (unsigned long)MAX_STEP_DURATION_MS);
  }
}

void initialiseSystem() {
  pinMode(PIN_FAULT_SET, OUTPUT);
  digitalWrite(PIN_FAULT_SET, LOW);
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

  if (digitalRead(PIN_HARD_DISABLE) == HIGH) {
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
  if (tempReadFailed) { fail("temperature read / heatsink NTC mismatch"); }

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
    if (measTemp_Ext[i] < SELF_TEST_MIN_TEMP_C || measTemp_Ext[i] > SELF_TEST_MAX_TEMP_C) {
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

  if (!ads7128.readTemperatures(measTemp_Int)) {
    allOk = false;
  } else if (!heatsinkTempsConsistent(measTemp_Int)) {
    allOk = false;
  }

  if (!ads1015.readTemperatures(measTemp_Ext)) {
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
}

static uint16_t currentToDacCode(float currentPerLegA) {
  if (currentPerLegA < 0.0f) { currentPerLegA = 0.0f; }
  if (currentPerLegA > MAX_CURRENT_PER_LEG_A) { currentPerLegA = MAX_CURRENT_PER_LEG_A; }
  return (uint16_t)((currentPerLegA / MAX_CURRENT_PER_LEG_A) * DAC_FULL_SCALE_CODE);
}

static void writeCurrentSetpoint(float totalCurrentA) {
  if (totalCurrentA < 0.0f) { totalCurrentA = 0.0f; }
  if (totalCurrentA > MAX_TOTAL_CURRENT_A) { totalCurrentA = MAX_TOTAL_CURRENT_A; }

  uint16_t code = currentToDacCode(totalCurrentA / DAC_NUM_SETPOINTS);
  uint16_t codes[DAC_NUM_SETPOINTS];
  for (uint8_t i = 0; i < DAC_NUM_SETPOINTS; i++) {
    codes[i] = code;
  }
  dac.writeAllSetpoints(codes);
}

void loadControl() {
  static float cvCurrentCommand = 0.0f;
  float commandCurrent = 0.0f;

  if (loadState == LoadState::RUNNING_CC || loadState == LoadState::RUNNING_CV) {
    testTimeElapsed = millis() - testStartMs;
  }

  if (loadState == LoadState::RUNNING_CV) {
    float voltageError = setVoltage - measV_Bus;
    cvCurrentCommand += CV_CURRENT_GAIN * voltageError;
    if (cvCurrentCommand < 0.0f) { cvCurrentCommand = 0.0f; }
    if (cvCurrentCommand > setCurrent) { cvCurrentCommand = setCurrent; }
    commandCurrent = cvCurrentCommand;
  } else {
    cvCurrentCommand = 0.0f;
  }

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
  if (measPower > overPower) {
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
    if (measTemp_Ext[i] > overTemperature) {
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
  bool running = isRunningState(loadState);

  Serial.printf("[loadState=%d] V=%.2f I=%.2f\n",
                (int)loadState, measV_Bus, totalCurrent);

  // Open/close the CSV on run start/end. Done here (1 s) so SD stalls stay out of the control step.
  if (running && !wasRunning) {
    sdLog.startTest();
  } else if (!running && wasRunning) {
    sdLog.endTest();
  }
  wasRunning = running;

  if (!sdLog.active() || !running) { return; }

  LogRecord rec;
  rec.timestampMs = testTimeElapsed;
  rec.state = loadState;
  rec.voltage = measV_Bus;
  rec.current = totalCurrent;
  rec.power = measPower;
  memcpy(rec.legCurrent, measCurrent, sizeof(rec.legCurrent));
  memcpy(rec.tempInt, measTemp_Int, sizeof(rec.tempInt));
  memcpy(rec.tempExt, measTemp_Ext, sizeof(rec.tempExt));
  sdLog.append(rec);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ELEC5552 Electronic Load - startup sequence");

  initialiseSystem();
  delay(100);  // let the INA/ADC finish their first conversions
  readSystemData();
  readTemperatureData();
  fanControl();
  runSelfTest();

  uint32_t now = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t stepStart;

  if (now - lastSystemMs >= SYSTEM_PERIOD_MS) {
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
  if (now - lastLoggingMs >= LOGGING_PERIOD_MS) {
    lastLoggingMs = now;
    stepStart = millis();
    logData();
    checkStepDuration("logData", stepStart);
  }
}
