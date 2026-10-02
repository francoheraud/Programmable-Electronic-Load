#include <Arduino.h>
#include <Wire.h>
#include <SystemState.h>
#include <Display.h>
#include <INA.h>
#include <ADS1015.h>
#include <ADS7128.h>
#include <DAC.h>
#include <UI.h>

//PINS
#define PIN_PWM          1
#define PIN_FAULT_SET    2  //MCU_DISABLE signal
#define PIN_CS_2         3
#define PIN_MOSI_2       4
#define PIN_SCK_2        5
#define PIN_HARD_DISABLE 6  //hardware signal. Pin altered from schematic
#define PIN_FAULT_RST    7  //MCU_RESET
#define PIN_SDA_1        8
#define PIN_SCL_1        9
#define PIN_BACK_BUTTON  14 //separate back/cancel/abort button
#define PIN_SPARE_4      15
#define PIN_SDA_2        17
#define PIN_SCL_2        18
#define PIN_SPARE_5      21
#define PIN_ENC_A        38 //quadrature phase A
#define PIN_ENC_B        39 //quadrature phase B
#define PIN_ENC_BUTTON   40 //encoder's integrated pushbutton (confirm/select)
//#define PIN_SPARE_1      46
//#define PIN_SPARE_2      47
//#define PIN_SPARE_3      48

const uint8_t inaAddrs[6] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
const uint16_t shuntCal = 3156;  // 25mΩ shunt, 5.05A/leg max
const uint8_t VBUS_LEG = 0;  // INA leg configured to read VBUS
static const float heatsinkDifferentialAlert = 0.05f;
static const float heatsinkTemperatureAlert = 100.0f;

static const uint16_t DAC_FULL_SCALE_CODE = 0xFFFF;
static const float MAX_CURRENT_PER_LEG_A = 5.05f;              // matches shuntCal above
static const float MAX_TOTAL_CURRENT_A = MAX_CURRENT_PER_LEG_A * DAC_NUM_SETPOINTS;
static const float CV_CURRENT_GAIN = 0.5f;                     // A of current demand per V of error

//TODO confirm fan below
#define FAN_PWM_FREQ_HZ   25000
#define FAN_PWM_RES_BITS  8

// setpoints
float overVoltage = 0.0f;
float overCurrent = 0.0f;
float overPower = 500.0f;
float overTemperature = 0.0f;
float setCurrent = 0.0f;
float setVoltage = 0.0f;
float setTime = 0.0f;
uint32_t testTimeElapsed = 0;

// Live measurements
float measV_Bus = 0.0f;
float measCurrent[6] = {0, 0, 0, 0, 0, 0};
float totalCurrent = 0.0f;
float measTemp_Int[6] = {0, 0, 0, 0, 0, 0};
float measTemp_Ext[4] = {0, 0, 0, 0};
float measPower = 0.0f;

struct LogRecord {
  uint32_t timestampMs;
  float voltage;
  float current;
  float tempC[4];
  LoadState log_loadstate;
};

LoadState loadState = LoadState::INIT;
FaultReason faultReason = FaultReason::NONE;
bool faultActive = false;
bool sensorReadFailed = false;
static bool inaReadFailed = false;
static bool tempReadFailed = false;
static uint32_t inaFailSinceMs = 0;
static uint32_t tempFailSinceMs = 0;

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
static const uint32_t LOGGING_PERIOD_MS = 1000;
static const uint32_t TEMPERATURE_PERIOD_MS = 500;
static const uint32_t SYSTEM_PERIOD_MS = 1000;
static const uint32_t RESOLVE_PERIOD_MS = 50;
static const uint32_t MAX_STEP_DURATION_MS = 8; // timeout delay

static inline void checkStepDuration(const char* stepName, uint32_t stepStartMs) {
  uint32_t elapsed = millis() - stepStartMs;
  if (elapsed > MAX_STEP_DURATION_MS) {
    Serial.printf("LOOP HEALTH: step '%s' took %lu ms (limit %lu ms) -- "
                  "check for a stalled I2C/SPI call\n",
                  stepName, (unsigned long)elapsed, (unsigned long)MAX_STEP_DURATION_MS);
  }
}

void initialiseSystem() {
  //TODO: initialise MicroSD

  pinMode(PIN_FAULT_SET, OUTPUT);   // MCU_DISABLE  (active-high)
  digitalWrite(PIN_FAULT_SET, LOW);
  pinMode(PIN_FAULT_RST, OUTPUT);   // MCU_RESET    (active-high, pulsed)
  digitalWrite(PIN_FAULT_RST, LOW);
 
  ledcSetup(0, FAN_PWM_FREQ_HZ, FAN_PWM_RES_BITS);
  ledcAttachPin(PIN_PWM, 0);
  ledcWrite(0, 0);
  
  pinMode(PIN_HARD_DISABLE, INPUT);
  pinMode(PIN_ENC_A,    INPUT_PULLUP);
  pinMode(PIN_ENC_B,    INPUT_PULLUP);
  pinMode(PIN_BACK_BUTTON, INPUT_PULLUP);
  pinMode(PIN_ENC_BUTTON, INPUT_PULLDOWN);

  Wire1.begin(PIN_SDA_1, PIN_SCL_1);
  Wire1.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  Wire2.begin(PIN_SDA_2, PIN_SCL_2);
  Wire2.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);

  if (!ina.begin(inaAddrs, shuntCal, VBUS_LEG)) {
    Serial.println("initialiseSystem: one or more INAs failed to configure");
  }

  if (!ads7128.begin()) {
    Serial.println("initialiseSystem: ADS7128 failed to configure");
  }

  if (!display.begin()) {
    Serial.println("initialiseSystem: OLED did not ACK on I2C_2");
  }

  ui.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BUTTON, PIN_BACK_BUTTON);

  dac.begin(SPI, PIN_SCK_2, PIN_MOSI_2, PIN_CS_2);

  Serial.println("initialised");
}

void runSelfTest() {
  // TODO: check values are within range
  // if all good then loadstate = IDLE else FAULT
  Serial.println("startup test complete");
  loadState = LoadState::IDLE;
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
    testTimeElapsed += CONTROL_PERIOD_MS;
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
  //update tacho based on internal temp readings.
  //waiting for Kurtis to send datasheet
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

void logData() {
  Serial.printf("[loadState=%d] V=%.2f I=%.2f\n",
                (int)loadState, measV_Bus, totalCurrent);
  // TODO: write to a flash log buffer, expose over USB CDC for download.
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ELEC5552 Electronic Load - startup sequence");

  initialiseSystem();
  readSystemData();
  readTemperatureData();
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
