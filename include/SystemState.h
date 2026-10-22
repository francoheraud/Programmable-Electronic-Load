#pragma once
#include <Arduino.h>

enum class LoadState : uint8_t { INIT, IDLE, RUNNING_CC, RUNNING_CV, FAULT };
enum class FaultReason : uint8_t { NONE, INT_TEMP, EXT_TEMP, VOLTAGE, CURRENT, POWER, SENSOR_UNRESPONSIVE, SELF_TEST };

// Setpoints (defined in main.cpp)
extern float overVoltage;
extern float overCurrent;
extern float overPower;
extern float overTemperature;
extern float setCurrent;
extern float setVoltage;
extern float setTime;
extern uint32_t testTimeElapsed;  // ms since the run started (real time)
extern uint32_t testStartMs;      // millis() when the run started (set by the UI)

// Live measurements (defined in main.cpp)
extern float measV_Bus;
extern float measCurrent[6];
extern float totalCurrent;
extern float measTemp_Int[6];
extern float measTemp_Ext[4];
extern float measPower;

// State (defined in main.cpp)
extern LoadState loadState;
extern FaultReason faultReason;
extern bool faultActive;
