#pragma once
#include <Arduino.h>

// Bring-up commands for the sensor / actuator devices. Each peripheral is started lazily on first use.
bool devicesCommand(int argc, String argv[]);  // true if the command was recognised
void devicesTick(uint32_t now);                // call every loop(): expires DAC holds
void devicesSafeState();                       // DAC setpoints to 0 and disarm
void devicesSense();                           // one-shot read of INA + NTC + external probes
void devicesPrintHelp();
