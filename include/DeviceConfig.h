#pragma once
#include <stdint.h>

// Shared by src/main.cpp and src/bringup so both use the same device settings.
static const uint8_t inaAddrs[6] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
static const uint16_t shuntCal = 3156;  // 25mΩ shunt, 5.05A/leg max
static const uint8_t VBUS_LEG = 0;  // INA leg configured to read VBUS

static const uint16_t DAC_FULL_SCALE_CODE = 0xFFFF;
static const float MAX_CURRENT_PER_LEG_A = 5.05f;
