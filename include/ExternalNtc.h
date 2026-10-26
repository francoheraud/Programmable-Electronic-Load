#pragma once
#include <math.h>
#include <stdint.h>

// Jaycar RN3440 datasheet, p2: R25=10k nominal (+/-10%), B25/50=4100 K.
// 10k pull-up to 3.3 V, NTC to ground; ADS1015 PGA +/-4.096 V, 2mV/LSB.
// Nominal conversion, not a per-probe calibration.
inline bool externalNtcCelsius(int16_t code, float& temperature) {
  if (code <= 0 || code >= 2047) return false;
  const float voltage = code * (4.096f / 2048.0f);
  // Reject near-ground and near-supply nodes; do not turn opens into temperatures.
  if (voltage <= 0.020f || voltage >= 3.250f) return false;
  const float resistance = 10000.0f * voltage / (3.3f - voltage);
  const float invT = 1.0f / 298.15f + logf(resistance / 10000.0f) / 4100.0f;
  const float result = 1.0f / invT - 273.15f;
  if (!isfinite(result) || result < -20.0f || result > 125.0f) return false;
  temperature = result;
  return true;
}
