#pragma once
#include <math.h>
#include <stdint.h>

// ELEC5552 2026 project brief, requirements 1-3. These are command limits,
// not evidence that the assembled hardware has been qualified at these ratings.
namespace LoadSetpoints {
constexpr float CURRENT_STEP_A = 0.01f;
constexpr float VOLTAGE_STEP_V = 0.1f;
constexpr float MAX_CURRENT_A = 30.0f;
constexpr float MAX_VOLTAGE_V = 30.0f;
constexpr float MAX_POWER_W = 500.0f;

inline float powerLimit(float requestedWatts) {
  return isfinite(requestedWatts) ? fminf(MAX_POWER_W, fmaxf(0, requestedWatts)) : 0;
}

inline float currentCeiling(float voltage, float requestedWatts, bool sampleValid) {
  if (!sampleValid || !isfinite(voltage) || voltage < 0) return 0;
  // At zero volts the current rating still applies; a fresh sample is required.
  return fminf(MAX_CURRENT_A, powerLimit(requestedWatts) / fmaxf(voltage, 0.001f));
}

inline float advanceCurrent(float value, int32_t detents, uint8_t stepIndex) {
  const int64_t ticksPerDetent = stepIndex == 2 ? 100 : (stepIndex == 1 ? 10 : 1);
  if (!isfinite(value)) value = 0;
  int64_t ticks = lroundf(fminf(MAX_CURRENT_A, fmaxf(0, value)) / CURRENT_STEP_A);
  ticks += (int64_t)detents * ticksPerDetent;
  if (ticks < 0) ticks = 0;
  if (ticks > 3000) ticks = 3000;
  return ticks * CURRENT_STEP_A;
}

// Work in integer ticks to prevent accumulated float drift over many detents.
inline float advance(float value, int32_t detents, float step, float maximum) {
  if (!isfinite(value)) value = 0;
  value = fminf(maximum, fmaxf(0, value));
  int64_t ticks = (int64_t)lroundf(value / step) + (int64_t)detents;
  const int64_t maxTicks = (int64_t)lroundf(maximum / step);
  if (ticks < 0) ticks = 0;
  if (ticks > maxTicks) ticks = maxTicks;
  return (float)ticks * step;
}
}
