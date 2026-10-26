#pragma once
#include <stdint.h>
#include <math.h>
// Analogue PCB: six equal 10k summing resistors average six 25mOhm shunts.
// OUT6 reaches Prog_OC via 15k/1k; bus reaches OV comparator via 15k/1k.
// DAC GAIN=0x01FF gives a nominal 2.5V range; DAC transfer uses 65536 steps.
inline bool protectionDacCodes(float amps, float volts, uint16_t &oc, uint16_t &ov) {
  if (!isfinite(amps) || !isfinite(volts) || amps <= 0 || amps > (5.05f * 6.0f) || volts <= 0 || volts > 30.0f) return false;
  const float ocCode = (amps * 0.025f / 6.0f * 16.0f) / 2.5f * 65536.0f;
  const float ovCode = (volts / 16.0f) / 2.5f * 65536.0f;
  // Round down: never raise the nominal requested trip threshold.
  oc = static_cast<uint16_t>(ocCode);
  ov = static_cast<uint16_t>(ovCode);
  return oc > 0 && ov > 0;
}
