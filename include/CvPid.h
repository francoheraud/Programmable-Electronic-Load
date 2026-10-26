#pragma once
#include <math.h>
#include <stdint.h>

// Conservative starting values for LOW-POWER bench commissioning, not a
// validated tuning for every source, cable capacitance or operating point.
struct CvPidSettings {
  float kp = 0.002f;       // A/V; reduced after source/load cycling on the bench
  float ki = 0.008f;       // A/(V*s); v3 trial, compare against logged v2 baseline
  float kd = 0.000f;       // A*s/V; disabled pending waveform-based tuning (PI)
  float derivativeTau = 0.300f; // seconds
  float riseRate = 0.015f; // A/s; bounded faster approach for v3 trial
  float fallRate = 0.030f; // A/s; faults/stop bypass this limit
  uint32_t maxSampleAgeMs = 600;
};

class CvPid {
 public:
  explicit CvPid(const CvPidSettings& settings = CvPidSettings()) : cfg(settings) {}

  void reset() {
    active = false;
    havePreviousVoltage = false;
    integral = derivative = output = 0.0f;
  }

  // sequence advances ONLY after a complete successful INA acquisition.
  // Called frequently, but integrates only once per new acquisition.
  float update(bool enabled, bool valid, uint32_t nowMs,
               uint32_t sampleMs, uint32_t sequence,
               float voltage, float target, float currentLimit) {
    if (!enabled || !valid || !isfinite(voltage) || !isfinite(target) ||
        !isfinite(currentLimit) || voltage < 0 || target <= 0 ||
        currentLimit <= 0 || nowMs - sampleMs > cfg.maxSampleAgeMs) {
      reset();
      return 0;
    }
    if (!active) {
      reset();
      active = true;
      lastSequence = sequence;
      lastUpdateMs = nowMs;
      return 0; // Require a fresh acquisition after enable/recovery.
    }
    output = clamp(output, 0, currentLimit); // Limit reductions act immediately.
    if (sequence == lastSequence) return output;
    lastSequence = sequence;
    const uint32_t elapsed = sampleMs - lastUpdateMs;
    lastUpdateMs = sampleMs;
    if (elapsed == 0) return output;
    if (elapsed > cfg.maxSampleAgeMs) {
      reset(); // Do not integrate a long scheduling gap.
      return 0;
    }
    const float dt = elapsed * 0.001f;
    const float error = voltage - target; // Sink more when voltage is too high.
    const float rate = havePreviousVoltage ? (voltage - previousVoltage) / dt : 0;
    derivative += (dt / (cfg.derivativeTau + dt)) * (rate - derivative);
    previousVoltage = voltage;
    havePreviousVoltage = true;
    const float p = cfg.kp * error;
    const float d = cfg.kd * derivative; // Positive sign for this sinking plant.
    const float candidateI = integral + cfg.ki * error * dt;
    const float requested = p + candidateI + d;
    const float low = fmaxf(0, output - cfg.fallRate * dt);
    const float high = fminf(currentLimit, output + cfg.riseRate * dt);
    output = clamp(requested, low, high);
    // Tracking anti-windup includes BOTH the current ceiling and slew limiter.
    // Store only the integral contribution consistent with achievable output.
    integral = output - p - d;
    return output;
  }

 private:
  static float clamp(float value, float low, float high) {
    return fminf(high, fmaxf(low, value));
  }
  CvPidSettings cfg;
  bool active = false, havePreviousVoltage = false;
  uint32_t lastSequence = 0, lastUpdateMs = 0;
  float integral = 0, derivative = 0, output = 0, previousVoltage = 0;
};
