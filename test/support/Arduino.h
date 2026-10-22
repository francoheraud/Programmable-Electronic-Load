#pragma once
// Minimal host-side stand-in for Arduino.h, used only by the `native` test environment.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>

#define IRAM_ATTR
#define HIGH 1
#define LOW  0
#define CHANGE 3
#define INPUT_PULLUP 2

using String = std::string;

inline uint32_t g_millis = 0;
inline uint8_t  g_pinLevel[64] = {0};

inline uint32_t millis() { return g_millis; }
inline int digitalRead(uint8_t pin) { return g_pinLevel[pin]; }
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(uint8_t, void (*)(), int) {}

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m)      ((void)(m))
#define portEXIT_CRITICAL(m)       ((void)(m))
#define portENTER_CRITICAL_ISR(m)  ((void)(m))
#define portEXIT_CRITICAL_ISR(m)   ((void)(m))

template <class T> T min(T a, T b) { return a < b ? a : b; }
template <class T> T max(T a, T b) { return a > b ? a : b; }
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
