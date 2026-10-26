#pragma once
#include <stdint.h>
#include <cmath>
using std::isfinite;
extern uint32_t nowMs;
inline uint32_t millis(){return nowMs;}
inline void delay(unsigned n){nowMs+=n;}
struct Console { template<class... T> void printf(const char*, T...){} void println(const char*){} };
extern Console Serial;
