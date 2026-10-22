#pragma once
#include <stdint.h>

// Hardware-free UI helpers. Kept free of Arduino headers so they can be unit-tested natively.

#if defined(__GNUC__)
#define UI_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define UI_ALWAYS_INLINE inline
#endif

// Index = (previousState << 2) | newState, state = (A << 1) | B.
// Returns +1 / -1 for a valid step, 0 for no movement or an invalid (skipped) transition.
UI_ALWAYS_INLINE int8_t decodeQuadrature(uint8_t prevState, uint8_t newState) {
  static const int8_t QUADRATURE_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
  };
  return QUADRATURE_TABLE[((prevState & 0x3) << 2) | (newState & 0x3)];
}

// Adds delta to the running carry and returns whole detents, leaving the remainder in carry.
inline int32_t extractDetents(int32_t& carry, int32_t delta, int32_t countsPerDetent) {
  carry += delta;
  int32_t detents = carry / countsPerDetent;
  carry -= detents * countsPerDetent;
  return detents;
}

struct ButtonState {
  bool rawPressed = false;
  bool stableLevelPressed = false;
  bool pressedEdge = false;
  uint32_t lastChangeMs = 0;
};

// pressedEdge is true for exactly one call after the level has been stable for stableMs.
inline void debounceUpdate(ButtonState& btn, bool rawLevel, uint32_t now, uint32_t stableMs) {
  btn.pressedEdge = false;
  if (rawLevel != btn.rawPressed) {
    btn.rawPressed = rawLevel;
    btn.lastChangeMs = now;
  }
  if ((now - btn.lastChangeMs) >= stableMs && btn.rawPressed != btn.stableLevelPressed) {
    btn.stableLevelPressed = btn.rawPressed;
    if (btn.stableLevelPressed) { btn.pressedEdge = true; }
  }
}
