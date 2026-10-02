#pragma once
#include <Wire.h>
// Simulated VL53L0X: the reading comes from whichever mux port is selected.
class VL53L0X {
 public:
  void setAddress(uint8_t) {}
  bool init(bool io_2v8 = true);
  void startContinuous(uint32_t period_ms = 0) {}
  uint16_t readRangeContinuousMillimeters();
  void setTimeout(uint16_t) {}
  bool timeoutOccurred() { return false; }
};
