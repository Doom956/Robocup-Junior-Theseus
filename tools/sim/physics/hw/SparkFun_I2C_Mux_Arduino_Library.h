#pragma once
#include <Wire.h>
class QWIICMUX {
 public:
  bool begin(uint8_t addr = 0x70, TwoWire &bus = Wire);
  bool setPort(uint8_t port); // selects which simulated sensor the next read talks to
};
