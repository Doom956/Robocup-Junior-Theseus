#pragma once
#include <Arduino.h>
// I2C bus: transactions cost a little time; scans find nothing (the simulated devices
// are reached through their own library classes).
class TwoWire {
 public:
  void begin();
  void beginTransmission(uint8_t addr);
  size_t write(uint8_t b);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t addr, uint8_t n);
  int available();
  int read();
};
extern TwoWire Wire;
