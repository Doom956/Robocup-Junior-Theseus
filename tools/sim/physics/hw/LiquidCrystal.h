#pragma once
#include <Arduino.h>
class LiquidCrystal {
 public:
  LiquidCrystal(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {}
  void begin(uint8_t cols, uint8_t rows) {}
  void setCursor(uint8_t col, uint8_t row) {}
  size_t print(const char *text); // the simulator watches for "back to start" / "no path found"
  size_t print(const String &t) { return print(t.c_str()); }
};
