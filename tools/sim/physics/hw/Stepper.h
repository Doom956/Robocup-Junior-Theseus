#pragma once
#include <Arduino.h>
class Stepper {
 public:
  Stepper(int steps, int p1, int p2, int p3, int p4) : steps_(steps) {}
  void setSpeed(long rpm) { rpm_ = rpm; }
  void step(int n) { if (rpm_ > 0) delay((unsigned long)(std::abs(n) * 60000.0 / (steps_ * rpm_))); }
 private:
  int steps_;
  long rpm_ = 0;
};
