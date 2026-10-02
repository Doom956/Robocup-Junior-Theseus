#pragma once
#include <Wire.h>
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
class Adafruit_DCMotor {
 public:
  int num = 0;            // motor number on the shield (1..4)
  void run(uint8_t cmd);  // direction
  void setSpeed(uint8_t speed);
};
class Adafruit_MotorShield {
 public:
  Adafruit_MotorShield(uint8_t addr = 0x60) {}
  bool begin(uint16_t freq = 1600, TwoWire *bus = &Wire);
  Adafruit_DCMotor *getMotor(uint8_t n);
};
