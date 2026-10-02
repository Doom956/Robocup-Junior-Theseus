#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
typedef enum { OPERATION_MODE_CONFIG = 0x00, OPERATION_MODE_IMUPLUS = 0x08, OPERATION_MODE_NDOF = 0x0C } adafruit_bno055_opmode_t;
// orientation.x = heading (0..360, clockwise, 0 = the direction it faced at start)
// orientation.z = pitch (nose up positive)
class Adafruit_BNO055 {
 public:
  Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = 0x28, TwoWire *bus = &Wire) {}
  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  bool getEvent(sensors_event_t *e);
  void setExtCrystalUse(bool) {}
  void setMode(adafruit_bno055_opmode_t) {}
};
