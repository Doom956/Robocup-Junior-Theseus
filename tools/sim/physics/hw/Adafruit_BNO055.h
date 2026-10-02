#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
typedef enum { OPERATION_MODE_CONFIG = 0x00, OPERATION_MODE_ACCONLY = 0x01, OPERATION_MODE_MAGONLY = 0x02, OPERATION_MODE_GYRONLY = 0x03,
  OPERATION_MODE_ACCMAG = 0x04, OPERATION_MODE_ACCGYRO = 0x05, OPERATION_MODE_MAGGYRO = 0x06, OPERATION_MODE_AMG = 0x07,
  OPERATION_MODE_IMUPLUS = 0x08, OPERATION_MODE_COMPASS = 0x09, OPERATION_MODE_M4G = 0x0A, OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
  OPERATION_MODE_NDOF = 0x0C } adafruit_bno055_opmode_t; // same values as the Adafruit library
// orientation.x = heading (0..360, clockwise). IMUPLUS: 0 = the direction it faced at start.
// NDOF (the library default): from magnetic north, as on the real BNO055.
// orientation.z = pitch (nose up positive)
class Adafruit_BNO055 {
 public:
  Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = 0x28, TwoWire *bus = &Wire) {}
  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  bool getEvent(sensors_event_t *e);
  void setExtCrystalUse(bool) {}
  void setMode(adafruit_bno055_opmode_t mode);
};
