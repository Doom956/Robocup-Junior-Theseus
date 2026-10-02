#pragma once
#include <cstdint>
struct sensors_vec_t { float x = 0, y = 0, z = 0; int8_t status = 0; };
struct sensors_event_t {
  int32_t version = 0, sensor_id = 0, type = 0, reserved0 = 0, timestamp = 0;
  sensors_vec_t orientation, acceleration, gyro, magnetic;
};
