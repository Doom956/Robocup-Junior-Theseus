#pragma once
#include <Wire.h>
typedef enum {
  TCS34725_INTEGRATIONTIME_2_4MS = 0xFF, TCS34725_INTEGRATIONTIME_24MS = 0xF6, TCS34725_INTEGRATIONTIME_50MS = 0xEB,
  TCS34725_INTEGRATIONTIME_101MS = 0xD5, TCS34725_INTEGRATIONTIME_154MS = 0xC0, TCS34725_INTEGRATIONTIME_614MS = 0x00
} tcs34725IntegrationTime_t;
typedef enum { TCS34725_GAIN_1X = 0, TCS34725_GAIN_4X = 1, TCS34725_GAIN_16X = 2, TCS34725_GAIN_60X = 3 } tcs34725Gain_t;
class Adafruit_TCS34725 {
 public:
  Adafruit_TCS34725(tcs34725IntegrationTime_t it = TCS34725_INTEGRATIONTIME_2_4MS, tcs34725Gain_t g = TCS34725_GAIN_1X) : it_(it) {}
  bool begin();
  void enable() {}
  void setInterrupt(bool) {}
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c); // waits one integration time, like the real library
 private:
  tcs34725IntegrationTime_t it_;
};
