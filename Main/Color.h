#ifndef Color_h
#define Color_h

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "Adafruit_TCS34725.h"

// TCS port on mux
#define TCS_PORT 7

// External globals (defined in main.cpp)
extern QWIICMUX myMux;
extern Adafruit_TCS34725 tcs;
extern rtos::Mutex i2cMutex;

// Function declarations
void init_color();
int  read_color();

#endif
