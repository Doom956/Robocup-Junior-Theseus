#ifndef Distance_h
#define Distance_h

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>

// PCA9685 / AllCall constants
#define PCA_ADDR    0x60
#define MODE1       0x00
#define ALLCALL_BIT 0x01

// Sensor & mux arrays (defined in Main.ino)
extern VL53L0X sensors[7];
extern QWIICMUX myMux;

// I2C mutex (defined in Main.ino)
#include <rtos.h>
extern rtos::Mutex i2cMutex;

// Motor objects (defined in Main.ino)
#include <Adafruit_MotorShield.h>
extern Adafruit_DCMotor *motorA;
extern Adafruit_DCMotor *motorB;
extern Adafruit_DCMotor *motorC;
extern Adafruit_DCMotor *motorD;

// Drivetrain object (defined in Main.ino)
#include "motors.h"
extern motors drivetrain;

// Gyro object (defined in Main.ino)
#include "gyro.h"
extern gyro myGyro;

// Pause flag (defined in Main.ino)
extern volatile bool Pausemaze;

// Obstacle avoidance step enum (defined in Globals.h / Main.ino)
enum Steps : int;
extern Steps steps;

// --- Function declarations ---

void disableAllCall();
void init_dist();
uint8_t scanI2COnCurrentBus();
void scanAllPorts();
int  measure(int sensor);
int  detectWall(int dir);
void parallel();
void centerFrontBack();
int  center();
int  centerLeft();
int  obstacleavoidance(int leftright);

#endif
