#include "gyro.h"
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <rtos.h>

extern Adafruit_BNO055 bno;
extern rtos::Mutex i2cMutex;

gyro::gyro(){
  
}
void gyro::init_Gyro(){
  i2cMutex.lock();
  bool found = bno.begin();
  i2cMutex.unlock();
  if(!found) Serial.println("can't find gyro");
  else        Serial.println("gyro found");
}
int gyro::inverse(int val, bool inv){
  if(inv == true) {
    if(val == 0) return val; 
    val = val - 360;
  }
  else{val = val%360;}
  return val;
}
int gyro::modulus(int val){
  // normalize to signed [-180, 180] so tilt magnitude is correct for both
  // positive and negative angles (a -6 deg tilt -> -6, not 366).
  val = val % 360;
  if(val > 180) val -= 360;
  else if(val < -180) val += 360;
  return val;
}
double gyro::heading(){
  sensors_event_t event;
  i2cMutex.lock();
  bno.getEvent(&event);
  i2cMutex.unlock();
  float heading = (double)event.orientation.x;
  
  return heading;
}
double gyro::pitch_heading(){
  sensors_event_t event;
  i2cMutex.lock();
  bno.getEvent(&event);
  i2cMutex.unlock();
  float pitch_heading = (double)event.orientation.z;
  if (abs(360-pitch_heading)< 5||abs(pitch_heading)<5) pitch_heading = 0;
  return pitch_heading;
}
void gyro::reset_accel_filter(){
  accelFilterInitialized = false;
  accelFiltered = 0.0;
}
double gyro::opposite_heading(double h) {
  h += 180.0;
  if (h >= 360.0) h -= 360.0;
  return h;
}
int gyro::headingToCardinal(double heading){
    // normalize angle
    if (heading < 0) heading += 360;
    if (heading >= 360) heading -= 360;

    if (heading >= 315 || heading < 45)
        return 0; // North
    else if (heading >= 45 && heading < 135)
        return 1; // East
    else if (heading >= 135 && heading < 225)
        return 2; // South
    else
        return 3; // West
}