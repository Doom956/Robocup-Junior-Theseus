
#include <Arduino.h>
#include "PID.h"

PID::PID(double _kp, double _ki, double _kd){
  
  kp = _kp; // public kp = inputted _kp
  ki = _ki;
  kd = _kd;
  previousTime = micros();

}
double PID::getPID(double _error){
  error = _error;
  currentTime = micros()-(end-start); // using micros since functions are slower
  delta = (error-prevError)/(currentTime - previousTime);
  cumError += error;
  // Anti-windup: clamp the integral term so a long stall (motors pushed against
  // a wall, sensor dropout keeping err non-zero) cannot grow cumError without
  // bound. Range is generous — normal steady-state sits well inside it.
  cumError = constrain(cumError, -3000.0, 3000.0);
  double output = kp*error + ki*cumError + kd*delta;
  previousTime = currentTime;
  prevError = error; // update previousTime and prevError
  return output;

}
void PID::pausePID(int on){
  if(on == 1) start = micros();
  if(on == 2) {
    end = micros();
    // Realign previousTime so the next getPID() call doesn't see a huge time
    // gap that causes a delta spike. Without this, (currentTime - previousTime)
    // includes the entire pause duration, making delta ≈ 0 for one cycle and
    // then spiking on the next.
    previousTime = micros() - (end - start);
  }
}

