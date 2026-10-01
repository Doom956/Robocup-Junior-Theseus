#include <Arduino.h>
#include "dispenser.h"
#include <Stepper.h>
extern Stepper myStepper;
extern int medkits;
dispenser::dispenser(int _incr, int _offset,int _steps_per_rev){
  incr = _incr;
  offset = _offset;
  steps_per_rev = _steps_per_rev;
  leftDispensed = 0; rightDispensed = 0;
  
}
void dispenser::rotate(int degrees){
  myStepper.setSpeed(7);
  long steps = ((long)degrees*steps_per_rev)/360;
  myStepper.step(steps);
}
void dispenser::dispenseLeft(char victim){ // clockwise
  if(medkits <= 0){
    Serial.println("dispenseLeft: no medkits remaining, skipping dispense");
    return;
  }
  switch(victim){
    case 'H':
      rotate((incr*2+offset+leftDispensed*incr));
      leftDispensed += 2;
      medkits -= 2;
      rotate(-(offset+leftDispensed*incr));
      break;
    case 'S':
      rotate((incr*1+offset+leftDispensed*incr));
      leftDispensed += 1;
      medkits -= 1;
      rotate(-(offset+leftDispensed*incr));
      break;
    case 'U':
      break;
  }
}
void dispenser::dispenseRight(char victim){ // clockwise
  if(medkits <= 0){
    Serial.println("dispenseRight: no medkits remaining, skipping dispense");
    return;
  }
  switch(victim){
    case 'H':
      rotate(-(incr*2+offset+rightDispensed*incr));
      rightDispensed += 2;
      medkits -= 2;
      rotate(offset+rightDispensed*incr);
      break;
    case 'S':
      rotate(-(incr*1+offset+rightDispensed*incr));
      rightDispensed += 1;
      medkits -= 1;
      rotate(offset+rightDispensed*incr);
      break;
    case 'U':
      break;
  }
}