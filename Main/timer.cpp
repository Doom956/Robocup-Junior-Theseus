#include<Arduino.h>
#include "timer.h"

timer::timer(){
  startTime = micros();
}
double timer::getTime(){
  currentTime = micros();
  return currentTime - startTime-(endTimeStamp-startTimeStamp);
}
double timer::delta_time(){
  current = getTime();
  delta = current - last;
  return delta;
}
double timer::reset_delta_time(){
  double prev = delta;
  last = current;
  return prev;
}
void timer::pause(int on){
  if(on == 1) startTimeStamp = micros();
  if(on == 2) endTimeStamp = micros();
  
}