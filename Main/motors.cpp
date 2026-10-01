#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <rtos.h>
#include "motors.h"
extern rtos::Mutex i2cMutex;
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *motorA;
extern Adafruit_DCMotor *motorB;
extern Adafruit_DCMotor *motorC;
extern Adafruit_DCMotor *motorD;
motors* motorsInstance = nullptr; // pointer to motor class

motors::motors(int encoderPin_A_A,int encoderPin_A_B,int encoderPin_B_A,int encoderPin_B_B,int encoderPin_D_A,int encoderPin_D_B){
  _encoderPin_A_A = encoderPin_A_A;
  _encoderPin_A_B = encoderPin_A_B;
  _encoderPin_B_A = encoderPin_B_A;
  _encoderPin_B_B = encoderPin_B_B;
  _encoderPin_D_A = encoderPin_D_A;
  _encoderPin_D_B = encoderPin_D_B;
  motorsInstance = this;

}
void encoder_update_A_ISR() {
  if (motorsInstance != nullptr) {
    motorsInstance->encoder_update_A();
  }
}

void encoder_update_B_ISR() {
  if (motorsInstance != nullptr) {
    motorsInstance->encoder_update_B();
  }
}
void encoder_update_D_ISR() {
  if (motorsInstance != nullptr) {
    motorsInstance->encoder_update_D();
  }
}
void motors::encoder_update_A(){
  int encoderState = digitalRead(_encoderPin_A_B);
 
  if(encoderState == LOW){
    encoderCountA--;
  }
  else{
    encoderCountA++;
  }
}
void motors::encoder_update_B(){
  int encoderState = digitalRead(_encoderPin_B_B);
  
  if(encoderState == HIGH){
    encoderCountB--;
  }
  else{
    encoderCountB++;
  }
}
void motors::encoder_update_D(){
  int encoderState = digitalRead(_encoderPin_D_B);
  if(encoderState == HIGH){
    encoderCountD--;
  }
  else{
    encoderCountD++;
  }
}
void motors::init_drive(){
  pinMode(_encoderPin_A_A, INPUT);
  pinMode(_encoderPin_A_B, INPUT);
  pinMode(_encoderPin_B_A, INPUT);
  pinMode(_encoderPin_B_B, INPUT);
  pinMode(_encoderPin_D_A, INPUT);
  pinMode(_encoderPin_D_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_A_A), encoder_update_A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(_encoderPin_B_A), encoder_update_B_ISR, RISING); // only counting rising of Pin: (20/2)/2
  attachInterrupt(digitalPinToInterrupt(_encoderPin_D_A),encoder_update_D_ISR,RISING);
  if(!AFMS.begin()){
    Serial.println("could not find motor shield");
  }
  else{
    Serial.println("motor shield found");
  }
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(BACKWARD);
  // initialize gyro
  
}
void motors::fullstop(){
  i2cMutex.lock();
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(0);
  motorB->setSpeed(0);
  motorC->setSpeed(0);
  motorD->setSpeed(0);
  i2cMutex.unlock();
}
void motors::reset_encoderCount(bool A, bool B, bool D){
  if(A == true) encoderCountA = 0;
  if(B == true) encoderCountB = 0;
  if(D == true) encoderCountD = 0;
}
void motors::set_encoderCountA(int A){
  encoderCountA = A;
}
void motors::set_encoderCountB(int B){
  encoderCountB = B;
}
void motors::set_encoderCountD(int D){
  encoderCountD = D;
}
void motors::set_interrupt(bool A, bool B, bool C){
  if(A == true){
    attachInterrupt(digitalPinToInterrupt(_encoderPin_A_A), encoder_update_A_ISR, RISING);
  }
  else{
    detachInterrupt(digitalPinToInterrupt(_encoderPin_A_A));
  }
  if(B == true){
    attachInterrupt(digitalPinToInterrupt(_encoderPin_B_A), encoder_update_B_ISR, RISING);
    
  }
  else{
    detachInterrupt(digitalPinToInterrupt(_encoderPin_B_A));
  }
  if(C == true){
    attachInterrupt(digitalPinToInterrupt(_encoderPin_D_A), encoder_update_D_ISR, RISING);
    
  }
  else{
    detachInterrupt(digitalPinToInterrupt(_encoderPin_D_A));
  }
}
void motors::drive(double A, double C, double B, double D){
  i2cMutex.lock();
  motorA->setSpeed((int)A);
  motorB->setSpeed((int)B);
  motorC->setSpeed((int)C);
  motorD->setSpeed((int)D);
  i2cMutex.unlock();
}
void motors::fw(int speed){
  i2cMutex.lock();
  motorA->run(FORWARD);
  motorB->run(FORWARD);
  motorC->run(FORWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
  i2cMutex.unlock();
}
void motors::backward(int speed){
  i2cMutex.lock();
  motorA->run(BACKWARD);
  motorB->run(BACKWARD);
  motorC->run(BACKWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
  i2cMutex.unlock();
}
void motors::turnright(int speed){
  i2cMutex.lock();
  motorA->run(FORWARD);
  motorB->run(BACKWARD);
  motorC->run(FORWARD);
  motorD->run(FORWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
  i2cMutex.unlock();
}
void motors::turnleft(int speed){
  i2cMutex.lock();
  motorA->run(BACKWARD);
  motorB->run(FORWARD);
  motorC->run(BACKWARD);
  motorD->run(BACKWARD);
  motorA->setSpeed(speed);
  motorB->setSpeed(speed);
  motorC->setSpeed(speed);
  motorD->setSpeed(speed);
  i2cMutex.unlock();
}