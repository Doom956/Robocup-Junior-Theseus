// camera communication code
// clear serial buffers.

#include "Globals.h"
#include "dispenser.h"

extern dispenser disp;
extern char classes[6];
extern int LEDPIN;
extern volatile int victimSide;

void lcdPrint(const char* msg) {
  lcdMutex.lock();
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(msg);
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcdMutex.unlock();
}

void clearSerialBuffer1() {
  while (Serial4.available() > 0) {
    Serial4.read();  // Read and discard one byte from the buffer
  }
}
void clearSerialBuffer2() {
  while (Serial3.available() > 0) {
    Serial3.read();  // Read and discard one byte from the buffer
  }
}

// classify a single camera byte: H/S/U = victim (0/1/2),
// L/c/I = camera "searching"/circle signal (-2), anything else -2.
int classifyCamByte(char sample){
  if(sample == 'H') return 0;
  if(sample == 'S') return 1;
  if(sample == 'U') return 2;
  return -2; // L / c / I / stray byte -> not a lettered victim
}
int readSerial1(){ // left
  // The camera streams continuously, so the RX FIFO backs up while we move.
  // Drain everything and keep only the NEWEST byte so we react to what the
  // camera sees now, not a byte from several frames (and tiles) ago.
  if(Serial4.available() <= 0) return -1; // nothing right now
  char sample = 0;
  while(Serial4.available() > 0){
    sample = Serial4.read();
  }
  return classifyCamByte(sample);
}
int readSerial2(){ //right
  if(Serial3.available() <= 0) return -1; // nothing right now
  char sample = 0;
  while(Serial3.available() > 0){
    sample = Serial3.read();
  }
  return classifyCamByte(sample);
}
bool detectCam1(){ // left camera serial4
   // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int n = 0;
  int maxCount = 0;
  char res = 0;
  timer myTimer;
  while(n<5){
    if(myTimer.getTime() > 4*1000000) return false;
    
    int value = readSerial1();
    
    if(value >= 0){   // only H/S/U
        samples[n] = value;
        Serial.println(value);
        n++;
    }
  }
  for(int i =0; i<5;i++){
    Serial.println(samples[i]);
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  for(int i = 0; i<5;i++){
    digitalWrite(LEDPIN,HIGH);
    delay(500);
    digitalWrite(LEDPIN,LOW);
    delay(500);
  }
  char msg[17]; snprintf(msg, sizeof(msg), "victim: %c", res);
  lcdPrint(msg);
  
  disp.dispenseLeft(res);
  return true;
}
bool detectCam2(){
  // read buffer
  // if there is content, take 5 samples and take the most common letter.
  int samples[5];
  int n = 0;
  int maxCount = 0;
  char res = 0;
  timer myTimer;
  while(n<5){
    if(myTimer.getTime() > 4*1000000) return false;
    
    int value = readSerial2();

    if(value >= 0){   // only H/S/U
        samples[n] = value;
        Serial.println(value);
        n++;
    }
  }
  for(int i =0; i<5;i++){
    Serial.println(samples[i]);
  }
  
  for(int i=0;i<6;i++){
    int count = 0;
    for(int j = 0;j<5;j++){
      if(samples[j] == i) count++;
    }
    if(count > maxCount){
      maxCount = count;
      res = classes[i];
    }
  }
  for(int i = 0; i<5;i++){
    digitalWrite(LEDPIN,HIGH);
    delay(500);
    digitalWrite(LEDPIN,LOW);
    delay(500);
  }
  char msg[17]; snprintf(msg, sizeof(msg), "victim: %c", res);
  lcdPrint(msg);
  
  disp.dispenseRight(res);
  return true;
}

// Service a camera victim that the RTOS thread flagged via victimPending.
// The caller (fwd/absoluteturn) has already paused PID + timer. This stops drivetrain, identifies victim, dispenses rescue kit, and labels the correct tile using encoder position. 
// serviceCameraVictim() is outside to prevent I2C conflict with centered.x
void serviceCameraVictim(){
  
  if(victimSide == 1){            // left camera (Serial3)
    if(detectWall(3) == 0){       // RCJ victims are wall-mounted
      Serial.println("victim at left");
      clearSerialBuffer1();
      if(detectCam1()==true) markVictimAtEncoderPosition(TILE_MM);
      victimtoggle = true;
    }
  }
  else if(victimSide == 2){       // right camera (Serial2)
    if(detectWall(1) == 0){
      Serial.println("victim at right");
      clearSerialBuffer2();
      if(detectCam2()==true) markVictimAtEncoderPosition(TILE_MM);
      victimtoggle = true;
    }
  }
  isVictim = true;       // at most one victim serviced per move
  victimPending = false; // re-enable the camera thread
}
