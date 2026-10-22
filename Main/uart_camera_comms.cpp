// camera communication code
// clear serial buffers.

#include "Globals.h"
#include "dispenser.h"

extern dispenser disp;
extern char classes[6];
extern int LEDPIN;
extern volatile int victimSide;
extern volatile int victimCode;

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
// CHANGED 2026-10-21 (Vision-branch): the cameras confirm a victim themselves before sending
// H/S/U (same letter / ring sum in 3 of their last 4 frames, every shape and ring check passed),
// so the 5-byte vote is gone: a letter is signalled and dispensed at once.
// "c" = a target passed every check in one frame but isn't confirmed yet (the camera's "stop and
// look"). The robot is stopped by then, so wait up to CAM_CONFIRM_US for the camera's letter.
const unsigned long CAM_CONFIRM_US = 1500000; // 4 camera frames at ~4-5.6 fps, plus margin

// Victim (0/1/2 = H/S/U) from the byte that woke the camera thread, or the camera's next letter
// within CAM_CONFIRM_US; -1 = no letter came.
int waitForLetter(int (*readCam)(), int code){
  timer myTimer;
  while(code < 0){
    if(myTimer.getTime() > CAM_CONFIRM_US) return -1;
    code = readCam();
  }
  return code;
}

void signalVictim(char res){
  for(int i = 0; i<5;i++){
    digitalWrite(LEDPIN,HIGH);
    delay(500);
    digitalWrite(LEDPIN,LOW);
    delay(500);
  }
  char msg[17]; snprintf(msg, sizeof(msg), "victim: %c", res);
  lcdPrint(msg);
}

bool detectCam1(int code){ // left camera serial4
  code = waitForLetter(readSerial1, code);
  if(code < 0) return false;
  char res = classes[code];
  Serial.print("left camera victim "); Serial.println(res);
  signalVictim(res);
  disp.dispenseLeft(res);
  return true;
}
bool detectCam2(int code){ // right camera serial3
  code = waitForLetter(readSerial2, code);
  if(code < 0) return false;
  char res = classes[code];
  Serial.print("right camera victim "); Serial.println(res);
  signalVictim(res);
  disp.dispenseRight(res);
  return true;
}

// Service a camera victim that the RTOS thread flagged via victimPending.
// The caller (fwd/absoluteturn) has already paused PID + timer. This stops drivetrain, identifies victim, dispenses rescue kit, and labels the correct tile using encoder position. 
// serviceCameraVictim() is outside to prevent I2C conflict with centered.x
void serviceCameraVictim(){
  
  // CHANGED 2026-10-21: the byte that woke the thread (victimCode) is acted on, not thrown away,
  // and the buffer is cleared afterwards: the letters the camera kept sending while the robot
  // blinked and dispensed are not a new victim.
  if(victimSide == 1){            // left camera (Serial3)
    if(detectWall(3) == 0){       // RCJ victims are wall-mounted
      Serial.println("victim at left");
      if(detectCam1(victimCode)==true) markVictimAtEncoderPosition(TILE_MM);
      clearSerialBuffer1();
      victimtoggle = true;
    }
  }
  else if(victimSide == 2){       // right camera (Serial2)
    if(detectWall(1) == 0){
      Serial.println("victim at right");
      if(detectCam2(victimCode)==true) markVictimAtEncoderPosition(TILE_MM);
      clearSerialBuffer2();
      victimtoggle = true;
    }
  }
  isVictim = true;       // at most one victim serviced per move
  victimPending = false; // re-enable the camera thread
}
