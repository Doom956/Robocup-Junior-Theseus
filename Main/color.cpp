#include "Color.h"
#include "Globals.h"

void init_color(){
  //tcs.setInterrupt(true);
  myMux.setPort(TCS_PORT);
  //digitalWrite(POWERPIN,HIGH);
  //tcs.enable();
  if (tcs.begin()) {
    Serial.println("Found tcs34725 sensor");
    
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1); // halt!
  }
  myMux.setPort(TCS_PORT);
  tcs.setInterrupt(true);  // turn on LED
  uint16_t r = 0, g = 0, b = 0, c = 0;

  // The TCS34725 has no valid data until one integration cycle (~24ms) after
  // enable; the first read(s) return all zeros. In the full sketch the I2C init
  // that runs before this (mux + 7 distance sensors) shifts the timing so a
  // single read sometimes lands too early -> c=0 -> clear=0 -> every later
  // (float)c/clear is inf. Retry until we get a real clear value.
  for (int i = 0; i < 10 && c == 0; i++) {
    delay(50);                 // > one 24ms integration period
    myMux.setPort(TCS_PORT);
    tcs.getRawData(&r, &g, &b, &c);
  }
  // The first non-zero reading is still too dark (2026-10-17: 66-145 on white, against ~2700 once running; then
  // black and blue tiles never read as black or blue, ratio ~28 on white). Let the sensor settle and average
  // 5 readings of the start tile.
  delay(200);
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 5; i++) {
    delay(60);                 // > one integration period, so each read is a new measurement
    myMux.setPort(TCS_PORT);
    tcs.getRawData(&r, &g, &b, &c);
    if (c > 0) { sum += c; n++; }
  }
  if (n > 0) c = sum / n;
  clear = (c > 0) ? c : 1;     // never store 0 -> never divide by zero
  Serial.println("clear value");
  Serial.println(clear);

}
int read_color(){
  // [DIAG-COLOR] snapshot the global `clear` divisor the instant this call starts,
  // before touching I2C at all. clear is written exactly once (init_color(), pre-thread)
  // and never reassigned anywhere else, so if this ever prints 0 here, its backing memory
  // has been clobbered by something other than this function -- not a bad sensor read.

  i2cMutex.lock();
  myMux.setPort(TCS_PORT);
  tcs.setInterrupt(true);  // turn on LED
  uint16_t r, g, b, c;

  tcs.getRawData(&r, &g, &b, &c);
  i2cMutex.unlock();
  //diagnosis
  
  Serial.print(r);
  Serial.print(" ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.print("c=");
  Serial.print(c);
  Serial.print(" ");
  Serial.print("ratio=");
  Serial.println((float)c/clear);
  
  
  //Serial.println((float)c/clear);
  if(c == 0) return 0;
  if((float)c/clear<BLACK_THRESHOLD){


    return -1; // black
  }
  
  // silver reflects more absolute light. Only classify here: the caller knows
  // which tile the sensor is over (fwd() past half a tile, or finishTileMove()
  // once the robot has stopped on the new tile) and records the checkpoint there.
  if(r>SILVER_THRESHOLD){
    return 3;
  }
  
  if((float)c/clear>WHITE_THRESHOLD) return 0;


  if(b>g+10&&b>r+10) return 1; //blue

  if(r>g+10&&r>b+10) return 2;

  return 0;

}

// White reference for read_color()'s black / white ratio, from the start tile when the run starts (first time out of
// PAUSE). The reading in init_color() at power-on is far too dark on this robot (2026-10-17: 66-145, also after a
// settling delay, against ~3100 on the same white tile once the run is going), so black and blue were never seen.
// Same calls as read_color(), so it is the light level the run will see.
void takeWhiteReference(){
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 5; i++) {
    uint16_t r, g, b, c;
    i2cMutex.lock();
    myMux.setPort(TCS_PORT);
    tcs.setInterrupt(true);
    tcs.getRawData(&r, &g, &b, &c);
    i2cMutex.unlock();
    if (c > 0) { sum += c; n++; }
    delay(60);
  }
  Serial.print("white reference: power-on ");
  Serial.print(clear, 0);
  if (n == 0) { Serial.println(", no reading now, kept"); return; }
  clear = (float)sum / n;
  Serial.print(", start tile now ");
  Serial.println(clear, 0);
}
