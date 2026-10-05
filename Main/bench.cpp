// Bench mode: measures the things the physics simulator can only guess (tools/sim/physics/README.md,
// "Where the numbers come from"), so the simulator can be set to match this robot.
//
// To use it: set BENCH_MODE to 1 in main.cpp and upload. No LCD or USB cable needed while it runs:
//   - LED blinking slowly: waiting. Place the robot for the next test (list below), then flip the pause
//     switch on and off. Keep hands off; the test starts 1 s later.
//   - LED on: a test is running.
//   - LED blinking fast: all done. Plug in USB, open the Serial monitor (115200) and flip the switch
//     on and off: every result is printed again. Copy the [BENCH] lines.
// With a cable plugged in the whole time, the instructions and results also appear as it goes.
// Set BENCH_MODE back to 0 for a competition run (with 0 none of this code runs).
//
// The tests, in order (what each prints and what it is compared with: the simulator's values at 11.8 V):
//   0 (no switch flip) heading at power-on: the code expects about 0 whichever way the robot faced when
//     switched on. If it isn't, the BNO055 is giving heading from magnetic north (NDOF mode).
//   1 middle of a tile with walls on all 4 sides, square to them: average of 50 readings per distance
//     sensor vs what the CAD says it should read there   -> tofOffsetSigma, SENSOR_OFFSET_MM
//   2 middle of a tile, room to turn: lowest PWM that turns the robot on the spot (sim 41) -> wheelMu, TURN_MIN_PWM
//   3 same: degrees turned by turnright(150) in 1 s (sim 75) -> skidFactor
//   4 open floor, 30 cm free ahead: lowest PWM that moves it straight (sim 13-17) -> frictionFracAt12V
//   5 open floor, 60 cm free ahead, mark its centre: fw(150) for 2 s with no steering: encoder distance and
//     heading change; measure the real distance and how far it went sideways -> tractionMean, motorGainSigma
//   6 anywhere, untouched for 2 minutes: heading change (sim 0.5 deg/min) -> gyroDriftSigmaDegPerMin
//   7-10 colour sensor over the middle of a white, blue, silver, then black tile: what read_color() says
//     and the raw values, vs the thresholds in Globals.h (the simulator's colour values are assumed)
#include "Globals.h"
#include "Movement.h"
#include "Color.h"

void lcdPrint(const char* msg); // uart_camera_comms.cpp
extern int LEDPIN;              // main.cpp

static const int BENCH_SWITCH_PIN = 22; // the pause (logic) switch, logicswitch in main.cpp

// ---- results: printed as they come and kept, so they can be printed again at the end ----
static const int MAX_RESULTS = 48, LINE_LEN = 100;
static char results[MAX_RESULTS][LINE_LEN];
static int resultCount = 0;
static char line[LINE_LEN];
static int lineLen = 0;

static void add(const char *s){
  while(*s && lineLen < LINE_LEN - 1) line[lineLen++] = *s++;
  line[lineLen] = 0;
}
static void addInt(long v){
  char b[16];
  snprintf(b, sizeof(b), "%ld", v);
  add(b);
}
static void addNum(double v, int decimals){ // fixed decimals without printf("%f")
  long scale = decimals >= 2 ? 100 : decimals == 1 ? 10 : 1;
  long x = lround(v * scale);
  char b[24];
  if(scale == 1) snprintf(b, sizeof(b), "%ld", x);
  else snprintf(b, sizeof(b), "%s%ld.%0*ld", x < 0 ? "-" : "", labs(x) / scale, decimals >= 2 ? 2 : 1, labs(x) % scale);
  add(b);
}
static void endLine(){
  Serial.print("[BENCH] ");
  Serial.println(line);
  if(resultCount < MAX_RESULTS){
    memcpy(results[resultCount], line, LINE_LEN);
    resultCount++;
  }
  lineLen = 0;
  line[0] = 0;
}

static void led(bool on){ digitalWrite(LEDPIN, on ? HIGH : LOW); }
static void blink(unsigned long periodMs){ led((millis() / (periodMs / 2)) % 2); }

static double wrap180(double a){
  while(a > 180.0) a -= 360.0;
  while(a < -180.0) a += 360.0;
  return a;
}

static int encoderAvg(){
  return (drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3;
}

// the "next" button: the pause switch flipped on and then off again (LED blinks slowly meanwhile)
static void waitForSwitch(const char *lcdText, const char *howToPlace){
  drivetrain.fullstop();
  Serial.print("[BENCH] next: ");
  Serial.println(howToPlace);
  Serial.println("[BENCH] flip the pause switch on and off to start");
  lcdPrint(lcdText);
  while(digitalRead(BENCH_SWITCH_PIN) == LOW){ blink(1000); delay(20); }
  while(digitalRead(BENCH_SWITCH_PIN) == HIGH){ blink(1000); delay(20); }
  led(true);   // on while the test runs
  delay(1000); // hands off the robot
}

static void benchSensors(){
  waitForSwitch("1 sensors", "1) robot in the middle of a tile with walls on all 4 sides, square to them");
  // CAD (tools/sim/cad/robot_geometry.json): wall faces 140 mm from the middle of a tile (280 mm path);
  // front sensors 98.4 mm ahead of the centre, side sensors 90.8 mm out, back sensor 102.8 mm behind
  const char *names[8] = {"", "front-right", "right-front", "right-back", "back", "left-back", "left-front", "front-left"};
  const double expected[8] = {0, 41.6, 49.2, 49.2, 37.2, 49.2, 49.2, 41.6};
  for(int s = 1; s <= 7; s++){
    long sum = 0;
    int n = 0;
    for(int i = 0; i < 50; i++){
      int v = measure(s);
      if(v > 0 && v < 2000){ sum += v; n++; }
    }
    add("1 sensor "); addInt(s); add(" "); add(names[s]);
    if(n == 0){ add(": no valid reading"); endLine(); continue; }
    double mean = (double)sum / n;
    add(": "); addNum(mean, 1); add(" mm, CAD says "); addNum(expected[s], 1);
    add(", off by "); addNum(mean - expected[s], 1);
    endLine();
  }
  Serial.println("[BENCH] (placing it in the exact middle is hard, +-5 mm; the left+right and front+back sums don't depend on it)");
}

static void benchTurnPwm(){
  waitForSwitch("2 turn PWM", "2) robot in the middle of a tile, room to turn");
  for(int pwm = 20; pwm <= 150; pwm += 5){
    double h0 = myGyro.heading();
    drivetrain.turnright(pwm);
    delay(1000);
    drivetrain.fullstop();
    delay(300);
    double moved = wrap180(myGyro.heading() - h0);
    Serial.print("[BENCH] turnright(");
    Serial.print(pwm);
    Serial.print("): ");
    Serial.print(moved, 1);
    Serial.println(" deg in 1 s");
    if(fabs(moved) > 5.0){
      add("2 lowest PWM that turns it on the spot: "); addInt(pwm); add(" (simulator 41; TURN_MIN_PWM is 20)");
      endLine();
      return;
    }
  }
  add("2 didn't turn even at 150");
  endLine();
}

static void benchTurnRate(){
  waitForSwitch("3 turn rate", "3) robot in the middle of a tile, room to turn");
  double h0 = myGyro.heading(), total = 0, last = h0;
  drivetrain.turnright(150);
  unsigned long start = millis();
  while(millis() - start < 1000){ // add up small steps so a turn past 180 deg is counted right
    double h = myGyro.heading();
    total += wrap180(h - last);
    last = h;
    delay(10);
  }
  drivetrain.fullstop();
  add("3 turnright(150) for 1 s: "); addNum(total, 1); add(" deg (simulator 75)");
  endLine();
}

static void benchStraightPwm(){
  waitForSwitch("4 drive PWM", "4) robot on open floor with 30 cm free ahead");
  for(int pwm = 5; pwm <= 60; pwm += 2){
    drivetrain.reset_encoderCount(true, true, true);
    drivetrain.fw(pwm);
    delay(1000);
    drivetrain.fullstop();
    delay(300);
    int counts = encoderAvg();
    if(counts > 10){
      add("4 lowest PWM that drives it straight from standstill: "); addInt(pwm); add(" (simulator 17)");
      endLine();
      return;
    }
  }
  add("4 didn't move even at 60");
  endLine();
}

static void benchStraightLine(){
  waitForSwitch("5 straight", "5) robot on open floor with 60 cm free ahead; mark where its centre is");
  double h0 = myGyro.heading();
  drivetrain.reset_encoderCount(true, true, true);
  drivetrain.fw(150);
  delay(2000);
  drivetrain.fullstop();
  delay(500);
  int a = drivetrain.encoderCountA, b = drivetrain.encoderCountB, d = drivetrain.encoderCountD;
  double mm = (a + b + d) / 3.0 / (wheel_cpr * gear_ratio) * wheel_diameter * M_PI;
  add("5 fw(150) 2 s: encoders A/B/D "); addInt(a); add("/"); addInt(b); add("/"); addInt(d);
  add(", the code makes that "); addNum(mm, 0); add(" mm; heading changed "); addNum(wrap180(myGyro.heading() - h0), 1); add(" deg");
  endLine();
  add("5 (measure with a ruler: distance driven, and how far it ended up to the side; simulator about 300 mm, encoders 4% over)");
  endLine();
}

static void benchGyroDrift(){
  waitForSwitch("6 gyro drift", "6) robot standing still anywhere; don't touch it for 2 minutes");
  double h0 = myGyro.heading();
  for(int s = 30; s <= 120; s += 30){
    delay(30000);
    add("6 heading change after "); addInt(s); add(" s standing still: "); addNum(wrap180(myGyro.heading() - h0), 2); add(" deg");
    endLine();
  }
  Serial.println("[BENCH] per minute = the 120 s value / 2 (simulator assumes 0.5 deg/min)");
}

static void benchFloorColours(){
  const char *tiles[4] = {"white", "blue", "silver", "black"};
  const char *classes[5] = {"black", "white", "blue", "red", "silver"}; // read_color() -1..3
  for(int t = 0; t < 4; t++){
    char lcdText[17];
    snprintf(lcdText, sizeof(lcdText), "%d %s", 7 + t, tiles[t]);
    char howTo[80];
    snprintf(howTo, sizeof(howTo), "%d) colour sensor over the middle of a %s tile", 7 + t, tiles[t]);
    waitForSwitch(lcdText, howTo);
    int count[5] = {0, 0, 0, 0, 0};
    long sr = 0, sg = 0, sb = 0, sc = 0;
    for(int i = 0; i < 10; i++){
      uint16_t r, g, b, c;
      i2cMutex.lock();
      myMux.setPort(TCS_PORT);
      tcs.getRawData(&r, &g, &b, &c);
      i2cMutex.unlock();
      sr += r; sg += g; sb += b; sc += c;
      int k = read_color(); // also prints r g b c and the clear ratio
      if(k >= -1 && k <= 3) count[k + 1]++;
      delay(60); // > one 50 ms measurement
    }
    add(""); addInt(7 + t); add(" "); add(tiles[t]); add(" tile: r "); addInt(sr / 10); add(" g "); addInt(sg / 10);
    add(" b "); addInt(sb / 10); add(" c "); addInt(sc / 10); add(" ratio "); addNum(clear > 0 ? (sc / 10.0) / clear : 0, 2);
    add("; read as");
    for(int k = 0; k < 5; k++){
      if(count[k] == 0) continue;
      add(" "); add(classes[k]); add(" x"); addInt(count[k]);
    }
    endLine();
  }
  add("thresholds: black if ratio < 0.1, silver if r > 800 (checked before white), white if ratio > 0.85");
  endLine();
}

void runBench(){
  pinMode(LEDPIN, OUTPUT);
  Serial.println("[BENCH] bench mode: measurements for the simulator (BENCH_MODE in main.cpp)");
  add("0 heading at power-on: "); addNum(myGyro.heading(), 1); add(" deg (should be about 0 whichever way the robot faced)");
  endLine();
  benchSensors();
  benchTurnPwm();
  benchTurnRate();
  benchStraightPwm();
  benchStraightLine();
  benchGyroDrift();
  benchFloorColours();
  drivetrain.fullstop();
  Serial.println("[BENCH] done. Flip the pause switch on and off to print every result again.");
  lcdPrint("bench done");
  while(true){ // fast blink; each switch flip prints all results again (plug in USB first)
    while(digitalRead(BENCH_SWITCH_PIN) == LOW){ blink(250); delay(20); }
    while(digitalRead(BENCH_SWITCH_PIN) == HIGH){ blink(250); delay(20); }
    Serial.println("[BENCH] ---- all results ----");
    for(int i = 0; i < resultCount; i++){
      Serial.print("[BENCH] ");
      Serial.println(results[i]);
    }
    Serial.println("[BENCH] ---- end ----");
  }
}
