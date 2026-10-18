#include <algorithm>
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
//   2 middle of a tile, room to turn: lowest PWM that turns the robot on the spot (sim 25) -> wheelMu, TURN_MIN_PWM
//   3 same: degrees turned by turnright(150) in 1 s (sim 95) -> skidFactor
//   4 open floor, 50 cm free ahead: lowest PWM that drives all three encoder wheels forward (sim 19)
//     -> frictionFracAt12V. Tests 2-4 also print the encoder counts A/B/D, so a weak or backwards wheel shows.
//   5 open floor, 60 cm free ahead, mark its centre: fw(150) for 2 s with no steering: encoder distance and
//     heading change; measure the real distance and how far it went sideways -> tractionMean, motorGainSigma
//   6 anywhere, untouched for 2 minutes: heading change (sim 0.03 deg/min) -> gyroDriftSigmaDegPerMin
//   7-10 colour sensor over the middle of a white, blue, silver, then black tile: what read_color() says
//     and the raw values, vs the thresholds in Globals.h (the simulator's colour values are assumed)
//   11 (one switch flip per sensor, 1 to 7) a flat wall exactly 100 mm in front of that sensor, measured from the
//     front of its board (hold a 100 mm spacer block between the sensor and the wall, then take it away, or
//     stand the robot on a line 100 mm from the wall), square to it: raw average of 100 readings, and the
//     offset it needs. The last line is SENSOR_OFFSET_MM (Main/Distance.cpp) ready to paste, with the four
//     side sensors left at 0 on purpose (correcting them loses points: see benchSensorOffsets()).
//     -> SENSOR_OFFSET_MM, tofOffsetMeanMm / tofOffsetSigma
#include "Globals.h"
#include "Movement.h"
#include "Color.h"

void lcdPrint(const char* msg); // uart_camera_comms.cpp
extern int LEDPIN;              // main.cpp
extern bool obstacleAhead;      // movement.cpp: fwd() saw an obstacle (or was blocked) and backed up

static const int BENCH_SWITCH_PIN = 22; // the pause (logic) switch, logicswitch in main.cpp

// ---- results: printed as they come and kept, so they can be printed again at the end ----
static const int MAX_RESULTS = 48, LINE_LEN = 160;
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

// encoder counts of wheels A/B/D since the last reset, as "a/b/d" (one counting backwards, or one stuck
// at 0, shows up here)
static void addEncoders(){
  addInt(drivetrain.encoderCountA); add("/"); addInt(drivetrain.encoderCountB); add("/"); addInt(drivetrain.encoderCountD);
}

static void benchTurnPwm(){
  waitForSwitch("2 turn PWM", "2) robot in the middle of a tile, room to turn");
  double firstHeading = myGyro.heading();
  bool gyroChanged = false;
  for(int pwm = 20; pwm <= 150; pwm += 5){
    double h0 = myGyro.heading();
    drivetrain.reset_encoderCount(true, true, true);
    drivetrain.turnright(pwm);
    delay(1000);
    drivetrain.fullstop();
    delay(300);
    double moved = wrap180(myGyro.heading() - h0);
    if(fabs(myGyro.heading() - firstHeading) > 0.01) gyroChanged = true;
    Serial.print("[BENCH] turnright(");
    Serial.print(pwm);
    Serial.print("): ");
    Serial.print(moved, 1);
    Serial.print(" deg in 1 s, encoders A/B/D ");
    Serial.print(drivetrain.encoderCountA); Serial.print("/");
    Serial.print(drivetrain.encoderCountB); Serial.print("/");
    Serial.println(drivetrain.encoderCountD);
    if(fabs(moved) > 5.0){
      add("2 lowest PWM that turns it on the spot: "); addInt(pwm); add(" (robot 25-30 on 2026-10-06, sim 25; TURN_MIN_PWM is 20); encoders ");
      addEncoders();
      endLine();
      return;
    }
  }
  add("2 didn't turn 5 deg even at 150; encoders at 150 "); addEncoders();
  add(gyroChanged ? "" : "; GYRO NEVER CHANGED (not reading?)");
  endLine();
}

static void benchTurnRate(){
  waitForSwitch("3 turn rate", "3) robot in the middle of a tile, room to turn");
  double h0 = myGyro.heading(), total = 0, last = h0;
  drivetrain.reset_encoderCount(true, true, true);
  drivetrain.turnright(150);
  unsigned long start = millis();
  while(millis() - start < 1000){ // add up small steps so a turn past 180 deg is counted right
    double h = myGyro.heading();
    total += wrap180(h - last);
    last = h;
    delay(10);
  }
  drivetrain.fullstop();
  add("3 turnright(150) for 1 s: "); addNum(total, 1); add(" deg (robot 92-97, sim 95); encoders "); addEncoders();
  endLine();
}

static void benchStraightPwm(){
  waitForSwitch("4 drive PWM", "4) robot on open floor with 50 cm free ahead");
  // up to 60 in small steps, then on to 150 in case it needs more; stops at the first PWM that turns all
  // three wheels forward 10 counts (about 3 mm), so it only creeps. (One wheel alone isn't enough: a motor
  // humming without turning made an encoder count noise, some of it backwards.)
  for(int pwm = 5; pwm <= 150; pwm += (pwm < 60 ? 2 : 10)){
    drivetrain.reset_encoderCount(true, true, true);
    drivetrain.fw(pwm);
    delay(1000);
    drivetrain.fullstop();
    delay(300);
    Serial.print("[BENCH] fw(");
    Serial.print(pwm);
    Serial.print("): encoders A/B/D ");
    Serial.print(drivetrain.encoderCountA); Serial.print("/");
    Serial.print(drivetrain.encoderCountB); Serial.print("/");
    Serial.println(drivetrain.encoderCountD);
    if(drivetrain.encoderCountA > 10 && drivetrain.encoderCountB > 10 && drivetrain.encoderCountD > 10){
      add("4 lowest PWM that drives it straight from standstill: "); addInt(pwm); add(" (robot 49-51, set by wheel A; sim 19); encoders ");
      addEncoders();
      endLine();
      return;
    }
  }
  add("4 the three wheels never all counted forward, even at fw(150): encoders "); addEncoders();
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
  Serial.println("[BENCH] per minute = the 120 s value / 2 (simulator 0.03 deg/min, from this test)");
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

static void benchSensorOffsets(){
  const char *names[8] = {"", "front-right", "right-front", "right-back", "back", "left-back", "left-front", "front-left"};
  int offset[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for(int s = 1; s <= 7; s++){
    char lcdText[17];
    snprintf(lcdText, sizeof(lcdText), "11 sensor %d", s);
    char howTo[110];
    snprintf(howTo, sizeof(howTo), "11) flat wall exactly 100 mm in front of sensor %d (%s), square to it", s, names[s]);
    waitForSwitch(lcdText, howTo);
    bool reads = false; // calibrateSensor() returns -1 for "no reading", which is also a possible offset
    for(int i = 0; i < 5 && !reads; i++) reads = measure(s) != -1;
    int off = reads ? calibrateSensor(s, 100) : 0; // averages 100 raw readings (SENSOR_OFFSET_MM not applied)
    add("11 sensor "); addInt(s); add(" "); add(names[s]);
    if(!reads) add(": no valid reading (offset left at 0)");
    else { add(" at 100 mm reads "); addInt(100 + off); add(", offset "); addInt(off); }
    endLine();
    offset[s] = off;
  }
  // Side sensors (2, 3, 5, 6) stay 0: the one-wall steering target (TARGET_SIDE_GAP_MM) was tuned with side
  // sensors that read ~10 mm long, as this robot's do. In the simulator, side readings 4-20 mm long all score
  // the same, but correcting them to exact (0) loses points: robot-test -10.0 +-6.1, main -4.5 +-7.2 on 200 comp
  // fields, more time lost (it holds further from a lone wall and jams corners turning). Front and back: no change.
  add("11 paste into Main/Distance.cpp (side sensors left at 0 on purpose, see bench.cpp): const int SENSOR_OFFSET_MM[8] = {0");
  for(int s = 1; s <= 7; s++){ add(", "); addInt((s == 2 || s == 3 || s == 5 || s == 6) ? 0 : offset[s]); }
  add("};");
  endLine();
}

// ---- move tests (BENCH_MODE 2): the firmware's own absoluteturn() and fwd() on an open floor, no maze ----
// Same LED and switch flips as bench mode. Switch the robot on standing on the floor it will drive on (the
// colour sensor's white reference is read at power-on; a darker floor could read as a black tile).
// Each test first takes whatever way the robot points as square (heading snapped to the nearest 90 deg), so
// place it pointing along the line it should drive.
//   M0 prints all distance sensors for 10 s (what does the back sensor see with nothing behind?)
//   M1 open floor, room to turn: 4 right 90s, 4 left 90s, two 180s: time and how far off each ends
//   M2 same place: small turns (5-15 deg) there and back: does each one move the robot?
//   M3 open floor, 1.2 m free ahead, a wall or box behind: 3 moves of one tile; the back sensor measures each (300 mm)
//   M4 a straight wall on one side, robot 4-5 cm from it: 4 moves; it should settle at the one-wall gap, no weaving
//   M5 a wall 34 cm ahead (like the far end of the next tile): one move + centreAlong(), no false "no progress"
//   M6 as M5 plus a heavy box (8-10 cm wide) in the middle, 10 cm ahead: expect "[FWD] no progress" and a back-up

static double norm360(double a){
  while(a >= 360.0) a -= 360.0;
  while(a < 0.0) a += 360.0;
  return a;
}

static double squareHere(){ // returns the snapped heading the test works from
  myGyro.resyncToNearestCardinal(45.0);
  return norm360(90.0 * round(myGyro.heading() / 90.0));
}

// one absoluteturn() to target: how long, how far off it stopped, and how far off once it has coasted
static void timedTurn(const char *label, double target){
  double h0 = myGyro.heading();
  unsigned long t0 = millis();
  absoluteturn(target);
  unsigned long ms = millis() - t0;
  double errEnd = wrap180(myGyro.heading() - target);
  delay(500);
  double h1 = myGyro.heading();
  add(label); add(": "); addNum(ms / 1000.0, 2); add(" s, stopped "); addNum(errEnd, 1); add(" deg off, after 0.5 s ");
  addNum(wrap180(h1 - target), 1); add(" (turned "); addNum(wrap180(h1 - h0), 1); add(")");
  endLine();
}

static void addSides(){ // side readings: right front/back (2/3), left front/back (6/5)
  add("right "); addInt(measure(2)); add("/"); addInt(measure(3)); add(", left "); addInt(measure(6)); add("/"); addInt(measure(5));
}

static void addFrontAndResult(){
  add("front "); addInt(measure(7)); add("/"); addInt(measure(1)); add(" (CAD 42 in the middle of a tile)");
  if(obstacleAhead) add(", treated as an obstacle (edge would be blocked)");
  else if(fwdShort) add(", short move (backed up)");
}

static void moveTurns(){
  waitForSwitch("M1 turns", "M1) open floor, room to turn: 4 right 90s, 4 left 90s, then 180 and back");
  add("M1 (robot-test: 90 in about 1-1.5 s, 180 in 2-2.5 s, within 2-3 deg; exactly 2.00 / 4.00 s = waited out its time limit)");
  endLine();
  double base = squareHere();
  char label[24];
  for(int i = 1; i <= 4; i++){ snprintf(label, sizeof(label), "M1 right 90 #%d", i); timedTurn(label, norm360(base + 90.0 * i)); }
  for(int i = 1; i <= 4; i++){ snprintf(label, sizeof(label), "M1 left 90 #%d", i); timedTurn(label, norm360(base - 90.0 * i)); }
  timedTurn("M1 180 #1", norm360(base + 180.0));
  timedTurn("M1 180 #2", base);
}

static void moveSmallTurns(){
  waitForSwitch("M2 small turns", "M2) same place: small turns of 5-15 deg, each followed by a turn back");
  add("M2 (each should turn the robot and stop within 2 deg; 'turned' about 0 = it didn't move)");
  endLine();
  double base = squareHere();
  const double steps[8] = {5, 0, -5, 0, 10, 0, -15, 0};
  char label[24];
  for(int i = 0; i < 8; i++){
    snprintf(label, sizeof(label), "M2 to %+d deg", (int)steps[i]);
    timedTurn(label, norm360(base + steps[i]));
  }
}

static int backAvg(){ // back sensor, average of 5 valid readings (-1 if none)
  long sum = 0;
  int n = 0;
  for(int i = 0; i < 10 && n < 5; i++){
    int v = measure(4);
    if(v > 0 && v < 1500){ sum += v; n++; }
  }
  return n > 0 ? (int)(sum / n) : -1;
}

static void moveOpenFloor(){
  waitForSwitch("M3 3 tiles", "M3) open floor, 1.2 m free ahead, 30 cm free each side, a wall or big box right behind the robot (the back sensor measures each move). 3 moves");
  double base = squareHere();
  int b0 = backAvg(), prev = b0;
  add("M3 start: back sensor "); addInt(b0); add(" mm");
  endLine();
  for(int i = 1; i <= 3; i++){
    unsigned long t0 = millis();
    fwd(TILE_MM);
    unsigned long ms = millis() - t0;
    delay(300);
    int b = backAvg();
    add("M3 move "); addInt(i); add(": "); addNum(ms / 1000.0, 2); add(" s, back sensor "); addInt(b);
    if(b > 0 && prev > 0){ add(", moved "); addInt(b - prev); add(" mm (should be 300)"); }
    add(", heading "); addNum(wrap180(myGyro.heading() - base), 1); add(" deg off");
    if(fwdShort) add(", short move (backed up)");
    endLine();
    prev = b;
    delay(2000);
  }
  add("M3 each move should be about 300 mm. Short: raise the 55 in fwd() to 60-65; past: lower it to 50");
  endLine();
}

static void moveAlongWall(){
  waitForSwitch("M4 along wall", "M4) straight wall on one side, 1.5 m long; robot parallel to it, its side 4-5 cm from the wall. 4 moves");
  double base = squareHere();
  add("M4 start: "); addSides(); add(" (the one-wall target is 80 on the wall side)");
  endLine();
  for(int i = 1; i <= 4; i++){
    fwd(TILE_MM);
    add("M4 move "); addInt(i); add(": "); addSides(); add(", heading "); addNum(wrap180(myGyro.heading() - base), 1); add(" deg off");
    endLine();
    delay(1000);
  }
  add("M4 the wall side should go toward 80 and stay there, no visible swing left and right ([CENTER] err shrinking)");
  endLine();
}

static void moveSensorWatch(){
  waitForSwitch("M0 sensors", "M0) all 7 distance sensors, twice a second for 10 s: open floor, nothing within 1 m behind it");
  int bMin = 9999, bMax = -1;
  unsigned long t0 = millis();
  while(millis() - t0 < 10000){
    int v[8];
    for(int s = 1; s <= 7; s++) v[s] = measure(s);
    if(v[4] > 0 && v[4] < 8000){ bMin = min(bMin, v[4]); bMax = max(bMax, v[4]); }
    Serial.print("[BENCH] M0 "); Serial.print((millis() - t0) / 1000.0, 1);
    Serial.print(" s: front "); Serial.print(v[7]); Serial.print("/"); Serial.print(v[1]);
    Serial.print(" right "); Serial.print(v[2]); Serial.print("/"); Serial.print(v[3]);
    Serial.print(" back "); Serial.print(v[4]);
    Serial.print(" left "); Serial.print(v[6]); Serial.print("/"); Serial.println(v[5]);
    delay(500);
  }
  add("M0 back sensor over 10 s: "); if(bMax < 0) add("never a reading under 8000"); else { addInt(bMin); add(" to "); addInt(bMax); add(" mm"); }
  add(" (nothing behind it should read 8190 or -1; a steady reading under 400 = it sees the floor or the cable)");
  endLine();
}

// Before M5/M6: the LED is on while both front sensors read 250-400 mm and agree within 30 mm (a wall at the far
// end of the next tile, both sensors seeing it, not the box) and off otherwise: move the robot until it comes on, then flip the switch
// on and off. (A blink here looked steady: each loop's sensor reads take about as long as a blink.)
static void waitForFrontWall(const char *lcdText, const char *howToPlace){
  drivetrain.fullstop();
  Serial.print("[BENCH] next: ");
  Serial.println(howToPlace);
  Serial.println("[BENCH] move the robot until the LED comes on (both front sensors 250-400 mm), then flip the pause switch on and off");
  lcdPrint(lcdText);
  unsigned long lastPrint = 0;
  for(int phase = 0; phase < 2; phase++){
    while(digitalRead(BENCH_SWITCH_PIN) == (phase == 0 ? LOW : HIGH)){
      int l = measure(7), r = measure(1);
      led(l >= 250 && l <= 400 && r >= 250 && r <= 400 && abs(l - r) <= 30);
      if(millis() - lastPrint > 1000){ lastPrint = millis(); Serial.print("[BENCH] front "); Serial.print(l); Serial.print("/"); Serial.println(r); }
      delay(20);
    }
  }
  led(true);
  delay(1000);
}

static void moveTowardWall(){
  waitForFrontWall("M5 wall ahead", "M5) a wall straight ahead, 30-40 cm from the front of the robot, nothing else in the way");
  squareHere();
  add("M5 start: front "); addInt(measure(7)); add("/"); addInt(measure(1)); add(" (about 340 = the far end of the next tile; the move should take 300 off it), back ");
  addInt(measure(4));
  endLine();
  fwd(TILE_MM);
  add("M5 after the move: "); addFrontAndResult(); add(", back "); addInt(measure(4));
  endLine();
  centreAlong(); // what SENSE_TILE does next in a run
  add("M5 after centreAlong(): "); addFrontAndResult(); add(". No 'no progress' line should appear in this test");
  endLine();
}

static void moveBlocked(){
  waitForFrontWall("M6 blocked", "M6) as M5, plus a heavy box 8-10 cm wide (taped down) in the middle, 10 cm ahead of the robot");
  squareHere();
  add("M6 start: front "); addInt(measure(7)); add("/"); addInt(measure(1)); add(" (both should still see the wall, not the box), back "); addInt(measure(4));
  endLine();
  fwd(TILE_MM);
  add("M6 after the move: "); addFrontAndResult(); add(", back "); addInt(measure(4)); add(". Expected: '[FWD] no progress' and the robot back near where it started");
  endLine();
}

// M7: wheels in the air (robot on a box, all wheels free): the motors' speed with no load, from the encoders
// (5 x 195 counts per wheel turn). The simulator assumes 170 RPM at 12 V (fitted to the move tests 2026-10-18; the
// BOM's #3493 would be 72), so free-running it expects about 158 RPM at PWM 255 and 90 at PWM 150 on an 11.8 V battery.
static void moveWheelsInAir(){
  waitForSwitch("M7 wheels up", "M7) robot lifted so no wheel touches anything (on a box or held), battery on");
  for(int pwm : {150, 255}){
    drivetrain.fw(pwm);
    delay(700);                                   // spin up
    drivetrain.reset_encoderCount(true, true, true);
    unsigned long t0 = millis();
    delay(1000);
    long a = drivetrain.encoderCountA, b = drivetrain.encoderCountB, d = drivetrain.encoderCountD;
    double s = (millis() - t0) / 1000.0;
    drivetrain.fullstop();
    delay(1000);
    add("M7 fw("); addInt(pwm); add(") wheels free: RPM A/B/D ");
    addNum(a / 975.0 / s * 60, 0); add("/"); addNum(b / 975.0 / s * 60, 0); add("/"); addNum(d / 975.0 / s * 60, 0);
    add(pwm == 255 ? " (sim about 158)" : " (sim about 90)");
    endLine();
  }
  add("M7 if these are about half the sim's, the motors are the 72 RPM ones and something else makes the robot fast; tell Claude");
  endLine();
}

// M8: an obstacle hit after a clear start (robot-test 2n): a box the front sensors can't see (it sits between their
// beams) partway into the next tile. The no-progress check must still stop the move.
static void moveBlockedLater(){
  waitForFrontWall("M8 blocked later", "M8) as M5 (wall 30-40 cm ahead), plus a heavy box 8-10 cm wide (taped down) in the middle, its front 15 cm ahead of the robot");
  squareHere();
  add("M8 start: front "); addInt(measure(7)); add("/"); addInt(measure(1)); add(" (both should still see the wall, not the box), back "); addInt(measure(4));
  endLine();
  fwd(TILE_MM);
  add("M8 after the move: "); addFrontAndResult(); add(", back "); addInt(measure(4)); add(". Expected: '[FWD] no progress' and the robot back near where it started");
  endLine();
}

static void printAllForever(const char *lcdText){
  drivetrain.fullstop();
  Serial.println("[BENCH] done. Flip the pause switch on and off to print every result again.");
  lcdPrint(lcdText);
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

// Which move tests run, in this order (0 = M0 sensor watch, 1-8 = M1-M8); change it to repeat a few.
// Without a maze (2026-10-18): wheels in the air, sensors on open floor, turns, 3 moves, blocked, blocked later.
static const int MOVE_TESTS[] = {7, 0, 1, 3, 6, 8};

void runMoveTests(){
  pinMode(LEDPIN, OUTPUT);
  Serial.println("[BENCH] move tests: absoluteturn() and fwd() on an open floor (BENCH_MODE 2 in main.cpp)");
  add("M heading at power-on: "); addNum(myGyro.heading(), 1); add(" deg (about 0; stuck at exactly 0.0 in M1 = gyro not found)");
  endLine();
  for(unsigned i = 0; i < sizeof(MOVE_TESTS) / sizeof(MOVE_TESTS[0]); i++){
    switch(MOVE_TESTS[i]){
      case 0: moveSensorWatch(); break;
      case 1: moveTurns(); break;
      case 2: moveSmallTurns(); break;
      case 3: moveOpenFloor(); break;
      case 4: moveAlongWall(); break;
      case 5: moveTowardWall(); break;
      case 6: moveBlocked(); break;
      case 7: moveWheelsInAir(); break;
      case 8: moveBlockedLater(); break;
    }
  }
  printAllForever("move tests done");
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
  benchSensorOffsets();
  printAllForever("bench done");
}


// LCD test (BENCH_MODE 3): writes two lines and a counter with the LiquidCrystal library on main.cpp's pins.
// (2026-10-17, new LCD: one row of boxes / stray characters were a loose GND, D5 bridged on the perfboard and a shaky
// contact; with those fixed the library works.)
void runLcdTest(){
  pinMode(LEDPIN, OUTPUT);
  Serial.println("[BENCH] LCD LiquidCrystal test");
  delay(500);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("HELLO LIBRARY   ");
  for(unsigned long n = 0; ; n++){
    char b[17];
    snprintf(b, sizeof(b), "lib count %-6lu", n);
    lcd.setCursor(0, 1);
    lcd.print(b);
    Serial.print("[BENCH] LCD count "); Serial.println(n);
    led(n % 2);
    delay(1000);
  }
}
