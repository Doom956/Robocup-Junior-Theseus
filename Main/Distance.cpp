// distance sensor code
// blue for SDA, yellow for SCL
// the motor shield takes up the I2C address at 0x70.

#include "Globals.h"
#include "Distance.h"

// Forward declaration of fwd() from movement.cpp
void fwd(double dist);

void disableAllCall() {
  // Point register to MODE1
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.endTransmission(false);

  // Read MODE1
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  if (Wire.available() < 1) return; // couldn't read
  uint8_t mode1 = Wire.read();

  // Clear ALLCALL bit
  mode1 &= (uint8_t)~ALLCALL_BIT;

  // Write MODE1 back
  Wire.beginTransmission((uint8_t)PCA_ADDR);
  Wire.write((uint8_t)MODE1);
  Wire.write(mode1);
  Wire.endTransmission(true);
}

void init_dist() {
  
  if(!myMux.begin()){
    Serial.println("can't find the Mux");
  }
  else{
    Serial.println("Mux initialized");
  }
 
  for(int i = 0; i<7; i++){
    myMux.setPort(i);
    sensors[i].setAddress(0x30); // conflict with TCS34725 for some reason.
    delay(10);
    
    
    if(!sensors[i].init()){
    Serial.println("Sensor "+String(i)+" failed to initialize");
    }
    else{
      Serial.println("Sensor "+String(i)+" is able to initialize");
    }
    // Without a timeout the Pololu library waits forever for a reading, so one sensor that stops
    // answering (loose cable, I2C glitch) freezes the whole robot. With it, a read gives up after
    // 100 ms and returns 65535, which measure() treats as "no reading". (A reading normally takes 33 ms.)
    sensors[i].setTimeout(100);
    sensors[i].startContinuous(); // start continuous ranging.
  }
    
  
  

}

uint8_t scanI2COnCurrentBus() {
  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" ");
      count++;
    }
  }
  return count;
}

void scanAllPorts() {
  for (uint8_t port = 0; port < 8; port++) {
    bool ok = myMux.setPort(port);

    Serial.print("Port ");
    Serial.print(port);
    Serial.print(ok ? ": " : ": (setPort FAILED) ");

    delay(20);

    uint8_t found = scanI2COnCurrentBus();
    if (found == 0) Serial.print("(none)");
    Serial.println();
  }
}
// measure distance
/*
int measure(int sensor){
  
  if(sensor ==1){
    myMux.setPort(2);
    int value = sensors[2].readRangeContinuousMillimeters();
    
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor == 2){
    myMux.setPort(1);
    int value = sensors[1].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
      
  }
  if(sensor==3){
    myMux.setPort(0);
    int value = sensors[0].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==4){
    myMux.setPort(3);
    int value = sensors[3].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==5){
    myMux.setPort(6);
    int value = sensors[6].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==6){
    myMux.setPort(5);
    int value = sensors[5].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }
  if(sensor==7){
    myMux.setPort(4);
    int value = sensors[4].readRangeContinuousMillimeters();
    if (value != -1 && value != 8191) { return value;}
    else { return -1;}
    
  }

  return -1;
}
old robot settings
*/
// Per-sensor distance offsets in mm, indexed by logical sensor number (1..7); [0] unused.
// Calibrate: place a flat matte target at a known distance D (near the ~80mm working
// range), average ~100 raw readings, set SENSOR_OFFSET_MM[n] = mean(raw) - D.
// Positive => sensor reads long; it is subtracted from every reading in measure().
const int SENSOR_OFFSET_MM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int measure(int sensor){
  // sensor→mux port mapping
  const int portMap[] = {-1, 1, 0, 6, 4, 5, 3, 2};
  if(sensor < 1 || sensor > 7) return -1;
  int port = portMap[sensor];
  int sensorIdx = port; // sensor index matches port number

  i2cMutex.lock();
  myMux.setPort(port);
  int value = sensors[sensorIdx].readRangeContinuousMillimeters();
  i2cMutex.unlock();

  if(value == -1 || value == 8191 || value == 65535) return -1; // no reading (65535 = timed out)
  int corrected = value - SENSOR_OFFSET_MM[sensor];     // apply per-sensor calibration
  return (corrected < 0) ? 0 : corrected;               // clamp: negative distance is nonsense
}

// Calibration helper. Place a flat matte target at a known true distance trueDistanceMm
// (perpendicular to sensor n, ideally near the working range), then call this once from
// setup() or a serial command, e.g. calibrateSensor(2, 80). It averages RAW readings
// (offset NOT applied) and prints the recommended SENSOR_OFFSET_MM[n] value to Serial.
// Copy that number into the SENSOR_OFFSET_MM array above and reflash. Returns the
// computed offset, or -1 if the sensor never returned a valid reading.
int calibrateSensor(int sensor, int trueDistanceMm){
  const int samples = 100;
  if(sensor < 1 || sensor > 7) return -1;
  const int portMap[] = {-1, 1, 0, 6, 4, 5, 3, 2};
  int port = portMap[sensor];
  int sensorIdx = port;

  long sum = 0;
  int valid = 0;
  for(int i = 0; i < samples; i++){
    i2cMutex.lock();
    myMux.setPort(port);
    int value = sensors[sensorIdx].readRangeContinuousMillimeters();
    i2cMutex.unlock();
    if(value != -1 && value != 8191 && value != 65535){
      sum += value;
      valid++;
    }
    delay(10); // ~let a fresh continuous-ranging sample accumulate between reads
  }

  if(valid == 0){
    Serial.print("calibrateSensor: sensor ");
    Serial.print(sensor);
    Serial.println(" returned no valid readings");
    return -1;
  }

  double meanRaw = (double)sum / valid;
  int offset = (int)lround(meanRaw - trueDistanceMm);
  Serial.print("[CAL] sensor ");
  Serial.print(sensor);
  Serial.print("  meanRaw=");
  Serial.print(meanRaw, 1);
  Serial.print("mm  true=");
  Serial.print(trueDistanceMm);
  Serial.print("mm  valid=");
  Serial.print(valid);
  Serial.print("/");
  Serial.print(samples);
  Serial.print("  -> SENSOR_OFFSET_MM[");
  Serial.print(sensor);
  Serial.print("] = ");
  Serial.println(offset);
  return offset;
}
// detects wall in a direction( 0 is north, 1 is east, etc..) If output = 0, there is a wall.
// realtive directions(local).
int detectWall(int dir){
  if(dir == 0){ // check if there is a wall at north
    int a = measure(1);
    int b = measure(7);
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0; // there is a wall.
    }
    else{
      return 1; // no wall
    }
  }
  if(dir == 1){
    int a = measure(2);
    int b = measure(3);
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 2){
    int a = measure(4);
    if(a<MIN_DIST&&a!=-1&&a!=8191){
      return 0;
    }
    else{
      return 1;
    }
  }
  if(dir == 3){
    int a = measure(5);
    int b = measure(6);
    
    if((a<MIN_DIST&&a!=-1&&a!=8191)&&(b<MIN_DIST&&b!=-1&&b!=8191)){
      return 0;
    }
    else{
      return 1;
    }
    
  }

  return 1;
}

void parallel(){
  const int PARALLEL_TOL_MM = 3;
  const int PARALLEL_SPEED = 90;
  const unsigned long PARALLEL_TIMEOUT_MS = 500;
  const double MAX_PARALLEL_ROTATION_DEG = 45.0;

  int sensorA = -1;
  int sensorB = -1;
  int wallDir;
  Serial.println("paralleling");
  
  
  // Prefer aligning to the right wall; otherwise use left wall.
  if (detectWall(1)==0) {
    sensorA = 2;
    sensorB = 3;
    wallDir=1;
  } else if (detectWall(3)==0) {
    sensorA = 6;
    sensorB = 5;
    wallDir=3;
  } else {
    drivetrain.fullstop();
    return;
  }

  unsigned long startMs = millis();
  double startHeading = myGyro.heading();

  while (true) {
    // abort the correction on pause so the caller can transition to PAUSE.
    if (Pausemaze == true) { drivetrain.fullstop(); break; }
    int a = measure(sensorA);
    int b = measure(sensorB);

    // Invalid reading: stop correction to avoid runaway spinning.
    if (a < 0 || b < 0) {
      Serial.println("parallel: invalid sensor reading, aborting correction");
      break;
    }
    // If either sensor no longer sees the side wall within range, stop correcting
    // (the wall ended / robot isn't beside one) to avoid spinning on a phantom reading.
    if(a>MIN_DIST||b>MIN_DIST){
      break;
    }

    int diff = a - b;
    if (abs(diff) <= PARALLEL_TOL_MM) {
      Serial.println("paralleled");
      // Square to a wall (both sensors within 3 mm, 176 mm apart: within ~1 deg): the true heading
      // is a compass direction, so take any gyro drift out here.
      myGyro.resyncToNearestCardinal(10.0);
      break;
    }
    // break out after rotation.

    double headingDelta = myGyro.heading() - startHeading;
    while (headingDelta > 180.0) headingDelta -= 360.0;
    while (headingDelta < -180.0) headingDelta += 360.0;

    if (abs(headingDelta) >= MAX_PARALLEL_ROTATION_DEG) {
      Serial.println("parallel: rotation limit hit, aborting correction");
      break;
    }

    if ((millis() - startMs) >= PARALLEL_TIMEOUT_MS) {
      Serial.println("parallel: timeout, aborting correction");
      break;
    }

    // Reset wheel directions then apply correction turn.
    i2cMutex.lock();
    motorA->run(FORWARD);
    motorB->run(FORWARD);
    motorC->run(FORWARD);
    motorD->run(BACKWARD);
    if ((diff > 0 && wallDir == 1)||(diff < 0 && wallDir==3)) {
      motorB->run(BACKWARD);
      motorD->run(FORWARD);
    } else {
      motorA->run(BACKWARD);
      motorC->run(BACKWARD);
    }
    i2cMutex.unlock();
    drivetrain.drive(PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED,PARALLEL_SPEED);
    
  }
  drivetrain.reset_encoderCount(true,true,true);
  drivetrain.fullstop();
}

// Self-centers the robot front-to-back within a tile using the front wall (avg of sensors 1+7).
// Only acts when a front wall is present (back-wall-only centering is not implemented yet)
// parallel() runs first so the robot is squared to a side wall before the front reading is trusted.

void centerFrontBack(){
  const int CENTERING_SPEED = 50;                   // mirrors PARALLEL_SPEED
  const unsigned long CENTERING_TIMEOUT_MS = 2000;
  // MAX_CENTER_CORRECTION_MM is a file-scope #define (Main.ino), shared with the SENSE_TILE trigger gate >> redundant safety abort 
  // -> in case conditions changed between the trigger check and this function actually running.

  Serial.println("centering front-back (front wall)");
  parallel();

  if(detectWall(0) != 0){ // 0 == wall present, matches detectWall's convention
    Serial.println("centerFrontBack: no front wall, nothing to center against");
    return;
  }

  int front1 = measure(1);
  int front7 = measure(7);
  if(front1 == -1 || front7 == -1){
    Serial.println("centerFrontBack: invalid initial reading, aborting");
    return;
  }

  double frontGap = (front1 + front7) / 2.0;
  double offset = frontGap - TARGET_GAP_MM; // +ve => too far from ront wall, drive forward; -ve => drive backward

  if(abs(offset) >= MAX_CENTER_CORRECTION_MM){
    Serial.println("centerFrontBack: offset exceeds sanity cap, aborting");
    return;
  }
  if(abs(offset) <= CENTER_TOL_MM){
    Serial.println("already centered");
    return;
  }

  bool driveForward = offset > 0;
  unsigned long startMs = millis();

  while(true){
    // abort the correction on pause so the caller can transition to PAUSE.
    if(Pausemaze == true){ drivetrain.fullstop(); break; }
    front1 = measure(1);
    front7 = measure(7);
    if(front1 == -1 || front7 == -1){
      Serial.println("centerFrontBack: invalid sensor reading mid-correction, aborting");
      break;
    }

    frontGap = (front1 + front7) / 2.0;
    offset = frontGap - TARGET_GAP_MM;

    if(abs(offset) <= CENTER_TOL_MM){
      Serial.println("centered");
      break;
    }
    // If the live offset flips sign vs. our initial decision >> overshot, stop rather than reversing (avoids oscillation).
    if((offset > 0) != driveForward){
      Serial.println("centerFrontBack: overshot target, stopping to avoid oscillation");
      break;
    }
    if((millis() - startMs) >= CENTERING_TIMEOUT_MS){
      Serial.println("centerFrontBack: timeout, aborting correction");
      break;
    }

    if(driveForward) drivetrain.fw(CENTERING_SPEED);
    else drivetrain.backward(CENTERING_SPEED);
  }

  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);
}

// Front/back position: drive so the robot ends up centred along the way it faces, from the
// wall ahead or behind (the nearer one in view, up to about one tile past this one). With the
// robot centred, a wall at the end of this tile is half a path, (TILE_MM - WALL_THICK_MM) / 2,
// from its centre, so the front sensors read that minus FRONT_SENSOR_FWD_MM, plus 300 mm for each
// tile further away; the back sensor likewise. A front wall is only used when both front sensors
// agree (seen square-on). Corrects at most MAX_ALONG_FIX_MM; further than that, which tile the
// wall belongs to gets ambiguous.
#define WALL_THICK_MM        20.0   // RCJ walls: 300 mm tiles, 280 mm between wall faces
#define FRONT_SENSOR_FWD_MM  98.4   // CAD (tools/sim/cad/robot_geometry.json): front ToF sensors ahead of the centre
#define BACK_SENSOR_BACK_MM  102.8  // CAD: back ToF sensor behind the centre
#define MAX_ALONG_FIX_MM     100.0
void centreAlong(){
  const int SPEED = 50;
  const unsigned long TIMEOUT_MS = 2000;
  const double halfPath = (TILE_MM - WALL_THICK_MM) / 2.0;
  parallel();
  int f1 = measure(1), f7 = measure(7), b = measure(4);
  bool useFront = f1 > 0 && f7 > 0 && f1 <= 450 && f7 <= 450 && abs(f1 - f7) <= 25;
  bool useBack = b > 0 && b <= 450;
  if(useFront && useBack){ if(b < (f1 + f7) / 2) useFront = false; else useBack = false; } // the nearer wall reads more accurately
  if(!useFront && !useBack) return;
  double now = useFront ? (f1 + f7) / 2.0 : b;
  double base = halfPath - (useFront ? FRONT_SENSOR_FWD_MM : BACK_SENSOR_BACK_MM); // reading when centred, wall at the end of this tile
  double k = round((now - base) / TILE_MM);                                       // how many tiles further the wall is
  double target = base + k * TILE_MM;
  double past = useFront ? target - now : now - target;                            // + = stopped past the centre
  if(fabs(past) > MAX_ALONG_FIX_MM || fabs(past) <= CENTER_TOL_MM) return;
  Serial.print("centring along: ");
  Serial.print(past, 0);
  Serial.println(useFront ? " mm off (front wall)" : " mm off (back wall)");
  bool forward = past < 0;
  unsigned long startMs = millis();
  while(true){
    if(Pausemaze == true) break;
    double r;
    if(useFront){ int a = measure(1), c = measure(7); if(a <= 0 || c <= 0) break; r = (a + c) / 2.0; }
    else { int a = measure(4); if(a <= 0) break; r = a; }
    double p = useFront ? target - r : r - target;
    if(fabs(p) <= CENTER_TOL_MM) break;
    if((p < 0) != forward) break;                 // overshot: stop rather than swing back and forth
    if(millis() - startMs >= TIMEOUT_MS) break;
    if(forward) drivetrain.fw(SPEED);
    else drivetrain.backward(SPEED);
  }
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);
}

// Right-wall follower error, fed to center_PID in movement.cpp.
// Uses the two right-side sensors (front = 2, back = 3) per Hanafi et al. (2013):
//   E_Tot = (ideal - D) + angle,  where D = avg gap, angle = back - front.
// Positive error steers away from the right wall (matches the old sign convention).
// Returns 0 when the right wall isn't present on BOTH sensors (no reliable reference).
int center(){
  int front = measure(2);   // right-front gap (mm)
  int back  = measure(3);   // right-back gap  (mm)
  bool wallPresent = front != -1 && front != 8191 && front <= SIDE_WALL_MAX_MM
                  && back  != -1 && back  != 8191 && back  <= SIDE_WALL_MAX_MM;
  if(!wallPresent) return 0;
  double D = (front + back) / 2.0;                       // distance term
  double e = (TARGET_SIDE_GAP_MM - D) + (back - front);  // (ideal - D) + angle
  return (int)e;
}

// Left-wall follower error — mirrors center() but uses left-side sensors (5 = back, 6 = front).
// Sign convention: positive error steers AWAY from the left wall (toward center/right),
// matching the same motor adjustment direction as the right-wall version so the same
// center_PID and drivetrain.drive(... +/- adjustment) formula works unchanged.
// Returns 0 when the left wall isn't present on BOTH sensors.
int centerLeft(){
  int front = measure(6);   // left-front gap (mm)
  int back  = measure(5);   // left-back gap  (mm)
  bool wallPresent = front != -1 && front != 8191 && front <= SIDE_WALL_MAX_MM
                  && back  != -1 && back  != 8191 && back  <= SIDE_WALL_MAX_MM;
  if(!wallPresent) return 0;
  double D = (front + back) / 2.0;                       // distance term
  // fwd() drives (120 - e) on the left wheels and (120 + e) on the right, so positive e turns
  // LEFT. center() relies on that: too close to the right wall gives positive e, away from it.
  // For the left wall it is the other way round: too close (small D) must give negative e
  // (turn right, away from the wall), so the distance term is (D - ideal), not (ideal - D).
  // (front - back) is negative when the nose points at the left wall → turns right. ✓
  double e = (D - TARGET_SIDE_GAP_MM) + (front - back);
  return (int)e;
}

static bool isSideWall(int v){ return v != -1 && v != 8191 && v <= SIDE_WALL_MAX_MM; }

// Side-wall centring error for fwd(), from one reading of the four side sensors.
// Positive turns the robot left (fwd() drives 120 - e on the left wheels, 120 + e on the right).
// Walls on both sides: balance the two gaps. That centres the robot whatever ROBOT_WIDTH_MM
// says (a wrong width only moves the one-wall target below). One wall: same as center() /
// centerLeft(). Returns false when neither side has a wall on both of its sensors.
bool sideCentringError(double &e){
  int rf = measure(2), rb = measure(3);   // right front / back gaps (mm)
  int lf = measure(6), lb = measure(5);   // left front / back gaps (mm)
  // A wall that ends beside the robot: one sensor already sees past its end (or into the gap
  // before the next wall) while the other still sees the wall. Read as one wall, that looks like
  // the robot turned 30-60 deg toward it and the follower steers hard enough to lose the heading.
  // The two sensors of a side are 176 mm apart, so a real wall reads more than SIDE_WALL_MAX_DIFF_MM
  // differently only when the robot is already turned ~10 deg; then that side isn't a usable reference.
  // (Was 60 mm, ~19 deg. Now that fwd() steers for the whole tile, the 17-30 mm a wall end adds in the
  // simulator steered the robot into weaving: 600 comp fields, -8.1 +-4.6 points without this change.)
  const int SIDE_WALL_MAX_DIFF_MM = 30;
  bool right = isSideWall(rf) && isSideWall(rb) && abs(rf - rb) <= SIDE_WALL_MAX_DIFF_MM;
  bool left  = isSideWall(lf) && isSideWall(lb) && abs(lf - lb) <= SIDE_WALL_MAX_DIFF_MM;
  if(right && left){
    double offset = ((lf + lb) - (rf + rb)) / 4.0;   // + = right of centre -> turn left
    double angle  = ((rb - rf) + (lf - lb)) / 2.0;   // + = nose toward the right wall -> turn left
    e = (int)(offset + angle);
    return true;
  }
  if(right){ e = (int)((TARGET_SIDE_GAP_MM - (rf + rb) / 2.0) + (rb - rf)); return true; }
  if(left){  e = (int)(((lf + lb) / 2.0 - TARGET_SIDE_GAP_MM) + (lf - lb)); return true; }
  return false;
}


extern bool avoidingObstacle; // movement.cpp: stops the closing fwd() from starting another avoidance

// Black under the colour sensor during the manoeuvre: stop and back off. The turns and
// short drives below don't go through fwd(), which is the only other black check.
static bool blackDuringAvoidance(){
  if(read_color() != -1) return false;
  Serial.println("black during obstacle avoidance, backing off");
  drivetrain.fullstop();
  delay(100);
  drivetrain.backward(150);
  delay(400);
  drivetrain.fullstop();
  return true;
}

static int obstacleavoidanceSteps(int leftright);

int obstacleavoidance(int leftright){ // leftright determines to manuver left or right.
// return distance to wall at front; -2 = paused, -3 = gave up (timeout or black)
  avoidingObstacle = true;
  int result = obstacleavoidanceSteps(leftright);
  avoidingObstacle = false;
  return result;
}

static int obstacleavoidanceSteps(int leftright){
  Serial.println("obstacle avoidance");
  int _ = -1;
  // Whole-manoeuvre limit: PARALLEL <-> BACKTRACK and FWD <-> WIGGLE can otherwise
  // cycle forever when the gap beside the obstacle is too tight.
  timer avoidTimer;
  const double AVOID_TIMEOUT_US = 12000000.0;
  while(true){
    if(avoidTimer.getTime() > AVOID_TIMEOUT_US){
      Serial.println("obstacle avoidance timeout, giving up");
      drivetrain.fullstop();
      steps = TURN;
      return -3;
    }
    // Single authoritative pause guard: gates every step boundary and transition
    // burst, not just the innermost drive loops. Reset steps so a resume after the
    // pause starts a fresh maneuver instead of re-entering mid-sequence.
    if(Pausemaze == true){
      drivetrain.fullstop();
      steps = TURN;
      return -2;
    }
    switch (steps){
      case TURN:{
        // Timeout to prevent infinite spinning if the far-side sensor never clears
        // (failed sensor, wall dead-ahead, or unusual obstacle geometry).
        timer turnTimer;
        const unsigned long TURN_TIMEOUT_US = 2000000; // 2 seconds

        if(leftright == 1){ // obstacle at left
          _ = measure(1);
          _ = (_!=-1&&_!=8191) ? _ : -1;
          Serial.println("turn step");
          while(measure(7) < MIN_DIST && turnTimer.getTime() < TURN_TIMEOUT_US){
            i2cMutex.lock();
            motorB->run(BACKWARD);
            motorD->run(FORWARD);
            i2cMutex.unlock();
            drivetrain.drive(255,255,255,255);
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          }
          
        }
        else if(leftright == 0){ // obstacle at right
          _ = measure(7);
          _ = (_!=-1&&_!=8191) ? _ : -1;
          while(measure(1)<MIN_DIST && turnTimer.getTime() < TURN_TIMEOUT_US){
            i2cMutex.lock();
            motorA->run(BACKWARD);
            motorC->run(BACKWARD);
            motorB->run(FORWARD);
            motorD->run(BACKWARD); // D is mounted reversed; BACKWARD raw = physically FORWARD, matching motorB
            i2cMutex.unlock();
            drivetrain.drive(255,255,255,255);
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          }
          
        }
        drivetrain.fullstop();
        delay(200);
        drivetrain.fw(255);
        delay(300);
        drivetrain.fullstop();
        delay(200);
        if(blackDuringAvoidance()){ steps = TURN; return -3; }
        steps = PARALLEL;
        break;
      }
      case PARALLEL:{
        PID pid(1,0,0.1);
        if(leftright == 1){
          int a = measure(2); int b = measure(3);
          while(true){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            if(avoidTimer.getTime() > AVOID_TIMEOUT_US) break;
            a=measure(2); b = measure(3);
            if(a<=30) break;
            Serial.println("paralleling step");
            double increment = pid.getPID(a-b); // signed error: positive turns one way, negative the other
            drivetrain.drive(constrain(100+increment,50,170),constrain(100+increment,50,170),constrain(100-increment,50,170),constrain(100-increment,50,170));
            Serial.println(a-b);
            
            
            if(abs(b-a)<=15){
              //fwd
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        else if(leftright == 0){
          while(true){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            if(avoidTimer.getTime() > AVOID_TIMEOUT_US) break;
            int a = measure(6); int b = measure(5);
            if(a<=30) break;
            double increment = pid.getPID(a-b); // signed error: positive turns one way, negative the other
            drivetrain.drive(constrain(100-increment,50,170),constrain(100-increment,50,170),constrain(100+increment,50,170),constrain(100+increment,50,170));
            
            if(abs(a-b)<=15){
              drivetrain.fullstop();
              delay(200);
              steps = FWD;
              goto end;
            }
          }
          
        }
        Serial.println("too close, backing up");
        Serial.println(measure(2));
        steps = BACKTRACK; // put switch step in front of end( always meet it)
        break;
        end:
          break;
        
      }
      case BACKTRACK:{
        Serial.println("backtracking step");
        // put a timer on to prevent it from taking too long
        timer myTime;
        if(leftright == 0){
          while(measure(6)<=40&&myTime.getTime()<800000){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            drivetrain.backward(120);
          }
        }
        else if(leftright == 1){
          while(measure(2)<=40&&myTime.getTime()<800000){
            if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
            drivetrain.backward(120);
          }
        }
        drivetrain.fullstop();
        delay(200);
        Serial.println("sensor 2, now reading");
        Serial.println(measure(2));
        steps = PARALLEL;
        break;
      }
      
      case FWD:{
        
        if(measure(6)<=35&&measure(2)<=35){
          steps = WIGGLE; // squeezed on both sides: wiggle, then retry FWD
          break;
        }
        
        Serial.println("fwd step");
        if(blackDuringAvoidance()){ steps = TURN; return -3; }
        parallel();
        drivetrain.reset_encoderCount(true,true,true);
        delay(200);
        
        // Drive the rest of the tile, subtracting distance already travelled.
        // Read the front sensor ONCE (a second read can differ and overshoot) and
        // clamp to [0, TILE_MM]: if either front reading is invalid the front is
        // open/garbage, so fall back to one tile instead of a runaway distance.
        int frontNow = measure(1);
        int travelled = (_ != -1 && frontNow != -1) ? (_ - frontNow) : 0;
        int remaining = constrain(TILE_MM - travelled, 0, TILE_MM);
        fwd(remaining);
        steps = TURN;
        if(moveInterrupted == true) return -2; // paused during the inner fwd()
        return _;
      }
      case WIGGLE:{
        PID pid(8,0,0.1);
        Serial.println("wiggle step");
        delay(2000);
        timer myTime;
        while(abs(measure(2)-measure(6))>=15&&myTime.getTime()<1000000){
          if(Pausemaze == true){
              drivetrain.fullstop();
              return -2;
            }
          double diff = pid.getPID(measure(2)-measure(6));
          drivetrain.drive(70+diff,70+diff,70-diff,70-diff);
        }
        drivetrain.fullstop();
        delay(200);
        steps = FWD;
        break;
      }
    }
  }
}



