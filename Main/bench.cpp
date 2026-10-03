// Bench mode: measures the things the physics simulator can only guess (tools/sim/physics/README.md,
// "Where the numbers come from"), so the simulator can be set to match this robot.
//
// To use it: set BENCH_MODE to 1 in main.cpp, upload, open the Serial monitor (115200). The robot runs
// the tests one by one. Before each one the LCD and Serial say how to place the robot; flip the pause
// switch on and off to start it, and keep hands off for the second it waits. Copy the [BENCH] lines.
// Set BENCH_MODE back to 0 for a competition run (with 0 none of this code runs).
//
// What each test prints and what it is compared with (the simulator's values at 11.8 V):
//   1 distance sensors in the middle of a tile with walls on all 4 sides: average of 50 readings per
//     sensor vs what the CAD says it should read there   -> tofOffsetSigma, SENSOR_OFFSET_MM
//   2 lowest PWM that turns the robot on the spot (sim 41) -> wheelMu, TURN_MIN_PWM
//   3 degrees turned by turnright(150) in 1 s (sim 75)    -> skidFactor
//   4 lowest PWM that moves it straight (sim 13-17)       -> frictionFracAt12V
//   5 fw(150) for 2 s with no steering: encoder distance and heading change; measure the real distance
//     and how far it went sideways with a ruler           -> tractionMean, motorGainSigma
//   6 heading change standing still for 2 minutes (sim 0.5 deg/min) -> gyroDriftSigmaDegPerMin
#include "Globals.h"
#include "Movement.h"

void lcdPrint(const char* msg); // uart_camera_comms.cpp

static const int BENCH_SWITCH_PIN = 22; // the pause (logic) switch, logicswitch in main.cpp

static double wrap180(double a){
  while(a > 180.0) a -= 360.0;
  while(a < -180.0) a += 360.0;
  return a;
}

static int encoderAvg(){
  return (drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3;
}

// the "next" button: the pause switch flipped on and then off again
static void waitForSwitch(const char *lcdText, const char *howToPlace){
  drivetrain.fullstop();
  Serial.print("[BENCH] next: ");
  Serial.println(howToPlace);
  Serial.println("[BENCH] flip the pause switch on and off to start");
  lcdPrint(lcdText);
  while(digitalRead(BENCH_SWITCH_PIN) == LOW) delay(20);
  while(digitalRead(BENCH_SWITCH_PIN) == HIGH) delay(20);
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
    Serial.print("[BENCH] sensor ");
    Serial.print(s);
    Serial.print(" ");
    Serial.print(names[s]);
    if(n == 0){ Serial.println(": no valid reading"); continue; }
    double mean = (double)sum / n;
    Serial.print(": ");
    Serial.print(mean, 1);
    Serial.print(" mm, CAD says ");
    Serial.print(expected[s], 1);
    Serial.print(", off by ");
    Serial.println(mean - expected[s], 1);
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
      Serial.print("[BENCH] lowest PWM that turns it on the spot: ");
      Serial.print(pwm);
      Serial.println(" (simulator 41; TURN_MIN_PWM is 20)");
      return;
    }
  }
  Serial.println("[BENCH] didn't turn even at 150");
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
  Serial.print("[BENCH] turnright(150) for 1 s: ");
  Serial.print(total, 1);
  Serial.println(" deg (simulator 75)");
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
      Serial.print("[BENCH] lowest PWM that drives it straight from standstill: ");
      Serial.print(pwm);
      Serial.println(" (simulator 17)");
      return;
    }
  }
  Serial.println("[BENCH] didn't move even at 60");
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
  Serial.print("[BENCH] fw(150) 2 s: encoders A/B/D ");
  Serial.print(a); Serial.print("/"); Serial.print(b); Serial.print("/"); Serial.print(d);
  Serial.print(", the code makes that ");
  Serial.print(mm, 0);
  Serial.print(" mm; heading changed ");
  Serial.print(wrap180(myGyro.heading() - h0), 1);
  Serial.println(" deg");
  Serial.println("[BENCH] measure with a ruler: distance driven, and how far it ended up to the side of the straight line");
  Serial.println("[BENCH] (simulator: about 300 mm, encoders say 4% more than the real distance)");
}

static void benchGyroDrift(){
  waitForSwitch("6 gyro drift", "6) robot standing still anywhere; don't touch it for 2 minutes");
  double h0 = myGyro.heading();
  for(int s = 30; s <= 120; s += 30){
    delay(30000);
    Serial.print("[BENCH] heading change after ");
    Serial.print(s);
    Serial.print(" s standing still: ");
    Serial.print(wrap180(myGyro.heading() - h0), 2);
    Serial.println(" deg");
  }
  Serial.println("[BENCH] per minute = the 120 s value / 2 (simulator assumes 0.5 deg/min)");
}

void runBench(){
  Serial.println("[BENCH] bench mode: measurements for the simulator (BENCH_MODE in main.cpp)");
  benchSensors();
  benchTurnPwm();
  benchTurnRate();
  benchStraightPwm();
  benchStraightLine();
  benchGyroDrift();
  drivetrain.fullstop();
  Serial.println("[BENCH] done");
  lcdPrint("bench done");
  while(true) delay(1000);
}
