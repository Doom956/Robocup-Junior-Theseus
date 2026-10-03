#include "Globals.h"
#include "Movement.h"
#include <chrono>
#include <cmath>

// Pitch change (deg) that counts as being on a ramp. RCJ ramps are up to 25 deg;
// the old 20 deg threshold missed shallower ramps and could drop out mid-ramp on a
// ~20 deg one. Speed bumps (<=2 cm) only tilt this robot ~7 deg. Bench-tune.
#define RAMP_PITCH_DEG 12
// fwd() gives up on a tile after this long (victim stops excluded): covers a stall
// at the low end of the Scale ramp-down or a wheel caught on debris.
#define FWD_TIMEOUT_US_PER_TILE 8000000.0
#define CLIMB_TIMEOUT_US 15000000.0
#define BACKUP_TIMEOUT_MS 2500
// absoluteturn(): finished when within this many degrees of the target.
#define TURN_DONE_DEG 2.0
// Lowest PWM absoluteturn() uses (same as before). Skid steering may need more than this to
// turn on the spot at all; bench-test the lowest drivetrain.turnright(pwm) that turns the
// robot and raise this to it plus a margin (the simulator's estimate is about 41).
#define TURN_MIN_PWM 20

// true while obstacleavoidance() runs (its closing fwd() must not start another avoidance)
bool avoidingObstacle = false;

static int avgEncoder(){
  return (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
}

// Reverse until the encoders are back at the start of the move (i.e. the centre of
// the tile we started from). Timeout so a dead encoder can't trap us here.
static void backUpToStart(){
  // Driven far enough that more than half the robot was on the next tile (RCJ 5.4.4: that's a visit
  // there), coming back is a new visit of this tile: on a blue one that's another 5 s stop before the
  // robot moves on (5.5.1c), or the referee calls lack of progress. 120 mm = half the tile less a margin
  // for not having started exactly in the middle.
  bool leftTile = avgEncoder() >= pulsesForDistanceMm(120);
  unsigned long startMs = millis();
  while(drivetrain.encoderCountA >= 0 && drivetrain.encoderCountB >= 0 && drivetrain.encoderCountD >= 0
        && millis() - startMs < BACKUP_TIMEOUT_MS){
    if(Pausemaze == true) break;
    drivetrain.backward(200);
  }
  drivetrain.fullstop();
  if(leftTile && Pausemaze == false && mapGrid[x_pos][y_pos].getType() == BLUE){
    Serial.println("back on a blue tile: 5 s stop");
    delay(5000);
  }
}

void init_drive(){
  drivetrain.init_drive();
  // initialize gyro
  myGyro.init_Gyro();
}

// obstacleavoidance(): >=0 / -1 = got past (its inner fwd() finished the tile),
// -2 = aborted by pause, -3 = gave up (timeout). Flags read by EXECUTE_MOVE / RETURN.
static void handleAvoidanceResult(int prevdist){
  if(prevdist == -2){
    moveInterrupted = true; // avoidance aborted by pause -> tile not completed
    steps = TURN;           // reset state machine so next call starts fresh
  }
  else if(prevdist == -3){
    fwdShort = true;        // couldn't get past: don't advance; caller blocks this edge
    steps = TURN;
  }
  else if(moveInterrupted == false){
    obstacle = true;
    fwdShort = false;       // inner fwd() covered only the remainder of the tile
  }
}

void fwd(double dist){ // in mm
  double pulses = dist/(wheel_diameter*M_PI)*wheel_cpr*gear_ratio; // easier to make a variable.
  bool black = false; // toggle for black tile
  bool climbtoggle = false; // toggle for climbing
  bool climbed = false; // if climbing occured.
  bool upwards = false; // up/ down for elevation
  int cnt = 0; // tiles traversed while climbing.
  PID climbPID(2,0,0.1); // pid for centering on ramp
  PID center_PID(2,0,0.5);
  PID gyroPID(1,0.001,0.03);
  PID Scale_PID(0.0045,0,0.0008); // pid for encoder 
  Serial.println("forwarding");
  // allow the camera RTOS thread to flag victims for this move
  fwdActive = true;
  isVictim = false;
  victimPending = false;
  moveInterrupted = false; // becomes true only if a pause aborts this move
  blacktoggle = false;
  fwdShort = false;
  silverDuringMove = false;
  const double fwdTimeoutUs = max(2000000.0, FWD_TIMEOUT_US_PER_TILE * dist / TILE_MM);
  int init_pitch = myGyro.modulus((int)myGyro.pitch_heading());
  int init_yaw = turnNeededDeg(myGyro.headingToCardinal(myGyro.heading()));
  Serial.println("init_yaw");
  Serial.println(init_yaw);
  // [DIAG] round-1 sideswipe instrumentation: show whether init_yaw matches actual heading
  double _entry_hdg = myGyro.heading();
  Serial.print("[FWD] entry hdg=");
  Serial.print(_entry_hdg, 1);
  Serial.print(" init_yaw=");
  Serial.print(init_yaw);
  Serial.print(" offset=");
  Serial.println(_entry_hdg - init_yaw, 1);
  const char* fwdExit = "normal";
  int _fwd_tick = 0;
  int front_left_current=measure(7); int front_right_current=measure(1);
  timer myTime;
  myTime.reset_delta_time();
  
  int front_left = measure(7);int front_right = measure(1);
  // Obstacle = one front sensor close while the other sees clear space. Not checked for the
  // fwd() that obstacleavoidance() itself runs at the end: that re-triggered avoidance again
  // and again (recursion with no limit), leaving the robot stuck until a lack-of-progress
  // restart, and could overflow the stack.
  bool canCheckObstacle = !avoidingObstacle;
  // outside loop
    if(canCheckObstacle&&front_left<=OBSTACLE_DIST&&front_left!=-1&&front_right>=MIN_DIST&&front_right!=-1){ // trigger obstacleavoidance
      Serial.println("obstacle left");
      int prevdist = obstacleavoidance(1);
      drivetrain.fullstop();
      delay(50);
      handleAvoidanceResult(prevdist);
      /*
      if(prevdist - (measure(1)+measure(7))/2 > TILE_MM){
        int pulses = pulsesForDistanceMm(prevdist - (measure(1)+measure(7))/2-TILE_MM); // don't "overmove"
        while(drivetrain.encoderCountA >= -pulses && drivetrain.encoderCountB >= -pulses && drivetrain.encoderCountD >= -pulses){ // too far in front, go back
          drivetrain.backward(150);
        }
      }
      else if(prevdist - (measure(1)+measure(7))/2 < TILE_MM){
        int pulses = pulsesForDistanceMm(TILE_MM-(prevdist - (measure(1)+measure(7))/2));
        while(drivetrain.encoderCountA <= pulses && drivetrain.encoderCountB <= pulses && drivetrain.encoderCountD <= pulses){ // too far in front, go back
          drivetrain.fw(150);
        }
      }
      drivetrain.fullstop();
      */
      Serial.println("[FWD] exit=obstacle-left");
      fwdActive = false;
      return;
    }
    else if(canCheckObstacle&&front_right<=OBSTACLE_DIST&&front_right!=-1&&front_left>=MIN_DIST&&front_left!=-1){ // same rule as the left side (was >= OBSTACLE_DIST)
      Serial.println("obstacle right");
      int prevdist = obstacleavoidance(0);
      drivetrain.fullstop();
      delay(50);
      /*
      if(prevdist - (measure(1)+measure(7))/2 > TILE_MM){
        int pulses = pulsesForDistanceMm(prevdist - (measure(1)+measure(7))/2-TILE_MM);
        while(drivetrain.encoderCountA >= -pulses && drivetrain.encoderCountB >= -pulses && drivetrain.encoderCountD >= -pulses){ // too far in front, go back
          drivetrain.backward(150);
        }
      }
      else if(prevdist - (measure(1)+measure(7))/2 < TILE_MM){
        int pulses = pulsesForDistanceMm(TILE_MM-(prevdist - (measure(1)+measure(7))/2));
        while(drivetrain.encoderCountA <= pulses && drivetrain.encoderCountB <= pulses && drivetrain.encoderCountD <= pulses){ // too far in front, go back
          drivetrain.fw(150);
        }
      }
      
      drivetrain.fullstop();
      */
      handleAvoidanceResult(prevdist);
      Serial.println("[FWD] exit=obstacle-right");
      fwdActive = false;
      return;
    }
    
  while((climbtoggle==true||(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3<=pulses)&&black!=true){
    Serial.print("distance travelled: ");
    Serial.println((((double)(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3)/5)/195*wheel_diameter*M_PI);
    //Serial.println((drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    if(Pausemaze==true) {drivetrain.fullstop(); moveInterrupted = true; break;}
    // Service a camera victim flagged by the RTOS thread: stop, pause PID +
    
    if(victimPending){
      drivetrain.fullstop(); // does not overide the thread
      climbPID.pausePID(1);
      gyroPID.pausePID(1);
      Scale_PID.pausePID(1);
      myTime.pause(1);
      while(victimPending == true){
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
      gyroPID.pausePID(2);
      climbPID.pausePID(2);
      Scale_PID.pausePID(2);
      myTime.pause(2);
    }
    
    // color: detect black (stop + back off) tiles ahead. Blue is read only
    // after the move completes (in EXECUTE_MOVE), not mid-motion here.
    // Rate-limit to every 10th loop iteration: the color sensor I2C read adds
    // ~2-3ms of latency that causes jitter in the centering PID loop. Black
    // tile detection is not so latency-sensitive that it must run every tick.
    int color = 0;
    if ((_fwd_tick % 10) == 0) {
      color = read_color(); // -1 black, 1 blue, 3 silver
      Serial.println("color");
      Serial.println(color);
    }
    if(color == -1){ // black tile ahead -> stop, mark next tile, back off
      drivetrain.fullstop();
      delay(100);
      Serial.println("black");
      int nx = x_pos; int ny = y_pos;
      stepForward(currentDir,nx,ny);
      if(inBounds(nx, ny)) mapGrid[nx][ny].setType(BLACK);
      backUpToStart();
      black = true;
      blacktoggle = true; // tells EXECUTE_MOVE not to advance the position
      fwdExit = "black";
      break;
    }
    // silver seen once the sensor is well into the next tile -> that tile is a checkpoint
    if(color == 3 && avgEncoder() >= pulses / 2.0) silverDuringMove = true;
    if(climbtoggle == false && myTime.getTime() > fwdTimeoutUs){
      fwdExit = "timeout";
      drivetrain.fullstop();
      break;
    }
    // PID centering — cascade: side walls → gyro heading hold.

    double adjustment;
    double _diag_pid_err;

    // 1) Side walls: both present -> balance the two gaps; one -> hold TARGET_SIDE_GAP_MM from it.
    bool sideWall = sideCentringError(_diag_pid_err);

    // 2) No side wall at all — hold the initial gyro heading so the robot
    //    doesn't drift in open areas or corridors with only front/back walls.
    if (!sideWall) {
      double yaw = myGyro.heading() - init_yaw;
      if (yaw > 180)  yaw -= 360;
      if (yaw < -180) yaw += 360;
      _diag_pid_err = yaw;
      adjustment = gyroPID.getPID(_diag_pid_err);
    } else {
      adjustment = center_PID.getPID(_diag_pid_err);
    }
    double Scale = Scale_PID.getPID(pulses-(drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3);
    
    // emergency stop
    
    front_left_current = measure(7);
    front_right_current = measure(1);
    
    if((front_left_current<=50&&front_left_current!=-1)&&(front_right_current<=50&&front_right_current!=-1)){
      Serial.println("stopping");
      // if the robot doesn't make it halfway across the tile, fwd failed.
      Serial.print("[FWD] emergency-stop fl=");
      Serial.print(front_left_current);
      Serial.print(" fr=");
      Serial.println(front_right_current);
      fwdExit = "emergency-front";
      drivetrain.fullstop();
      delay(50);
      break;
    }
    
    // check pitch: past RAMP_PITCH_DEG the robot is on a slope, so the encoder is turned off.
    // One reading decides both "on a ramp" and "up or down": the up/down test used to read the gyro
    // again, and with the pitch just past the threshold (noise, whole degrees) that second reading could
    // miss it, so an up-ramp was taken as a down-ramp: descend() instead of elevation(), wrong floor.
    int tilt = myGyro.modulus(myGyro.pitch_heading()) - init_pitch;
    if(abs(tilt) > RAMP_PITCH_DEG){
      Serial.println("climbing");
      int _encoderCountA = drivetrain.encoderCountA; // save values before ramp
      int _encoderCountB = drivetrain.encoderCountB;
      int _encoderCountD = drivetrain.encoderCountD;
      climbtoggle = true; // prevent outer loop from exiting on encoder count
      climbed = true;
      Serial.println(tilt);
      upwards = tilt > 0; // distinguish between moving up and moving down.
      double sectionPulses = pulses; // slope length of one tile at the current pitch
      timer climbTime;
      drivetrain.reset_encoderCount(true,true,true); // count ramp distance from the ramp start
      while(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch) > RAMP_PITCH_DEG){
        if(Pausemaze == true) { drivetrain.fullstop(); moveInterrupted = true; break; }
        if(climbTime.getTime() > CLIMB_TIMEOUT_US) { Serial.println("climb timeout"); fwdExit = "climb-timeout"; break; }
        // PID centering
        double yaw = myGyro.heading()-init_yaw;
        if(yaw>180) yaw = yaw-360;
        if(yaw<-180) yaw+= 360;
    
        // Steer between the ramp's side walls when both are in view (as on flat ground; walls are
        // vertical, so tilting along the slope doesn't change the side readings). Holding the gyro
        // heading alone let an off-centre start scrape a side wall all the way up: the wheels slip,
        // the encoders over-count and the ramp comes out a tile or two too long.
        double sideErr;
        double adjustment = sideCentringError(sideErr) ? center_PID.getPID(sideErr) : climbPID.getPID(yaw);
        
        Serial.println("climbing");
        //Serial.println(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch));
        Serial.println(adjustment);
        // center during climbing
        if(upwards == true) drivetrain.drive(180-adjustment,180-adjustment,180+adjustment,180+adjustment);
        if(upwards == false) drivetrain.drive(120-adjustment,120-adjustment,120+adjustment,120+adjustment);
        //Serial.println((drivetrain.encoderCountD+drivetrain.encoderCountA+drivetrain.encoderCountB)/3);
        sectionPulses = pulses/cos(abs(myGyro.modulus(myGyro.pitch_heading())-init_pitch)*(M_PI/180));
        if(avgEncoder() >= sectionPulses){
          Serial.println("1 section of the ramp climbed");
          cnt++;
          drivetrain.reset_encoderCount(true,true,true);
        } // track tiles
      }
      // A ramp section left partly counted (the encoder resets above discard it) still
      // occupies a tile if more than half of it was climbed; previously a ramp a
      // little shorter than one slope-length gave cnt = 0 and no floor change.
      if(moveInterrupted == false && avgEncoder() >= sectionPulses / 2.0) cnt++;
      int carried = 0; // tilted distance that still counts toward this tile
      if(cnt == 0){
        Serial.println("short tilt, not a ramp (bump/debris)");
        climbed = false;
        carried = avgEncoder();
      }
      // ramp crested: restore pre-ramp encoder values so outer loop finishes the tile
      drivetrain.set_encoderCountA(_encoderCountA + carried);
      drivetrain.set_encoderCountB(_encoderCountB + carried);
      drivetrain.set_encoderCountD(_encoderCountD + carried);
      climbtoggle = false; // re-enable encoder-based exit in outer loop
      myTime = timer();    // restart the tile timeout for the remainder after the ramp
      if(moveInterrupted == true) break;
    }
    
    
    // [DIAG] right-wall follower trace. front=sensor2, back=sensor3.
    // Watch: are m2/m3 valid (not -1) and <= SIDE_WALL_MAX_MM? is err non-zero when off-center?
    _fwd_tick++;
    if((_fwd_tick % 5) == 0){
      int _diag_front = measure(2);
      int _diag_back  = measure(3);
      Serial.print("[CENTER] m2(front)=");
      Serial.print(_diag_front);
      Serial.print(" m3(back)=");
      Serial.print(_diag_back);
      Serial.print(" err=");
      Serial.print(_diag_pid_err, 1);
      Serial.print(" adj=");
      Serial.println(adjustment, 1);
    }
    if(Scale*120 < 25) break;
    // Steering: left = base - adjustment, right = base + adjustment. Scale starts around 5, so the old
    // constrain(Scale*(120 -+ adjustment), 20, 150) gave 150 on both sides and no steering at all until
    // the last ~60 mm of the tile. Lower the base instead, so the whole correction always fits under 150.
    double base = min(Scale * 120, 150 - fabs(adjustment));
    double driveL = constrain(base - adjustment, 20, 150), driveR = constrain(base + adjustment, 20, 150);
    drivetrain.drive(driveL, driveL, driveR, driveR);
    //drivetrain.drive(150+adjustment,(150+adjustment)*1.25,(150-adjustment)*1.25,150+adjustment);
  }
  // Didn't get halfway (front blocked, stall, timeout): the robot is still in the
  // tile it started from. Back up to its centre and report it so the caller
  // doesn't advance the map position.
  if(climbed == false && black == false && moveInterrupted == false && avgEncoder() < pulses / 2.0){
    fwdShort = true;
    Serial.println("[FWD] short move, backing up to start tile");
    backUpToStart();
  }
  Serial.print("[FWD] exit=");
  Serial.println(fwdExit);
  Serial.println("stop- end of fwd");
  // sometimes it barely makes it over the slope
  if(climbed == true){
    for(int i = 0; i<cnt;i++){
      // Never step off the 40 x 40 map: writing a tile outside it corrupts memory and
      // crashes the program (it happened after a lost robot counted a long ramp).
      int nx = x_pos, ny = y_pos;
      stepForward(currentDir, nx, ny);
      if(!inBounds(nx, ny)) { Serial.println("ramp would leave the map, stopping the count"); break; }
      Serial.println("adding ramp to map");
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
      // moving between floors
      // only elevate the first tile of a ramp.
      // transition to another map
      if(i==0){
        if(upwards==true) elevation(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor); // elevate
        else descend(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
      }
    }
    Serial.println("compensating");
    drivetrain.fw(200);
    delay(300);
    drivetrain.fullstop();
    delay(200);
    absoluteturn(turnNeededDeg(currentDir)); // snap direction.
  }
  
  fwdActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true);
  victimtoggle = false;
}
// absolute turning

void absoluteturn(double angle){
  // create PID instance.
  PID myPID(4.5,0,0.3);
  double MOTORSPEED = 0;
  // allow the camera RTOS thread to flag victims during the turn
  turnActive = true;
  isVictim = false;
  victimPending = false;
  // Shortest signed-path error, wrapped into [-180, 180]:
  //   sign of diff  = direction to turn (+CW/turnright, -CCW/turnleft)
  //   |diff|        = shortest angular distance to target
  // Replaces the old fasterway + inverse() pair, which had a discontinuity at
  // 0/360 that caused left-turns through NORTH to go the 270-degree long way.
  double diff = angle - myGyro.heading();
  while(diff > 180.0)  diff -= 360.0;
  while(diff < -180.0) diff += 360.0;
  double init_abs = fabs(diff);
  Serial.print("[TURN] target=");
  Serial.print(angle);
  Serial.print(" hdg=");
  Serial.print(myGyro.heading(), 1);
  Serial.print(" init_diff=");
  Serial.println(init_abs, 1);
   // create timer to cut of turning
  timer myTimer;
  // Turning limit: 2 s per 90 deg, but at least 1 s so a small correction gets time to finish.
  const double turnLimitUs = max(1000000.0, 2.0 * init_abs / 90.0 * 1000000.0);

  // One loop for both directions: stop once within TURN_DONE_DEG of the target, and
  // turn back if it overshoots. (The old loop had no "reached" exit -- it only stopped
  // when its time ran out -- and steered by |error|, so an overshoot kept turning the
  // same way. Its minimum power (20) may also be below what turns the robot on the spot,
  // so near the target it could stall and wait out the clock, stopping short.)
  while(true){
    if(Pausemaze==true) {drivetrain.fullstop(); break;}
    if(victimPending){ // service camera victim mid-turn
      drivetrain.fullstop();
      myPID.pausePID(1); myTimer.pause(1);
      while(victimPending==true){
        rtos::ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
      myPID.pausePID(2); myTimer.pause(2);
    }
    // Recompute the wrapped error every tick.
    double d = angle - myGyro.heading();
    while(d > 180.0)  d -= 360.0;
    while(d < -180.0) d += 360.0;

    if(fabs(d) <= TURN_DONE_DEG) break;            // reached
    if(myTimer.getTime() > turnLimitUs) break;     // turning limit

    MOTORSPEED = constrain(myPID.getPID(fabs(d)), TURN_MIN_PWM, 150);
    if(d > 0) drivetrain.turnright(MOTORSPEED);
    else      drivetrain.turnleft(MOTORSPEED);
  }
  victimtoggle = false;
  Serial.println("finished turning");
  turnActive = false; // camera thread idles until the next move
  drivetrain.fullstop();
  drivetrain.reset_encoderCount(true,true,true); // reset encoder counters.
}

// Corrects left-right position within the tile by turning the robot a small amount before the next forward drive
// -> so fwd()'s heading-hold behavior moves the robot diagonally back towards the center.
// (it locks onto whatever heading it starts at) 
// Must run AFTER turnCompletedSuccessfully() has validated the cardinal turn, so this intentional small heading offset isn't mistaken for a botched turn.

