#include <mbed.h> // access arduino mbed OS (rtos::Thread)
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Stepper.h>
#include <LiquidCrystal.h> // lcd screen
#include <array> // std::array (Grid type for multi-floor maps)
#include <deque>
#include <vector>
#include <utility>

#include <ArduinoQueue.h> // queue
#include <Vector.h> // vector
#include "PID.h"
#include "timer.h"
#include "gyro.h"
#include "dispenser.h"
#include "motors.h"
#include "Distance.h"
#include "Color.h"
#include "Movement.h"

// Forward declarations from other TUs
int  readSerial1();
int  readSerial2();
void serviceCameraVictim();
void victimTileFromEncoder(int distanceMm, int encoderCount, int &victimX, int &victimY);
int  turnNeededDeg(int cardinalDir);
int  calibrateSensor(int sensor, int trueDistanceMm);
void initializeMap();
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL);
bool checkTileMismatch(bool wallF, bool wallR, bool wallB, bool wallL);
bool planExploreDir(Direction &outDir);
void syncActiveFloor();
Grid& floorGrid(int floor);
void lcdPrint(const char* msg);
void runBench(); // bench.cpp

// 1 = bench mode: instead of a run, measure the values the simulator guesses (see bench.cpp). 0 for competition.
#define BENCH_MODE 0

// movement constants
#define MIN_DIST 120         // mm (tune this)
#define OBSTACLE_DIST 90
#define TILE_MM 300         // one tile = 300mm (RCJ tile)
#define ROBOT_LENGTH_MM 170                                      // mm, robot front-to-back length
#define TARGET_GAP_MM (((double)TILE_MM - ROBOT_LENGTH_MM) / 2.0) // mm, ideal front/back clearance when centered (52.5)
#define CENTER_TOL_MM 10                                          // mm, front-back centering tolerance
#define MAX_CENTER_CORRECTION_MM 300.0                            // mm, one tile — offset this large means an unreliable reading or the robot isn't really in-tile; skip/abort centering
#define ROBOT_WIDTH_MM 140                                          // mm, robot left-right width
#define TARGET_SIDE_GAP_MM (((double)TILE_MM - ROBOT_WIDTH_MM) / 2.0) // mm, ideal side-wall clearance when centered (80)
#define SIDE_WALL_MAX_MM 200                                        // mm; a side reading beyond this is the next tile through a gap, not this tile's wall
#define LATERAL_TOL_MM 15                                            // mm, lateral correction tolerance (looser than CENTER_TOL_MM)
#define MAX_LATERAL_OFFSET_MM 90.0                                   // mm, sanity cap — offset this large means an unreliable reading; skip
#define LATERAL_CORRECTION_GAIN 1                                // multiplier on the computed turn angle; bench-tune upward since fwd() partially fights the pre-turn (pulls back toward cardinal)
#define BLACK_THRESHOLD 0.1f // color clear-channel threshold ratio for black
#define SILVER_THRESHOLD 800 // use red value
#define WHITE_THRESHOLD 0.85f
#define MULTIPLER 1.1
#define WALL_MISMATCH_THRESHOLD 2 // >= this many of the 4 absolute walls disagreeing with the stored tile flags a position mismatch
#define TARGET_WALL_DISTANCE 80
float clear;

#include "MazeTile.h"

// set up mux and distance senosrs
VL53L0X sensors[7];
QWIICMUX myMux;
// shut down allcall
#define PCA_ADDR 0x60
#define MODE1    0x00
#define ALLCALL_BIT 0x01  // MODE1 bit0

// set up color sensor
#define TCS_PORT 7
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
// set up gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
gyro myGyro;
// set up motorshield and motors.
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorA = AFMS.getMotor(1);
Adafruit_DCMotor *motorB = AFMS.getMotor(2);
Adafruit_DCMotor *motorC = AFMS.getMotor(3);
Adafruit_DCMotor *motorD = AFMS.getMotor(4);

// set up encoder pins
const int encoderPin_A_A = 3;
const int encoderPin_A_B = 5; 
const int encoderPin_B_A = 2;
const int encoderPin_B_B = 4; 
const int encoderPin_D_A = 18;
const int encoderPin_D_B = 19;


//drivetrain class object
motors drivetrain(encoderPin_A_A,encoderPin_A_B,encoderPin_B_A,encoderPin_B_B,encoderPin_D_A,encoderPin_D_B);
// wheel cpr
const double wheel_cpr = 5; // 20/4
//gear ratio
const double gear_ratio = 195;
// wheel diameter
const double wheel_diameter = 80; // millimeters.
// detection classes

char classes[6] = {'H','S','U','R','Y','G'};

// create stepper object
const int steps_per_revolution = 2048;
Stepper myStepper = Stepper(steps_per_revolution, 8, 9,10,11); 
// create lcd object
int en = 25; int rs = 27; int d4 = 23; int d5 = 53; int d6 = 29; int d7 = 31;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// map grids 
// MAP_SIZE and Grid are defined in MazeTile.h
Grid mapGrid; // active floor's tiles
Grid m1;      // floor storage ("basement"/floor 0)
Grid m2;      // floor 1
Grid m3;      // floor 2

int currentFloor = START_FLOOR; // current floor (0..2) for elevation()/descend()
int LEDPIN = 51;


//states that the robot will be in
enum RobotState {
  SENSE_TILE,
  CENTERING,
  UPDATE_MAP,
  PLAN_NEXT,
  VICTIM_DETECT,
  EXECUTE_MOVE,
  BOTCHED_TURN_RECOVERY,
  BOTCHED_FWD_RECOVERY,
  BACKPEDAL,
  PAUSE,
  RETURN
};
enum Steps : int {
  TURN,
  PARALLEL,
  BACKTRACK,
  WIGGLE,
  FWD
};
// coord struct
struct coord {
  int x;
  int y;
};
Steps steps = TURN;

// initialize 

Direction currentDir = NORTH;     // robot heading in map coords (0..3)
int plannedTurnDeg = 0;           // -90,0,+90,180
Direction plannedMoveDir = NORTH; // absolute direction robot will move next
bool turnCompletedForMove = false;
// bound BOTCHED_TURN_RECOVERY so a persistently un-completable turn can't cycle forever
int botchedTurnAttempts = 0;
const int MAX_BOTCHED_TURN_ATTEMPTS = 3;
int x_pos = MAP_SIZE/2;
int y_pos = MAP_SIZE/2;
RobotState state = SENSE_TILE;
// maze return to start condition variables
int medkits = 8;
timer mazeTime;
// black blue toggles
bool blacktoggle = false;      // set by fwd() when it backs off a black tile
bool fwdShort = false;         // set by fwd() when it didn't reach the next tile
bool silverDuringMove = false; // set by fwd() when it crossed onto silver
bool returning = false;        // RETURN has started (a pause resumes the return, not exploration)
bool bluetoggle = false;
bool stairtoggle = false;
// obstacle toggle
bool obstacle = false;
// victim toggles
bool victimtoggle = false;
bool victimAtCurrent = false;
// camera GPIOs
const int gpio1 = 13;
const int gpio2 = 12;
// stepper variables
const int angle_offset = 44;
const int angle_increment = 22;
dispenser disp(angle_increment,angle_offset,steps_per_revolution);
// logic switch pin
const int logicswitch = 22;
volatile bool Pausemaze = false; // set by pauseThread, read by loop()
volatile bool moveInterrupted = false; // fwd() sets true when a pause aborts the move before the tile is completed
int x_checkpoint = MAP_SIZE/2, y_checkpoint = MAP_SIZE/2;
int floor_checkpoint = START_FLOOR; // floor the last checkpoint was recorded on (0..2)
bool tilecheck = false;

// Forward declaration: Arduino can't auto-prototype template return types.
std::deque<std::pair<int, std::pair<int,int>>> BFS(std::pair<int, std::pair<int,int>> currentpos, Grid& m1, Grid& m2, Grid& m3, std::pair<int, std::pair<int,int>> endpos, bool allowBlue = false, bool allowObstacle = false);

double headingErrorDeg(double targetDeg, double actualDeg) {
  double err = targetDeg - actualDeg;
  while (err > 180.0) err -= 360.0;
  while (err < -180.0) err += 360.0;
  return abs(err);
}

// ===== camera victim-detection RTOS thread =====
// The thread only checks the camera UARTs (Serial3 = left, Serial2 = right).
// -> It never touches the I2C bus (mux/distance/color) so it cannot interfere w/ the main context's measure()/detectWall() calls. 

// When a camera reports a letter while the robot is moving, the thread raises victimPending; fwd()/absoluteturn()
// then stop the drivetrain, pause their PID + timer, run detectCam(), and use markVictimAtEncoderPosition() to label the correct tile before resuming.

volatile bool fwdActive = false; // true only while inside fwd()
volatile bool turnActive = false;
volatile bool victimPending = false; // a camera reported -> movement must service it
volatile int  victimSide = 0;        // 1 = left (Serial3), 2 = right (Serial2)
volatile bool isVictim = false;      // a victim already handled during current move

rtos::Thread cameraThread;
rtos::Mutex i2cMutex;
rtos::Mutex lcdMutex; // lcd mutex to prevent conflict
void cameraTask(){
  while(true){
    int encoderCount = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
    int nx = x_pos; int ny=y_pos;
    if((fwdActive||turnActive) && !victimPending && !isVictim){
      
      //if(encoderCount>=0.3*pulsesForDistanceMm(TILE_MM)||encoderCount<=0.7*pulsesForDistanceMm(TILE_MM)){
        if(readSerial1() != -1){        // left camera (Serial4)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            victimSide = 1;
            drivetrain.fullstop();           // locks i2cMutex internally
            victimPending = true;
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            serviceCameraVictim();           // locks i2cMutex internally
          }
        }
        else if(readSerial2() != -1){   // right camera (Serial3)
          if(fwdActive) victimTileFromEncoder(TILE_MM,encoderCount,nx,ny);
          Serial.println("nx, ny");
          Serial.println(nx);
          Serial.println(ny);
          Serial.println(mapGrid[nx][ny].getVictim());
          if(mapGrid[nx][ny].getVictim() == false){
            victimSide = 2;
            drivetrain.fullstop();           // locks i2cMutex internally
            victimPending = true;
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
            serviceCameraVictim();           // locks i2cMutex internally
          }
        }
      }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
}


// pause maze thread: watches the logic switch and requests a stop.
rtos::Thread pauseThread;
void pauseTask(){
  while(true){
    
    if(digitalRead(logicswitch)==HIGH){
      
      Pausemaze = true;
    }
    else{
      
      Pausemaze = false;
    }
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool turnCompletedSuccessfully(Direction intendedDir) {
  const double TURN_SUCCESS_TOLERANCE_DEG = 20.0;
  double targetHeading = turnNeededDeg(intendedDir);
  double actualHeading = myGyro.heading();
  double err = headingErrorDeg(targetHeading, actualHeading);
  Serial.print("turn target=");
  Serial.print(targetHeading);
  Serial.print(", actual=");
  Serial.print(actualHeading);
  Serial.print(", err=");
  Serial.println(err);
  return err <= TURN_SUCCESS_TOLERANCE_DEG;
}

// RCJ run time is 8 minutes. Head home once the time used plus an estimate of the
// trip back would run past it. Tune RETURN_SEC_PER_TILE from a timed test run
// (average seconds per tile on the way home, including turns).
const double RUN_TIME_S = 480.0;
const double RETURN_SEC_PER_TILE = 5.0;
const double RETURN_MARGIN_S = 30.0;
const std::pair<int, std::pair<int,int>> HOME = {START_FLOOR, {MAP_SIZE/2, MAP_SIZE/2}};

// The route RETURN drives: avoid blue tiles, then allow them, then allow recorded obstacles.
std::deque<std::pair<int, std::pair<int,int>>> homePath(){
  syncActiveFloor();
  std::pair<int, std::pair<int, int>> currentpos = {currentFloor, {x_pos, y_pos}};
  std::deque<std::pair<int, std::pair<int,int>>> path = BFS(currentpos, m1, m2, m3, HOME, false, false);
  if(path.empty()) path = BFS(currentpos, m1, m2, m3, HOME, true, false); // allow blue
  if(path.empty()) path = BFS(currentpos, m1, m2, m3, HOME, true, true);  // allow recorded obstacles
  return path;
}

bool timeToReturn(){
  double elapsedS = mazeTime.getTime() / 1000000.0;
  // estimate the same route RETURN will take (it detours around blue tiles), plus
  // the 5 s stop on every blue tile it can't avoid
  std::deque<std::pair<int, std::pair<int,int>>> path = homePath();
  int tiles = path.empty() ? 0 : (int)path.size() - 1;
  int blueTiles = 0;
  for(const auto &p : path) if(floorGrid(p.first)[p.second.first][p.second.second].getType() == BLUE) blueTiles++;
  return elapsedS + tiles * RETURN_SEC_PER_TILE + blueTiles * 5.0 + RETURN_MARGIN_S >= RUN_TIME_S;
}

// Forget the edges blocked by blockEdge(), on all floors, remembering them so
// restoreBlockedEdges() can put them back. Returns how many were cleared.
struct ClearedEdge { Grid *g; uint8_t x, y, d; };
const int MAX_CLEARED_EDGES = 128;
ClearedEdge clearedEdges[MAX_CLEARED_EDGES];
int clearedEdgeCount = 0;
int exploreRetries = 0;
int clearBlockedEdges(){
  clearedEdgeCount = 0;
  Grid *grids[] = {&mapGrid, &m1, &m2, &m3};
  for(Grid *g : grids)
    for(int x = 0; x < MAP_SIZE; x++)
      for(int y = 0; y < MAP_SIZE; y++)
        for(int d = 0; d < 4; d++)
          if((*g)[x][y].getObstacle(d) && clearedEdgeCount < MAX_CLEARED_EDGES){
            (*g)[x][y].setObstacle(d, false);
            clearedEdges[clearedEdgeCount++] = {g, (uint8_t)x, (uint8_t)y, (uint8_t)d};
          }
  return clearedEdgeCount;
}
void restoreBlockedEdges(){
  for(int i = 0; i < clearedEdgeCount; i++)
    (*clearedEdges[i].g)[clearedEdges[i].x][clearedEdges[i].y].setObstacle(clearedEdges[i].d, true);
  clearedEdgeCount = 0;
}

// Record an obstacle on the edge between (x,y) and its neighbour in direction d
// (both sides), so the planners stop routing across it.
void blockEdge(int x, int y, Direction d){
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  mapGrid[x][y].setObstacle(d, true);
  if(nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) mapGrid[nx][ny].setObstacle(opposite(d), true);
}

// fwd() came back without reaching the next tile and reversed to this tile's centre.
// First time: re-sense and re-plan (a missed front wall is picked up by SENSE_TILE).
// Second time from the same tile in the same direction: block that edge so the
// robot routes around it instead of retrying forever.
int shortMoveCount = 0;
int shortX = -1, shortY = -1, shortFloor = -1;
Direction shortDir = NORTH;
void handleShortMove(){
  if(x_pos == shortX && y_pos == shortY && currentFloor == shortFloor && currentDir == shortDir){
    shortMoveCount++;
  } else {
    shortMoveCount = 1;
    shortX = x_pos; shortY = y_pos; shortFloor = currentFloor; shortDir = currentDir;
  }
  Serial.print("short move, attempt ");
  Serial.println(shortMoveCount);
  if(shortMoveCount >= 2){
    Serial.println("blocking edge ahead");
    blockEdge(x_pos, y_pos, currentDir);
    shortMoveCount = 0;
  }
}

// After fwd() completed a tile: record the traversed edge, advance the position and
// handle the floor of the tile just entered (blue = 5 s stop, silver = checkpoint).
// fwd() has already advanced x_pos/y_pos (and the floor) over any ramp tiles.
void finishTileMove(){
  // Never step off the 40 x 40 map (only possible once the position is already wrong):
  // a tile outside it would be written into other memory and crash the program.
  int nx = x_pos, ny = y_pos;
  stepForward(currentDir, nx, ny);
  if(!inBounds(nx, ny)){
    Serial.println("move would leave the map, position not advanced");
    return;
  }
  markEdgeBothWays(x_pos, y_pos, currentDir);
  stepForward(currentDir, x_pos, y_pos); // x_pos/y_pos now = new tile
  int color = read_color();
  if(color == 3 || silverDuringMove == true){
    mapGrid[x_pos][y_pos].setType(CHECKPOINT);
    x_checkpoint = x_pos; y_checkpoint = y_pos;
    floor_checkpoint = currentFloor;
    Serial.println("checkpoint recorded");
  }
  // RCJ: stop 5 s on a blue tile every time it is entered (the map flag covers a
  // missed colour read on later visits, e.g. on the way home)
  if(color == 1 || mapGrid[x_pos][y_pos].getType() == BLUE){
    mapGrid[x_pos][y_pos].setType(BLUE);
    drivetrain.fullstop();
    delay(5000);
  }
  if(obstacle == true){
    // obstacle avoided while entering this tile: block the edge ahead of it
    blockEdge(x_pos, y_pos, currentDir);
  }
}

// Turn to an absolute direction and confirm it with the gyro (same check as
// EXECUTE_MOVE). A botched turn is snapped back to the nearest cardinal and retried.
bool turnToDirection(Direction d){
  for(int attempt = 0; attempt < MAX_BOTCHED_TURN_ATTEMPTS; attempt++){
    if(Pausemaze == true) return false;
    if(d != currentDir || attempt > 0) absoluteturn(turnNeededDeg(d));
    delay(200);
    parallel();
    delay(100);
    if(turnCompletedSuccessfully(d)){
      currentDir = d;
      return true;
    }
    Direction snapped = (Direction)myGyro.headingToCardinal(myGyro.heading());
    absoluteturn(turnNeededDeg(snapped));
    currentDir = snapped;
  }
  return false;
}
void setup(){
  // initialize camera gpio pins
  pinMode(gpio1, INPUT);
  pinMode(gpio2, INPUT);
  // initialize logic switch pin
  pinMode(logicswitch, INPUT);
  pinMode(LEDPIN,OUTPUT);
  // begin UART communication.
  Serial.begin(115200);
  Serial3.begin(115200); // switch to 9600 for reliability
  Serial4.begin(115200);
  
  
  Wire.begin();
  disableAllCall();
  myMux.begin();
  init_dist(); // initialize mux before distance sensors.
  calibrateSensor(2,80); // must run AFTER init_dist() so sensors are initialized
  scanAllPorts();
  init_color();
  init_drive();
  //detect();
  //initialize map
  initializeMap(); // initialize mapgrid
  // every floor starts as a copy of the freshly initialized (empty) grid.
  m1 = mapGrid;
  m2 = mapGrid;
  m3 = mapGrid;
  currentFloor = START_FLOOR;
  x_pos=MAP_SIZE/2;
  y_pos=MAP_SIZE/2;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  state = SENSE_TILE;
  // start lcd
  lcd.begin(16, 2);
#if BENCH_MODE
  runBench(); // never returns
#endif
  // start RTOS threads: camera victim detection + pause-switch watcher.
  cameraThread.start(cameraTask);
  cameraThread.set_priority(osPriorityAboveNormal);
  pauseThread.start(pauseTask);
  //Serial.println("starting");
  mazeTime = timer(); // run clock starts now, after sensor init/calibration
  
  
}
int iterator = 0;



void loop(){
  //diagPrintStackUsage();
  
  /*
  for(int i = 1;i<=7;i++){
    Serial.print("sensor ");
    Serial.println(i);
    Serial.println(measure(i));
    delay(500);
  }
  
  */
  
  
  //lcdPrint("working");
  //delay(500);
  //drivetrain.drive(150,150*1.25,150*1.25,150);
  //drivetrain.drive(150,150,150,150);
  
  
  static bool wallF, wallR, wallB, wallL;
  switch (state) {
    case SENSE_TILE: {
      // reset per-tile toggles
      blacktoggle = false; bluetoggle = false; victimtoggle = false; obstacle = false;
      // Stop where it should have: centred along the way it faces (from a wall ahead or behind),
      // so the walls below are read from the middle of the tile.
      centreAlong();
      // Read for walls
      Serial.println("reading walls");
      readWallsRel(wallF, wallR, wallB, wallL);
      // re-sense: does this tile actually match what the map already recorded for it?
      tilecheck = checkTileMismatch(wallF, wallR, wallB, wallL);

      delay(200);
      state = UPDATE_MAP; // next state.
      // (front/back centring: centreAlong() above, before the walls are read)
      
      if(Pausemaze == true){
        Serial.println("pause");
        state=PAUSE;
        break;
      }
      break;
    }
    case CENTERING: {
      Serial.println("front/back centering in tile");
      centerFrontBack();
      state = UPDATE_MAP;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case UPDATE_MAP: {
      Serial.println("updating tile");
      // skip the write on a mismatch: preserve the already-trusted wall data for
      // this cell rather than overwriting it with a reading taken while the
      // robot's position belief may be wrong.
      if(!tilecheck) writeWallsToCurrentTile(wallF, wallR, wallB, wallL);
      else Serial.println("tile mismatch detected - preserving existing map data for this tile");
      updateFullyExploredAt(x_pos, y_pos);
      state = VICTIM_DETECT; // poll cameras while stopped before planning.
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case VICTIM_DETECT: {
      
      Serial.println("victim detect");
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case PLAN_NEXT: {
      Serial.println("plan next");
      if(returning){ // re-sensed a tile on the way home (failed move): keep heading home
        state = RETURN;
        break;
      }
      if(timeToReturn()){
        Serial.println("time to return home");
        state = RETURN;
        break;
      }
      Direction next;
      bool found = planExploreDir(next);
      // Nothing left on the map with plenty of time to spare usually means the map is wrong:
      // edges blocked after failed moves (often a wall end or wheel slip, not a real obstacle).
      // Clear them and look again before heading home (at most 3 times per run). If that
      // opens nothing new, put them back so the way home doesn't route through them.
      if(!found && exploreRetries < 3 && mazeTime.getTime() / 1000000.0 < RUN_TIME_S - 150 && clearBlockedEdges() > 0){
        found = planExploreDir(next);
        if(found){
          exploreRetries++;
          Serial.println("map looks finished early: cleared blocked edges, exploring again");
        }
        else restoreBlockedEdges();
      }
      if(found == false){
        Serial.println("maze fully explored");
        lcdPrint("maze explored");
        state = RETURN;
        break;
      }
      plannedMoveDir = next;
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;
      Serial.println(plannedTurnDeg);
      state = EXECUTE_MOVE;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case EXECUTE_MOVE: {
      if (turnCompletedForMove == false) {
        if(plannedMoveDir != currentDir){
          absoluteturn(plannedTurnDeg);
        }
        delay(200);
        parallel();
        delay(100);

        if (turnCompletedSuccessfully(plannedMoveDir) == false) {
          state = BOTCHED_TURN_RECOVERY;
          break;
        }
        currentDir = plannedMoveDir;
        turnCompletedForMove = true;
        botchedTurnAttempts = 0; // clean turn -> reset the recovery counter
      }
      // as in RETURN: only an avoidance during this move may block an edge in finishTileMove()
      // (BACKPEDAL goes straight back to planning, so one from a move that ended on black stayed set)
      obstacle = false;
      fwd(TILE_MM);
      // A pause aborted the move before the tile was completed: don't advance
      // position or write walls/edges (the robot didn't actually traverse the tile).
      if(moveInterrupted == true){
        state = PAUSE;
        break;
      }
      if(blacktoggle == true){
        state = BACKPEDAL; // black tile ahead (marked BLACK by fwd), robot reversed into this tile
        turnCompletedForMove = false;
        break;
      }
      if(fwdShort == true){
        handleShortMove();
        turnCompletedForMove = false;
        tilecheck = false;
        state = SENSE_TILE; // still in the same tile: re-sense and re-plan
        if(Pausemaze == true) state = PAUSE;
        break;
      }
      // update map + robot position only on a completed move
      finishTileMove();

      delay(200);
      parallel();
      delay(100);
      iterator += 1;

      isVictim = false;
      turnCompletedForMove = false;
      tilecheck = false;
      state = SENSE_TILE;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case BACKPEDAL: {
      // fwd() saw black, marked the tile ahead BLACK and reversed back into this
      // tile. Re-plan from here; the planner never routes into BLACK tiles.
      blacktoggle = false;
      turnCompletedForMove = false;
      delay(200);
      parallel();
      state = PLAN_NEXT;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case BOTCHED_TURN_RECOVERY: {
      if(Pausemaze == true){
        state = PAUSE;
        break;
      }
      botchedTurnAttempts += 1;
      Direction snappedDir = (Direction)myGyro.headingToCardinal(myGyro.heading());
      int snappedHeading = turnNeededDeg(snappedDir);
      Serial.println("botched turn detected, snapping to cardinal");
      absoluteturn(snappedHeading);
      delay(150);
      parallel();
      delay(100);
      currentDir = snappedDir;
      plannedTurnDeg = turnNeededDeg(plannedMoveDir);
      turnCompletedForMove = false;

      // Repeated failures on the same planned turn (wall, gyro drift, motor slip):
      // stop retrying it. Re-plan a fresh direction from the now-clean cardinal
      // heading instead of bouncing between EXECUTE_MOVE and recovery forever.
      if(botchedTurnAttempts >= MAX_BOTCHED_TURN_ATTEMPTS){
        Serial.println("max botched-turn retries reached, re-planning");
        botchedTurnAttempts = 0;
        state = PLAN_NEXT;
        break;
      }

      state = EXECUTE_MOVE;
      break;
    }
    case BOTCHED_FWD_RECOVERY: {
      // A forward move was interrupted / failed. Re-align and retry once.
      if(Pausemaze == true){
        state = PAUSE;
        break;
      }
      Serial.println("botched fwd detected, re-centering and retrying");
      parallel();
      delay(200);
      absoluteturn(turnNeededDeg(currentDir));
      delay(150);
      state = EXECUTE_MOVE;
      break;
    }
    case RETURN: {
      // Drive home one tile per loop(), re-planning from the current position each
      // time. fwd() handles ramps itself (advances x_pos/y_pos and switches floors via
      // elevation()/descend()), so the floor bookkeeping stays in sync.
      returning = true;
      if(currentFloor == HOME.first && x_pos == HOME.second.first && y_pos == HOME.second.second){
        drivetrain.fullstop();
        Serial.println("back at start");
        // Done: stop and blink, but keep watching the pause switch. If the robot is actually on
        // the wrong tile, a lack-of-progress restart puts it back on the last checkpoint and it
        // heads home again from there. (This was a while(true) that ignored the switch, so only
        // a power cycle -- which loses the map -- could get it going again.)
        lcdPrint("back to start");
        while(Pausemaze == false){
          drivetrain.fullstop();
          digitalWrite(LEDPIN, (millis() / 1000) % 2 ? HIGH : LOW);
          delay(20);
        }
        digitalWrite(LEDPIN, LOW);
        state = PAUSE;
        break;
      }
      std::deque<std::pair<int, std::pair<int,int>>> path = homePath();
      if(path.size() < 2){
        // No route home in the map (usually the position is off and a black tile or a blocked
        // edge cut the planned route). Stop, but keep watching the pause switch as above, so a
        // lack-of-progress restart can recover the run instead of it ending here.
        lcdPrint("no path found");
        while(Pausemaze == false){
          drivetrain.fullstop();
          delay(20);
        }
        state = PAUSE;
        break;
      }
      // path[0] = current tile, path[1] = next tile (may be on another floor for a ramp)
      int dx = path[1].second.first  - path[0].second.first;
      int dy = path[1].second.second - path[0].second.second;
      Direction moveDir;
      if(dy == 0) moveDir = (dx == 1) ? EAST : WEST;
      else        moveDir = (dy == 1) ? NORTH : SOUTH;

      if(turnToDirection(moveDir) == false){
        if(Pausemaze == true) state = PAUSE;
        break; // re-plan and retry next loop
      }
      obstacle = false;
      fwd(TILE_MM);
      if(moveInterrupted == true){
        state = PAUSE;
        break;
      }
      if(blacktoggle == true){
        blacktoggle = false; // tile ahead is now BLACK; the next BFS routes around it
        break;
      }
      if(fwdShort == true){
        handleShortMove();
        // Re-read this tile's walls before re-planning: the move most likely failed on a
        // wall the map is missing, and RETURN never senses walls itself, so it drove into
        // the same wall again and again until a lack-of-progress restart. PLAN_NEXT sends
        // the robot straight back to RETURN while returning == true.
        state = SENSE_TILE;
        if(Pausemaze == true) state = PAUSE;
        break;
      }
      finishTileMove();
      delay(200);
      centreAlong(); // squares up (parallel) and stops in the middle of the tile, as when exploring
      delay(100);
      isVictim = false;
      if(Pausemaze == true) state = PAUSE;
      break;
    }
    case PAUSE: {
      drivetrain.fullstop();
      delay(200);
      if(digitalRead(logicswitch)==LOW){
        Pausemaze = false;
        // Restore the checkpoint's FLOOR as well as its tile. Save the grid we were
        // working on back into its floor slot, then load the checkpoint floor's grid
        // -> so victim flags / walls are looked up on the correct floor.
        syncActiveFloor();
        currentFloor = floor_checkpoint;
        mapGrid = floorGrid(currentFloor);
        x_pos = x_checkpoint; y_pos = y_checkpoint; // resume from last checkpoint
        // Deterministic reset: rotate to the gyro's zero and declare it NORTH.
        // Removes the ambiguous headingToCardinal snap (which could bucket a near-45 deg
        // reading into the wrong cardinal and leave the robot diagonal).
        absoluteturn(0);        // turnNeededDeg(NORTH) == 0
        currentDir = NORTH;

        Serial.println("checkpoint coordinates");
        Serial.println(x_checkpoint);
        Serial.println(y_checkpoint);
        Serial.println(currentDir);
        steps = TURN; // reset avoidance steps
        turnCompletedForMove = false;
        blacktoggle = false;
        fwdShort = false;
        tilecheck = false;
        // keep going home if the pause happened during the return
        state = returning ? RETURN : SENSE_TILE;
      }
      break;
    }
 }


}
