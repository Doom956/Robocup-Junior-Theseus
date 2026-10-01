#ifndef Globals_h
#define Globals_h

#include <Arduino.h>
#include <Wire.h>
#include <mbed.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_BNO055.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include "MazeTile.h"
#include "motors.h"
#include "gyro.h"
#include "PID.h"
#include "timer.h"

// ============================================================
// Constants (mirrored from main.cpp — each TU needs its own)
// ============================================================
#define MIN_DIST              120
#define OBSTACLE_DIST         90
#define TILE_MM               300
#define ROBOT_LENGTH_MM       170
#define ROBOT_WIDTH_MM        140
#define TARGET_GAP_MM         (((double)TILE_MM - ROBOT_LENGTH_MM) / 2.0)
#define CENTER_TOL_MM         10
#define MAX_CENTER_CORRECTION_MM 300.0
#define TARGET_SIDE_GAP_MM    (((double)TILE_MM - ROBOT_WIDTH_MM) / 2.0)
#define SIDE_WALL_MAX_MM      200
#define LATERAL_TOL_MM        15
#define MAX_LATERAL_OFFSET_MM 90.0
#define LATERAL_CORRECTION_GAIN 1
#define BLACK_THRESHOLD       0.1f
#define SILVER_THRESHOLD      800
#define WHITE_THRESHOLD       0.85f
#define MULTIPLER             1.1
#define WALL_MISMATCH_THRESHOLD 2
#define TARGET_WALL_DISTANCE 80

// MAP_SIZE is defined in MazeTile.h; Grid uses it
using Grid = std::array<std::array<Tile, MAP_SIZE>, MAP_SIZE>;
const double wheel_diameter = 80.0;
const double wheel_cpr = 5.0;
const double gear_ratio = 195.0;

// ============================================================
// Hardware / driver objects (defined in main.cpp)
// ============================================================
extern VL53L0X sensors[7];
extern QWIICMUX myMux;
extern Adafruit_TCS34725 tcs;
extern Adafruit_BNO055 bno;
extern gyro myGyro;
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *motorA;
extern Adafruit_DCMotor *motorB;
extern Adafruit_DCMotor *motorC;
extern Adafruit_DCMotor *motorD;
extern motors drivetrain;
extern LiquidCrystal lcd;
extern Stepper myStepper;

// ============================================================
// Globals (defined in main.cpp)
// ============================================================
extern rtos::Mutex i2cMutex;
extern rtos::Mutex lcdMutex;
extern volatile bool Pausemaze;
extern volatile bool moveInterrupted;
extern volatile bool fwdActive;
extern volatile bool turnActive;
extern volatile bool isVictim;
extern volatile bool victimPending;
extern volatile bool victimtoggle;
extern bool obstacle;
extern bool blacktoggle;      // fwd() saw black ahead, marked it BLACK and backed off
extern bool fwdShort;         // fwd() covered < half a tile (blocked/stalled) and backed up to the start tile
extern bool silverDuringMove; // fwd() saw silver after the half-tile point (tile being entered is a checkpoint)
extern float clear;

// Map / navigation
extern Grid mapGrid;
extern Grid m1, m2, m3;
extern int x_pos, y_pos;
extern int currentFloor;
extern int x_checkpoint, y_checkpoint;
extern int floor_checkpoint;
extern Direction currentDir;
extern int plannedTurnDeg;
extern Direction plannedMoveDir;
extern bool turnCompletedForMove;
extern int botchedTurnAttempts;
extern const int MAX_BOTCHED_TURN_ATTEMPTS;
extern int medkits;

// State enums (defined here for all TUs except main.cpp which defines its own)
enum RobotState {
  SENSE_TILE, CENTERING, UPDATE_MAP, PLAN_NEXT, VICTIM_DETECT,
  EXECUTE_MOVE, BOTCHED_TURN_RECOVERY, BOTCHED_FWD_RECOVERY,
  BACKPEDAL, PAUSE, RETURN
};
enum Steps : int { TURN, PARALLEL, BACKTRACK, WIGGLE, FWD };
extern RobotState state;
extern Steps steps;

// ============================================================
// Forward declarations of functions in other TUs
// ============================================================

// Distance.cpp
int  measure(int sensor);
int  calibrateSensor(int sensor, int trueDistanceMm);
int  detectWall(int dir);
void parallel();
void centerFrontBack();
int  center();
int  centerLeft();
int  obstacleavoidance(int leftright);

// Color.cpp
int read_color();

// Navigation.cpp
Direction rotateDir(Direction base, int offset);
void stepForward(Direction d, int &x, int &y);
double pulsesForDistanceMm(double distanceMm);
void victimTileFromEncoder(int distanceMm, int encoderCount, int &victimX, int &victimY);
void markVictimAtEncoderPosition(int distanceMm);
bool inBounds(int x, int y);
void initializeMap();
void markEdgeBothWays(int x, int y, Direction d);
void updateFullyExploredAt(int x, int y);
void writeWallsToCurrentTile(bool n, bool e, bool s, bool w);
void elevation(Grid &srcGrid, int x, int y, Grid &m1, Grid &m2, Grid &m3, int &currentFloor);
void descend(Grid &srcGrid, int x, int y, Grid &m1, Grid &m2, Grid &m3, int &currentFloor);
Direction opposite(Direction d);
int turnNeededDeg(int cardinalDir);
Grid& floorGrid(int floor);
void syncActiveFloor();
bool planExploreDir(Direction &outDir);

// Movement.cpp
void init_drive();
void fwd(double dist);
void absoluteturn(double angle);

// UART / Camera
int  readSerial1();
int  readSerial2();
bool detectCam1();
bool detectCam2();

#endif
