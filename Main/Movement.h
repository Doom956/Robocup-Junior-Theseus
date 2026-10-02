#ifndef Movement_h
#define Movement_h

#include <Arduino.h>
#include "MazeTile.h"  // for Direction, Tile, Grid, enum values

// Drivetrain extern
#include "motors.h"
extern motors drivetrain;

// Gyro extern
#include "gyro.h"
extern gyro myGyro;

// Pause flag
extern volatile bool Pausemaze;

// Forward declarations from other modules
int  measure(int sensor);
int  center();
int  centerLeft();
bool sideCentringError(double &e);
void centreAlong();
int  read_color();
int  obstacleavoidance(int leftright);
void stepForward(Direction d, int &x, int &y);
bool inBounds(int x, int y);
double pulsesForDistanceMm(double distanceMm);
void markEdgeBothWays(int x, int y, Direction d);
void writeWallsToCurrentTile(bool n, bool e, bool s, bool w);
void updateFullyExploredAt(int x, int y);
void elevation(Grid &srcGrid, int x, int y, Grid &m1, Grid &m2, Grid &m3, int &currentFloor);
void descend(Grid &srcGrid, int x, int y, Grid &m1, Grid &m2, Grid &m3, int &currentFloor);

// Function declarations
void init_drive();
void fwd(double dist);
void absoluteturn(double angle);

#endif
