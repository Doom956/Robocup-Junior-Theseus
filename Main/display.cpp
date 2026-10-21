// Status on the 16x2 LCD for testing without a laptop (navigation only; victim messages still use lcdPrint()).
//   line 1: tile relative to the start, floor, facing, run time   e.g. "2,3 F1 N    85s"
//   line 2: the last event                                         e.g. "NO PROGRESS"
// Writes take a few ms and never wait, so they can be called from the main loop and from fwd() when it stops.
#include "Globals.h"
#include "timer.h"

extern timer mazeTime; // main.cpp

#ifndef FW_VERSION
#define FW_VERSION "unknown" // set by tools/fw_version.py when built with PlatformIO
#endif

// Characters per line: 16 on the character LCD; the 2inch screen shrinks the font for longer lines (up to 32).
#if USE_TFT
#define LINE_W 32
#else
#define LINE_W 16
#endif
static char eventLine[LINE_W + 1] = "";
static char problemLine[LINE_W + 1] = ""; // a start-up problem: shown instead of events until the next power-on

static void writeLine(int row, const char *text){
  char buf[LINE_W + 1];
  snprintf(buf, sizeof(buf), "%-*s", LINE_W, text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void lcdStatus(){
  tftMinimapUpdate(); // the screen's minimap (does nothing with the old LCD)
  static const char dirs[4] = {'N', 'E', 'S', 'W'};
  char buf[LINE_W + 1];
  snprintf(buf, sizeof(buf), "%d,%d F%d %c %4lus", x_pos - MAP_SIZE / 2, y_pos - MAP_SIZE / 2, currentFloor,
           dirs[((int)currentDir) & 3], (unsigned long)(mazeTime.getTime() / 1000000.0));
  // Never wait for the LCD: the camera code holds it for 1 s while it shows a victim. Skipped now, shown next tile.
  if(!lcdMutex.trylock()) return;
  writeLine(0, buf);
  writeLine(1, eventLine);
  lcdMutex.unlock();
}

static bool runStarted = false;

void lcdEvent(const char *msg){
  if(strcmp(msg, "RUNNING") == 0 || strcmp(msg, "WHITE REF LOW!") == 0 || strcmp(msg, "RESTART") == 0) runStarted = true;
  if(!runStarted && strcmp(msg, "PAUSED") == 0) msg = "READY"; // the pause at power-on: all checks passed
  // A start-up problem stays on the screen: "PAUSED" used to replace it within a second of power-on.
  // The end of the run still shows (and the simulator ends runs on this text).
  if(problemLine[0] && strcmp(msg, "back to start") != 0 && strcmp(msg, "no path found") != 0) msg = problemLine;
  snprintf(eventLine, sizeof(eventLine), "%s", msg);
  if(!lcdMutex.trylock()) return; // busy (victim message): the event shows with the next update
  writeLine(1, eventLine);
  lcdMutex.unlock();
}

int checkDistanceSensors(char *out, int outLen); // Distance.cpp
extern bool colourMissing;                       // Color.cpp

// Power-on check: gyro, colour sensor, the 7 distance sensors. Everything missing goes on line 2 and stays there,
// e.g. "!GYRO COLR LB FR" (gyro, colour sensor, left-back and front-right distance sensors); the Serial log has
// the full names. All fine: "READY" until the run starts.
void lcdStartupCheck(){
  Serial.print("firmware ");
  Serial.println(FW_VERSION);
  if(lcdMutex.trylock()){ writeLine(0, "fw " FW_VERSION); lcdMutex.unlock(); } // replaced by the position line
  char dead[24];
  checkDistanceSensors(dead, sizeof(dead));
  char line[40] = "";
  if(gyroMissing) strcat(line, " GYRO");
  if(colourMissing) strcat(line, " COLR");
  if(dead[0]){ strcat(line, " "); strcat(line, dead); }
  if(line[0]) snprintf(problemLine, sizeof(problemLine), "!%s", line + 1);
  else problemLine[0] = 0;
  lcdEvent(problemLine[0] ? problemLine : "READY");
}
