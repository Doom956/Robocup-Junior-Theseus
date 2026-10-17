// Status on the 16x2 LCD for testing without a laptop (navigation only; victim messages still use lcdPrint()).
//   line 1: tile relative to the start, floor, facing, run time   e.g. "2,3 F1 N    85s"
//   line 2: the last event                                         e.g. "NO PROGRESS"
// Writes take a few ms and never wait, so they can be called from the main loop and from fwd() when it stops.
#include "Globals.h"
#include "timer.h"

extern timer mazeTime; // main.cpp

static char eventLine[17] = "";

static void writeLine(int row, const char *text){
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void lcdStatus(){
  static const char dirs[4] = {'N', 'E', 'S', 'W'};
  char buf[17];
  snprintf(buf, sizeof(buf), "%d,%d F%d %c %4lus", x_pos - MAP_SIZE / 2, y_pos - MAP_SIZE / 2, currentFloor,
           dirs[((int)currentDir) & 3], (unsigned long)(mazeTime.getTime() / 1000000.0));
  lcdMutex.lock();
  writeLine(0, buf);
  writeLine(1, eventLine);
  lcdMutex.unlock();
}

void lcdEvent(const char *msg){
  snprintf(eventLine, sizeof(eventLine), "%s", msg);
  lcdMutex.lock();
  writeLine(1, eventLine);
  lcdMutex.unlock();
}
