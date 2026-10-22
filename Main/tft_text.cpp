// Waveshare 2inch LCD Module (ST7789V) as a big 16x2 text display: see tft_text.h.
#include "Globals.h"
#if USE_TFT
#include <mbed.h>

// Layout, landscape 320 x 240: two text bands of 48 px at the top, the minimap below. The text is a 5x7 font scaled
// to fit the line: up to 17 characters at 3 x 6 (letters ~5.4 mm tall, like the old LCD's), 26 at 2 x 4, 52 at 1 x 2.
#define TFT_W 320
#define TFT_H 240
#define TEXT_X 4
#define BAND_H 48
static const int Y0[2] = {4, 56};
#define COL_BLACK  0x0000
#define COL_WHITE  0xFFFF
#define COL_YELLOW 0xFFE0
#define COL_RED    0xF800
#define COL_GREEN  0x07E0
#define WAKE_MS 1000     // wake the panel up again this often (motor noise on the RST/power wires blanks it while driving)
#define REFRESH_MS 10000 // and redraw everything this often, in case its memory was lost too

static const uint8_t FONT[95][5] = { // classic 5x7, ASCII 32-126, one byte per column, bit 0 at the top
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08}};

static mbed::DigitalOut *pDin, *pClk, *pCs, *pDc, *pRst, *pBl;
// Below normal: it only draws while the main loop sleeps, so the screen never slows navigation.
// Above normal, but each pass draws for at most DRAW_BUDGET_US and then sleeps PASS_MS: the screen always gets its turn
// (below normal it never did while the robot drove without a laptop, 2026-10-20) and takes at most ~15% of the
// processor, in short slices, so navigation is never held up for long.
static rtos::Thread tftThread(osPriorityAboveNormal, 4096);
#define DRAW_BUDGET_US 3000
#define PASS_MS 20
static unsigned long passStart;
static bool overBudget(){ return micros() - passStart > DRAW_BUDGET_US; }

// What the callers wrote: 2 rows of 40, like the HD44780's memory (only the first 16 are visible), so code written
// for the old LCD (e.g. 32 spaces to clear) behaves the same.
static char text[2][40];
static int curCol = 0, curRow = 0;

static inline void spiByte(uint8_t b){
  for(int i = 7; i >= 0; i--){
    pDin->write((b >> i) & 1);
    pClk->write(1);
    pClk->write(0);
  }
}
static void cmd(uint8_t c){ pDc->write(0); spiByte(c); pDc->write(1); }
static void dat(uint8_t d){ spiByte(d); }
static void window(int x0, int y0, int x1, int y1){
  cmd(0x2A); dat(x0 >> 8); dat(x0); dat(x1 >> 8); dat(x1);
  cmd(0x2B); dat(y0 >> 8); dat(y0); dat(y1 >> 8); dat(y1);
  cmd(0x2C);
}
static void fill(int x, int y, int w, int h, uint16_t c){
  window(x, y, x + w - 1, y + h - 1);
  for(long n = (long)w * h; n > 0; n--){ dat(c >> 8); dat(c); }
}
// Set the panel up: hardware reset (start-up only), sleep out, 16-bit colour, landscape, inversion, display on. The
// sequence that worked on the robot (2026-10-20 wiring finder); a software reset (0x01) first left this module dark.
static void panelInit(bool hardReset){
  if(hardReset){ pRst->write(0); delay(5); pRst->write(1); delay(10); }
  pCs->write(0); // the only device on these pins: selected for good
  cmd(0x11); delay(10);                  // sleep out
  cmd(0x3A); dat(0x55);                  // 16-bit colour
  cmd(0x36); dat(TFT_ROTATION == 3 ? 0xA0 : 0x60); // landscape (MV + MX or MY)
  cmd(0x21);                             // inversion on (IPS panel)
  cmd(0x13);                             // normal display
  cmd(0x29);                             // display on
}
// one character at (x, y), the font scaled sx wide and sy tall (cell 6sx x 8sy)
static void drawCell(int x, int y, char ch, uint16_t fg, int sx, int sy){
  if(ch < 32 || ch > 126) ch = ' ';
  const uint8_t *g = FONT[ch - 32];
  int cw = 6 * sx, chh = 8 * sy;
  window(x, y, x + cw - 1, y + chh - 1);
  for(int py = 0; py < chh; py++){
    int fy = py / sy;
    for(int px = 0; px < cw; px++){
      int fx = px / sx;
      uint16_t c = (fx < 5 && fy < 7 && ((g[fx] >> fy) & 1)) ? fg : COL_BLACK;
      dat(c >> 8); dat(c);
    }
  }
}

// ---- Minimap: the robot's map around it, under the text (text rows end at y 187) ----
// 26 x 10 tiles of 12 x 12 px at y 108-227, north up, in the simulator's map colours (MC_ below): white = visited (pale
// blue on an upper floor), off-white = seen, grey = unknown, blue, silver, black, sand = ramp, a ring = the start tile;
// dark lines = walls, amber lines = edges blocked for an obstacle; the robot is an orange square with a nub ahead.
#define MCELL 12
#define MCOLS 26
#define MROWS 10
#define MX0 ((TFT_W - MCOLS * MCELL) / 2)
#define MY0 108
// one tile as drawn: 0 = nothing (outside the map or never seen)
#define M_WALLS     0x0000F   // bits 0-3: walls N E S W
#define M_OBST      0x000F0   // bits 4-7: obstacle edges N E S W
#define M_VISITED   0x00100
#define M_SEEN      0x00200
#define M_TYPE_SH   10        // bits 10-11: TileTypes
#define M_RAMP      0x01000
#define M_ROBOT     0x02000
#define M_DIR_SH    14        // bits 14-15: the robot's direction
#define M_START     0x10000
#define M_UP        0x20000   // a tile on an upper floor (the start floor is START_FLOOR)
static rtos::Mutex mapMutex;          // guards mapSnap / mapValid (held for a copy only)
static uint32_t mapSnap[MROWS][MCOLS]; // written by the main loop, read by the screen thread
static bool mapValid = false;

// One map tile as a code. Reads only, and only inside the map (0 for anything outside).
static uint32_t mapCode(int mx, int my, int rx, int ry, int dir, int floor){
  if(mx < 0 || my < 0 || mx >= MAP_SIZE || my >= MAP_SIZE) return 0;
  Tile &t = mapGrid[mx][my];
  uint32_t c = 0;
  int type = (int)t.getType() & 3;
  bool visited = t.getVisited(), seen = t.getDiscovered() || type == BLACK;
  if(visited || seen){
    for(int d = 0; d < 4; d++){
      if(t.getWall(d)) c |= 1u << d;
      if(t.getObstacle(d)) c |= 1u << (4 + d);
    }
    if(visited) c |= M_VISITED;
    if(floor != START_FLOOR) c |= M_UP;
    if(seen) c |= M_SEEN;
    c |= (uint32_t)type << M_TYPE_SH;
    if(t.getElevate() || t.getDescend()) c |= M_RAMP;
  }
  if(mx == MAP_SIZE / 2 && my == MAP_SIZE / 2 && floor == START_FLOOR) c |= M_START;
  if(mx == rx && my == ry) c |= M_ROBOT | ((uint32_t)(dir & 3) << M_DIR_SH);
  return c;
}

// Main loop: copy the window around the robot. The window only moves when the robot gets near its edge (or changes
// floor), so a move usually changes two tiles and the screen thread redraws just those.
void tftMinimapUpdate(){
  static uint32_t snap[MROWS][MCOLS];
  static int ox = 0, oy = 0, of = -1000;
  int rx = x_pos, ry = y_pos, f = currentFloor, dir = (int)currentDir;
  if(f != of || rx - ox < 4 || rx - ox > MCOLS - 5 || ry - oy < 2 || ry - oy > MROWS - 3){
    ox = rx - MCOLS / 2; oy = ry - MROWS / 2; of = f;
  }
  for(int r = 0; r < MROWS; r++)
    for(int c = 0; c < MCOLS; c++)
      snap[r][c] = mapCode(ox + c, oy + (MROWS - 1 - r), rx, ry, dir, f); // screen row 0 = the north edge
  if(!mapMutex.trylock()) return; // the screen thread is copying: next tile
  memcpy(mapSnap, snap, sizeof(mapSnap));
  mapValid = true;
  mapMutex.unlock();
}

// Colours as in the simulator's map (tools/sim/physics/live.html, light theme), in RGB565
#define MC_UNKNOWN 0xE71B  // #e2e2de never seen
#define MC_SEEN    0xF79D  // #f0f0ec seen, not visited yet
#define MC_TILE    0xFFFF  // #ffffff visited
#define MC_TILE_UP 0xDF3E  // #dde7f3 visited, upper floor
#define MC_WALL    0x18E3  // #1d1d1b
#define MC_BLACK   0x1082  // #121212
#define MC_BLUE    0x2319  // #2463c9
#define MC_SILVER  0xAD96  // #a9b0b7
#define MC_RAMP    0xDE54  // #d8c9a3
#define MC_ROBOT   0xDA41  // #d9480f
#define MC_BLOCKED 0xE508  // #e0a240 (amber: an edge blocked for an obstacle)

static void drawMapCell(int r, int c, uint32_t code){
  uint16_t px[MCELL][MCELL];
  uint16_t bg = MC_UNKNOWN;
  if(code & (M_VISITED | M_SEEN)){
    int type = (code >> M_TYPE_SH) & 3;
    bg = (code & M_VISITED) ? ((code & M_UP) ? MC_TILE_UP : MC_TILE) : MC_SEEN;
    if(code & M_RAMP) bg = MC_RAMP;
    if(type == BLUE) bg = MC_BLUE;
    else if(type == CHECKPOINT) bg = MC_SILVER;
    else if(type == BLACK) bg = MC_BLACK;
  }
  for(int y = 0; y < MCELL; y++) for(int x = 0; x < MCELL; x++) px[y][x] = bg;
  if(code & M_START){ // the start tile: a dark ring inside the tile
    for(int i = 3; i < 9; i++){ px[3][i] = MC_WALL; px[8][i] = MC_WALL; px[i][3] = MC_WALL; px[i][8] = MC_WALL; }
  }
  for(int d = 0; d < 4; d++){
    uint16_t col = (code & (1u << (4 + d))) ? MC_BLOCKED : (code & (1u << d)) ? MC_WALL : 0;
    if(!col) continue;
    for(int i = 0; i < MCELL; i++) for(int t = 0; t < 2; t++){ // 2 px lines
      if(d == 0) px[t][i] = col;              // north: top rows
      if(d == 1) px[i][MCELL - 1 - t] = col;  // east: right columns
      if(d == 2) px[MCELL - 1 - t][i] = col;  // south: bottom rows
      if(d == 3) px[i][t] = col;              // west: left columns
    }
  }
  if(code & M_ROBOT){
    for(int y = 3; y < 9; y++) for(int x = 3; x < 9; x++) px[y][x] = MC_ROBOT;
    int d = (code >> M_DIR_SH) & 3;
    for(int i = 5; i < 7; i++) for(int t = 1; t < 3; t++){ // nub toward the way it faces
      if(d == 0) px[t][i] = MC_ROBOT;
      if(d == 1) px[i][MCELL - 1 - t] = MC_ROBOT;
      if(d == 2) px[MCELL - 1 - t][i] = MC_ROBOT;
      if(d == 3) px[i][t] = MC_ROBOT;
    }
  }
  int x0 = MX0 + c * MCELL, y0 = MY0 + r * MCELL;
  window(x0, y0, x0 + MCELL - 1, y0 + MCELL - 1);
  for(int y = 0; y < MCELL; y++) for(int x = 0; x < MCELL; x++){ dat(px[y][x] >> 8); dat(px[y][x]); }
}

// Screen thread: draw the tiles that changed since the last pass.
static void drawMinimap(bool all){
  static uint32_t want[MROWS][MCOLS];
  static uint32_t shown[MROWS][MCOLS];
  static bool any = false, init = false;
  if(!init || all){ memset(shown, 0xFF, sizeof(shown)); init = true; } // 0xFFFFFFFF is never a tile code: redraw all
  if(mapMutex.trylock()){
    if(mapValid){ memcpy(want, mapSnap, sizeof(want)); any = true; }
    mapMutex.unlock();
  }
  if(!any) return;
  for(int r = 0; r < MROWS; r++)
    for(int c = 0; c < MCOLS; c++){
      if(overBudget()) return; // the rest next pass
      if(want[r][c] != shown[r][c]){ drawMapCell(r, c, want[r][c]); shown[r][c] = want[r][c]; }
    }
}

static int rowLen(const char *r){ int n = 40; while(n > 0 && r[n - 1] == ' ') n--; return n; }

static void tftLoop(){
  // No Serial here: on the battery alone (no USB host) a print from this thread may never return.
  fill(0, 0, TFT_W, TFT_H, COL_BLACK);
  char shown[2][40]; memset(shown, 0, sizeof(shown)); // 0 never matches: everything is drawn once
  uint16_t shownFg[2] = {0, 0};
  int shownScale[2] = {0, 0};
  int clearRow[2] = {BAND_H, BAND_H}; // a band being cleared (font size changed): next pixel row to clear
  unsigned long lastRefresh = millis(), lastWake = millis();
  while(true){
    passStart = micros();
    bool all = false;
    if(millis() - lastWake >= WAKE_MS){ panelInit(false); lastWake = millis(); passStart = micros(); } // sleep out, display on
    if(millis() - lastRefresh >= REFRESH_MS){ lastRefresh = millis(); all = true; memset(shown, 0, sizeof(shown)); }
    char snap[2][40];
    bool got = false;
    if(lcdMutex.trylock()){ memcpy(snap, text, sizeof(snap)); lcdMutex.unlock(); got = true; }
    for(int r = 0; got && r < 2; r++){
      // red for a warning ("!GYRO ..."), green for READY, else white (position line) / yellow (event line)
      uint16_t fg = snap[r][0] == '!' ? COL_RED : (r == 1 && strncmp(snap[r], "READY", 5) == 0) ? COL_GREEN
                    : (r == 0 ? COL_WHITE : COL_YELLOW);
      int len = rowLen(snap[r]);
      int sc = len <= 17 ? 3 : len <= 26 ? 2 : 1;               // shrink the font to fit the line
      if(sc != shownScale[r]){ clearRow[r] = 0; shownScale[r] = sc; memset(shown[r], 0, sizeof(shown[r])); }
      if(fg != shownFg[r]){ shownFg[r] = fg; memset(shown[r], 0, sizeof(shown[r])); }
      while(clearRow[r] < BAND_H && !overBudget()){ fill(0, Y0[r] + clearRow[r], TFT_W, 2, COL_BLACK); clearRow[r] += 2; }
      if(clearRow[r] < BAND_H) continue; // still clearing: text next pass
      int cols = (TFT_W - 2 * TEXT_X) / (6 * sc); if(cols > 40) cols = 40;
      int y = Y0[r] + (BAND_H - 8 * 2 * sc) / 2;
      for(int c = 0; c < cols && !overBudget(); c++){
        if(snap[r][c] != shown[r][c]){ drawCell(TEXT_X + c * 6 * sc, y, snap[r][c], fg, sc, 2 * sc); shown[r][c] = snap[r][c]; }
      }
    }
    if(!overBudget()) drawMinimap(all);
    else if(all) drawMinimap(true); // over budget: this only marks the map for redrawing
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(PASS_MS));
  }
}

void TftText::begin(uint8_t, uint8_t){
  static bool started = false;
  if(started) return; // bench and main both call begin()
  started = true;
  memset(text, ' ', sizeof(text));
  pCs  = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_CS), 1);
  pClk = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_CLK), 0);
  pDin = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_DIN), 0);
  pDc  = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_DC), 1);
  pRst = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_RST), 1);
  pBl  = new mbed::DigitalOut(digitalPinToPinName(TFT_PIN_BL), 1);  // backlight on
  panelInit(true);
  tftThread.start(tftLoop);
}

void TftText::setCursor(uint8_t col, uint8_t row){ curCol = col % 40; curRow = row % 2; }

size_t TftText::print(const char *s){
  size_t n = 0;
  for(; *s; s++, n++){
    text[curRow][curCol] = *s;
    if(++curCol >= 40){ curCol = 0; curRow = (curRow + 1) % 2; }
  }
  return n;
}
#endif
