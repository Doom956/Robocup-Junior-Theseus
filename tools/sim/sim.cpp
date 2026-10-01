// Maze simulator for the robot's REAL navigation code.
//
// Compiles Main/navigation.cpp + Main/MazeTile.cpp unchanged (hardware headers are
// replaced by tools/sim/stubs) and drives them through random RCJ-style fields:
// walls with loops, black, blue and silver tiles, and ramps between two levels.
// Black tiles are only placed where they don't wall off any part of the field.
//
// Real code under test: planExploreDir, BFS, readWallsRel/checkTileMismatch,
// writeWallsToCurrentTile, updateFullyExploredAt, markEdgeBothWays, stepForward,
// elevation/descend/neighbourFloor, syncActiveFloor.
// Copied here because they live in hardware files (keep them in sync):
//   main.cpp     : SENSE_TILE/UPDATE_MAP, finishTileMove, blockEdge, handleShortMove,
//                  timeToReturn, the RETURN loop, PAUSE (lack-of-progress restart)
//   movement.cpp : the ramp loop at the end of fwd()
//
// --robot sets how imperfect the simulated robot is:
//   perfect   : sensors never misread, wheels never slip. Tests the logic; must pass 100%.
//   realistic : occasional wall misreads, stalls, wheel slip, miscounted ramps, missed black
//   harsh     : the same faults, more often
// Every fault goes through the same recovery path the robot uses (emergency stop ->
// short move -> handleShortMove, checkTileMismatch, lack-of-progress restart at the last
// checkpoint). An 8-minute clock runs in every mode, using the time model below.
//
// Usage: sim.exe [--robot perfect|realistic|harsh] [--runs N] [--scenario NAME|all]
//                [--seed S] [--show] [--verbose]

#include "Globals.h"
#include <deque>
#include <vector>
#include <random>
#include <string>
#include <map>

// ---- globals navigation.cpp expects (defined in main.cpp on the robot) ----
bool serialEcho = false;
SimSerial Serial;
Grid mapGrid, m1, m2, m3;
int x_pos = MAP_SIZE / 2, y_pos = MAP_SIZE / 2;
int currentFloor = START_FLOOR;
Direction currentDir = NORTH;
int x_checkpoint = MAP_SIZE / 2, y_checkpoint = MAP_SIZE / 2;
int floor_checkpoint = START_FLOOR;
motors::motors(int, int, int, int, int, int) {}
motors drivetrain(0, 0, 0, 0, 0, 0);

// navigation.cpp functions that main.cpp declares itself
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL);
bool checkTileMismatch(bool wallF, bool wallR, bool wallB, bool wallL);
std::deque<std::pair<int, std::pair<int, int>>> BFS(std::pair<int, std::pair<int, int>> currentpos, Grid &m1, Grid &m2, Grid &m3,
                                                    std::pair<int, std::pair<int, int>> endpos, bool allowBlue, bool allowObstacle);

// ============================================================
// Robot realism and time model
// ============================================================
struct Realism { const char *name; double wallMiss, wallGhost, stall, slip, rampErr, blackMiss; };
static const Realism REALISM[] = {
  {"perfect",   0,    0,    0,    0,     0,    0},
  {"realistic", 0.01, 0.01, 0.03, 0.003, 0.10, 0.02},
  {"harsh",     0.03, 0.03, 0.08, 0.01,  0.25, 0.06},
};
// main.cpp constants
static const double RUN_TIME_S = 480.0, RETURN_SEC_PER_TILE = 5.0, RETURN_MARGIN_S = 30.0;
// seconds per action (simple model; tune to your robot)
static const double T_FWD = 2.5, T_TURN90 = 1.2, T_TURN180 = 2.0, T_SENSE = 0.8, T_BLUE = 5.0,
                    T_RAMP_TILE = 3.0, T_SHORT = 3.5, T_BLACK = 3.0, T_LOP = 10.0;

// ============================================================
// The real field
// ============================================================
static const int SDX[4] = {0, 1, 0, -1}; // N E S W, same as stepForward()
static const int SDY[4] = {1, 0, -1, 0};
enum CellType { W_FLOOR, W_BLACK, W_BLUE, W_SILVER, W_RAMP, W_SOLID };

struct World {
  int W = 0, H = 0, sx = 0, sy = 0;
  std::vector<std::array<bool, 4>> wall;
  std::vector<int> type, level;
  int id(int x, int y) const { return y * W + x; }
  bool in(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
  bool hasWall(int x, int y, int d) const { return !in(x, y) || wall[id(x, y)][d]; }
  int typeAt(int x, int y) const { return in(x, y) ? type[id(x, y)] : W_SOLID; }
  void open(int x, int y, int d) {
    int nx = x + SDX[d], ny = y + SDY[d];
    if (!in(x, y) || !in(nx, ny)) return;
    wall[id(x, y)][d] = false;
    wall[id(nx, ny)][(d + 2) % 4] = false;
  }
  // tiles reachable from the start without crossing black (ramps crossed, never stopped on)
  std::vector<bool> reach() const {
    std::vector<bool> r(W * H, false);
    std::deque<std::pair<int, int>> q;
    q.push_back({sx, sy});
    r[id(sx, sy)] = true;
    while (!q.empty()) {
      int x = q.front().first, y = q.front().second;
      q.pop_front();
      for (int d = 0; d < 4; d++) {
        int nx = x + SDX[d], ny = y + SDY[d];
        if (hasWall(x, y, d) || r[id(nx, ny)]) continue;
        int t = typeAt(nx, ny);
        if (t == W_BLACK || t == W_SOLID) continue;
        r[id(nx, ny)] = true;
        q.push_back({nx, ny});
      }
    }
    return r;
  }
  bool allReachable() const {
    std::vector<bool> r = reach();
    for (int i = 0; i < W * H; i++)
      if (!r[i] && type[i] != W_BLACK && type[i] != W_SOLID) return false;
    return true;
  }
};

struct Scenario {
  const char *name;
  int minW, maxW, minH, maxH;
  int extraOpenPct; // extra wall openings (loops), % of cells
  int blackPct, bluePct, silverPct;
  int ramps;        // 0, 1 or 2 ramps joining two areas on different levels
};

static const Scenario SCENARIOS[] = {
  {"flat",    5, 8,   5, 8,   0,  6, 4, 4, 0},
  {"loops",   5, 9,   5, 9,   25, 6, 4, 4, 0},
  {"big",     12, 16, 12, 16, 20, 5, 3, 3, 0},
  {"ramp",    4, 7,   4, 8,   20, 5, 3, 3, 1},
  {"bigramp", 7, 8,   10, 14, 20, 5, 3, 4, 2},
};

static World generate(const Scenario &sc, std::mt19937 &rng) {
  auto rnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  World w;
  int rampLen = sc.ramps ? rnd(1, 3) : 0;
  int wA = rnd(sc.minW, sc.maxW), wB = sc.ramps ? rnd(sc.minW, sc.maxW) : 0;
  w.W = wA + rampLen + wB; // <= 19, so the field always fits the 40x40 map from any start tile
  w.H = rnd(sc.minH, sc.maxH);
  int n = w.W * w.H;
  w.wall.assign(n, {{true, true, true, true}});
  w.type.assign(n, W_FLOOR);
  w.level.assign(n, 0);
  std::vector<int> rows;
  while ((int)rows.size() < sc.ramps) {
    int r = rnd(0, w.H - 1);
    bool ok = true;
    for (int o : rows) if (std::abs(o - r) < 2) ok = false;
    if (ok) rows.push_back(r);
  }
  auto isRampRow = [&](int y) { for (int r : rows) if (r == y) return true; return false; };
  int levelA = sc.ramps ? rnd(0, 1) : 0;
  for (int y = 0; y < w.H; y++)
    for (int x = 0; x < w.W; x++) {
      int i = w.id(x, y);
      if (x < wA) w.level[i] = levelA;
      else if (x < wA + rampLen) w.type[i] = isRampRow(y) ? W_RAMP : W_SOLID;
      else w.level[i] = 1 - levelA;
    }

  // carve a perfect maze inside each area, then knock out extra walls for loops
  for (int reg = 0; reg < (sc.ramps ? 2 : 1); reg++) {
    int x0 = reg == 0 ? 0 : wA + rampLen, x1 = reg == 0 ? wA : w.W;
    std::vector<bool> seen(n, false);
    std::vector<std::pair<int, int>> st;
    st.push_back({rnd(x0, x1 - 1), rnd(0, w.H - 1)});
    seen[w.id(st[0].first, st[0].second)] = true;
    while (!st.empty()) {
      int x = st.back().first, y = st.back().second;
      int opts[4], k = 0;
      for (int d = 0; d < 4; d++) {
        int nx = x + SDX[d], ny = y + SDY[d];
        if (nx >= x0 && nx < x1 && ny >= 0 && ny < w.H && !seen[w.id(nx, ny)]) opts[k++] = d;
      }
      if (!k) { st.pop_back(); continue; }
      int d = opts[rnd(0, k - 1)];
      w.open(x, y, d);
      seen[w.id(x + SDX[d], y + SDY[d])] = true;
      st.push_back({x + SDX[d], y + SDY[d]});
    }
    int extra = (x1 - x0) * w.H * sc.extraOpenPct / 100;
    for (int e = 0; e < extra; e++) {
      int x = rnd(x0, x1 - 1), y = rnd(0, w.H - 1), d = rnd(0, 3);
      int nx = x + SDX[d], ny = y + SDY[d];
      if (nx >= x0 && nx < x1 && ny >= 0 && ny < w.H) w.open(x, y, d);
    }
  }
  // ramp corridors: open along their length, walls on both sides
  for (int r : rows) for (int x = wA - 1; x < wA + rampLen; x++) w.open(x, r, EAST);

  // start tile, then black/blue/silver on plain tiles (never on the start or next to a ramp end)
  std::vector<int> plain;
  for (int i = 0; i < n; i++) if (w.type[i] == W_FLOOR) plain.push_back(i);
  int s = plain[rnd(0, (int)plain.size() - 1)];
  w.sx = s % w.W; w.sy = s / w.W;
  auto nearRampEnd = [&](int i) { int x = i % w.W, y = i / w.W; return isRampRow(y) && (x == wA - 1 || x == wA + rampLen); };
  std::shuffle(plain.begin(), plain.end(), rng);
  int nb = n * sc.blackPct / 100, nbl = n * sc.bluePct / 100, nsi = n * sc.silverPct / 100;
  for (int i : plain) {
    if (i == s || nearRampEnd(i)) continue;
    if (nb > 0) {
      w.type[i] = W_BLACK; // keep it only if nothing gets walled off
      if (w.allReachable()) nb--; else w.type[i] = W_FLOOR;
    }
    else if (nbl > 0) { w.type[i] = W_BLUE; nbl--; }
    else if (nsi > 0) { w.type[i] = W_SILVER; nsi--; }
  }
  return w;
}

// ============================================================
// Simulated robot
// ============================================================
static World world;
static Realism prof;
static std::mt19937 noise;
static int rx, ry;           // where the robot really is
static double simTime;       // seconds since the start
static bool perfectRobot;
static bool flagWallPlanned, flagMismatch;
static double chance() { return std::uniform_real_distribution<double>(0.0, 1.0)(noise); }

// Distance.cpp stand-in: dir is relative (0 front, 1 right, 2 back, 3 left); 0 = wall.
// Each reading can be wrong: a real wall missed, or a wall seen where there is none.
int detectWall(int dir) {
  int absDir = (currentDir + dir) % 4;
  bool real = world.hasWall(rx, ry, absDir);
  bool flip = real ? chance() < prof.wallMiss : chance() < prof.wallGhost;
  return (real != flip) ? 0 : 1;
}

static int mapX(int wx) { return wx - world.sx + MAP_SIZE / 2; }
static int mapY(int wy) { return wy - world.sy + MAP_SIZE / 2; }
static int expectedFloor(int x, int y) { return START_FLOOR + world.level[world.id(x, y)] - world.level[world.id(world.sx, world.sy)]; }

// ---- main.cpp copies ----
static int shortMoveCount = 0, shortX = -1, shortY = -1, shortFloor = -1;
static Direction shortDir = NORTH;
static int realCpX, realCpY; // the silver tile the robot really visited last (where the referee restarts it)

static void blockEdge(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  mapGrid[x][y].setObstacle(d, true);
  if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) mapGrid[nx][ny].setObstacle(opposite(d), true);
}
static void handleShortMove() {
  if (x_pos == shortX && y_pos == shortY && currentFloor == shortFloor && currentDir == shortDir) shortMoveCount++;
  else { shortMoveCount = 1; shortX = x_pos; shortY = y_pos; shortFloor = currentFloor; shortDir = currentDir; }
  if (shortMoveCount >= 2) { blockEdge(x_pos, y_pos, currentDir); shortMoveCount = 0; }
}
static const std::pair<int, std::pair<int, int>> HOME = {START_FLOOR, {MAP_SIZE / 2, MAP_SIZE / 2}};
static std::deque<std::pair<int, std::pair<int, int>>> homePath() {
  syncActiveFloor();
  std::pair<int, std::pair<int, int>> cur = {currentFloor, {x_pos, y_pos}};
  auto path = BFS(cur, m1, m2, m3, HOME, false, false);
  if (path.empty()) path = BFS(cur, m1, m2, m3, HOME, true, false);
  if (path.empty()) path = BFS(cur, m1, m2, m3, HOME, true, true);
  return path;
}
static bool timeToReturn() {
  auto path = homePath();
  int tiles = path.empty() ? 0 : (int)path.size() - 1, blueTiles = 0;
  for (const auto &p : path) if (floorGrid(p.first)[p.second.first][p.second.second].getType() == BLUE) blueTiles++;
  return simTime + tiles * RETURN_SEC_PER_TILE + blueTiles * 5.0 + RETURN_MARGIN_S >= RUN_TIME_S;
}

static std::vector<bool> visitedReal; // tiles the robot really sensed from

static void senseTile() { // SENSE_TILE + UPDATE_MAP
  bool f, rt, b, l;
  readWallsRel(f, rt, b, l);
  if (!checkTileMismatch(f, rt, b, l)) writeWallsToCurrentTile(f, rt, b, l);
  else if (perfectRobot) flagMismatch = true;
  updateFullyExploredAt(x_pos, y_pos);
  simTime += T_SENSE;
  visitedReal[world.id(rx, ry)] = true;
}

static void applyFloorColour() { // finishTileMove(): colour of the tile really under the robot
  int t = world.typeAt(rx, ry);
  if (t == W_SILVER) {
    mapGrid[x_pos][y_pos].setType(CHECKPOINT);
    x_checkpoint = x_pos; y_checkpoint = y_pos; floor_checkpoint = currentFloor;
    realCpX = rx; realCpY = ry;
  }
  if (t == W_BLUE || mapGrid[x_pos][y_pos].getType() == BLUE) { mapGrid[x_pos][y_pos].setType(BLUE); simTime += T_BLUE; }
}

static int rampCrossings = 0;
enum MoveResult { MOVED, SHORT, STOPPED_BLACK, ON_BLACK };

static void turnTo(Direction d) {
  int diff = (d - currentDir + 4) % 4;
  simTime += diff == 0 ? 0 : diff == 2 ? T_TURN180 : T_TURN90;
  currentDir = d;
}

// One tile forward in currentDir: fwd() + finishTileMove(), with faults.
static MoveResult moveForward() {
  int d = currentDir;
  if (world.hasWall(rx, ry, d)) { if (perfectRobot) flagWallPlanned = true; simTime += T_SHORT; return SHORT; } // emergency stop
  if (chance() < prof.stall) { simTime += T_SHORT; return SHORT; }
  int nx = rx + SDX[d], ny = ry + SDY[d];
  if (world.typeAt(nx, ny) == W_BLACK) {
    if (chance() < prof.blackMiss) { rx = nx; ry = ny; simTime += T_FWD; return ON_BLACK; }
    int mx = x_pos, my = y_pos; // fwd(): marks the tile ahead BLACK and backs up
    stepForward(currentDir, mx, my);
    if (inBounds(mx, my)) mapGrid[mx][my].setType(BLACK);
    simTime += T_BLACK;
    return STOPPED_BLACK;
  }
  if (chance() < prof.slip) { // wheels spin in place: encoders say one tile, robot didn't move
    markEdgeBothWays(x_pos, y_pos, currentDir);
    stepForward(currentDir, x_pos, y_pos);
    simTime += T_FWD;
    applyFloorColour();
    return MOVED;
  }
  if (world.typeAt(nx, ny) == W_RAMP) {
    // fwd() ramp loop (movement.cpp): one map tile per ramp section, floor change on the first
    int cnt = 0;
    while (world.typeAt(rx + SDX[d] * (cnt + 1), ry + SDY[d] * (cnt + 1)) == W_RAMP) cnt++;
    int ax = rx + SDX[d] * (cnt + 1), ay = ry + SDY[d] * (cnt + 1);
    bool upwards = world.level[world.id(ax, ay)] > world.level[world.id(rx, ry)];
    int counted = cnt;
    if (chance() < prof.rampErr) counted = std::max(0, cnt + (chance() < 0.5 ? -1 : 1));
    for (int i = 0; i < counted; i++) {
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
      if (i == 0) {
        if (upwards) elevation(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
        else descend(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
      }
    }
    rx += SDX[d] * cnt;
    ry += SDY[d] * cnt;
    simTime += T_RAMP_TILE * cnt;
    rampCrossings++;
  }
  markEdgeBothWays(x_pos, y_pos, currentDir);
  stepForward(currentDir, x_pos, y_pos);
  rx += SDX[d];
  ry += SDY[d];
  simTime += T_FWD;
  applyFloorColour();
  return MOVED;
}

struct RunResult {
  bool done = false, failed = false, home = false, fullCov = false, timeReturn = false, lost = false, lostAtEnd = false, pass = false;
  std::string reason;
  double coverage = 0, mapAcc = 0, time = 0;
  int moves = 0, returnMoves = 0, blackStops = 0, shorts = 0, lops = 0, rampCrossings = 0;
};

static bool lostNow;
static void checkSync(RunResult &r) {
  bool lost = x_pos != mapX(rx) || y_pos != mapY(ry) || currentFloor != expectedFloor(rx, ry);
  if (lost) r.lost = true;
  lostNow = lost;
}

static void lackOfProgress(RunResult &r, bool returning, int &consecShort) { // PAUSE: restart at the checkpoint
  r.lops++;
  simTime += T_LOP;
  syncActiveFloor();
  currentFloor = floor_checkpoint;
  mapGrid = floorGrid(currentFloor);
  x_pos = x_checkpoint; y_pos = y_checkpoint;
  currentDir = NORTH; // absoluteturn(0)
  rx = realCpX; ry = realCpY;
  consecShort = 0;
  checkSync(r);
  if (!returning) senseTile(); // state = SENSE_TILE
}

static void handleMove(RunResult &r, MoveResult m, bool returning, int &consecShort) {
  if (m == MOVED) { consecShort = 0; checkSync(r); if (!returning) senseTile(); }
  else if (m == STOPPED_BLACK) r.blackStops++; // BACKPEDAL -> PLAN_NEXT (or re-plan in RETURN)
  else if (m == SHORT) {
    r.shorts++; consecShort++;
    handleShortMove();
    if (!returning) senseTile(); // SENSE_TILE again
    if (consecShort >= 4) lackOfProgress(r, returning, consecShort);
  }
  else lackOfProgress(r, returning, consecShort); // ON_BLACK
}

static RunResult runMaze() {
  RunResult r;
  initializeMap();
  m1 = mapGrid; m2 = mapGrid; m3 = mapGrid;
  currentFloor = START_FLOOR;
  x_pos = MAP_SIZE / 2; y_pos = MAP_SIZE / 2;
  x_checkpoint = x_pos; y_checkpoint = y_pos; floor_checkpoint = START_FLOOR;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  rx = world.sx; ry = world.sy; realCpX = rx; realCpY = ry;
  simTime = 0; rampCrossings = 0; lostNow = false;
  shortMoveCount = 0; shortX = shortY = shortFloor = -1;
  flagWallPlanned = flagMismatch = false;
  visitedReal.assign(world.W * world.H, false);
  const int MOVE_LIMIT = 12 * world.W * world.H + 100;
  int consecShort = 0;
  auto fail = [&](const char *why) { if (!r.failed) { r.failed = true; r.reason = why; } };

  // ---- exploration: SENSE_TILE -> PLAN_NEXT -> EXECUTE_MOVE ----
  senseTile();
  bool returning = false;
  while (!r.failed && !returning) {
    if (simTime >= RUN_TIME_S) { fail("8:00 ran out while exploring"); break; }
    if (timeToReturn()) { r.timeReturn = true; break; }
    Direction next;
    if (!planExploreDir(next)) break; // maze explored -> RETURN
    turnTo(next);
    r.moves++;
    handleMove(r, moveForward(), false, consecShort);
    if (r.moves > MOVE_LIMIT) fail("exploration never finished (looping)");
  }

  // ---- RETURN: re-plan home one tile at a time ----
  returning = true;
  while (!r.failed) {
    if (currentFloor == HOME.first && x_pos == HOME.second.first && y_pos == HOME.second.second) {
      if (rx == world.sx && ry == world.sy) r.done = true;
      else fail("thinks it is home but is not");
      break;
    }
    if (simTime >= RUN_TIME_S) { fail("8:00 ran out on the way home"); break; }
    auto path = homePath();
    if (path.size() < 2) { fail("no path home in its map"); break; }
    int dx = path[1].second.first - path[0].second.first;
    int dy = path[1].second.second - path[0].second.second;
    turnTo((dy == 0) ? (dx == 1 ? EAST : WEST) : (dy == 1 ? NORTH : SOUTH));
    r.returnMoves++;
    handleMove(r, moveForward(), true, consecShort);
    if (r.returnMoves > MOVE_LIMIT) fail("never got home (looping)");
  }
  syncActiveFloor();
  r.rampCrossings = rampCrossings;
  r.time = simTime;
  r.lostAtEnd = lostNow;

  // ---- results ----
  std::vector<bool> reach = world.reach();
  int total = 0, covered = 0, wallsOk = 0, wallsAll = 0;
  for (int i = 0; i < world.W * world.H; i++) {
    if (!reach[i] || world.type[i] == W_RAMP) continue;
    total++;
    if (visitedReal[i]) covered++;
  }
  for (int i = 0; i < world.W * world.H; i++) {
    if (!visitedReal[i]) continue;
    int x = i % world.W, y = i / world.W, f = expectedFloor(x, y);
    if (f < 0 || f >= NUM_FLOORS) continue;
    Tile &t = floorGrid(f)[mapX(x)][mapY(y)];
    for (int d = 0; d < 4; d++) { wallsAll++; if (t.getVisited() && t.getWall(d) == world.hasWall(x, y, d)) wallsOk++; }
  }
  r.coverage = total ? (double)covered / total : 1;
  r.mapAcc = wallsAll ? (double)wallsOk / wallsAll : 1;
  r.fullCov = covered == total;
  r.home = r.done && simTime <= RUN_TIME_S;
  r.pass = r.home && (r.fullCov || r.timeReturn);
  if (perfectRobot) r.pass = r.pass && r.mapAcc == 1 && !r.lost && !flagWallPlanned && !flagMismatch;
  if (!r.pass && r.reason.empty()) {
    if (!r.home) r.reason = "back home after 8:00";
    else if (!r.fullCov && !r.timeReturn) r.reason = "finished early and missed part of the field";
    else r.reason = flagWallPlanned ? "planner chose a direction with a real wall"
                  : flagMismatch ? "checkTileMismatch fired with perfect sensors"
                  : r.lost ? "map position drifted from the real position" : "map wall does not match the real field";
  }
  return r;
}

// ============================================================
// ASCII output (--show)
// ============================================================
static void printWorld() {
  std::printf("\nReal field (S start, X black, b blue, s silver, R ramp, # solid, . = upper level):\n");
  for (int y = world.H - 1; y >= 0; y--) {
    std::string top, mid;
    for (int x = 0; x < world.W; x++) {
      top += "+"; top += world.hasWall(x, y, NORTH) ? "---" : "   ";
      mid += world.hasWall(x, y, WEST) ? "|" : " ";
      int t = world.typeAt(x, y);
      char c = world.level[world.id(x, y)] ? '.' : ' ';
      if (t == W_BLACK) c = 'X'; else if (t == W_BLUE) c = 'b'; else if (t == W_SILVER) c = 's';
      else if (t == W_RAMP) c = 'R'; else if (t == W_SOLID) c = '#';
      if (x == world.sx && y == world.sy) c = 'S';
      mid += ' '; mid += c; mid += ' ';
    }
    top += "+"; mid += world.hasWall(world.W - 1, y, EAST) ? "|" : " ";
    std::printf("%s\n%s\n", top.c_str(), mid.c_str());
  }
  std::string bot;
  for (int x = 0; x < world.W; x++) bot += world.hasWall(x, 0, SOUTH) ? "+---" : "+   ";
  std::printf("%s+\n", bot.c_str());
}

static void printMapFloor(int f) {
  Grid &g = floorGrid(f);
  int x0 = MAP_SIZE, x1 = -1, y0 = MAP_SIZE, y1 = -1;
  for (int x = 0; x < MAP_SIZE; x++)
    for (int y = 0; y < MAP_SIZE; y++)
      if (g[x][y].getVisited() || g[x][y].getType() == BLACK) { x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y); }
  if (x1 < 0) return;
  std::printf("\nRobot's map, floor %d (S home, X black, b blue, s checkpoint, ^ v ramp up/down to other floor, ? not visited):\n", f);
  for (int y = y1; y >= y0; y--) {
    std::string top, mid;
    for (int x = x0; x <= x1; x++) {
      Tile &t = g[x][y];
      bool v = t.getVisited();
      top += "+"; top += (v && t.getWall(NORTH)) ? "---" : "   ";
      mid += (v && t.getWall(WEST)) ? "|" : " ";
      char c = v ? ' ' : '?';
      if (t.getType() == BLACK) c = 'X'; else if (t.getType() == BLUE) c = 'b'; else if (t.getType() == CHECKPOINT) c = 's';
      if (t.getElevate()) c = '^';
      if (t.getDescend()) c = 'v';
      if (f == START_FLOOR && x == MAP_SIZE / 2 && y == MAP_SIZE / 2) c = 'S';
      mid += ' '; mid += c; mid += ' ';
    }
    Tile &e = g[x1][y];
    top += "+"; mid += (e.getVisited() && e.getWall(EAST)) ? "|" : " ";
    std::printf("%s\n%s\n", top.c_str(), mid.c_str());
  }
  std::string bot;
  for (int x = x0; x <= x1; x++) bot += (g[x][y0].getVisited() && g[x][y0].getWall(SOUTH)) ? "+---" : "+   ";
  std::printf("%s+\n", bot.c_str());
}

static std::string pct(double v) { char b[16]; std::snprintf(b, sizeof b, "%.0f%%", v * 100); return b; }

// ============================================================
int main(int argc, char **argv) {
  int runs = 500;
  long seed = -1;
  std::string only = "all", robot = "perfect";
  bool show = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) seed = std::atol(argv[++i]);
    else if (a == "--scenario" && i + 1 < argc) only = argv[++i];
    else if (a == "--robot" && i + 1 < argc) robot = argv[++i];
    else if (a == "--show") show = true;
    else if (a == "--verbose") serialEcho = true;
    else { std::printf("usage: sim [--robot perfect|realistic|harsh] [--runs N] [--scenario flat|loops|big|ramp|bigramp|all] [--seed S] [--show] [--verbose]\n"); return 2; }
  }
  bool found = false;
  for (const Realism &p : REALISM) if (robot == p.name) { prof = p; found = true; }
  if (!found) { std::printf("unknown --robot %s (perfect, realistic or harsh)\n", robot.c_str()); return 2; }
  perfectRobot = robot == "perfect";

  if (seed < 0) std::printf("robot: %s%s\n\n", prof.name, perfectRobot ? " (logic test: every run must pass)" : " (faults on: expect failures, compare the numbers)");
  int totalFail = 0;
  for (const Scenario &sc : SCENARIOS) {
    if (only != "all" && only != sc.name) continue;
    int n = seed >= 0 ? 1 : runs, pass = 0, home = 0, full = 0, timeRet = 0, lost = 0, lostEnd = 0, crossed = 0, lops = 0, firstFailSeed = -1;
    double cov = 0, acc = 0;
    std::map<std::string, int> reasons;
    for (int k = 0; k < n; k++) {
      long s = seed >= 0 ? seed : k + 1;
      std::mt19937 rng((unsigned)s * 2654435761u + (unsigned)(&sc - SCENARIOS));
      noise.seed((unsigned)s * 7919u + 12345u);
      world = generate(sc, rng);
      RunResult r = runMaze();
      pass += r.pass; home += r.home; full += r.fullCov; timeRet += r.timeReturn; lost += r.lost; lostEnd += r.lostAtEnd;
      crossed += r.rampCrossings > 0; lops += r.lops; cov += r.coverage; acc += r.mapAcc;
      if (!r.pass) { reasons[r.reason]++; if (firstFailSeed < 0) firstFailSeed = (int)s; }
      if (seed >= 0) {
        std::printf("%s / %s / seed %ld: %s%s\n  time %d:%02d  explore moves %d  return moves %d  short moves %d  black stops %d  restarts %d  ramp crossings %d\n"
                    "  coverage %s%s  map walls correct %s  %s\n",
                    sc.name, prof.name, s, r.pass ? "PASS" : "FAIL: ", r.pass ? "" : r.reason.c_str(),
                    (int)r.time / 60, (int)r.time % 60, r.moves, r.returnMoves, r.shorts, r.blackStops, r.lops, r.rampCrossings,
                    pct(r.coverage).c_str(), r.timeReturn ? " (headed home early for time)" : "", pct(r.mapAcc).c_str(),
                    r.lost ? (r.lostAtEnd ? "got lost, never recovered" : "got lost, recovered") : "never lost");
        if (show) { printWorld(); for (int f = 0; f < NUM_FLOORS; f++) printMapFloor(f); }
      }
    }
    if (seed < 0) {
      std::printf("%-8s %4d/%-4d passed (%s)   home in time %s   full coverage %s   avg coverage %s   map walls %s   lost %s (still lost at end %s)   restarts/run %.2f",
                  sc.name, pass, n, pct((double)pass / n).c_str(), pct((double)home / n).c_str(), pct((double)full / n).c_str(),
                  pct(cov / n).c_str(), pct(acc / n).c_str(), pct((double)lost / n).c_str(), pct((double)lostEnd / n).c_str(), (double)lops / n);
      if (sc.ramps) std::printf("   ramp crossed in %d", crossed);
      std::printf("\n");
      if (!reasons.empty()) {
        std::vector<std::pair<int, std::string>> sorted;
        for (auto &kv : reasons) sorted.push_back({kv.second, kv.first});
        std::sort(sorted.rbegin(), sorted.rend());
        for (size_t i = 0; i < sorted.size() && i < 4; i++) std::printf("           %5dx %s\n", sorted[i].first, sorted[i].second.c_str());
        std::printf("           replay: --robot %s --scenario %s --seed %d --show\n", prof.name, sc.name, firstFailSeed);
      }
    }
    totalFail += n - pass;
  }
  if (seed < 0) {
    if (perfectRobot) std::printf(totalFail ? "\nLOGIC FAILURES: %d\n" : "\nALL PASSED\n", totalFail);
    else std::printf("\n(with faults on, failures are expected; use the numbers to compare code changes)\n");
  }
  return perfectRobot && totalFail ? 1 : 0;
}
