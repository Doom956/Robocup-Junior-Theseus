// Maze simulator for the robot's REAL navigation code.
//
// Compiles Main/navigation.cpp + Main/MazeTile.cpp unchanged (hardware headers are
// replaced by tools/sim/stubs) and drives them through random RCJ-style mazes:
// walls with loops, black, blue and silver tiles, and ramps between two levels.
//
// Real code under test: planExploreDir, BFS, readWallsRel/checkTileMismatch,
// writeWallsToCurrentTile, updateFullyExploredAt, markEdgeBothWays, stepForward,
// elevation/descend/neighbourFloor, syncActiveFloor.
// Copied here (they live in hardware files): the map bookkeeping around a move
// from main.cpp (SENSE_TILE/UPDATE_MAP, finishTileMove, RETURN) and the ramp loop
// at the end of fwd() in movement.cpp. Keep those in sync if you change them.
//
// The simulated robot moves perfectly and its sensors never lie, so this checks
// the navigation LOGIC, not motors, sensors or tuning.
//
// Each run checks:
//   - every reachable tile was visited (black tiles avoided)
//   - the robot never planned a move into a real wall
//   - x_pos/y_pos and currentFloor always match where the robot really is
//   - every wall stored in the map matches the real maze
//   - the robot gets back to the start tile
//
// Usage: sim.exe [--runs N] [--seed S] [--scenario flat|loops|big|ramp|all] [--show] [--verbose]
//   --seed S --scenario X --show   replays one maze and prints the real maze and the robot's map

#include "Globals.h"
#include <deque>
#include <vector>
#include <random>
#include <string>

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
// The real maze
// ============================================================
static const int SDX[4] = {0, 1, 0, -1}; // N E S W, same as stepForward()
static const int SDY[4] = {1, 0, -1, 0};
enum CellType { W_FLOOR, W_BLACK, W_BLUE, W_SILVER, W_RAMP, W_SOLID };

struct World {
  int W = 0, H = 0, sx = 0, sy = 0;
  std::vector<std::array<bool, 4>> wall;
  std::vector<int> type, level, region;
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
};

struct Scenario {
  const char *name;
  int minW, maxW, minH, maxH;
  int extraOpenPct; // extra wall openings (loops), % of cells
  int blackPct, bluePct, silverPct;
  bool ramp;
};

static const Scenario SCENARIOS[] = {
  {"flat",  5, 8,  5, 8,  0,  6, 4, 4, false},
  {"loops", 5, 9,  5, 9,  25, 6, 4, 4, false},
  {"big",   12, 16, 12, 16, 20, 5, 3, 3, false},
  {"ramp",  4, 7,  4, 8,  20, 5, 3, 3, true},
};

static World generate(const Scenario &sc, std::mt19937 &rng) {
  auto rnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  World w;
  int rampLen = sc.ramp ? rnd(1, 3) : 0;
  int wA = rnd(sc.minW, sc.maxW), wB = sc.ramp ? rnd(sc.minW, sc.maxW) : 0;
  w.W = wA + rampLen + wB;
  w.H = rnd(sc.minH, sc.maxH);
  int n = w.W * w.H;
  w.wall.assign(n, {{true, true, true, true}});
  w.type.assign(n, W_FLOOR);
  w.level.assign(n, 0);
  w.region.assign(n, 0);
  int rampRow = rnd(0, w.H - 1);
  int levelA = sc.ramp ? rnd(0, 1) : 0, levelB = 1 - levelA;
  for (int y = 0; y < w.H; y++)
    for (int x = 0; x < w.W; x++) {
      int i = w.id(x, y);
      if (x < wA) { w.region[i] = 0; w.level[i] = levelA; }
      else if (x < wA + rampLen) { w.region[i] = 2; w.type[i] = (y == rampRow) ? W_RAMP : W_SOLID; }
      else { w.region[i] = 1; w.level[i] = levelB; }
    }

  // carve a perfect maze inside each area, then knock out extra walls for loops
  for (int reg = 0; reg < (sc.ramp ? 2 : 1); reg++) {
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
  // ramp corridor: open along its length, walls on both sides
  if (sc.ramp) for (int x = wA - 1; x < wA + rampLen; x++) w.open(x, rampRow, EAST);

  // start tile, then black/blue/silver on plain tiles (never on start or next to a ramp end)
  std::vector<int> plain;
  for (int i = 0; i < n; i++) if (w.type[i] == W_FLOOR) plain.push_back(i);
  int s = plain[rnd(0, (int)plain.size() - 1)];
  w.sx = s % w.W; w.sy = s / w.W;
  auto nearRampEnd = [&](int i) {
    if (!sc.ramp) return false;
    int x = i % w.W, y = i / w.W;
    return y == rampRow && (x == wA - 1 || x == wA + rampLen);
  };
  std::shuffle(plain.begin(), plain.end(), rng);
  int nb = n * sc.blackPct / 100, nbl = n * sc.bluePct / 100, nsi = n * sc.silverPct / 100;
  for (int i : plain) {
    if (i == s || nearRampEnd(i)) continue;
    if (nb > 0) { w.type[i] = W_BLACK; nb--; }
    else if (nbl > 0) { w.type[i] = W_BLUE; nbl--; }
    else if (nsi > 0) { w.type[i] = W_SILVER; nsi--; }
  }
  return w;
}

// ============================================================
// Simulated robot
// ============================================================
static World world;
static int rx, ry; // where the robot really is

// Distance.cpp stand-in: dir is relative (0 front, 1 right, 2 back, 3 left); 0 = wall
int detectWall(int dir) {
  int absDir = (currentDir + dir) % 4;
  return world.hasWall(rx, ry, absDir) ? 0 : 1;
}

struct RunResult {
  bool ok = true;
  std::string reason;
  int moves = 0, returnMoves = 0, blackHits = 0, rampCrossings = 0;
  void fail(const std::string &r) { if (ok) { ok = false; reason = r; } }
};

static int expectedFloor(int x, int y) { return START_FLOOR + world.level[world.id(x, y)] - world.level[world.id(world.sx, world.sy)]; }

enum MoveResult { MOVED, STOPPED_BLACK, HIT_WALL };

// One tile forward in currentDir: what fwd() + finishTileMove() do to the map.
static int rampCrossings = 0;
static MoveResult moveForward() {
  if (world.hasWall(rx, ry, currentDir)) return HIT_WALL;
  int nx = rx + SDX[currentDir], ny = ry + SDY[currentDir];
  if (world.typeAt(nx, ny) == W_BLACK) {
    // fwd(): sees black, marks the tile ahead BLACK, backs up
    int mx = x_pos, my = y_pos;
    stepForward(currentDir, mx, my);
    if (inBounds(mx, my)) mapGrid[mx][my].setType(BLACK);
    return STOPPED_BLACK;
  }
  if (world.typeAt(nx, ny) == W_RAMP) {
    // fwd() ramp loop (movement.cpp): one map tile per ramp section, floor change on the first
    int cnt = 0;
    while (world.typeAt(rx + SDX[currentDir] * (cnt + 1), ry + SDY[currentDir] * (cnt + 1)) == W_RAMP) cnt++;
    int ax = rx + SDX[currentDir] * (cnt + 1), ay = ry + SDY[currentDir] * (cnt + 1);
    bool upwards = world.level[world.id(ax, ay)] > world.level[world.id(rx, ry)];
    for (int i = 0; i < cnt; i++) {
      markEdgeBothWays(x_pos, y_pos, currentDir);
      stepForward(currentDir, x_pos, y_pos);
      writeWallsToCurrentTile(0, 1, 0, 1);
      updateFullyExploredAt(x_pos, y_pos);
      if (i == 0) {
        if (upwards) elevation(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
        else descend(mapGrid, x_pos, y_pos, m1, m2, m3, currentFloor);
      }
    }
    rx += SDX[currentDir] * cnt;
    ry += SDY[currentDir] * cnt;
    rampCrossings++;
  }
  // finishTileMove() (main.cpp)
  markEdgeBothWays(x_pos, y_pos, currentDir);
  stepForward(currentDir, x_pos, y_pos);
  rx += SDX[currentDir];
  ry += SDY[currentDir];
  int t = world.typeAt(rx, ry);
  if (t == W_SILVER) {
    mapGrid[x_pos][y_pos].setType(CHECKPOINT);
    x_checkpoint = x_pos; y_checkpoint = y_pos; floor_checkpoint = currentFloor;
  }
  if (t == W_BLUE) mapGrid[x_pos][y_pos].setType(BLUE);
  return MOVED;
}

static int mapX(int wx) { return wx - world.sx + MAP_SIZE / 2; }
static int mapY(int wy) { return wy - world.sy + MAP_SIZE / 2; }

static void checkPosition(RunResult &r, const char *when) {
  if (x_pos != mapX(rx) || y_pos != mapY(ry))
    r.fail(std::string("map position drifted from real position (") + when + ")");
  int ef = expectedFloor(rx, ry);
  if (ef >= 0 && ef < NUM_FLOORS && currentFloor != ef)
    r.fail(std::string("currentFloor wrong (") + when + ")");
}

static std::vector<int> standFloor; // floor the robot was on when it sensed each real tile (-1 = never)

static void senseTile(RunResult &r) {
  // SENSE_TILE + UPDATE_MAP (main.cpp)
  bool f, rt, b, l;
  readWallsRel(f, rt, b, l);
  bool mismatch = checkTileMismatch(f, rt, b, l);
  if (!mismatch) writeWallsToCurrentTile(f, rt, b, l);
  else r.fail("checkTileMismatch fired with perfect sensors (position bookkeeping bug)");
  updateFullyExploredAt(x_pos, y_pos);
  standFloor[world.id(rx, ry)] = currentFloor;
}

static RunResult runMaze() {
  RunResult r;
  initializeMap();
  m1 = mapGrid; m2 = mapGrid; m3 = mapGrid;
  currentFloor = START_FLOOR;
  x_pos = MAP_SIZE / 2; y_pos = MAP_SIZE / 2;
  mapGrid[x_pos][y_pos].setDiscovered(true);
  currentDir = NORTH;
  rx = world.sx; ry = world.sy;
  standFloor.assign(world.W * world.H, -1);
  rampCrossings = 0;
  const int MOVE_LIMIT = 6 * world.W * world.H + 50;

  // ---- exploration: SENSE_TILE -> PLAN_NEXT -> EXECUTE_MOVE ----
  senseTile(r);
  while (r.ok) {
    Direction next;
    if (!planExploreDir(next)) break; // maze explored -> RETURN
    currentDir = next;                // turns always succeed here
    MoveResult m = moveForward();
    r.moves++;
    if (m == HIT_WALL) { r.fail("planner chose a direction with a real wall"); break; }
    if (m == STOPPED_BLACK) { r.blackHits++; continue; } // BACKPEDAL -> PLAN_NEXT
    checkPosition(r, "exploring");
    senseTile(r);
    if (r.moves > MOVE_LIMIT) { r.fail("exploration did not finish (move limit)"); break; }
  }

  // ---- RETURN: re-plan home one tile at a time ----
  const std::pair<int, std::pair<int, int>> HOME = {START_FLOOR, {MAP_SIZE / 2, MAP_SIZE / 2}};
  while (r.ok && !(currentFloor == HOME.first && x_pos == HOME.second.first && y_pos == HOME.second.second)) {
    syncActiveFloor();
    std::pair<int, std::pair<int, int>> cur = {currentFloor, {x_pos, y_pos}};
    auto path = BFS(cur, m1, m2, m3, HOME, false, false);
    if (path.empty()) path = BFS(cur, m1, m2, m3, HOME, true, false);
    if (path.empty()) path = BFS(cur, m1, m2, m3, HOME, true, true);
    if (path.size() < 2) { r.fail("RETURN: no path home"); break; }
    int dx = path[1].second.first - path[0].second.first;
    int dy = path[1].second.second - path[0].second.second;
    currentDir = (dy == 0) ? (dx == 1 ? EAST : WEST) : (dy == 1 ? NORTH : SOUTH);
    MoveResult m = moveForward();
    r.returnMoves++;
    if (m == HIT_WALL) { r.fail("RETURN: path went through a real wall"); break; }
    if (m == STOPPED_BLACK) { r.fail("RETURN: path went onto a black tile"); break; }
    checkPosition(r, "returning");
    if (r.returnMoves > MOVE_LIMIT) { r.fail("RETURN: did not reach home (move limit)"); break; }
  }
  if (r.ok && (rx != world.sx || ry != world.sy)) r.fail("RETURN: robot thinks it is home but is not");
  syncActiveFloor();
  r.rampCrossings = rampCrossings;

  // ---- coverage: every tile reachable without crossing black ----
  if (r.ok) {
    std::vector<bool> reach(world.W * world.H, false);
    std::deque<std::pair<int, int>> q;
    q.push_back({world.sx, world.sy});
    reach[world.id(world.sx, world.sy)] = true;
    while (!q.empty()) {
      int x = q.front().first, y = q.front().second;
      q.pop_front();
      for (int d = 0; d < 4; d++) {
        int nx = x + SDX[d], ny = y + SDY[d];
        if (world.hasWall(x, y, d) || !world.in(nx, ny) || reach[world.id(nx, ny)]) continue;
        int t = world.typeAt(nx, ny);
        if (t == W_BLACK || t == W_SOLID) continue;
        reach[world.id(nx, ny)] = true;
        q.push_back({nx, ny});
      }
    }
    for (int i = 0; i < world.W * world.H && r.ok; i++)
      if (reach[i] && world.type[i] != W_RAMP && standFloor[i] < 0) r.fail("a reachable tile was never visited");
  }
  // ---- map accuracy: stored walls == real walls on every visited tile ----
  for (int i = 0; i < world.W * world.H && r.ok; i++) {
    if (standFloor[i] < 0) continue;
    int x = i % world.W, y = i / world.W;
    Tile &t = floorGrid(standFloor[i])[mapX(x)][mapY(y)];
    for (int d = 0; d < 4; d++)
      if (t.getWall(d) != world.hasWall(x, y, d)) { r.fail("map wall does not match the real maze"); break; }
  }
  return r;
}

// ============================================================
// ASCII output (--show)
// ============================================================
static void printWorld() {
  std::printf("\nReal maze (S start, X black, b blue, s silver, R ramp, # solid, . = upper level):\n");
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
      if (t.getElevate()) c = '^'; if (t.getDescend()) c = 'v';
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

// ============================================================
int main(int argc, char **argv) {
  int runs = 500;
  long seed = -1;
  std::string only = "all";
  bool show = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) seed = std::atol(argv[++i]);
    else if (a == "--scenario" && i + 1 < argc) only = argv[++i];
    else if (a == "--show") show = true;
    else if (a == "--verbose") serialEcho = true;
    else { std::printf("usage: sim [--runs N] [--seed S] [--scenario flat|loops|big|ramp|all] [--show] [--verbose]\n"); return 2; }
  }

  int totalFail = 0;
  for (const Scenario &sc : SCENARIOS) {
    if (only != "all" && only != sc.name) continue;
    int n = seed >= 0 ? 1 : runs, pass = 0, sumMoves = 0, firstFailSeed = -1, crossedRuns = 0;
    std::string firstReason;
    for (int k = 0; k < n; k++) {
      long s = seed >= 0 ? seed : k + 1;
      std::mt19937 rng((unsigned)s * 2654435761u + (unsigned)(&sc - SCENARIOS));
      world = generate(sc, rng);
      RunResult r = runMaze();
      if (r.ok) { pass++; sumMoves += r.moves + r.returnMoves; }
      else if (firstFailSeed < 0) { firstFailSeed = (int)s; firstReason = r.reason; }
      if (r.rampCrossings > 0) crossedRuns++;
      if (seed >= 0) {
        std::printf("scenario %s seed %ld: %s%s  (explore moves %d, black stops %d, ramp crossings %d, return moves %d)\n", sc.name, s,
                    r.ok ? "PASS" : "FAIL: ", r.ok ? "" : r.reason.c_str(), r.moves, r.blackHits, r.rampCrossings, r.returnMoves);
        if (show) {
          printWorld();
          for (int f = 0; f < NUM_FLOORS; f++) printMapFloor(f);
        }
      }
    }
    if (seed < 0) {
      std::printf("%-6s %4d/%-4d passed", sc.name, pass, n);
      if (pass) std::printf("   avg %.1f moves/run", (double)sumMoves / pass);
      if (sc.ramp) std::printf("   ramp crossed in %d runs", crossedRuns);
      if (firstFailSeed >= 0) std::printf("   first failure: seed %d (%s)", firstFailSeed, firstReason.c_str());
      std::printf("\n");
    }
    totalFail += n - pass;
  }
  if (seed < 0) std::printf(totalFail ? "\nFAILURES: %d (replay one with --seed S --scenario NAME --show)\n" : "\nALL PASSED\n", totalFail);
  return totalFail ? 1 : 0;
}
