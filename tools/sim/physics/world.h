// Random RCJ-style fields for the physics simulator (same generator as tools/sim/sim.cpp,
// plus ramp heights). Tiles are 300 mm; walls sit on tile edges.
#pragma once
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <random>
#include <vector>
#include <algorithm>

namespace field {

const double TILE = 300.0;
const int DX[4] = {0, 1, 0, -1}; // N E S W
const int DY[4] = {1, 0, -1, 0};
enum CellType { FLOOR, BLACK_T, BLUE_T, SILVER_T, RAMP_T, SOLID_T };
struct Obstacle { double x, y, r; }; // upright cylinder, centre and radius in mm (RCJ 3.4.3: >= 15 cm tall)
// what the organisers add on top of walls and floor colours (RCJ 2026, 3.1 and 3.4)
struct Extras {
  double wallT = 20;        // wall thickness: two facing walls leave a 280 mm path (3.3.3)
  double obstacleRate = 0;  // obstacles per plain tile
  double bumpRate = 0;      // speed bumps per plain tile
  double bumpH = 10;        // speed bump height, mm (3.4.1: at most 1 cm)
  double bumpW = 40;        // speed bump width across, mm
};

struct World {
  int W = 0, H = 0, sx = 0, sy = 0;
  std::vector<std::array<bool, 4>> wall;
  std::vector<int> type, level;
  // ramp corridor (columns rampX0 .. rampX0+rampLen-1), heights in mm
  int rampX0 = 0, rampLen = 0;
  double rampDeg = 0, levelHeight[2] = {0, 0};
  std::vector<Obstacle> obstacles;
  std::vector<int> bump;     // per tile: 0 none, 1 = strip running east-west (crossed going N/S), 2 = running north-south
  double bumpH = 10, bumpW = 40;
  int id(int x, int y) const { return y * W + x; }
  bool in(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
  bool hasWall(int x, int y, int d) const { return !in(x, y) || wall[id(x, y)][d]; }
  int typeAt(int x, int y) const { return in(x, y) ? type[id(x, y)] : SOLID_T; }
  void open(int x, int y, int d) {
    int nx = x + DX[d], ny = y + DY[d];
    if (!in(x, y) || !in(nx, ny)) return;
    wall[id(x, y)][d] = false;
    wall[id(nx, ny)][(d + 2) % 4] = false;
  }
  // floor height (mm) at a point; ramps rise linearly between the two levels
  double height(double px, double py) const {
    int x = (int)std::floor(px / TILE), y = (int)std::floor(py / TILE);
    if (!in(x, y)) return 0;
    if (type[id(x, y)] == RAMP_T) {
      double f = (px - rampX0 * TILE) / (rampLen * TILE);
      f = f < 0 ? 0 : f > 1 ? 1 : f;
      double hA = levelHeight[level[id(rampX0 - 1, y)]], hB = levelHeight[level[id(rampX0 + rampLen, y)]];
      return hA + (hB - hA) * f;
    }
    if (type[id(x, y)] == SOLID_T) return 0;
    double h = levelHeight[level[id(x, y)]];
    int b = bump.empty() ? 0 : bump[id(x, y)];
    if (b) { // speed bump across the middle of the tile, smooth profile
      double d = std::fabs(b == 1 ? py - (y + 0.5) * TILE : px - (x + 0.5) * TILE);
      if (d < bumpW / 2) h += bumpH * 0.5 * (1 + std::cos(2 * M_PI * d / bumpW));
    }
    return h;
  }
  std::vector<bool> reach() const {
    std::vector<bool> r(W * H, false);
    std::deque<std::pair<int, int>> q;
    q.push_back({sx, sy});
    r[id(sx, sy)] = true;
    while (!q.empty()) {
      int x = q.front().first, y = q.front().second;
      q.pop_front();
      for (int d = 0; d < 4; d++) {
        int nx = x + DX[d], ny = y + DY[d];
        if (hasWall(x, y, d) || r[id(nx, ny)]) continue;
        int t = typeAt(nx, ny);
        if (t == BLACK_T || t == SOLID_T) continue;
        r[id(nx, ny)] = true;
        q.push_back({nx, ny});
      }
    }
    return r;
  }
  bool allReachable() const {
    std::vector<bool> r = reach();
    for (int i = 0; i < W * H; i++)
      if (!r[i] && type[i] != BLACK_T && type[i] != SOLID_T) return false;
    return true;
  }
};

struct Scenario {
  const char *name;
  int minW, maxW, minH, maxH, extraOpenPct, blackPct, bluePct, silverPct, ramps;
};
static const Scenario SCENARIOS[] = {
  {"flat",    5, 8,   5, 8,   0,  6, 4, 4, 0},
  {"loops",   5, 9,   5, 9,   25, 6, 4, 4, 0},
  {"big",     12, 16, 12, 16, 20, 5, 3, 3, 0},
  {"ramp",    4, 7,   4, 8,   20, 5, 3, 3, 1},
  {"bigramp", 7, 8,   10, 14, 20, 5, 3, 4, 2},
};

inline void addExtras(World &w, std::mt19937 &rng, const Extras &ex);

inline World generate(const Scenario &sc, std::mt19937 &rng, double rampMinDeg, double rampMaxDeg, const Extras &ex = Extras()) {
  auto rnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  World w;
  int rampLen = sc.ramps ? rnd(1, 3) : 0;
  int wA = rnd(sc.minW, sc.maxW), wB = sc.ramps ? rnd(sc.minW, sc.maxW) : 0;
  w.W = wA + rampLen + wB;
  w.H = rnd(sc.minH, sc.maxH);
  int n = w.W * w.H;
  w.wall.assign(n, {{true, true, true, true}});
  w.type.assign(n, FLOOR);
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
      else if (x < wA + rampLen) w.type[i] = isRampRow(y) ? RAMP_T : SOLID_T;
      else w.level[i] = 1 - levelA;
    }
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
        int nx = x + DX[d], ny = y + DY[d];
        if (nx >= x0 && nx < x1 && ny >= 0 && ny < w.H && !seen[w.id(nx, ny)]) opts[k++] = d;
      }
      if (!k) { st.pop_back(); continue; }
      int d = opts[rnd(0, k - 1)];
      w.open(x, y, d);
      seen[w.id(x + DX[d], y + DY[d])] = true;
      st.push_back({x + DX[d], y + DY[d]});
    }
    int extra = (x1 - x0) * w.H * sc.extraOpenPct / 100;
    for (int e = 0; e < extra; e++) {
      int x = rnd(x0, x1 - 1), y = rnd(0, w.H - 1), d = rnd(0, 3);
      int nx = x + DX[d], ny = y + DY[d];
      if (nx >= x0 && nx < x1 && ny >= 0 && ny < w.H) w.open(x, y, d);
    }
  }
  for (int r : rows) for (int x = wA - 1; x < wA + rampLen; x++) w.open(x, r, 1);
  w.rampX0 = wA;
  w.rampLen = rampLen;
  if (sc.ramps) {
    w.rampDeg = std::uniform_real_distribution<double>(rampMinDeg, rampMaxDeg)(rng);
    w.levelHeight[0] = 0;
    w.levelHeight[1] = rampLen * TILE * std::tan(w.rampDeg * M_PI / 180.0);
  }
  std::vector<int> plain;
  for (int i = 0; i < n; i++) if (w.type[i] == FLOOR) plain.push_back(i);
  int s = plain[rnd(0, (int)plain.size() - 1)];
  w.sx = s % w.W; w.sy = s / w.W;
  auto nearRampEnd = [&](int i) { int x = i % w.W, y = i / w.W; return isRampRow(y) && (x == wA - 1 || x == wA + rampLen); };
  std::shuffle(plain.begin(), plain.end(), rng);
  int nb = n * sc.blackPct / 100, nbl = n * sc.bluePct / 100, nsi = n * sc.silverPct / 100;
  for (int i : plain) {
    if (i == s || nearRampEnd(i)) continue;
    if (nb > 0) { w.type[i] = BLACK_T; if (w.allReachable()) nb--; else w.type[i] = FLOOR; }
    else if (nbl > 0) { w.type[i] = BLUE_T; nbl--; }
    else if (nsi > 0) { w.type[i] = SILVER_T; nsi--; }
  }
  // drawn from their own generator, so walls and colours stay the same as without them
  std::mt19937 extraRng(rng());
  addExtras(w, extraRng, ex);
  return w;
}

// Speed bumps (RCJ 3.4.1-2: fixed, <= 1 cm, not on ramps or stairs) and obstacles (3.4.3-4:
// >= 15 cm tall; either at least 20 cm from every wall, or touching a wall and at least 20 cm
// from the opposite edge of the tile and from other obstacles). Only on plain tiles, not the
// start or the tiles at the ends of a ramp.
inline void addExtras(World &w, std::mt19937 &rng, const Extras &ex) {
  auto uni = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
  int n = w.W * w.H;
  w.bump.assign(n, 0);
  w.bumpH = ex.bumpH; w.bumpW = ex.bumpW;
  std::vector<int> plain;
  for (int i = 0; i < n; i++) {
    int x = i % w.W, y = i / w.W;
    bool rampEnd = (x == w.rampX0 - 1 || x == w.rampX0 + w.rampLen) && w.rampLen &&
                   ((x + 1 < w.W && w.type[w.id(x + 1, y)] == RAMP_T) || (x > 0 && w.type[w.id(x - 1, y)] == RAMP_T));
    if (w.type[i] == FLOOR && !(x == w.sx && y == w.sy) && !rampEnd) plain.push_back(i);
  }
  std::shuffle(plain.begin(), plain.end(), rng);
  int bumps = (int)std::lround(plain.size() * ex.bumpRate), obstacles = (int)std::lround(plain.size() * ex.obstacleRate);
  std::vector<int> rest;
  for (int i : plain) {
    if (bumps > 0) { w.bump[i] = 1 + (int)(rng() % 2); bumps--; }
    else rest.push_back(i);
  }
  const double S = TILE, h = ex.wallT / 2;
  auto clearOfOthers = [&](double x, double y, double r) {
    for (const Obstacle &o : w.obstacles)
      if (std::hypot(o.x - x, o.y - y) - o.r - r < 200) return false;
    return true;
  };
  for (int i : rest) {
    if (obstacles <= 0) break;
    int tx = i % w.W, ty = i / w.W;
    double r = uni(25, 45);
    // in the open: at least 20 cm from every wall face
    double cx = (tx + 0.5) * S, cy = (ty + 0.5) * S;
    bool open = true;
    for (int y = ty - 2; y <= ty + 2 && open; y++)
      for (int x = tx - 2; x <= tx + 2 && open; x++)
        for (int d = 0; d < 4 && open; d++) {
          if (!w.hasWall(x, y, d)) continue;
          double ex0 = x * S, ey0 = y * S, ex1 = ex0 + S, ey1 = ey0 + S;
          if (d == 0) ey0 = ey1; else if (d == 2) ey1 = ey0; else if (d == 1) ex0 = ex1; else ex1 = ex0;
          double qx = std::max(ex0, std::min(cx, ex1)), qy = std::max(ey0, std::min(cy, ey1));
          if (std::hypot(cx - qx, cy - qy) - h - r < 200) open = false;
        }
    if (open && clearOfOthers(cx, cy, r)) { w.obstacles.push_back({cx, cy, r}); obstacles--; continue; }
    // against a wall of this tile, reaching at most 10 cm in (20 cm left to the opposite edge)
    int sides[4], k = 0;
    for (int d = 0; d < 4; d++) if (w.wall[i][d]) sides[k++] = d;
    if (!k) continue;
    int d = sides[rng() % k];
    double along = uni(r + h, S - r - h), off = h + r;
    if (d == 0) { cx = tx * S + along; cy = (ty + 1) * S - off; }
    else if (d == 2) { cx = tx * S + along; cy = ty * S + off; }
    else if (d == 1) { cx = (tx + 1) * S - off; cy = ty * S + along; }
    else { cx = tx * S + off; cy = ty * S + along; }
    if (clearOfOthers(cx, cy, r)) { w.obstacles.push_back({cx, cy, r}); obstacles--; }
  }
}

} // namespace field
