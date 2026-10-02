// Random RCJ-style fields for the physics simulator (same generator as tools/sim/sim.cpp,
// plus ramp heights). Tiles are 300 mm; walls sit on tile edges.
#pragma once
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <random>
#include <vector>

namespace field {

const double TILE = 300.0;
const int DX[4] = {0, 1, 0, -1}; // N E S W
const int DY[4] = {1, 0, -1, 0};
enum CellType { FLOOR, BLACK_T, BLUE_T, SILVER_T, RAMP_T, SOLID_T };

struct World {
  int W = 0, H = 0, sx = 0, sy = 0;
  std::vector<std::array<bool, 4>> wall;
  std::vector<int> type, level;
  // ramp corridor (columns rampX0 .. rampX0+rampLen-1), heights in mm
  int rampX0 = 0, rampLen = 0;
  double rampDeg = 0, levelHeight[2] = {0, 0};
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
    return levelHeight[level[id(x, y)]];
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

inline World generate(const Scenario &sc, std::mt19937 &rng, double rampMinDeg, double rampMaxDeg) {
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
  return w;
}

} // namespace field
