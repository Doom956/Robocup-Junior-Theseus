// Physics simulator: runs the robot's real firmware (Main/*.cpp, unchanged) against a
// simulated robot on a random RCJ field.
//
// The Arduino/hardware libraries are replaced (tools/sim/physics/hw) by versions backed by:
//   - a simulated clock: millis/micros/delay, and every hardware call costs roughly its real time
//   - 4 DC motors (dead band, lag, per-motor gain), skid-steer driving, walls that block the robot
//   - encoders counting real wheel rotation (wheel slip shows up as encoder error)
//   - 7 VL53L0X beams cast from the CAD positions (tools/sim/cad/robot_geometry.json)
//   - BNO055 heading (with drift and noise) and pitch from the ramp under the wheels
//   - TCS34725 colour of the tile under its CAD position
//   - a referee: lack-of-progress restart (pause switch) on black or when stuck
//
// Usage: physics.exe --scenario NAME --seed S [--ideal] [--verbose] [--trace FILE] [--live [--live-speed X]]
// Prints one JSON line with the result. --trace writes a recording of the run (one JSON object per
// line) that viewer.html plays back. --live writes the same recording to stdout while the run happens,
// paced to the wall clock, and takes commands on stdin (pause, resume, speed X, lop, nudge, stop).

#include "Globals.h"
#include "world.h"
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <random>
#include <sstream>
#include <string>

// =====================================================================================
// Assumptions (tune these to the real robot; see README)
// =====================================================================================
struct Params {
  // Motors: Pololu 195.3125:1 Metal Gearmotor 20Dx44L mm 12V (#3493) on a Carobot V3 shield
  // (TB6612FNG + PCA9685, like the Adafruit Motor Shield V2), powered by a 3S LiPo.
  double batteryVoltage = 11.76;  // 3S LiPo, measured before the bench tests (2026-10-06)
  double noLoadRpmAt12V = 72;     // datasheet
  double stallKgcmAt12V = 10;     // datasheet (extrapolated); gearbox limit is 5 kg.cm
  double frictionFracAt12V = 0.05; // no-load current / stall current = 80 mA / 1.6 A (datasheet)
  double stictionFactor = 1.3;    // extra friction to start from standstill
  // Motor A (left front) only: extra friction at low power, fraction of stall torque, fading to none at
  // PWM 128. FITTED to bench mode on the robot (2026-10-06, 11.76 V): test 4 needed PWM 49-51 before wheel A
  // counted forward, and even then A counted 42-56 against 250-340 for B and D (it feels free by hand, so motor
  // or driver channel); at PWM 150 (test 3) A counted like the others (1010 vs 984-1105). 0.16 gives --selftest
  // tests 2/3/4 = 30 / 93 deg / 51 (robot 25-30 / 92-97 deg / 49-51); 0 = motor A like the others.
  double motorAExtraFriction = 0.16;
  double robotMassKg = 1.15;      // ESTIMATE from the parts' datasheets + printed parts (README "Robot mass") - weigh it
  double driverOhms = 0.5;        // TB6612FNG output ON resistance, upper + lower, typ (Toshiba datasheet); motor 12 V / 1.6 A = 7.5 ohm
  // wheelMu and skidFactor FITTED to the robot's bench mode (Main/bench.cpp, 3 runs on 2026-10-06): lowest
  // turnright() PWM that turns it 5 deg in 1 s 25/30/25 (sim 25), turnright(150) for 1 s 97/92/95 deg (sim 95).
  // (Before: 0.8 and 1.3, assumed, gave 45 and 70.) wheelMu is the sideways drag when turning on the spot; it is
  // also the grip for pushing a loose obstacle (off by default).
  double wheelMu = 0.2;
  double motorTau = 0.05;         // motor + gearbox speed time constant, s
  double motorGainSigma = 0.03;   // per-motor speed difference (fraction)
  double trackWidth = 156;        // left-right wheel spacing, mm (CAD)
  double skidFactor = 1.1;        // skid steering turns slower than the ideal track predicts (fitted, see wheelMu)
  double tractionMean = 0.97, tractionSigma = 0.02, tractionTau = 0.5; // wheel grip (ground speed / wheel speed)
  double wallNudgeMm = 0.3;       // how far a wall can shove a turning robot sideways, mm per ms (0 = it jams)
  double wheelDiameter = 80;
  double encoderCountsPerRev = 5 * 195.3125; // 20 CPR encoder, code counts rising edges of one channel = 5 per motor turn
  // Gyro: "magnetic" = the BNO055 reports heading from magnetic north (team notes: "the heading is
  // always global") instead of from the start direction. Maze-to-north angle, deg; -1 = random per run.
  // -1 = do what the firmware asks for: bno.begin() defaults to NDOF, which the BNO055 datasheet (3.3.3.5)
  // defines as absolute orientation (heading from magnetic north); IMUPLUS is relative to the start.
  // In NDOF the heading only becomes magnetic once the magnetometer is calibrated, which needs the robot
  // turned through many orientations. The real robot explored like a relative heading, so by default the
  // calibration never completes during a run (gyroMagneticAfterS = 1e9). Set it to 0 (calibrated at the
  // start) or e.g. 60 to see what happens if the heading switches to magnetic.
  double gyroMagnetic = -1, gyroMagneticOffsetDeg = -1, gyroMagneticAfterS = 1e9; // switch to magnetic after this many s of the run
  double magErrorDeg = 2.5;       // BNO055 datasheet: magnetometer heading accuracy +-2.5 deg (fully calibrated, ideal)
  double tofMinReliableMm = 30;   // team notes: "can't handle below 30mm"
  // VL53L0X datasheet table 12: standard deviation 4 % at 33 ms (white target, incl. part-to-part); table 14:
  // offset drift < 3 %. Here ~3 % reading-to-reading + a fixed per-sensor offset.
  double tofNoiseMm = 1.5, tofNoisePct = 0.03, tofOffsetSigma = 5, tofMaxRange = 1200, tofConeDeg = 25, tofPeriodUs = 33000;
  // Bench test 1 (3 runs, robot centred by hand): opposite sensors added up read 21/19/20 mm (left + right) and
  // 14/8/17 mm (front + back) more than the CAD says, whatever the centring: about +8 mm per sensor on average.
  // (Measured on the team's practice tile; walls thinner than the 20 mm used here would explain part of it.)
  double tofOffsetMeanMm = 8;
  // Bench test 6: 0.00 deg in 2 min standing still (BNO055 steps are 1/16 deg), so under 0.03 deg/min at rest.
  // Drift while moving isn't measured. (Before: 0.5, assumed.)
  double gyroNoiseDeg = 0.2, gyroDriftSigmaDegPerMin = 0.03, pitchNoiseDeg = 0.5;
  double colourNoise = 0.04;
  double placeSigmaMm = 8, placeSigmaDeg = 2; // how accurately a person places the robot
  double rampMinDeg = 15, rampMaxDeg = 25;
  // RCJ 2026 field (3.1, 3.3, 3.4): walls ~2 cm thick (280 mm path), obstacles and speed bumps
  double wallThicknessMm = 20, obstacleRate = 0.03, bumpRate = 0.05, bumpHeightMm = 10;
  // RCJ 2026 3.4.3: obstacles are "large, heavy items" and "may be fixed to the floor"; 3.4.5: one that is
  // moved stays where it ends up. ASSUMED (the rules give no mass or material): this share of the obstacles
  // is loose, each with a mass between obstacleMassMinKg and obstacleMassMaxKg and floor friction
  // obstacleMu; the rest are fixed. 0 = all fixed (the default, as before).
  double obstacleMovableFrac = 0, obstacleMassMinKg = 0.5, obstacleMassMaxKg = 2.0, obstacleMu = 0.4;
  double runTimeS = 480, stuckTimeoutS = 60, lopPauseS = 3;
  // Analysis only, NOT the real robot: 1 = put the code's map position right each time it starts
  // reading a tile (same floor only). Shows how much never getting lost would be worth.
  double oracleTile = 0;
  // Analysis only: 1 = put the robot itself in the middle of its tile (heading untouched) each time
  // the code starts reading a tile. Shows how much being off-centre costs.
  double oracleCentre = 0;
} P;

// --set name=value overrides any of the numbers above
static std::map<std::string, double *> paramTable() {
  return {
    {"batteryVoltage", &P.batteryVoltage}, {"noLoadRpmAt12V", &P.noLoadRpmAt12V}, {"stallKgcmAt12V", &P.stallKgcmAt12V},
    {"frictionFracAt12V", &P.frictionFracAt12V}, {"stictionFactor", &P.stictionFactor}, {"motorAExtraFriction", &P.motorAExtraFriction}, {"robotMassKg", &P.robotMassKg},
    {"wheelMu", &P.wheelMu}, {"motorTau", &P.motorTau},
    {"motorGainSigma", &P.motorGainSigma}, {"trackWidth", &P.trackWidth}, {"skidFactor", &P.skidFactor},
    {"tractionMean", &P.tractionMean}, {"tractionSigma", &P.tractionSigma},
    {"gyroMagnetic", &P.gyroMagnetic}, {"magErrorDeg", &P.magErrorDeg}, {"driverOhms", &P.driverOhms}, {"gyroMagneticOffsetDeg", &P.gyroMagneticOffsetDeg}, {"gyroMagneticAfterS", &P.gyroMagneticAfterS},
    {"tofMinReliableMm", &P.tofMinReliableMm},
    {"wallNudgeMm", &P.wallNudgeMm}, {"tofNoiseMm", &P.tofNoiseMm}, {"tofNoisePct", &P.tofNoisePct},
    {"tofOffsetSigma", &P.tofOffsetSigma}, {"tofOffsetMeanMm", &P.tofOffsetMeanMm}, {"tofMaxRange", &P.tofMaxRange}, {"tofConeDeg", &P.tofConeDeg},
    {"gyroNoiseDeg", &P.gyroNoiseDeg}, {"gyroDriftSigmaDegPerMin", &P.gyroDriftSigmaDegPerMin},
    {"pitchNoiseDeg", &P.pitchNoiseDeg}, {"colourNoise", &P.colourNoise}, {"placeSigmaMm", &P.placeSigmaMm},
    {"placeSigmaDeg", &P.placeSigmaDeg}, {"rampMinDeg", &P.rampMinDeg}, {"rampMaxDeg", &P.rampMaxDeg},
    {"wallThicknessMm", &P.wallThicknessMm}, {"obstacleRate", &P.obstacleRate},
    {"obstacleMovableFrac", &P.obstacleMovableFrac}, {"obstacleMassMinKg", &P.obstacleMassMinKg}, {"obstacleMassMaxKg", &P.obstacleMassMaxKg}, {"obstacleMu", &P.obstacleMu}, {"bumpRate", &P.bumpRate}, {"bumpHeightMm", &P.bumpHeightMm},
    {"runTimeS", &P.runTimeS}, {"stuckTimeoutS", &P.stuckTimeoutS}, {"oracleTile", &P.oracleTile}, {"oracleCentre", &P.oracleCentre}};
}

static bool setParam(const std::string &kv) {
  size_t eq = kv.find('=');
  if (eq == std::string::npos) return false;
  std::string k = kv.substr(0, eq);
  double v = std::atof(kv.c_str() + eq + 1);
  std::map<std::string, double *> m = paramTable();
  auto it = m.find(k);
  if (it == m.end()) return false;
  *it->second = v;
  return true;
}

static void makeIdeal() { // no noise, no drift, identical motors, perfect grip
  P.motorGainSigma = 0; P.motorAExtraFriction = 0; P.tractionMean = 1; P.tractionSigma = 0;
  P.tofNoiseMm = 0; P.tofNoisePct = 0; P.tofOffsetSigma = 0; P.tofOffsetMeanMm = 0; P.magErrorDeg = 0;
  P.gyroNoiseDeg = 0; P.gyroDriftSigmaDegPerMin = 0; P.pitchNoiseDeg = 0; P.colourNoise = 0;
  P.placeSigmaMm = 0; P.placeSigmaDeg = 0;
}

// =====================================================================================
// Simulation state
// =====================================================================================
struct SimEnd { std::string reason; };
bool simSerialEcho = false;
SimSerial Serial, Serial3, Serial4;
TwoWire Wire;

static std::mt19937 rng;
static double gauss(double s) { return s > 0 ? std::normal_distribution<double>(0, s)(rng) : 0; }
static field::World W;
static double tUs = 0;                 // simulated time
static bool runStarted = false;
static double runStartUs = 0;

// robot pose: x east, y north (mm, field corner = 0,0); heading clockwise from north (deg)
static double rx, ry, rhead, rpitch = 0;
struct Motor { int dir = RELEASE; int pwm = 0; double speed = 0, gain = 1, traction = 1, encFrac = 0; };
static Motor mot[5];                   // 1 = A (left), 2 = B (right), 3 = C (left), 4 = D (right, mounted reversed)
static double gyroBiasDegPerUs = 0;

// sensor geometry (robot frame: forward, left), by code sensor number 1..7
struct TofSensor { double fwd = 0, left = 0, height = 106; int facing = 0; double offset = 0; long lastSample = -1; double phaseUs = 0; bool ok = false; };
static TofSensor tof[8];
static int portToSensor[8] = {2, 1, 7, 6, 4, 5, 3, 0}; // inverse of measure()'s portMap; port 7 = colour sensor
static int muxPort = 0;
static double colourFwd = 92.8, colourLeft = 0, colourHeight = 23.7;
static double bodyFront = 99.1, bodyBack = 103.5, bodyHalfWidth = 91.5;
static std::vector<double> outF, outL;   // robot outline seen from above (forward, left), convex, from the CAD
static double outlineRadius = 140;      // furthest outline point from the centre
static double axleOffset = 57.8;

// referee / metrics
static bool switchHigh = false;
static double pauseUntilUs = 0;
static int lastTileX = -1, lastTileY = -1, cpX = 0, cpY = 0;
static double lastProgressUs = 0;
static int lops = 0, rampCrossings = 0, lastLevel = -1;
static std::vector<bool> visited;
// RCJ 2026 scoring (navigation part): per-tile records of what the robot visited
static std::vector<int> blueVisits, blueGood;   // visits of each blue tile; visits where it stopped 5 s
static std::vector<bool> cpVisited;             // checkpoints visited
static std::set<int> rampsDone;                 // ramps navigated (by ramp row), 10 points each once
static std::set<int> bumpsDone;                 // speed-bump tiles crossed, 5 points each once
static int curTile = -1, lastRampRow = -1;                        // tile the robot is "visiting" (more than half of it inside)
static double stillUs = 0, lastStillX = 0, lastStillY = 0; // how long it has stood still on the current tile
static bool blueStopped = false;                // stood still 5 s on the current blue tile
static double contactUs = 0;
static bool inContact = false;
static int syncSamples = 0, lostSamples = 0;
static std::string lopReasons;
// A robot that stopped for good away from home ("no path found", or "back to start" on the wrong
// tile): its captain calls lack of progress, which puts it back on the last checkpoint. If the code
// doesn't respond (it ignores the pause switch), the run ends where it stopped, and that restart isn't
// counted: a captain who saw it wouldn't bother.
static double stopClaimUs = -1, stopLopUs = -1;
static std::string stopWhy, stopEnd, stopReasonsBefore;
static int stopLopsBefore = 0;
static double stopX = 0, stopY = 0, stopHead = 0;
static bool movedSinceStopLop = false;
static void stopClaim(const char *why, const char *end) {
  if (stopClaimUs >= 0) return; // already handling this stop
  stopClaimUs = tUs; stopWhy = why; stopEnd = end;
}
static std::ofstream traceFile;
static std::ostream *traceOut = nullptr; // the recording: traceFile (--trace) or stdout (--live)
static bool live = false;
static double nextTraceUs = 0;
static int tofLast[8] = {0};            // last reading of each distance sensor, for the trace
static double gyroLast = 0;
static std::string lcdText, lcdTraced, serialLine;
static std::vector<uint32_t> mapTraced; // code's map as last written to the trace

static std::string jsonEsc(const std::string &in) {
  std::string o;
  for (char ch : in) {
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if ((unsigned char)ch < 0x20) o += ' ';
    else o += ch;
  }
  return o;
}

static double rad(double d) { return d * M_PI / 180.0; }
static double wrap360(double a) { a = std::fmod(a, 360.0); return a < 0 ? a + 360 : a; }

// =====================================================================================
// Geometry
// =====================================================================================
// sin and cos of an angle in degrees, remembered for the last two angles asked for. The same
// headings come up again and again (every footprint point, every collision test), and the maths
// library's sin/cos were almost half the run time. Same values, so results don't change.
struct SinCos {
  double deg[2] = {1e300, 1e300}, s[2] = {0, 0}, c[2] = {1, 1};
  int next = 0;
  void at(double d, double &sv, double &cv) {
    for (int i = 0; i < 2; i++) if (deg[i] == d) { sv = s[i]; cv = c[i]; return; }
    double h = rad(d);
    deg[next] = d; s[next] = sv = std::sin(h); c[next] = cv = std::cos(h);
    next ^= 1;
  }
};
static SinCos headSC, collideSC, pitchSC;

static void robotToWorld(double fwd, double left, double &wx, double &wy) {
  double sh, ch;
  headSC.at(rhead, sh, ch);
  wx = rx + fwd * sh - left * ch;
  wy = ry + fwd * ch + left * sh;
}

// Walls are P.wallThicknessMm thick, centred on the tile edges, and reach half that past their ends
// (the posts holding them count as wall, RCJ 3.1.4), so two facing walls leave a 280 mm path
// (RCJ 3.3.3). Obstacles are upright cylinders (RCJ 3.4.3).
static std::vector<std::vector<int>> obstaclesInCell; // obstacle indices overlapping each tile
static std::vector<double> obstacleMass;              // per obstacle: kg if loose, 0 if fixed (empty = all fixed)
static double pushedMm = 0;                           // how far the robot pushed obstacles in this run
static void indexObstacles() {
  obstaclesInCell.assign(W.W * W.H, {});
  for (size_t k = 0; k < W.obstacles.size(); k++) {
    const field::Obstacle &o = W.obstacles[k];
    for (int y = (int)std::floor((o.y - o.r) / field::TILE); y <= (int)std::floor((o.y + o.r) / field::TILE); y++)
      for (int x = (int)std::floor((o.x - o.r) / field::TILE); x <= (int)std::floor((o.x + o.r) / field::TILE); x++)
        if (W.in(x, y)) obstaclesInCell[W.id(x, y)].push_back((int)k);
  }
}
// box of the wall on a tile edge: horizontal edge from grid point (gx, gy) to (gx + 1, gy), or vertical to (gx, gy + 1)
static void edgeBox(int gx, int gy, bool horizontal, double b[4]) {
  const double S = field::TILE, h = P.wallThicknessMm / 2;
  if (horizontal) { b[0] = gx * S - h; b[1] = gy * S - h; b[2] = (gx + 1) * S + h; b[3] = gy * S + h; }
  else            { b[0] = gx * S - h; b[1] = gy * S - h; b[2] = gx * S + h; b[3] = (gy + 1) * S + h; }
}
// every wall that can reach into tile (cx, cy): the walls touching one of its four corners
template <class Fn> static void wallsNearCell(int cx, int cy, Fn f) {
  double b[4];
  for (int gy = cy; gy <= cy + 1; gy++)
    for (int gx = cx; gx <= cx + 1; gx++) {
      for (int a = gx - 1; a <= gx; a++) if (W.hasWall(a, gy, 2)) { edgeBox(a, gy, true, b); f(b); }  // along y = gy
      for (int a = gy - 1; a <= gy; a++) if (W.hasWall(gx, a, 3)) { edgeBox(gx, a, false, b); f(b); } // along x = gx
    }
}
static bool rayBox(double ox, double oy, double dx, double dy, const double *b, double &t, double &facing) {
  double tmin = -1e18, tmax = 1e18; int axis = 0;
  if (std::fabs(dx) < 1e-12) { if (ox < b[0] || ox > b[2]) return false; }
  else { double p = (b[0] - ox) / dx, q = (b[2] - ox) / dx; if (p > q) std::swap(p, q); if (p > tmin) { tmin = p; axis = 0; } tmax = std::min(tmax, q); }
  if (std::fabs(dy) < 1e-12) { if (oy < b[1] || oy > b[3]) return false; }
  else { double p = (b[1] - oy) / dy, q = (b[3] - oy) / dy; if (p > q) std::swap(p, q); if (p > tmin) { tmin = p; axis = 1; } tmax = std::min(tmax, q); }
  if (tmax < std::max(tmin, 0.0)) return false;
  if (tmin < 0) { t = 0; facing = 1; return true; } // starts inside the wall
  t = tmin; facing = axis == 0 ? std::fabs(dx) : std::fabs(dy);
  return true;
}
static bool rayCircle(double ox, double oy, double dx, double dy, const field::Obstacle &o, double &t, double &facing) {
  double px = ox - o.x, py = oy - o.y, b = px * dx + py * dy, c = px * px + py * py - o.r * o.r, disc = b * b - c;
  if (disc < 0) return false;
  t = -b - std::sqrt(disc);
  if (t < 0) { if (c > 0) return false; t = 0; facing = 1; return true; }
  double nx = (px + t * dx) / o.r, ny = (py + t * dy) / o.r;
  facing = std::fabs(nx * dx + ny * dy);
  return true;
}

// distance from (ox,oy) along (dx,dy) to the first wall or obstacle, walking the tile grid.
// *facing = how square-on the surface is to the ray (1 = head-on).
static double raycast(double ox, double oy, double dx, double dy, double maxd, double *facing = nullptr) {
  const double S = field::TILE, INF = 1e9;
  int ix = (int)std::floor(ox / S), iy = (int)std::floor(oy / S);
  int stepX = dx > 0 ? 1 : -1, stepY = dy > 0 ? 1 : -1;
  double tMaxX = std::fabs(dx) < 1e-12 ? INF : ((dx > 0 ? (ix + 1) * S - ox : ox - ix * S) / std::fabs(dx));
  double tMaxY = std::fabs(dy) < 1e-12 ? INF : ((dy > 0 ? (iy + 1) * S - oy : oy - iy * S) / std::fabs(dy));
  double tDX = std::fabs(dx) < 1e-12 ? INF : S / std::fabs(dx), tDY = std::fabs(dy) < 1e-12 ? INF : S / std::fabs(dy);
  for (int guard = 0; guard < 64; guard++) {
    double tExit = std::min(tMaxX, tMaxY), best = INF, bestFacing = 1;
    wallsNearCell(ix, iy, [&](const double *b) {
      double t, fc;
      if (rayBox(ox, oy, dx, dy, b, t, fc) && t < best) { best = t; bestFacing = fc; }
    });
    if (W.in(ix, iy))
      for (int k : obstaclesInCell[W.id(ix, iy)]) {
        double t, fc;
        if (rayCircle(ox, oy, dx, dy, W.obstacles[k], t, fc) && t < best) { best = t; bestFacing = fc; }
      }
    if (best <= tExit + 1e-9) {
      if (best > maxd) return INF;
      if (facing) *facing = bestFacing;
      return best;
    }
    if (tExit > maxd) return INF;
    if (tMaxX < tMaxY) { ix += stepX; tMaxX += tDX; } else { iy += stepY; tMaxY += tDY; }
  }
  return INF;
}

// does the robot body at pose (x,y,head) touch any wall or obstacle? The body is the outline from
// the CAD (convex); walls are boxes (thickness + posts), obstacles circles.
// wallsOnly: ignore obstacles. hit: if not null, set to the obstacle touched (or -1 for a wall).
static bool collides(double x, double y, double head, bool wallsOnly = false, int *hit = nullptr) {
  if (hit) *hit = -1;
  const int n = (int)outF.size();
  double sh, ch, px[32], py[32];
  collideSC.at(head, sh, ch);
  for (int k = 0; k < n; k++) { px[k] = x + outF[k] * sh - outL[k] * ch; py[k] = y + outF[k] * ch + outL[k] * sh; }
  // separating-axis test against an axis-aligned box b = x0, y0, x1, y1
  auto hitsBox = [&](const double *b) {
    // quick reject: box further than the outline's radius
    double qx = std::max(b[0], std::min(x, b[2])), qy = std::max(b[1], std::min(y, b[3]));
    if ((qx - x) * (qx - x) + (qy - y) * (qy - y) > outlineRadius * outlineRadius) return false;
    double mnx = 1e18, mxx = -1e18, mny = 1e18, mxy = -1e18;
    for (int k = 0; k < n; k++) { mnx = std::min(mnx, px[k]); mxx = std::max(mxx, px[k]); mny = std::min(mny, py[k]); mxy = std::max(mxy, py[k]); }
    if (mxx < b[0] || mnx > b[2] || mxy < b[1] || mny > b[3]) return false;
    for (int k = 0; k < n; k++) {
      int j = (k + 1) % n;
      double ax = -(py[j] - py[k]), ay = px[j] - px[k];
      double pmin = 1e18, pmax = -1e18;
      for (int m = 0; m < n; m++) { double d = px[m] * ax + py[m] * ay; pmin = std::min(pmin, d); pmax = std::max(pmax, d); }
      double c[4] = {b[0] * ax + b[1] * ay, b[2] * ax + b[1] * ay, b[0] * ax + b[3] * ay, b[2] * ax + b[3] * ay};
      double bmin = std::min(std::min(c[0], c[1]), std::min(c[2], c[3])), bmax = std::max(std::max(c[0], c[1]), std::max(c[2], c[3]));
      if (pmax < bmin || pmin > bmax) return false;
    }
    return true;
  };
  int cx = (int)std::floor(x / field::TILE), cy = (int)std::floor(y / field::TILE);
  double b[4];
  for (int gy = cy - 1; gy <= cy + 2; gy++)       // walls along y = gy
    for (int gx = cx - 2; gx <= cx + 1; gx++)
      if (W.hasWall(gx, gy, 2)) { edgeBox(gx, gy, true, b); if (hitsBox(b)) return true; }
  for (int gx = cx - 1; gx <= cx + 2; gx++)       // walls along x = gx
    for (int gy = cy - 2; gy <= cy + 1; gy++)
      if (W.hasWall(gx, gy, 3)) { edgeBox(gx, gy, false, b); if (hitsBox(b)) return true; }
  if (wallsOnly) return false;
  for (int ty = cy - 1; ty <= cy + 1; ty++)
    for (int tx = cx - 1; tx <= cx + 1; tx++) {
      if (!W.in(tx, ty)) continue;
      for (int k : obstaclesInCell[W.id(tx, ty)]) {
        const field::Obstacle &o = W.obstacles[k];
        if (std::hypot(o.x - x, o.y - y) > outlineRadius + o.r) continue;
        // inside the outline, or closer than r to one of its edges
        bool inside = true;
        double best = 1e18;
        for (int m = 0; m < n; m++) {
          int j = (m + 1) % n;
          double ex = px[j] - px[m], ey = py[j] - py[m], wx = o.x - px[m], wy = o.y - py[m];
          if (ex * wy - ey * wx < 0) inside = false; // outline is counter-clockwise
          double t = std::max(0.0, std::min(1.0, (wx * ex + wy * ey) / (ex * ex + ey * ey)));
          double dx = wx - t * ex, dy = wy - t * ey;
          best = std::min(best, dx * dx + dy * dy);
        }
        if (inside || best < o.r * o.r) { if (hit) *hit = k; return true; }
      }
    }
  return false;
}

// =====================================================================================
// Physics + referee, advanced by every hardware call
// =====================================================================================
static void lackOfProgress(const char *why);

// Does obstacle k (a circle) overlap a wall or another obstacle?
static bool obstacleBlocked(int k) {
  const field::Obstacle &o = W.obstacles[k];
  int cx = (int)std::floor(o.x / field::TILE), cy = (int)std::floor(o.y / field::TILE);
  bool hit = false;
  for (int ty = cy - 1; ty <= cy + 1 && !hit; ty++)
    for (int tx = cx - 1; tx <= cx + 1 && !hit; tx++)
      wallsNearCell(tx, ty, [&](const double *b) {
        double qx = std::max(b[0], std::min(o.x, b[2])), qy = std::max(b[1], std::min(o.y, b[3]));
        if (std::hypot(o.x - qx, o.y - qy) < o.r) hit = true;
      });
  for (size_t j = 0; j < W.obstacles.size() && !hit; j++)
    if ((int)j != k && std::hypot(W.obstacles[j].x - o.x, W.obstacles[j].y - o.y) < W.obstacles[j].r + o.r) hit = true;
  return hit;
}

// The robot's next pose (nx, ny, nh) runs into a loose obstacle: push it along if the wheels' grip can
// overcome its floor friction (the robot slows down by the share of its push the friction takes), unless
// that would shove it into a wall or another obstacle. On success nx, ny are the robot's pose after the push.
static bool pushObstacle(double &nx, double &ny, double nh) {
  if (obstacleMass.empty() || collides(nx, ny, nh, true)) return false; // no loose obstacles, or a wall in the way
  int k;
  collides(nx, ny, nh, false, &k);
  if (k < 0 || obstacleMass[k] <= 0) return false;                     // a fixed obstacle
  double need = P.obstacleMu * obstacleMass[k] * 9.81, can = P.wheelMu * P.robotMassKg * 9.81;
  if (need >= can) return false;                                        // too heavy for the wheels' grip
  double f = 1 - need / can, px = rx + (nx - rx) * f, py = ry + (ny - ry) * f;
  field::Obstacle before = W.obstacles[k];
  W.obstacles[k].x += px - rx; W.obstacles[k].y += py - ry;             // moves with the robot
  indexObstacles();
  for (int it = 0, j; it < 40 && collides(px, py, nh, false, &j) && j == k; it++) { // still overlapping (turning): out of the way
    double d = std::max(1e-6, std::hypot(W.obstacles[k].x - px, W.obstacles[k].y - py));
    W.obstacles[k].x += (W.obstacles[k].x - px) / d; W.obstacles[k].y += (W.obstacles[k].y - py) / d;
    indexObstacles();
  }
  if (obstacleBlocked(k) || collides(px, py, nh)) { W.obstacles[k] = before; indexObstacles(); return false; }
  pushedMm += std::hypot(W.obstacles[k].x - before.x, W.obstacles[k].y - before.y);
  nx = px; ny = py;
  return true;
}

static double motorOhms() { return 12.0 / 1.6; } // Pololu #3493: 12 V, 1.6 A stall

static void physicsStep(double dt) {
  // motors -> wheel surface speeds. DC motor: steady speed = no-load speed x (duty - load torque /
  // stall torque), all scaled to the battery voltage. Load = gearbox friction (more from standstill)
  // + sideways wheel drag when turning (skid steering) + gravity on a ramp.
  double V = P.batteryVoltage;
  double vNoLoad = P.noLoadRpmAt12V * V / 12.0 / 60.0 * M_PI * P.wheelDiameter; // mm/s at full PWM
  double stallNm = P.stallKgcmAt12V * V / 12.0 * 0.0980665 * motorOhms() / (motorOhms() + P.driverOhms);
  double rM = P.wheelDiameter / 2000.0, wheelLoadN = P.robotMassKg * 9.81 / 4;
  double friction = P.frictionFracAt12V * 12.0 / V;                                       // fraction of stall torque
  double skidSpin = P.wheelMu * wheelLoadN * (axleOffset / (P.trackWidth / 2)) * rM / stallNm; // turning on the spot
  double sinPitch, cosPitch;
  pitchSC.at(rpitch, sinPitch, cosPitch);
  double gravity = wheelLoadN * sinPitch * rM / stallNm;                                 // nose up = positive
  double duty[5] = {0, 0, 0, 0, 0};
  for (int i = 1; i <= 4; i++) {
    duty[i] = (mot[i].dir == FORWARD ? 1 : mot[i].dir == BACKWARD ? -1 : 0) * mot[i].pwm / 255.0;
    if (i == 4) duty[i] = -duty[i]; // D is mounted reversed
  }
  double dL = (duty[1] + duty[3]) / 2, dR = (duty[2] + duty[4]) / 2;
  double turning = std::min(1.0, std::fabs(dL - dR) / std::max(1e-9, std::fabs(dL) + std::fabs(dR))); // 0 straight .. 1 spin
  for (int i = 1; i <= 4; i++) {
    Motor &m = mot[i];
    double d = duty[i];
    double extraA = i == 1 ? P.motorAExtraFriction * std::max(0.0, 1 - std::fabs(d) / (128 / 255.0)) : 0;
    double resist = friction + extraA + skidSpin * turning;
    if (std::fabs(m.speed) < 1) resist *= P.stictionFactor;
    double eff = std::fabs(d) - resist - gravity * (d >= 0 ? 1 : -1); // driving uphill costs, downhill helps
    double target = eff > 0 ? (d > 0 ? 1 : -1) * eff * vNoLoad * m.gain : 0;
    static double lastDt = -1, lastTau = -1, lag = 0; // same step length almost every time: don't redo exp()
    if (dt != lastDt || P.motorTau != lastTau) { lastDt = dt; lastTau = P.motorTau; lag = 1 - std::exp(-dt / P.motorTau); }
    m.speed += (target - m.speed) * lag;
    // grip wanders slowly (Ornstein-Uhlenbeck)
    m.traction += (P.tractionMean - m.traction) * dt / P.tractionTau + gauss(P.tractionSigma * std::sqrt(2 * dt / P.tractionTau));
    m.traction = std::min(1.0, std::max(0.5, m.traction));
  }
  // encoders count wheel rotation, whether or not the robot actually moves
  auto encode = [&](int i, volatile int &count) {
    Motor &m = mot[i];
    m.encFrac += m.speed * dt / (M_PI * P.wheelDiameter) * P.encoderCountsPerRev;
    int whole = (int)m.encFrac;
    count += whole; m.encFrac -= whole;
  };
  encode(1, drivetrain.encoderCountA);
  encode(2, drivetrain.encoderCountB);
  encode(4, drivetrain.encoderCountD);
  // skid-steer body motion
  double vL = (mot[1].speed * mot[1].traction + mot[3].speed * mot[3].traction) / 2;
  double vR = (mot[2].speed * mot[2].traction + mot[4].speed * mot[4].traction) / 2;
  double v = (vL + vR) / 2 * cosPitch;
  double turnDeg = (vL - vR) / (P.trackWidth * P.skidFactor) * dt * 180 / M_PI; // left faster -> clockwise
  double nh = rhead + turnDeg, mid = rad(rhead + turnDeg / 2);
  double nx = rx + v * dt * std::sin(mid), ny = ry + v * dt * std::cos(mid);
  inContact = false;
  if (!collides(nx, ny, nh)) { rx = nx; ry = ny; rhead = nh; }
  else if (pushObstacle(nx, ny, nh)) { rx = nx; ry = ny; rhead = nh; contactUs += dt * 1e6; inContact = true; }
  else {
    contactUs += dt * 1e6;
    inContact = true;
    bool turned = false;
    if (std::fabs(turnDeg) > 1e-9) {
      if (!collides(rx, ry, nh)) { rhead = nh; turned = true; } // can still turn on the spot
      else {
        // turning with a corner on a wall: the wall shoves the robot sideways as it scrapes round
        // (up to P.wallNudgeMm per ms), instead of locking it in place
        double step = P.wallNudgeMm * dt * 1000;
        const double dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0.7, 0.7}, {0.7, -0.7}, {-0.7, 0.7}, {-0.7, -0.7}};
        for (const auto &d : dirs)
          if (step > 0 && !collides(rx + d[0] * step, ry + d[1] * step, nh)) { rx += d[0] * step; ry += d[1] * step; rhead = nh; turned = true; break; }
      }
    }
    if (!turned && !collides(nx, ny, rhead)) { rx = nx; ry = ny; } // or slide along without turning
  }
  rhead = wrap360(rhead);
  // pitch from the floor under the two axles
  double fx, fy, bx, by;
  robotToWorld(axleOffset, 0, fx, fy);
  robotToWorld(-axleOffset, 0, bx, by);
  rpitch = std::atan2(W.height(fx, fy) - W.height(bx, by), 2 * axleOffset) * 180 / M_PI;
}

// one frame of the recording: true pose, what the code believes, sensors, motors, and the tiles of
// the code's map that changed since the last frame
static int tileCode(Tile &t) {
  int c = 0;
  for (int d = 0; d < 4; d++) c |= t.getWall(d) << d;
  c |= t.getVisited() << 4 | (int)t.getType() << 5 | t.getElevate() << 7 | t.getDescend() << 8 | t.getDiscovered() << 13;
  for (int d = 0; d < 4; d++) c |= t.getObstacle(d) << (9 + d); // edges blocked by blockEdge() (or a real obstacle)
  return c;
}
static void traceFrame(double runS) {
  nextTraceUs = tUs + 50000;
  char b[320];
  auto signedPwm = [](int i) { return mot[i].pwm * (mot[i].dir == BACKWARD ? -1 : mot[i].dir == FORWARD ? 1 : 0); };
  std::snprintf(b, sizeof b, "{\"t\":%d,\"x\":%.1f,\"y\":%.1f,\"h\":%.1f,\"p\":%.1f,\"g\":%.1f,\"mx\":%d,\"my\":%d,\"f\":%d,\"d\":%d,\"s\":%d,\"c\":%d,\"pw\":[%d,%d,%d,%d],\"tof\":[",
                (int)(runS * 1000), rx, ry, rhead, rpitch, gyroLast, x_pos, y_pos, currentFloor, (int)currentDir, (int)state, (int)inContact,
                signedPwm(1), signedPwm(2), signedPwm(3), -signedPwm(4));
  *traceOut << b;
  for (int i = 1; i <= 7; i++) *traceOut << (i > 1 ? "," : "") << tofLast[i];
  *traceOut << "]";
  if (lcdText != lcdTraced) { lcdTraced = lcdText; *traceOut << ",\"lcd\":\"" << jsonEsc(lcdText) << "\""; }
  if (mapTraced.empty()) mapTraced.assign(NUM_FLOORS * MAP_SIZE * MAP_SIZE, 0);
  bool first = true;
  for (int f = 0; f < NUM_FLOORS; f++) {
    Grid &g = f == currentFloor ? mapGrid : floorGrid(f); // mapGrid is the working copy of the current floor
    for (int x = 0; x < MAP_SIZE; x++)
      for (int y = 0; y < MAP_SIZE; y++) {
        uint32_t c = tileCode(g[x][y]), &prev = mapTraced[(f * MAP_SIZE + x) * MAP_SIZE + y];
        if (c == prev) continue;
        prev = c;
        *traceOut << (first ? ",\"m\":[" : ",") << "[" << f << "," << x << "," << y << "," << c << "]";
        first = false;
      }
  }
  if (!first) *traceOut << "]";
  // obstacles the robot pushed since the last frame: "ob":[[index, x, y], ...]
  static std::vector<std::array<int, 2>> obTraced;
  if (obTraced.size() != W.obstacles.size()) { obTraced.clear(); for (const field::Obstacle &o : W.obstacles) obTraced.push_back({(int)o.x, (int)o.y}); }
  bool firstOb = true;
  for (size_t k = 0; k < W.obstacles.size(); k++) {
    int x = (int)W.obstacles[k].x, y = (int)W.obstacles[k].y;
    if (x == obTraced[k][0] && y == obTraced[k][1]) continue;
    obTraced[k] = {x, y};
    *traceOut << (firstOb ? ",\"ob\":[" : ",") << "[" << k << "," << x << "," << y << "]";
    firstOb = false;
  }
  if (!firstOb) *traceOut << "]";
  *traceOut << "}\n";
  if (live) traceOut->flush();
}

void simSerialOut(const char *text) {
  if (simSerialEcho) std::fputs(text, stdout);
  if (!traceOut) return;
  for (const char *c = text; *c; c++) {
    if (*c != '\n') { serialLine += *c; continue; }
    *traceOut << "{\"log\":\"" << jsonEsc(serialLine) << "\",\"t\":" << (runStarted ? (int)((tUs - runStartUs) / 1000) : 0) << "}\n";
    serialLine.clear();
  }
}

// RCJ 2026, 5.4.4: a tile is "visited" when more than half of the robot is inside it (seen from
// above). Returns that tile's id, or -1 while the robot straddles tiles.
static double nextFootprintUs = 0;
// floor() for the footprint test (a library call there was 14% of the run time); exact for any
// value an int can hold
static inline int floorInt(double v) { int i = (int)v; return (v < i) ? i - 1 : i; }

static int majorityTile() {
  const int N = 12; // N x N sample points over the footprint (CAD outline)
  int ids[8], counts[8], n = 0;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) {
      double fwd = -bodyBack + (bodyFront + bodyBack) * (i + 0.5) / N;
      double left = -bodyHalfWidth + 2 * bodyHalfWidth * (j + 0.5) / N;
      double wx, wy;
      robotToWorld(fwd, left, wx, wy);
      int x = floorInt(wx / field::TILE), y = floorInt(wy / field::TILE);
      if (!W.in(x, y)) continue;
      int id = W.id(x, y), k = 0;
      while (k < n && ids[k] != id) k++;
      if (k == n) { if (n == 8) continue; ids[n] = id; counts[n++] = 0; }
      counts[k]++;
    }
  for (int k = 0; k < n; k++)
    if (counts[k] * 2 > N * N) return ids[k];
  return -1;
}

static void refereeTick() {
  Pausemaze = switchHigh; // what pauseTask() does every 10 ms
  if (!runStarted) return;
  if (switchHigh && tUs >= pauseUntilUs) switchHigh = false;
  double runS = (tUs - runStartUs) / 1e6;
  if (stopClaimUs >= 0) {
    if (stopLopUs < 0) {
      if (runS >= P.runTimeS) throw SimEnd{stopEnd};
      if (tUs - stopClaimUs >= 2e6) { // the captain sees it standing still
        stopLopsBefore = lops; stopReasonsBefore = lopReasons; stopX = rx; stopY = ry; stopHead = rhead;
        lackOfProgress(stopWhy.c_str());
        stopLopUs = tUs; movedSinceStopLop = false;
      }
    } else {
      if (!switchHigh)
        for (int i = 1; i <= 4; i++)
          if (mot[i].pwm > 0 && (mot[i].dir == FORWARD || mot[i].dir == BACKWARD)) movedSinceStopLop = true;
      if (movedSinceStopLop) stopClaimUs = stopLopUs = -1; // it carried on: the restart counts
      else if (tUs - stopLopUs > (P.lopPauseS + 15) * 1e6 || runS >= P.runTimeS) {
        lops = stopLopsBefore; lopReasons = stopReasonsBefore; rx = stopX; ry = stopY; rhead = stopHead;
        throw SimEnd{stopEnd};
      }
    }
  }
  if (runS >= P.runTimeS) throw SimEnd{"time"};
  int tx = (int)std::floor(rx / field::TILE), ty = (int)std::floor(ry / field::TILE);
  if (!switchHigh && tUs >= nextFootprintUs) {
    nextFootprintUs = tUs + 5000;
    // standing still on the current tile (needed on blue tiles: 5 s)
    if (std::hypot(rx - lastStillX, ry - lastStillY) < 0.5) stillUs += 5000; else stillUs = 0;
    lastStillX = rx; lastStillY = ry;
    if (curTile >= 0 && W.type[curTile] == field::BLUE_T && stillUs >= 5e6 && !blueStopped) { blueStopped = true; blueGood[curTile]++; }
    int m = majorityTile();
    if (m >= 0 && m != curTile) {
      // RCJ 5.5.1c: visiting another tile without first stopping 5 s on a blue tile
      bool leftBlueEarly = curTile >= 0 && W.type[curTile] == field::BLUE_T && !blueStopped;
      int from = curTile;
      if (from >= 0 && !W.bump.empty() && W.bump[from]) bumpsDone.insert(from); // left a speed-bump tile: crossed
      curTile = m; stillUs = 0; blueStopped = false;
      lastTileX = m % W.W; lastTileY = m / W.W; lastProgressUs = tUs;
      int t = W.type[m];
      if (t != field::RAMP_T) {
        visited[m] = true;
        if (lastLevel >= 0 && W.level[m] != lastLevel) {
          rampCrossings++;
          rampsDone.insert(lastRampRow); // up or down: 10 points per ramp, once
        }
        lastLevel = W.level[m];
      } else lastRampRow = m / W.W;
      if (t == field::SILVER_T) { cpX = m % W.W; cpY = m / W.W; cpVisited[m] = true; }
      if (t == field::BLUE_T) blueVisits[m]++;
      if (t == field::BLACK_T) lackOfProgress("drove onto a black tile");          // RCJ 5.5.1b
      else if (leftBlueEarly) { (void)from; lackOfProgress("left a blue tile before stopping 5 s"); }
    }
  }
  if (!switchHigh && (tUs - lastProgressUs) / 1e6 > P.stuckTimeoutS) lackOfProgress("stuck");
  // is the code's map position right? (checked while it reads the walls of a tile)
  static int prevState = -1;
  bool enteredSense = state == SENSE_TILE && prevState != SENSE_TILE;
  prevState = state;
  if (state == SENSE_TILE && !switchHigh && W.in(tx, ty) && W.type[W.id(tx, ty)] != field::RAMP_T) {
    int ef = START_FLOOR + W.level[W.id(tx, ty)] - W.level[W.id(W.sx, W.sy)];
    if (P.oracleTile > 0 && enteredSense && currentFloor == ef) { x_pos = tx - W.sx + MAP_SIZE / 2; y_pos = ty - W.sy + MAP_SIZE / 2; }
    if (P.oracleCentre > 0 && enteredSense && W.level[W.id(tx, ty)] == W.level[curTile >= 0 ? curTile : W.id(tx, ty)]) {
      double ox = rx, oy = ry, cx = (tx + 0.5) * field::TILE, cy = (ty + 0.5) * field::TILE;
      // 1 = both directions, 2 = only along the way it faces, 3 = only sideways
      bool facesY = ((int)std::lround(rhead / 90.0) % 2) == 0;
      if (P.oracleCentre != 3) { if (facesY) ry = cy; else rx = cx; }
      if (P.oracleCentre != 2) { if (facesY) rx = cx; else ry = cy; }
      if (collides(rx, ry, rhead)) { rx = ox; ry = oy; }
    }
    syncSamples++;
    if (x_pos != tx - W.sx + MAP_SIZE / 2 || y_pos != ty - W.sy + MAP_SIZE / 2 || currentFloor != ef) lostSamples++;
  }
  if (traceOut && tUs >= nextTraceUs) traceFrame(runS);
}

static void lackOfProgress(const char *why) {
  // referee restart: a teammate flips the pause switch and puts the robot on the last checkpoint
  lops++;
  if (!lopReasons.empty()) lopReasons += "; ";
  lopReasons += why;
  switchHigh = true;
  pauseUntilUs = tUs + P.lopPauseS * 1e6;
  rx = (cpX + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  ry = (cpY + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  rhead = wrap360(gauss(P.placeSigmaDeg)); // placed facing the start direction
  for (int i = 1; i <= 4; i++) mot[i].speed = 0;
  lastTileX = cpX; lastTileY = cpY; lastProgressUs = tUs + P.lopPauseS * 1e6;
  curTile = W.id(cpX, cpY); stillUs = 0; blueStopped = false; lastStillX = rx; lastStillY = ry;
  if (traceOut) *traceOut << "{\"event\":\"lop\",\"why\":\"" << why << "\",\"t\":" << (int)((tUs - runStartUs) / 1000)
                             << ",\"cx\":" << cpX << ",\"cy\":" << cpY << "}\n";
}

// ---- live mode: run at wall-clock speed (x liveSpeed) and obey commands from the server ----
double liveWallMs();
void liveSleepMs(double ms);
std::string liveReadInput();
static bool livePaused = false;
static double liveSpeed = 1, liveSimAnchorUs = 0, liveWallAnchorMs = 0, liveNextUs = 0;
static std::string liveInbox;

static void liveRebase() { liveSimAnchorUs = tUs; liveWallAnchorMs = liveWallMs(); }
static void liveEvent(const char *what) {
  *traceOut << "{\"event\":\"" << what << "\",\"t\":" << (int)((tUs - runStartUs) / 1000) << "}\n";
  traceOut->flush();
}
static void liveCommand(const std::string &c) {
  if (c == "pause" && !livePaused) { livePaused = true; liveEvent("paused"); }
  else if (c == "resume" && livePaused) { livePaused = false; liveRebase(); liveEvent("resumed"); }
  else if (c.compare(0, 6, "speed ") == 0) { liveSpeed = std::atof(c.c_str() + 6); liveRebase(); } // 0 = as fast as possible
  else if (c == "lop" && !switchHigh) lackOfProgress("restart button");
  else if (c == "nudge") { // someone bumps the robot: up to 40 mm and 15 degrees, never into a wall
    std::uniform_real_distribution<double> u(-1, 1);
    for (int k = 0; k < 30; k++) {
      double nx = rx + 40 * u(rng), ny = ry + 40 * u(rng), nh = wrap360(rhead + 15 * u(rng));
      if (!collides(nx, ny, nh)) { rx = nx; ry = ny; rhead = nh; break; }
    }
    liveEvent("nudge");
  }
  else if (c == "stop") throw SimEnd{"stopped"};
}
static void livePace() {
  liveInbox += liveReadInput();
  for (size_t nl; (nl = liveInbox.find('\n')) != std::string::npos;) {
    std::string c = liveInbox.substr(0, nl);
    liveInbox.erase(0, nl + 1);
    while (!c.empty() && (c.back() == '\r' || c.back() == ' ')) c.pop_back();
    liveCommand(c);
  }
  while (livePaused) {
    liveSleepMs(40);
    liveInbox += liveReadInput();
    for (size_t nl; (nl = liveInbox.find('\n')) != std::string::npos;) {
      std::string c = liveInbox.substr(0, nl);
      liveInbox.erase(0, nl + 1);
      while (!c.empty() && (c.back() == '\r' || c.back() == ' ')) c.pop_back();
      liveCommand(c);
    }
  }
  if (liveSpeed > 0) {
    double ahead = (tUs - liveSimAnchorUs) / 1000.0 / liveSpeed - (liveWallMs() - liveWallAnchorMs);
    if (ahead > 2) liveSleepMs(ahead);
    else if (ahead < -500) liveRebase(); // a slow PC fell behind: carry on from here instead of rushing
  }
}

static bool simReady = false; // false while global objects are being constructed (before main)

static void simAdvance(double us) {
  if (!simReady) { tUs += us; return; } // firmware globals (timers, PIDs) call micros() before main runs
  double end = tUs + us;
  while (tUs < end) {
    double h = std::min(1000.0, end - tUs);
    // a leftover below a nanosecond: the 32-bit build keeps `end` at higher precision than tUs, so
    // tUs += h can round back to the same value and this loop would never finish
    if (h < 1e-3) { tUs = end; break; }
    physicsStep(h / 1e6);
    tUs += h;
    refereeTick();
    if (live && runStarted && tUs >= liveNextUs) { liveNextUs = tUs + 10000; livePace(); }
  }
}

// =====================================================================================
// Simulated Arduino core and libraries
// =====================================================================================
unsigned long millis() { simAdvance(1); return (unsigned long)(tUs / 1000); }
unsigned long micros() { simAdvance(1); return (unsigned long)tUs; }
void delay(unsigned long ms) { simAdvance(ms * 1000.0); }
void delayMicroseconds(unsigned int us) { simAdvance(us); }
namespace rtos { namespace ThisThread { void sleep_for(std::chrono::milliseconds ms) { simAdvance(ms.count() * 1000.0); } } }
void pinMode(int, int) {}
int digitalRead(int pin) { return pin == 22 && switchHigh ? HIGH : LOW; } // 22 = logic (pause) switch
void digitalWrite(int, int) {}
void attachInterrupt(int, void (*)(), int) {}
void detachInterrupt(int) {}

void TwoWire::begin() {}
void TwoWire::beginTransmission(uint8_t) {}
size_t TwoWire::write(uint8_t) { return 1; }
uint8_t TwoWire::endTransmission(bool) { simAdvance(150); return 2; } // nothing answers a raw scan
uint8_t TwoWire::requestFrom(uint8_t, uint8_t) { simAdvance(150); return 0; }
int TwoWire::available() { return 0; }
int TwoWire::read() { return -1; }

bool QWIICMUX::begin(uint8_t, TwoWire &) { return true; }
bool QWIICMUX::setPort(uint8_t p) { muxPort = p; simAdvance(150); return true; }

bool VL53L0X::init(bool) { simAdvance(2000); return true; }
uint16_t VL53L0X::readRangeContinuousMillimeters() {
  int code = portToSensor[muxPort & 7];
  if (code < 1 || code > 7 || !tof[code].ok) { simAdvance(500); return 65535; }
  TofSensor &s = tof[code];
  // continuous mode: wait for a sample newer than the last one read
  long k = (long)std::floor((tUs - s.phaseUs) / P.tofPeriodUs);
  if (k <= s.lastSample) { k = s.lastSample + 1; simAdvance(s.phaseUs + k * P.tofPeriodUs - tUs); }
  s.lastSample = k;
  simAdvance(400); // I2C read
  double ox, oy;
  robotToWorld(s.fwd, s.left, ox, oy);
  double base = rhead + (s.facing == 0 ? 0 : s.facing == 1 ? 90 : s.facing == 2 ? 180 : -90); // forward/right/back/left
  // The beam is a ~25 degree cone, strongest in the middle. With several surfaces in view the
  // sensor reports roughly the signal-weighted distance: weight = beam strength x how square-on
  // the surface is / distance^2 (nearer, face-on walls return far more light).
  const int RAYS = 11;
  double sigma = P.tofConeDeg / 2.355, half = P.tofConeDeg * 0.6;
  double sumW = 0, sumWD = 0, sumProfile = 0;
  for (int r = 0; r < RAYS; r++) {
    double off = -half + 2 * half * r / (RAYS - 1), prof = std::exp(-off * off / (2 * sigma * sigma)), facing = 1;
    sumProfile += prof;
    double a = rad(base + off);
    double d = raycast(ox, oy, std::sin(a), std::cos(a), P.tofMaxRange * 1.5, &facing);
    if (d > P.tofMaxRange * 1.5) continue;
    double w = prof * std::max(0.05, facing) / std::max(d * d, 100.0);
    sumW += w; sumWD += w * d;
  }
  // too little light back (only far or grazing surfaces) = out of range
  if (sumW < sumProfile / (P.tofMaxRange * P.tofMaxRange)) return tofLast[code] = 8190;
  double dist = sumWD / sumW;
  // below ~30 mm the VL53L0X can't measure properly (team notes); it reads about its minimum, never less
  if (dist < P.tofMinReliableMm) dist = P.tofMinReliableMm + std::fabs(gauss(5));
  double d = dist + s.offset + gauss(P.tofNoiseMm + P.tofNoisePct * dist);
  return tofLast[code] = (uint16_t)std::max(0.0, std::round(d));
}

bool Adafruit_TCS34725::begin() { simAdvance(3000); return true; }
void Adafruit_TCS34725::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  simAdvance((256 - (int)it_) * 2400.0 + 500); // integration time, like the real library
  double wx, wy;
  robotToWorld(colourFwd, colourLeft, wx, wy);
  int t = W.typeAt((int)std::floor(wx / field::TILE), (int)std::floor(wy / field::TILE));
  // raw counts at 24 ms / 1x gain (assumed; red > 800 is the code's silver test, clear ratio < 0.1 is black)
  double R = 600, G = 650, B = 600, C = 2000;              // white
  if (t == field::BLACK_T) { R = 40; G = 45; B = 40; C = 120; }
  if (t == field::BLUE_T) { R = 150; G = 220; B = 520; C = 900; }
  if (t == field::SILVER_T) { R = 1100; G = 1150; B = 1100; C = 3500; }
  double maxCount = std::min(65535.0, (256 - (int)it_) * 1024.0); // AMS datasheet: max RGBC count
  auto noisy = [maxCount](double v) { return (uint16_t)std::min(maxCount, std::max(0.0, std::round(v * (1 + gauss(P.colourNoise))))); };
  *r = noisy(R); *g = noisy(G); *b = noisy(B); *c = noisy(C);
}

void Adafruit_DCMotor::run(uint8_t cmd) { mot[num].dir = cmd; simAdvance(250); }
void Adafruit_DCMotor::setSpeed(uint8_t s) { mot[num].pwm = s; simAdvance(250); }
bool Adafruit_MotorShield::begin(uint16_t, TwoWire *) { return true; }
Adafruit_DCMotor *Adafruit_MotorShield::getMotor(uint8_t n) {
  static Adafruit_DCMotor m[5];
  m[n].num = n;
  return &m[n];
}

static int bnoMode = OPERATION_MODE_NDOF;
bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode) { bnoMode = mode; simAdvance(20000); return true; }
void Adafruit_BNO055::setMode(adafruit_bno055_opmode_t mode) { bnoMode = mode; }
static double magOffsetDeg = 0;                // maze "north" vs magnetic north
static double magPhase[4] = {0, 0, 0, 0};      // where the heading error from local magnetic distortion peaks
static bool magneticMode() {                   // NDOF, NDOF_FMC_OFF, COMPASS, M4G use the magnetometer for heading
  if (P.gyroMagnetic >= 0) return P.gyroMagnetic > 0;
  return bnoMode == OPERATION_MODE_NDOF || bnoMode == OPERATION_MODE_NDOF_FMC_OFF || bnoMode == OPERATION_MODE_COMPASS || bnoMode == OPERATION_MODE_M4G;
}
bool Adafruit_BNO055::getEvent(sensors_event_t *e) {
  simAdvance(900);
  bool magnetic = magneticMode() && (runStarted ? (tUs - runStartUs) / 1e6 >= P.gyroMagneticAfterS : P.gyroMagneticAfterS <= 0);
  // magnetic heading: no gyro drift, but the frame is magnetic north and local distortion bends it a little
  double magErr = P.magErrorDeg * std::sin(2 * M_PI * rx / 900 + magPhase[0]) * std::cos(2 * M_PI * ry / 1100 + magPhase[1]);
  e->orientation.x = (float)wrap360(rhead + (magnetic ? magOffsetDeg + magErr : gyroBiasDegPerUs * tUs) + gauss(P.gyroNoiseDeg));
  gyroLast = e->orientation.x;
  e->orientation.y = 0;
  e->orientation.z = (float)(rpitch + gauss(P.pitchNoiseDeg));
  return true;
}

size_t LiquidCrystal::print(const char *text) {
  simAdvance(2000);
  std::string s = text;
  if (stopClaimUs >= 0) {} // already handling a stop: repeats of its message change nothing
  else if (s.find("back to start") != std::string::npos) {
    if ((int)std::floor(rx / field::TILE) == W.sx && (int)std::floor(ry / field::TILE) == W.sy) throw SimEnd{"home"};
    stopClaim("stopped on the wrong tile", "home");
  }
  else if (s.find("no path found") != std::string::npos) stopClaim("no path home in its map", "no path");
  if (!s.empty() && s.find_first_not_of(' ') != std::string::npos) lcdText = s;
  return s.size();
}

// =====================================================================================
// Setup + run
// =====================================================================================
void setup();
void loop();

static bool loadGeometry(const std::string &path) {
  std::ifstream f(path);
  if (!f) return false;
  std::stringstream ss; ss << f.rdbuf();
  std::string js = ss.str();
  auto num = [&](const std::string &obj, const char *key) {
    size_t p = obj.find(std::string("\"") + key + "\"");
    return p == std::string::npos ? 0.0 : std::atof(obj.c_str() + obj.find(':', p) + 1);
  };
  size_t pos = js.find("\"sensors\"");
  int found = 0;
  while (true) {
    size_t a = js.find('{', pos), b = js.find('}', a);
    if (a == std::string::npos || a > js.find(']', pos)) break;
    std::string obj = js.substr(a, b - a);
    int code = (int)num(obj, "code_sensor");
    if (code >= 1 && code <= 7) {
      TofSensor &s = tof[code];
      s.fwd = num(obj, "forward_mm"); s.left = num(obj, "left_mm"); s.height = num(obj, "height_mm"); s.ok = true;
      s.facing = obj.find("\"forward\"") != std::string::npos ? 0 : obj.find("\"right\"") != std::string::npos ? 1
               : obj.find("\"backward\"") != std::string::npos ? 2 : 3;
      found++;
    }
    pos = b;
  }
  size_t c = js.find("\"colour_sensor\"");
  if (c != std::string::npos) { std::string obj = js.substr(c, js.find('}', c) - c); colourFwd = num(obj, "forward_mm"); colourLeft = num(obj, "left_mm"); colourHeight = num(obj, "height_mm"); }
  size_t ax = js.find("\"axles_forward_mm\"");
  if (ax != std::string::npos) axleOffset = std::fabs(std::atof(js.c_str() + js.find('[', ax) + 1));
  // body outline = the outer faces of the distance sensors (boards are 1.3 mm thick)
  bodyFront = bodyBack = bodyHalfWidth = 0;
  for (int i = 1; i <= 7; i++) {
    bodyFront = std::max(bodyFront, tof[i].fwd + 0.7);
    bodyBack = std::max(bodyBack, -tof[i].fwd + 0.7);
    bodyHalfWidth = std::max(bodyHalfWidth, std::fabs(tof[i].left) + 0.7);
  }
  // the real outline (all parts that can touch a wall), if the geometry file has it
  outF.clear(); outL.clear();
  size_t o = js.find("\"outline_mm\"");
  if (o != std::string::npos) {
    size_t a = js.find('[', o), b = a;
    for (int depth = 0; b < js.size(); b++) { // the matching ']' (the list spans many lines)
      if (js[b] == '[') depth++;
      else if (js[b] == ']' && --depth == 0) break;
    }
    std::string arr = js.substr(a, b - a + 1);
    std::vector<double> v;
    for (size_t i = 0; i < arr.size();) {
      if (arr[i] == '-' || (arr[i] >= '0' && arr[i] <= '9')) { char *e; v.push_back(std::strtod(arr.c_str() + i, &e)); i = e - arr.c_str(); }
      else i++;
    }
    for (size_t i = 0; i + 1 < v.size(); i += 2) { outF.push_back(v[i]); outL.push_back(v[i + 1]); }
  }
  if (outF.size() < 3) { // no outline: the box around the sensors
    outF = {bodyFront, bodyFront, -bodyBack, -bodyBack};
    outL = {-bodyHalfWidth, bodyHalfWidth, bodyHalfWidth, -bodyHalfWidth};
  }
  outlineRadius = 0;
  bodyFront = bodyBack = bodyHalfWidth = 0;
  for (size_t i = 0; i < outF.size(); i++) {
    outlineRadius = std::max(outlineRadius, std::hypot(outF[i], outL[i]));
    bodyFront = std::max(bodyFront, outF[i]); bodyBack = std::max(bodyBack, -outF[i]);
    bodyHalfWidth = std::max(bodyHalfWidth, std::fabs(outL[i]));
  }
  return found == 7;
}

// --selftest: fixed experiments with the simulated hardware, through the robot's own functions.
// The same tests on the real robot (see README) show which assumptions in Params need changing.
int measure(int sensor);
static int selfTest() {
  // 3 x 3 field; middle tile walled on N, E and W, open to the south
  W = field::World();
  W.W = 3; W.H = 3;
  W.wall.assign(9, {{false, false, false, false}});
  W.type.assign(9, field::FLOOR); W.level.assign(9, 0);
  for (int y = 0; y < 3; y++) for (int x = 0; x < 3; x++) for (int d = 0; d < 4; d++) {
    int nx = x + field::DX[d], ny = y + field::DY[d];
    if (!W.in(nx, ny)) W.wall[W.id(x, y)][d] = true;
  }
  for (int d : {0, 1, 3}) { int nx = 1 + field::DX[d], ny = 1 + field::DY[d]; W.wall[W.id(1, 1)][d] = true; W.wall[W.id(nx, ny)][(d + 2) % 4] = true; }
  W.sx = 1; W.sy = 1;
  indexObstacles();
  simReady = true;
  auto place = [](double x, double y, double h) { rx = x; ry = y; rhead = h; for (int i = 1; i <= 4; i++) mot[i].speed = 0; };
  place(450, 450, 0);
  std::printf("SENSORS (robot centred in a tile, walls front/left/right, open behind)\n");
  const char *where[8] = {"", "front-right", "right-front", "right-back", "back", "left-back", "left-front", "front-left"};
  for (int i = 1; i <= 7; i++) {
    double sum = 0; int n = 20;
    for (int k = 0; k < n; k++) sum += measure(i);
    std::printf("  measure(%d) %-11s %6.1f mm\n", i, where[i], sum / n);
  }
  std::printf("  code expects (centred): side %.0f mm, front %.0f mm\n", (300.0 - 140) / 2, (300.0 - 170) / 2);

  std::printf("DRIVE drivetrain.fw(150) for 2 s from rest, then fullstop (open floor)\n");
  W.wall.assign(9, {{false, false, false, false}});
  for (int y = 0; y < 3; y++) for (int x = 0; x < 3; x++) for (int d = 0; d < 4; d++)
    if (!W.in(x + field::DX[d], y + field::DY[d])) W.wall[W.id(x, y)][d] = true;
  W.H = 3; place(450, 120, 0);
  drivetrain.reset_encoderCount(true, true, true);
  double y0 = ry;
  drivetrain.fw(150); delay(2000); drivetrain.fullstop(); delay(500);
  int enc = (drivetrain.encoderCountA + drivetrain.encoderCountB + drivetrain.encoderCountD) / 3;
  std::printf("  moved %.0f mm, encoders A/B/D %d/%d/%d, %.4f mm per count (code assumes %.4f = 5 x 195 counts per wheel turn)\n", ry - y0,
              (int)drivetrain.encoderCountA, (int)drivetrain.encoderCountB, (int)drivetrain.encoderCountD, (ry - y0) / std::max(1, enc),
              M_PI * wheel_diameter / (wheel_cpr * gear_ratio));
  for (int pwm : {20, 30, 60, 90, 120, 150}) {
    place(450, 120, 0); drivetrain.fw(pwm); delay(1500); double a = ry; delay(1000); double v = ry - a; drivetrain.fullstop(); delay(500);
    std::printf("  fw(%3d): %5.0f mm/s\n", pwm, v);
  }
  std::printf("TURN drivetrain.turnright(pwm), turn rate after 1 s, centred in a tile\n");
  for (int pwm : {20, 30, 40, 60, 90, 150}) {
    place(450, 450, 0);
    double unwrapped = 0, last = rhead;
    drivetrain.turnright(pwm);
    for (int k = 0; k < 200; k++) {
      delay(10); double d = rhead - last; if (d < -180) d += 360; if (d > 180) d -= 360;
      if (k >= 100) unwrapped += d;
      last = rhead;
    }
    drivetrain.fullstop(); delay(500);
    std::printf("  turnright(%3d): %5.0f deg/s\n", pwm, unwrapped);
  }
  // The robot's bench mode (Main/bench.cpp) tests 2-4, with the same steps and the same pass rules, so the
  // numbers compare one to one with the [BENCH] lines from the real robot.
  std::printf("BENCH (as Main/bench.cpp)\n");
  {
    place(450, 450, 0);
    int found = -1;
    for (int pwm = 20; pwm <= 150 && found < 0; pwm += 5) {
      double h0 = rhead;
      drivetrain.turnright(pwm); delay(1000); drivetrain.fullstop(); delay(300);
      double moved = rhead - h0; if (moved > 180) moved -= 360; if (moved < -180) moved += 360;
      if (std::fabs(moved) > 5.0) found = pwm;
    }
    std::printf("  2 lowest PWM that turns it on the spot: %d\n", found);
    place(450, 450, 0);
    double total = 0, last = rhead;
    drivetrain.turnright(150);
    for (int k = 0; k < 100; k++) { delay(10); double d = rhead - last; if (d < -180) d += 360; if (d > 180) d -= 360; total += d; last = rhead; }
    drivetrain.fullstop(); delay(500);
    std::printf("  3 turnright(150) for 1 s: %.1f deg\n", total);
    found = -1;
    for (int pwm = 5; pwm <= 150 && found < 0; pwm += (pwm < 60 ? 2 : 10)) {
      place(450, 120, 0);
      drivetrain.reset_encoderCount(true, true, true);
      drivetrain.fw(pwm); delay(1000); drivetrain.fullstop(); delay(300);
      if (drivetrain.encoderCountA > 10 && drivetrain.encoderCountB > 10 && drivetrain.encoderCountD > 10) found = pwm;
    }
    std::printf("  4 lowest PWM that drives all three encoder wheels forward: %d\n", found);
  }
  {
    double V = P.batteryVoltage, stallNm = P.stallKgcmAt12V * V / 12.0 * 0.0980665 * motorOhms() / (motorOhms() + P.driverOhms), rM = P.wheelDiameter / 2000.0;
    double fr = P.frictionFracAt12V * 12.0 / V, spin = P.wheelMu * P.robotMassKg * 9.81 / 4 * (axleOffset / (P.trackWidth / 2)) * rM / stallNm;
    std::printf("MODEL at %.1f V: top speed %.0f mm/s; lowest PWM that moves it straight %.0f (from standstill %.0f); "
                "lowest PWM that turns it on the spot %.0f\n", V, P.noLoadRpmAt12V * V / 12 / 60 * M_PI * P.wheelDiameter,
                255 * fr, 255 * fr * P.stictionFactor, 255 * (fr + spin) * P.stictionFactor);
  }
  std::printf("COLOUR raw r g b c over each tile type\n");
  const char *names[] = {"white", "black", "blue", "silver"};
  int types[] = {field::FLOOR, field::BLACK_T, field::BLUE_T, field::SILVER_T};
  for (int k = 0; k < 4; k++) {
    W.type.assign(9, types[k]); uint16_t r, g, b, c; tcs.getRawData(&r, &g, &b, &c);
    std::printf("  %-6s %5u %5u %5u %5u\n", names[k], r, g, b, c);
  }
  return 0;
}

int main(int argc, char **argv) {
  std::string scenario = "loops", tracePath, geomPath;
  long seed = 1;
  bool ideal = false, selftest = false;
  std::vector<std::string> sets;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
    else if (a == "--seed" && i + 1 < argc) seed = std::atol(argv[++i]);
    else if (a == "--ideal") ideal = true;
    else if (a == "--verbose") { simSerialEcho = true; std::setvbuf(stdout, nullptr, _IONBF, 0); } // unbuffered: nothing lost if the firmware crashes
    else if (a == "--trace" && i + 1 < argc) tracePath = argv[++i];
    else if (a == "--live") live = true;
    else if (a == "--live-speed" && i + 1 < argc) liveSpeed = std::atof(argv[++i]);
    else if (a == "--geometry" && i + 1 < argc) geomPath = argv[++i];
    else if (a == "--set" && i + 1 < argc) sets.push_back(argv[++i]);
    else if (a == "--selftest") selftest = true;
    else if (a == "--params") {
      std::printf("{");
      bool first = true;
      for (const auto &kv : paramTable()) { std::printf("%s\"%s\":%g", first ? "" : ",", kv.first.c_str(), *kv.second); first = false; }
      std::printf("}\n");
      return 0;
    }
    else { std::printf("usage: physics --scenario flat|loops|big|ramp|bigramp|comp --seed S [--ideal] [--set name=value]... [--verbose] [--trace FILE] [--geometry robot_geometry.json]\n"); return 2; }
  }
  if (geomPath.empty()) {
    std::string exe = argv[0];
    size_t slash = exe.find_last_of("/\\");
    geomPath = (slash == std::string::npos ? std::string(".") : exe.substr(0, slash)) + "/../cad/robot_geometry.json";
  }
  if (!loadGeometry(geomPath)) { std::printf("{\"error\":\"could not read 7 sensors from %s\"}\n", geomPath.c_str()); return 2; }
  if (ideal) makeIdeal();
  if (scenario == "comp") P.bumpRate = 0.12; // the 2025 international fields had 2-13 speed-bump tiles (about 6 in 48)
  for (const std::string &s : sets)
    if (!setParam(s)) { std::printf("{\"error\":\"unknown --set %s\"}\n", s.c_str()); return 2; }
  if (selftest) {
    rng.seed(1);
    for (int i = 1; i <= 4; i++) { mot[i].gain = 1 + gauss(P.motorGainSigma); mot[i].traction = P.tractionMean; }
    for (int i = 1; i <= 7; i++) tof[i].offset = P.tofOffsetMeanMm + gauss(P.tofOffsetSigma);
    return selfTest();
  }

  const field::Scenario *sc = nullptr;
  for (const auto &s : field::SCENARIOS) if (scenario == s.name) sc = &s;
  if (!sc) { std::printf("{\"error\":\"unknown scenario %s\"}\n", scenario.c_str()); return 2; }
  std::mt19937 fieldRng((unsigned)seed * 2654435761u + (unsigned)(sc - field::SCENARIOS));
  field::Extras ex;
  ex.wallT = P.wallThicknessMm; ex.obstacleRate = P.obstacleRate; ex.bumpRate = P.bumpRate; ex.bumpH = P.bumpHeightMm;
  W = field::generate(*sc, fieldRng, P.rampMinDeg, P.rampMaxDeg, ex);
  indexObstacles();
  rng.seed((unsigned)seed * 7919u + 4242u);
  if (P.obstacleMovableFrac > 0) { // which obstacles are loose and how heavy: own generator, the rest of the run is unchanged
    std::mt19937 orng((unsigned)seed * 104729u + 17u);
    std::uniform_real_distribution<double> u(0, 1);
    obstacleMass.assign(W.obstacles.size(), 0);
    for (double &m : obstacleMass)
      if (u(orng) < P.obstacleMovableFrac) m = P.obstacleMassMinKg + (P.obstacleMassMaxKg - P.obstacleMassMinKg) * u(orng);
  }

  for (int i = 1; i <= 4; i++) { mot[i].gain = 1 + gauss(P.motorGainSigma); mot[i].traction = P.tractionMean; }
  for (int i = 1; i <= 7; i++) { tof[i].offset = P.tofOffsetMeanMm + gauss(P.tofOffsetSigma); tof[i].phaseUs = std::uniform_real_distribution<double>(0, P.tofPeriodUs)(rng); }
  gyroBiasDegPerUs = gauss(P.gyroDriftSigmaDegPerMin) / 60e6;
  magOffsetDeg = P.gyroMagneticOffsetDeg >= 0 ? P.gyroMagneticOffsetDeg : std::uniform_real_distribution<double>(0, 360)(rng);
  for (double &ph : magPhase) ph = std::uniform_real_distribution<double>(0, 2 * M_PI)(rng);
  rx = (W.sx + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  ry = (W.sy + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  rhead = wrap360(gauss(P.placeSigmaDeg));
  cpX = W.sx; cpY = W.sy;
  visited.assign(W.W * W.H, false);
  blueVisits.assign(W.W * W.H, 0); blueGood.assign(W.W * W.H, 0); cpVisited.assign(W.W * W.H, false);
  if (!tracePath.empty() || live) {
    if (live) traceOut = &std::cout;
    else { traceFile.open(tracePath); traceOut = &traceFile; }
    *traceOut << "{\"scenario\":\"" << sc->name << "\",\"seed\":" << seed << ",\"ideal\":" << (ideal ? "true" : "false") << ",\"sets\":\"";
    for (const std::string &x : sets) *traceOut << jsonEsc(x) << " ";
    *traceOut << "\",\"robot\":{\"front\":" << bodyFront << ",\"back\":" << bodyBack << ",\"half\":" << bodyHalfWidth << ",\"axle\":" << axleOffset
          << ",\"track\":" << P.trackWidth << ",\"wheel\":" << P.wheelDiameter << ",\"colour\":[" << colourFwd << "," << colourLeft << "," << colourHeight << "],\"tof\":[";
    for (int i = 1; i <= 7; i++) *traceOut << (i > 1 ? "," : "") << "[" << tof[i].fwd << "," << tof[i].left << "," << tof[i].facing << "," << tof[i].height << "]";
    *traceOut << "],\"cone\":" << P.tofConeDeg << ",\"outline\":[";
    for (size_t k = 0; k < outF.size(); k++) *traceOut << (k ? "," : "") << "[" << outF[k] << "," << outL[k] << "]";
    *traceOut << "]},\"map\":{\"size\":" << MAP_SIZE << ",\"floors\":" << NUM_FLOORS << ",\"start\":" << START_FLOOR << "}";
    *traceOut << ",\"field\":{\"W\":" << W.W << ",\"H\":" << W.H << ",\"sx\":" << W.sx << ",\"sy\":" << W.sy << ",\"rampDeg\":" << W.rampDeg
          << ",\"rampX0\":" << W.rampX0 << ",\"rampLen\":" << W.rampLen << ",\"levelHeight\":[" << W.levelHeight[0] << "," << W.levelHeight[1] << "]"
          << ",\"wallT\":" << P.wallThicknessMm << ",\"bumpH\":" << W.bumpH << ",\"bumpW\":" << W.bumpW << ",\"obstacles\":[";
    for (size_t k = 0; k < W.obstacles.size(); k++) *traceOut << (k ? "," : "") << "[" << (int)W.obstacles[k].x << "," << (int)W.obstacles[k].y << "," << (int)W.obstacles[k].r << "]";
    *traceOut << "],\"obstacleKg\":[";
    for (size_t k = 0; k < obstacleMass.size(); k++) *traceOut << (k ? "," : "") << obstacleMass[k];
    *traceOut << "],\"bumps\":[";
    bool firstBump = true;
    for (int i = 0; i < W.W * W.H; i++) if (W.bump[i]) { *traceOut << (firstBump ? "" : ",") << "[" << i << "," << W.bump[i] << "]"; firstBump = false; }
    *traceOut << "],\"tiles\":[";
    for (int i = 0; i < W.W * W.H; i++) {
      int walls = 0;
      for (int d = 0; d < 4; d++) walls |= W.wall[i][d] << d;
      *traceOut << (i ? "," : "") << "[" << W.type[i] << "," << W.level[i] << "," << walls << "]";
    }
    *traceOut << "]}}\n";
  }

  std::string reason;
  simReady = true;
  try {
    setup();
    runStarted = true;
    runStartUs = tUs;
    if (live) liveRebase();
    lastProgressUs = tUs;
    lastTileX = W.sx; lastTileY = W.sy;
    visited[W.id(W.sx, W.sy)] = true;
    curTile = W.id(W.sx, W.sy); lastStillX = rx; lastStillY = ry;
    lastLevel = W.level[W.id(W.sx, W.sy)];
    while (true) loop();
  } catch (const SimEnd &e) { reason = e.reason; }

  // ---- results ----
  int tx = (int)std::floor(rx / field::TILE), ty = (int)std::floor(ry / field::TILE);
  bool home = reason == "home" && tx == W.sx && ty == W.sy;
  std::vector<bool> reach = W.reach();
  int total = 0, covered = 0, wallsOk = 0, wallsAll = 0;
  syncActiveFloor();
  for (int i = 0; i < W.W * W.H; i++) {
    if (!reach[i] || W.type[i] == field::RAMP_T) continue;
    total++;
    if (!visited[i]) continue;
    covered++;
    int x = i % W.W, y = i / W.W, f = START_FLOOR + W.level[i] - W.level[W.id(W.sx, W.sy)];
    if (f < 0 || f >= NUM_FLOORS) continue;
    Tile &t = floorGrid(f)[x - W.sx + MAP_SIZE / 2][y - W.sy + MAP_SIZE / 2];
    for (int d = 0; d < 4; d++) { wallsAll++; if (t.getVisited() && t.getWall(d) == W.hasWall(x, y, d)) wallsOk++; }
  }
  double runS = runStarted ? (tUs - runStartUs) / 1e6 : 0;
  // RCJ 2026 scoring, navigation part (no victims or rescue kits in the simulator)
  int sbv = 0, bluePts = 0, checkpoints = 0;
  for (int i = 0; i < W.W * W.H; i++) {
    if (blueGood[i] > 0) { sbv++; bluePts += std::max(0, 30 - 10 * (blueVisits[i] - 1)); } // 5.6.6: revisits cost 10 each
    if (cpVisited[i]) checkpoints++;
  }
  int srn = (int)rampsDone.size(), bumpsCrossed = (int)bumpsDone.size();
  int reliability = std::max(0, sbv * 10 - lops * 15);          // 5.6.7
  int exitBonus = home ? sbv * 10 + srn * 5 : 0;                // 5.6.12
  int score = bluePts + checkpoints * 10 + srn * 10 + bumpsCrossed * 5 + reliability + exitBonus; // 5.6.8: 5 per speed-bump tile
  std::vector<char> resultBuf(1024 + lopReasons.size() + 2 * lcdText.size()); // a run with many restarts has a long list of reasons
  char *result = resultBuf.data();
  std::snprintf(result, resultBuf.size(), "{\"scenario\":\"%s\",\"seed\":%ld,\"ideal\":%s,\"end\":\"%s\",\"home\":%s,\"time_s\":%.1f,\"coverage\":%.3f,"
              "\"tiles\":%d,\"reachable\":%d,\"map_walls\":%.3f,\"lops\":%d,\"lop_reasons\":\"%s\",\"wall_contact_s\":%.1f,"
              "\"score\":%d,\"score_parts\":\"blue %d (%d tiles), checkpoints %d, ramps %d, speed bumps %d, reliability %d, exit %d\","
              "\"lost_fraction\":%.3f,\"ramp_crossings\":%d,\"ramp_deg\":%.1f,\"obstacle_pushed_mm\":%.0f,\"last_lcd\":\"%s\"}\n",
              sc->name, seed, ideal ? "true" : "false", reason.c_str(), home ? "true" : "false", runS, total ? (double)covered / total : 1.0,
              covered, total, wallsAll ? (double)wallsOk / wallsAll : 1.0, lops, lopReasons.c_str(), contactUs / 1e6,
              score, bluePts, sbv, checkpoints * 10, srn * 10, bumpsCrossed * 5, reliability, exitBonus,
              syncSamples ? (double)lostSamples / syncSamples : 0.0, rampCrossings, W.rampDeg, pushedMm, jsonEsc(lcdText).c_str());
  if (traceOut) {
    if (runStarted) traceFrame(runS);
    *traceOut << "{\"result\":" << std::string(result, std::strlen(result) - 1) << ",\"t\":" << (int)(runS * 1000) << "}\n";
  }
  if (live) traceOut->flush();
  else std::fputs(result, stdout);
  return 0;
}
