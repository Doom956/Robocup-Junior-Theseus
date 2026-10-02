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
#include <random>
#include <sstream>
#include <string>

// =====================================================================================
// Assumptions (tune these to the real robot; see README)
// =====================================================================================
struct Params {
  // Motors: Pololu 195.3125:1 Metal Gearmotor 20Dx44L mm 12V (#3493) on a Carobot V3 shield
  // (TB6612FNG + PCA9685, like the Adafruit Motor Shield V2), powered by a 3S LiPo.
  double batteryVoltage = 11.8;   // 3S LiPo: 12.6 V full, 11.1 V nominal (assumed part-charged)
  double noLoadRpmAt12V = 72;     // datasheet
  double stallKgcmAt12V = 10;     // datasheet (extrapolated); gearbox limit is 5 kg.cm
  double frictionFracAt12V = 0.05; // no-load current / stall current = 80 mA / 1.6 A (datasheet)
  double stictionFactor = 1.3;    // extra friction to start from standstill
  double robotMassKg = 1.15;      // ESTIMATE from the parts' datasheets + printed parts (README "Robot mass") - weigh it
  double driverOhms = 0.5;        // TB6612FNG output ON resistance, upper + lower, typ (Toshiba datasheet); motor 12 V / 1.6 A = 7.5 ohm
  double wheelMu = 0.8;           // ASSUMED - silicone wheels on the field's floor
  double motorTau = 0.05;         // motor + gearbox speed time constant, s
  double motorGainSigma = 0.03;   // per-motor speed difference (fraction)
  double trackWidth = 156;        // left-right wheel spacing, mm (CAD)
  double skidFactor = 1.3;        // skid steering turns slower than the ideal track predicts
  double tractionMean = 0.97, tractionSigma = 0.02, tractionTau = 0.5; // wheel grip (ground speed / wheel speed)
  double wallNudgeMm = 0.3;       // how far a wall can shove a turning robot sideways, mm per ms (0 = it jams)
  double wheelDiameter = 80;
  double encoderCountsPerRev = 5 * 195.3125; // 20 CPR encoder, code counts rising edges of one channel = 5 per motor turn
  // Gyro: "magnetic" = the BNO055 reports heading from magnetic north (team notes: "the heading is
  // always global") instead of from the start direction. Maze-to-north angle, deg; -1 = random per run.
  // -1 = do what the firmware asks for: bno.begin() defaults to NDOF, which the BNO055 datasheet (3.3.3.5)
  // defines as absolute orientation (heading from magnetic north); IMUPLUS is relative to the start.
  double gyroMagnetic = -1, gyroMagneticOffsetDeg = -1, gyroMagneticAfterS = 0; // switch to magnetic after this many s of the run
  double magErrorDeg = 2.5;       // BNO055 datasheet: magnetometer heading accuracy +-2.5 deg (fully calibrated, ideal)
  double tofMinReliableMm = 30;   // team notes: "can't handle below 30mm"
  // VL53L0X datasheet table 12: standard deviation 4 % at 33 ms (white target, incl. part-to-part); table 14:
  // offset drift < 3 %. Here ~3 % reading-to-reading + a fixed per-sensor offset.
  double tofNoiseMm = 1.5, tofNoisePct = 0.03, tofOffsetSigma = 5, tofMaxRange = 1200, tofConeDeg = 25, tofPeriodUs = 33000;
  double gyroNoiseDeg = 0.2, gyroDriftSigmaDegPerMin = 0.5, pitchNoiseDeg = 0.5;
  double colourNoise = 0.04;
  double placeSigmaMm = 8, placeSigmaDeg = 2; // how accurately a person places the robot
  double rampMinDeg = 15, rampMaxDeg = 25;
  double runTimeS = 480, stuckTimeoutS = 60, lopPauseS = 3;
} P;

// --set name=value overrides any of the numbers above
static std::map<std::string, double *> paramTable() {
  return {
    {"batteryVoltage", &P.batteryVoltage}, {"noLoadRpmAt12V", &P.noLoadRpmAt12V}, {"stallKgcmAt12V", &P.stallKgcmAt12V},
    {"frictionFracAt12V", &P.frictionFracAt12V}, {"stictionFactor", &P.stictionFactor}, {"robotMassKg", &P.robotMassKg},
    {"wheelMu", &P.wheelMu}, {"motorTau", &P.motorTau},
    {"motorGainSigma", &P.motorGainSigma}, {"trackWidth", &P.trackWidth}, {"skidFactor", &P.skidFactor},
    {"tractionMean", &P.tractionMean}, {"tractionSigma", &P.tractionSigma},
    {"gyroMagnetic", &P.gyroMagnetic}, {"magErrorDeg", &P.magErrorDeg}, {"driverOhms", &P.driverOhms}, {"gyroMagneticOffsetDeg", &P.gyroMagneticOffsetDeg}, {"gyroMagneticAfterS", &P.gyroMagneticAfterS},
    {"tofMinReliableMm", &P.tofMinReliableMm},
    {"wallNudgeMm", &P.wallNudgeMm}, {"tofNoiseMm", &P.tofNoiseMm}, {"tofNoisePct", &P.tofNoisePct},
    {"tofOffsetSigma", &P.tofOffsetSigma}, {"tofMaxRange", &P.tofMaxRange}, {"tofConeDeg", &P.tofConeDeg},
    {"gyroNoiseDeg", &P.gyroNoiseDeg}, {"gyroDriftSigmaDegPerMin", &P.gyroDriftSigmaDegPerMin},
    {"pitchNoiseDeg", &P.pitchNoiseDeg}, {"colourNoise", &P.colourNoise}, {"placeSigmaMm", &P.placeSigmaMm},
    {"placeSigmaDeg", &P.placeSigmaDeg}, {"rampMinDeg", &P.rampMinDeg}, {"rampMaxDeg", &P.rampMaxDeg},
    {"runTimeS", &P.runTimeS}, {"stuckTimeoutS", &P.stuckTimeoutS}};
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
  P.motorGainSigma = 0; P.tractionMean = 1; P.tractionSigma = 0;
  P.tofNoiseMm = 0; P.tofNoisePct = 0; P.tofOffsetSigma = 0; P.magErrorDeg = 0;
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
static double axleOffset = 57.8;

// referee / metrics
static bool switchHigh = false;
static double pauseUntilUs = 0;
static int lastTileX = -1, lastTileY = -1, cpX = 0, cpY = 0;
static double lastProgressUs = 0;
static int lops = 0, rampCrossings = 0, lastLevel = -1;
static std::vector<bool> visited;
static double contactUs = 0;
static bool inContact = false;
static int syncSamples = 0, lostSamples = 0;
static std::string lopReasons;
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
static void robotToWorld(double fwd, double left, double &wx, double &wy) {
  double h = rad(rhead);
  wx = rx + fwd * std::sin(h) - left * std::cos(h);
  wy = ry + fwd * std::cos(h) + left * std::sin(h);
}

// distance from (ox,oy) along (dx,dy) to the first wall, walking the tile grid.
// *facing = how square-on the wall is to the ray (1 = head-on).
static double raycast(double ox, double oy, double dx, double dy, double maxd, double *facing = nullptr) {
  const double S = field::TILE, INF = 1e9;
  int ix = (int)std::floor(ox / S), iy = (int)std::floor(oy / S);
  int stepX = dx > 0 ? 1 : -1, stepY = dy > 0 ? 1 : -1;
  double tMaxX = std::fabs(dx) < 1e-12 ? INF : ((dx > 0 ? (ix + 1) * S - ox : ox - ix * S) / std::fabs(dx));
  double tMaxY = std::fabs(dy) < 1e-12 ? INF : ((dy > 0 ? (iy + 1) * S - oy : oy - iy * S) / std::fabs(dy));
  double tDX = std::fabs(dx) < 1e-12 ? INF : S / std::fabs(dx), tDY = std::fabs(dy) < 1e-12 ? INF : S / std::fabs(dy);
  for (int guard = 0; guard < 64; guard++) {
    if (tMaxX < tMaxY) {
      if (tMaxX > maxd) return INF;
      if (W.hasWall(ix, iy, stepX > 0 ? 1 : 3)) { if (facing) *facing = std::fabs(dx); return tMaxX; }
      ix += stepX; tMaxX += tDX;
    } else {
      if (tMaxY > maxd) return INF;
      if (W.hasWall(ix, iy, stepY > 0 ? 0 : 2)) { if (facing) *facing = std::fabs(dy); return tMaxY; }
      iy += stepY; tMaxY += tDY;
    }
  }
  return INF;
}

// does the robot body at pose (x,y,head) touch any wall?
static bool collides(double x, double y, double head) {
  double h = rad(head), fx = std::sin(h), fy = std::cos(h), lx = -std::cos(h), ly = std::sin(h);
  int cx = (int)std::floor(x / field::TILE), cy = (int)std::floor(y / field::TILE);
  for (int ty = cy - 1; ty <= cy + 1; ty++)
    for (int tx = cx - 1; tx <= cx + 1; tx++)
      for (int d = 0; d < 4; d++) {
        if (!W.hasWall(tx, ty, d)) continue;
        double x0 = tx * field::TILE, y0 = ty * field::TILE, S = field::TILE;
        double ax, ay, bx, by; // wall segment
        if (d == 0) { ax = x0; ay = y0 + S; bx = x0 + S; by = y0 + S; }
        else if (d == 1) { ax = x0 + S; ay = y0; bx = x0 + S; by = y0 + S; }
        else if (d == 2) { ax = x0; ay = y0; bx = x0 + S; by = y0; }
        else { ax = x0; ay = y0; bx = x0; by = y0 + S; }
        // segment in robot frame, clipped against the body box (Liang-Barsky)
        double pf = (ax - x) * fx + (ay - y) * fy, pl = (ax - x) * lx + (ay - y) * ly;
        double qf = (bx - x) * fx + (by - y) * fy - pf, ql = (bx - x) * lx + (by - y) * ly - pl;
        double t0 = 0, t1 = 1, p[4] = {-qf, qf, -ql, ql}, q[4] = {pf + bodyBack, bodyFront - pf, pl + bodyHalfWidth, bodyHalfWidth - pl};
        bool hit = true;
        for (int k = 0; k < 4 && hit; k++) {
          if (std::fabs(p[k]) < 1e-12) { if (q[k] < 0) hit = false; continue; }
          double r = q[k] / p[k];
          if (p[k] < 0) { if (r > t1) hit = false; else if (r > t0) t0 = r; }
          else { if (r < t0) hit = false; else if (r < t1) t1 = r; }
        }
        if (hit) return true;
      }
  return false;
}

// =====================================================================================
// Physics + referee, advanced by every hardware call
// =====================================================================================
static void lackOfProgress(const char *why);

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
  double gravity = wheelLoadN * std::sin(rad(rpitch)) * rM / stallNm;                    // nose up = positive
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
    double resist = friction + skidSpin * turning;
    if (std::fabs(m.speed) < 1) resist *= P.stictionFactor;
    double eff = std::fabs(d) - resist - gravity * (d >= 0 ? 1 : -1); // driving uphill costs, downhill helps
    double target = eff > 0 ? (d > 0 ? 1 : -1) * eff * vNoLoad * m.gain : 0;
    m.speed += (target - m.speed) * (1 - std::exp(-dt / P.motorTau));
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
  double v = (vL + vR) / 2 * std::cos(rad(rpitch));
  double turnDeg = (vL - vR) / (P.trackWidth * P.skidFactor) * dt * 180 / M_PI; // left faster -> clockwise
  double nh = rhead + turnDeg, mid = rad(rhead + turnDeg / 2);
  double nx = rx + v * dt * std::sin(mid), ny = ry + v * dt * std::cos(mid);
  inContact = false;
  if (!collides(nx, ny, nh)) { rx = nx; ry = ny; rhead = nh; }
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

// largest share of the robot's footprint (CAD outline) over any one black tile, 0..1
static double nextBlackCheckUs = 0;
static double blackShare() {
  int cx = (int)std::floor(rx / field::TILE), cy = (int)std::floor(ry / field::TILE);
  bool near = false;
  for (int y = cy - 1; y <= cy + 1 && !near; y++)
    for (int x = cx - 1; x <= cx + 1; x++)
      if (W.typeAt(x, y) == field::BLACK_T) { near = true; break; }
  if (!near) return 0;
  const int N = 12; // N x N sample points over the footprint
  std::map<int, int> hits;
  int best = 0;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) {
      double fwd = -bodyBack + (bodyFront + bodyBack) * (i + 0.5) / N;
      double left = -bodyHalfWidth + 2 * bodyHalfWidth * (j + 0.5) / N;
      double wx, wy;
      robotToWorld(fwd, left, wx, wy);
      int x = (int)std::floor(wx / field::TILE), y = (int)std::floor(wy / field::TILE);
      if (W.typeAt(x, y) == field::BLACK_T) best = std::max(best, ++hits[W.id(x, y)]);
    }
  return (double)best / (N * N);
}

static void refereeTick() {
  Pausemaze = switchHigh; // what pauseTask() does every 10 ms
  if (!runStarted) return;
  if (switchHigh && tUs >= pauseUntilUs) switchHigh = false;
  double runS = (tUs - runStartUs) / 1e6;
  if (runS >= P.runTimeS) throw SimEnd{"time"};
  int tx = (int)std::floor(rx / field::TILE), ty = (int)std::floor(ry / field::TILE);
  if (W.in(tx, ty) && (tx != lastTileX || ty != lastTileY)) {
    lastTileX = tx; lastTileY = ty; lastProgressUs = tUs;
    int i = W.id(tx, ty), t = W.type[i];
    if (t != field::RAMP_T) {
      visited[i] = true;
      if (lastLevel >= 0 && W.level[i] != lastLevel) rampCrossings++;
      lastLevel = W.level[i];
    }
    if (t == field::SILVER_T) { cpX = tx; cpY = ty; }
  }
  // RCJ rule: more than half of the robot over a black tile is a lack of progress
  if (!switchHigh && tUs >= nextBlackCheckUs) {
    nextBlackCheckUs = tUs + 5000;
    if (blackShare() > 0.5) lackOfProgress("drove onto a black tile");
  }
  if (!switchHigh && (tUs - lastProgressUs) / 1e6 > P.stuckTimeoutS) lackOfProgress("stuck");
  // is the code's map position right? (checked while it reads the walls of a tile)
  if (state == SENSE_TILE && !switchHigh && W.in(tx, ty) && W.type[W.id(tx, ty)] != field::RAMP_T) {
    syncSamples++;
    int ef = START_FLOOR + W.level[W.id(tx, ty)] - W.level[W.id(W.sx, W.sy)];
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
  if (s.find("back to start") != std::string::npos) throw SimEnd{"home"};
  if (s.find("no path found") != std::string::npos) throw SimEnd{"no path"};
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
    else if (a == "--verbose") simSerialEcho = true;
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
    else { std::printf("usage: physics --scenario flat|loops|big|ramp|bigramp --seed S [--ideal] [--set name=value]... [--verbose] [--trace FILE] [--geometry robot_geometry.json]\n"); return 2; }
  }
  if (geomPath.empty()) {
    std::string exe = argv[0];
    size_t slash = exe.find_last_of("/\\");
    geomPath = (slash == std::string::npos ? std::string(".") : exe.substr(0, slash)) + "/../cad/robot_geometry.json";
  }
  if (!loadGeometry(geomPath)) { std::printf("{\"error\":\"could not read 7 sensors from %s\"}\n", geomPath.c_str()); return 2; }
  if (ideal) makeIdeal();
  for (const std::string &s : sets)
    if (!setParam(s)) { std::printf("{\"error\":\"unknown --set %s\"}\n", s.c_str()); return 2; }
  if (selftest) {
    rng.seed(1);
    for (int i = 1; i <= 4; i++) { mot[i].gain = 1 + gauss(P.motorGainSigma); mot[i].traction = P.tractionMean; }
    for (int i = 1; i <= 7; i++) tof[i].offset = gauss(P.tofOffsetSigma);
    return selfTest();
  }

  const field::Scenario *sc = nullptr;
  for (const auto &s : field::SCENARIOS) if (scenario == s.name) sc = &s;
  if (!sc) { std::printf("{\"error\":\"unknown scenario %s\"}\n", scenario.c_str()); return 2; }
  std::mt19937 fieldRng((unsigned)seed * 2654435761u + (unsigned)(sc - field::SCENARIOS));
  W = field::generate(*sc, fieldRng, P.rampMinDeg, P.rampMaxDeg);
  rng.seed((unsigned)seed * 7919u + 4242u);

  for (int i = 1; i <= 4; i++) { mot[i].gain = 1 + gauss(P.motorGainSigma); mot[i].traction = P.tractionMean; }
  for (int i = 1; i <= 7; i++) { tof[i].offset = gauss(P.tofOffsetSigma); tof[i].phaseUs = std::uniform_real_distribution<double>(0, P.tofPeriodUs)(rng); }
  gyroBiasDegPerUs = gauss(P.gyroDriftSigmaDegPerMin) / 60e6;
  magOffsetDeg = P.gyroMagneticOffsetDeg >= 0 ? P.gyroMagneticOffsetDeg : std::uniform_real_distribution<double>(0, 360)(rng);
  for (double &ph : magPhase) ph = std::uniform_real_distribution<double>(0, 2 * M_PI)(rng);
  rx = (W.sx + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  ry = (W.sy + 0.5) * field::TILE + gauss(P.placeSigmaMm);
  rhead = wrap360(gauss(P.placeSigmaDeg));
  cpX = W.sx; cpY = W.sy;
  visited.assign(W.W * W.H, false);
  if (!tracePath.empty() || live) {
    if (live) traceOut = &std::cout;
    else { traceFile.open(tracePath); traceOut = &traceFile; }
    *traceOut << "{\"scenario\":\"" << sc->name << "\",\"seed\":" << seed << ",\"ideal\":" << (ideal ? "true" : "false") << ",\"sets\":\"";
    for (const std::string &x : sets) *traceOut << jsonEsc(x) << " ";
    *traceOut << "\",\"robot\":{\"front\":" << bodyFront << ",\"back\":" << bodyBack << ",\"half\":" << bodyHalfWidth << ",\"axle\":" << axleOffset
          << ",\"track\":" << P.trackWidth << ",\"wheel\":" << P.wheelDiameter << ",\"colour\":[" << colourFwd << "," << colourLeft << "," << colourHeight << "],\"tof\":[";
    for (int i = 1; i <= 7; i++) *traceOut << (i > 1 ? "," : "") << "[" << tof[i].fwd << "," << tof[i].left << "," << tof[i].facing << "," << tof[i].height << "]";
    *traceOut << "],\"cone\":" << P.tofConeDeg << "},\"map\":{\"size\":" << MAP_SIZE << ",\"floors\":" << NUM_FLOORS << ",\"start\":" << START_FLOOR << "}";
    *traceOut << ",\"field\":{\"W\":" << W.W << ",\"H\":" << W.H << ",\"sx\":" << W.sx << ",\"sy\":" << W.sy << ",\"rampDeg\":" << W.rampDeg
          << ",\"rampX0\":" << W.rampX0 << ",\"rampLen\":" << W.rampLen << ",\"levelHeight\":[" << W.levelHeight[0] << "," << W.levelHeight[1] << "],\"tiles\":[";
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
  std::vector<char> resultBuf(1024 + lopReasons.size() + 2 * lcdText.size()); // a run with many restarts has a long list of reasons
  char *result = resultBuf.data();
  std::snprintf(result, resultBuf.size(), "{\"scenario\":\"%s\",\"seed\":%ld,\"ideal\":%s,\"end\":\"%s\",\"home\":%s,\"time_s\":%.1f,\"coverage\":%.3f,"
              "\"tiles\":%d,\"reachable\":%d,\"map_walls\":%.3f,\"lops\":%d,\"lop_reasons\":\"%s\",\"wall_contact_s\":%.1f,"
              "\"lost_fraction\":%.3f,\"ramp_crossings\":%d,\"ramp_deg\":%.1f,\"last_lcd\":\"%s\"}\n",
              sc->name, seed, ideal ? "true" : "false", reason.c_str(), home ? "true" : "false", runS, total ? (double)covered / total : 1.0,
              covered, total, wallsAll ? (double)wallsOk / wallsAll : 1.0, lops, lopReasons.c_str(), contactUs / 1e6,
              syncSamples ? (double)lostSamples / syncSamples : 0.0, rampCrossings, W.rampDeg, jsonEsc(lcdText).c_str());
  if (traceOut) {
    if (runStarted) traceFrame(runS);
    *traceOut << "{\"result\":" << std::string(result, std::strlen(result) - 1) << ",\"t\":" << (int)(runS * 1000) << "}\n";
  }
  if (live) traceOut->flush();
  else std::fputs(result, stdout);
  return 0;
}
