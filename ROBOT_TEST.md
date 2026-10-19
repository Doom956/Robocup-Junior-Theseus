# Robot tests

**Not yet confirmed on the robot** (keep this list current; at most ~5 before the next maze session):
- 2l ramp exit (drive 155 mm from the ramp's end)
- 2m centring: back sensor only for this tile's wall
- 2n no-progress 80 mm window + end-of-move check (bench: M6/M8 caught 5 of 5, a loose box pushed without a false stop,
  2026-10-18; still to do: count false stops in a maze run)
- 2o no-progress check stays on after two stops at one edge (revert if the maze shows repeated stops at one edge with
  nothing there, as on 2026-10-17 partB8)
Confirmed 2026-10-18 (no maze): run clock starts at 0 at the first switch flip (switched on 2.5 min earlier; firmware 9cdce30).
Last good maze version: tag `known-good-2026-10-17`.

Changes on this branch depend on how real the simulator's motors, grip, turning, gyro or sensors are,
so they stay off `main` until they pass on the real robot. Each one has a short test below: what to
set up, what to run, and what to watch for in the Serial monitor.

Section 3 lists logic fixes that are on main now (ramp up/down, 5 s stop when backing into a blue
tile, bench mode); worth checking in the same session. Whole branch vs main in the simulator (600
comp fields, main at d8c8c75): score +19.2 +-4.5, time lost 29% -> 10%, back home 64% -> 86%. With the
simulator fitted to the bench results (main at d0d87ee): +25.8 +-4.5, time lost 32% -> 10%, home 62% -> 84%.

**Results so far (2026-10-06, robot-test at 816307b)**

| Test | Result |
|---|---|
| 1. Bench mode | done, 3 runs (2 on main, 1 on robot-test); see CLAUDE.md. Turning needs PWM 25-30, wheel A lags at low power |
| 2d c. Gyro drift | IMUPLUS 0.00 deg in 2 min. On main (NDOF) one run jumped 12.5 deg while standing still: passed, and NDOF is a real risk |
| 2e / steering a | corrects itself, but then waits ~4 s; drives very close to a wall on its right. **Repeat with the Serial log** (wall on the right, then on the left) |
| 3. Ramp up and down | 6 of 6 |
| 2026-10-07: pause switch | read "pause" almost always: the board's pull-down resistor came loose (pin 22 floats in the run position). Fixed in code on main (`INPUT_PULLDOWN`); re-solder the resistor (pin 22 to GND, ~10 k) when you can |
| 2026-10-07: 4 s wait | the camera thread, not navigation: it stops for any camera byte (`!= -1`) and `detectCam` waits 4 s for a letter. Gone with the cameras off. Victim code: not changed (team decision) |
| 2026-10-07: stops at the back of tiles | seen on the robot; fixed on this branch in 2f, to test |
| 2026-10-07: weaves left and right | seen on the robot (main); steering gain halved on this branch in 2g, to test |
| 2026-10-07: `can't find gyro` at one start-up | heading then stuck at 0.00, turns never finish. Cause: the gyro cable had come unplugged. Check it is seated before each session |
| 2026-10-16/17: move tests (BENCH_MODE 2) | turns 90 deg in 0.75-1.06 s, 180 in 1.4-1.8 s, all within 2 deg; 5-15 deg turns all move (2h passes). One-tile moves 297-326 mm by the back sensor. Wall following holds 80 mm +-8. Blocked by an obstacle between the front sensors after a free start: not caught (fixed in 2k) |
| 2026-10-17: maze runs (B1, logs/2026-10-17) | first runs lost (white reference wrong, walls missed when 8+ cm out of place, ramp misread, loops). After the fixes in 2k: home in 3 of the last 4 runs, the last 2 clean (2:28 each, both ramps up and down, black stop, no false alarms) |
| 1. Obstacles, 2. relocalize | not yet |

**Suggested order for a test session**

1. **Bench mode first** (`BENCH_MODE 1` in `Main/main.cpp`, see `Main/bench.cpp`): ~15 minutes, gives the
   sensor offsets, turning power, gyro drift and floor colours. No LCD or USB cable needed: the LED blinks
   slowly while it waits for the pause switch, stays on during a test and blinks fast when done; then plug
   in USB and flip the switch to print every result. Copy the `[BENCH]` lines; they tell us how far the
   simulator is from this robot. Set `BENCH_MODE` back to 0. New test 11 (at the end, one switch flip per
   sensor): a flat wall exactly 100 mm in front of each distance sensor (a printed 100 mm spacer); its last
   line is `SENSOR_OFFSET_MM` for `Main/Distance.cpp`, ready to paste. It leaves the side sensors at 0 on
   purpose: the one-wall steering is tuned to side sensors that read ~10 mm long, as these do, and correcting
   them lost points in the simulator (-10). Bench test 1 doesn't show the right sensors reading more than the
   left (right minus left was 24, -2 and 30 mm in 3 runs: where the robot was placed by hand). Tests 1-10 can be skipped by
   flipping through them if they were done already.
2. **Steering** (last section, 2e and 2f): a few tiles along a corridor; no weaving, no big turns off a wall, each move ends in the middle of the tile.
3. **Ramp up and down** (section 3): the floor change.
4. **Obstacles** (section 1, tests a-e), then **relocalize** (section 2).
5. A few full runs on a practice field with an obstacle or two; save the Serial logs.

**Tile accuracy check (over/undershoot), every session.** Cameras off, record with `python tools/serial_log.py
log.txt`, let it explore a few minutes, then `python tools/robot_log.py log.txt`. Look at three lines:

| Line | Good | If not |
|---|---|---|
| `stops along the tile` | most moves within 10 mm; under 1 in 10 more than 30 mm short or past | mostly **short** (negative median): raise the 55 in `fwd()` (2f) to 60-65. Mostly **past** (positive): lower it to 50, or check the battery (a full pack drives further) |
| `turns: ... median error` | under 2 deg, none over 5 | overshoots: turn gain 8 -> 6 (2h); stops short: `TURN_MIN_PWM` 45 -> 50 |
| `moves started ... deg off` | median under 2 deg | squaring up (`parallel()`) isn't working: check the side sensors |

The sim's own run of this branch gives about: 55 of 96 moves within 10 mm, the rest median -12 mm, 3 short and 8
past by more than 30 mm; turns median 0.7 deg. A real robot close to that is behaving like the sim says.

## 1. Obstacle ahead: treat it like a wall first (strategy change)

**What changed** (`Main/movement.cpp` `fwd()`, `Main/main.cpp` `handleShortMove()`):

- Start of a move, one front sensor at 90 mm or less and the other at 120 mm or more (the old
  obstacle trigger). If the robot is more than 3 deg off the tile direction, it first turns square and
  reads again. Still one-sided: the first time at this edge it doesn't drive around it. It stops,
  the edge is blocked in the map like a wall, and it plans elsewhere.
- During the move, the same one-sided reading in the first 250 mm, while pointing within 4 deg of the
  tile direction: stop, back up to the centre of the start tile, block the edge.
- If the planner comes back to that edge later (nothing else left to explore, or the only way home
  crosses it), the robot drives around it with `obstacleavoidance()` as before.

**Why:** in the simulator, obstacle avoidance was the first thing that put the map position wrong in
62 of 146 lost runs. 42 of those were false alarms: the end of the next tile's wall, seen by one front
sensor when the robot was turned or off-centre. Another 23 runs got lost pushing into an obstacle the
start-of-move check hadn't seen yet.

**Simulator, 600 comp fields vs main (`cc0fa71`):** score +8.9 +-4.6, time lost 44% -> 24%
(-20 +-3 pts), back home 48% -> 66% (+17 +-5 pts), explored 66% -> 67%, restarts per run 1.19 -> 0.82.
With the assumed values varied (300 fields each), time lost went down 13-25 pts and back home went up
9-19 pts every time:

| Changed in the sim | Score | Time lost | Back home |
|---|---|---|---|
| wheelMu 0.6 | +13.4 +-6.8 | -21 +-5 | +9 +-7 |
| wheelMu 1.2 | +7.5 +-6.8 | -17 +-5 | +9 +-8 |
| skidFactor 1.6 | +6.9 +-7.1 | -16 +-5 | +19 +-7 |
| motorGainSigma 0.08 | +8.8 +-6.3 | -13 +-5 | +19 +-7 |
| robotMassKg 1.3 | +11.5 +-6.6 | -21 +-5 | +17 +-7 |
| tofOffsetSigma 10 | +0.2 +-6.6 | -25 +-5 | +13 +-7 |
| gyroDriftSigmaDegPerMin 2 | +11.3 +-6.7 | -18 +-5 | +16 +-7 |
| obstacleRate 0.06 | +11.9 +-6.2 | -22 +-4 | +14 +-7 |

**Tests on the real robot:**

a. **Wall end, no obstacle (false alarm check).** A tile open on its left, with the next tile ahead
   walled on its left. Put the robot 40-50 mm toward the open side, then again turned about 10 deg
   toward it, and let it drive forward. Expected: `one-sided reading while not square: squaring up and
   looking again`, then it drives on. Bad: `obstacle ahead: blocking edge` with nothing there. Note the
   `measure(1)`/`measure(7)` values when it happens.
b. **Obstacle touching a side wall of the next tile** (5-9 cm cylinder reaching up to 10 cm into the
   path, RCJ 3.4.4), at the start, the middle and the far end of that tile. Expected: `obstacle left`/
   `obstacle right` then `obstacle ahead: blocking edge`, or `obstacle ahead during the move` then
   `[FWD] obstacle, backing up to start tile`. The robot never touches it, ends in the middle of its own
   tile and goes somewhere else.
c. **Dead end behind an obstacle** (the only way out passes it). After the block, once nothing else is
   left: `map looks finished early: cleared blocked edges, exploring again`, then the avoidance steps
   (`obstacle avoidance`, `turn step`, `paralleling step`, `fwd step`). Check that the tile the code
   thinks it is on matches the real one afterwards.
d. **Full run with no obstacles.** Count `obstacle ahead during the move` and `obstacle ahead: blocking
   edge` in the log. It should be about zero. Frequent ones mean the 4 deg / 250 mm limits need changing.

e. **Obstacle just past a blue tile's edge.** Blue tile ahead, obstacle touching a side wall in its
   first half. If the robot stops for it during the move with its nose already on blue, expect
   `[FWD] on blue: waiting 5 s first` before it backs up (otherwise the referee calls lack of progress
   for leaving a blue tile early). Sim: +1.2 +-0.9 points on 600 comp fields.

## 2. Relocalize: shift the position one tile when the walls don't match the map

**What changed** (`Main/main.cpp`, `relocateOnMismatch()`, from the `relocalize` branch): when the walls
read in SENSE_TILE disagree with what the map recorded for that (visited) tile, and the tile behind, or
exactly one other neighbour, matches the reading on all four sides with no wall in between, the code
moves its position there. Otherwise nothing changes.

**Simulator, 600 comp fields vs robot-test without it:** score +3.5 +-1.5, time lost -4 +-1 pts,
back home +2 +-2 pts, restarts 0.80 -> 0.64 per run. With noisier distance sensors (offsets x2, noise
x2, 5 mm extra noise), more grip or 4x gyro drift (300 fields each): +2.1 to +4.0 points, time lost
3-4 pts lower every time.

**Tests on the real robot:**

a. **It corrects a real mistake.** Let the robot explore 3-4 tiles, then pause, carry it back one tile
   (same heading) and resume. On the next tile read expect `walls don't match this tile but match the
   neighbour N: position corrected`, and the map position (LCD / Serial) is right again.
b. **It doesn't move a right position.** A normal run with no interference: count `position corrected`
   in the log. Each one should be at a place where the robot really was off by a tile (check against
   what you saw). A correction with the robot where the code thought means a wall was misread there:
   note the four `reading walls` values and the distance readings.

**2c. Check the walls before stopping at home.** When the code thinks it is back on the start tile, it
reads the walls first; if they don't match the start tile and relocalize can place the robot, it carries
on home instead of stopping (a stop on the wrong tile is a lack of progress). Sim, 600 comp fields on two
seed sets: +0.3 +-0.3 points and back home +1 pt both times; "stopped on the wrong tile" 40 -> 35.
Test: carry the robot one tile away from the start, on its way home, pointing along the route (pause,
move it, resume). Expect `walls don't match the start tile: not home yet`, then it drives to the start.

## 2b. Steering from a wall one tile away (open areas) - now also on main

**What changed** (`Main/Distance.cpp`, `sideCentringError()`): the steering only used side walls closer
than 200 mm, so in open areas the robot drifted 100-150 mm off-centre (then false obstacle alarms and
scraped corners). A side reading of 260-440 mm is the wall one tile further out (a centred robot reads
49 mm to its own tile's wall, 349 mm to the next; tiles are 300 mm), so it now counts as that reading
less 300. 200-260 mm stays unused (can't tell which wall). The 30 mm front/back check still applies.

**Simulator** (robot-test before it, same seeds): nominal 600 comp fields +8.9 +-4.1 points, time lost
-3 +-3 pts, back home +5 +-4 pts. With the assumed values varied (200 fields each): sensor offsets x2
+18.4 +-7.8, x0.4 -3.3 +-5.8, low grip -2.0 +-6.9, high grip +7.5 +-7.4, motor spread +9.8 +-8.1, gyro
drift x4 +3.4 +-7.5, slow robot +4.1 +-6.5: no clear harm in any.

**Depends on:** how well the VL53L0X reads a wall 25-45 cm away to the side (datasheet range 120 cm).

**Tests on the real robot:**

a. **Open area, wall one tile away.** A 2-tile-wide open area with a wall along one side only. Start
   the robot in the tile away from the wall, 5-6 cm off-centre, and let it drive 3 tiles along. It
   should end closer to the tile centre than it started; the `[CENTER]` lines show err moving to 0.
b. **Sensor readings at that range.** In bench mode test 1 (or with `measure(2/3/5/6)` printed), put a
   wall 35 cm from a side sensor: readings should be steady within about 1 cm.

## 2e. Steering limit: side walls can't turn the robot more than 10 deg off

**What changed** (`Main/movement.cpp`, `fwd()`, `MAX_STEER_DEG`): the side-wall steering had no limit on
how far it turned the robot. Once the robot is more than 10 deg off the tile direction, the side walls may
only steer it back toward that direction; steering further away is replaced by the gyro heading hold.
Within 10 deg nothing changes.

**Why:** in the simulator (200 comp fields, main), moves where the heading got more than 15 deg off were 12%
of all moves but held 53% of the moments the map position first went wrong (26% of moves past 30 deg lost
the position, about 1% of steady ones). Typical case: one wall, robot close to it; the one-wall target
(80 mm) asks for a big turn away, the two side sensors pressed under ~30 mm keep asking for more, and the
robot rotates 30-60 deg with its corner on the wall while the encoders count a tile. Turned that far, the
mid-move obstacle check (4 deg) is also off, so it pushed obstacles it had seen.

**Simulator:**

| Test | Score | Time lost | Back home |
|---|---|---|---|
| robot-test, 600 comp fields (seeds 1-600) | +6.1 +-3.7 | -4 +-2 pts | +8 +-4 pts |
| robot-test, seeds 601-1200 | +3.1 +-3.8 | -1 +-2 pts | +2 +-4 pts |
| main, seeds 1-600 | +6.2 +-4.0 | -4 +-3 pts | +4 +-5 pts |
| main, seeds 601-1200 | +7.4 +-4.2 | -2 +-3 pts | +1 +-5 pts |

With the assumed values varied (main, 300 fields each): wheelMu 0.6 +8.5 +-5.5, wheelMu 1.2 +5.7 +-5.6,
skidFactor 1.6 +2.3 +-5.0, motorGainSigma 0.08 +4.6 +-5.5, tofOffsetSigma 10 +2.8 +-5.9, tofOffsetSigma 2
+5.6 +-5.3, gyroDriftSigmaDegPerMin 2 +5.8 +-5.6, robotMassKg 1.3 +6.6 +-5.7: no harm in any.

**Depends on:** how hard the real robot's wall following steers, and what the side VL53L0X read when
they are pressed close to a wall (the sim follows the datasheet: unreliable under ~30 mm).

**Tests on the real robot:**

a. **One wall, robot close to it.** A corridor with a wall on one side only. Start the robot 2-3 cm from
   that wall, turned about 5 deg toward it, and let it drive 3 tiles. Expected: it turns away by at most
   about 10 deg, straightens, and the next `[FWD] entry hdg=` is within a few degrees of the tile direction.
   `[FWD] steering limited N times` may appear on the first tile.
b. **Full runs.** Count `[FWD] steering limited` in the log. A few per run is expected. On most moves means
   the real steering needs more than 10 deg: raise `MAX_STEER_DEG` to 15. Note also any move where the
   robot ended visibly turned or scraped a wall.

## 2f. End each tile at its target, never under PWM 55

**What changed** (`Main/movement.cpp`, `fwd()`): two things in the slow-down at the end of each tile.
- The drive power never goes under 55. It used to slow to about 20, but this robot needs about PWM 50 to
  keep all wheels turning (bench test 4: wheel A), so it stalled short of the middle.
- The loop stopped once the power fell under 25, which was always 46 encoder counts (12 mm) before its
  target. It now counts the slow-down from 46 counts past the target, so it stops on the target.

**Why:** on 2026-10-07 the robot was often at the back of the tiles. In the sim (with the weak motor A from
bench test 4) a quarter of the moves ended more than 30 mm short.

**Simulator** (600 comp fields vs robot-test before it):

| Test | Score | Time lost | Back home |
|---|---|---|---|
| minimum PWM 55, seeds 1-600 | +7.2 +-3.3 | -1 +-2 pts | +0 +-3 pts |
| minimum PWM 55, seeds 601-1200 | +5.5 +-3.0 | -0 +-2 pts | +2 +-3 pts |
| + stop on the target, seeds 1-600 | +2.9 +-2.9 | -1 +-2 pts | +4 +-3 pts |
| + stop on the target, seeds 601-1200 | +1.9 +-3.0 | -1 +-2 pts | +2 +-3 pts |

Where moves end along the tile (`stopcentre.py`, 60 fields; negative = short of the middle): before, median
-18 mm when `fwd()` ends, 25% more than 30 mm short, -5 mm after `centreAlong()`; now -6 mm, 6%, and 0 mm.

**Depends on:** the real robot's lowest moving power (bench test 4 said ~50) and how far it coasts at PWM 55.

**Tests on the real robot:**

a. **Middle of the tile.** A corridor of 4-5 tiles, cameras off. Let it drive along; after each move, before
   it reads the walls, check where it stopped. It should be within about 1-2 cm of the middle, not at the
   back of the tile. The end of each move shouldn't creep or pause.
b. **No overshoot.** Same corridor ending in a wall: on the last tile it must not touch the end wall.
   `centring along:` lines in the log should mostly show small numbers (under 20 mm).
## 2g. Side-wall steering gain 1 instead of 2

**What changed** (`Main/movement.cpp`, `fwd()`): `center_PID(2,0,0.5)` -> `center_PID(1,0,0.5)`.

**Why:** on 2026-10-07 the robot wove left and right along walls. The steering is in effect only a P
controller: `PID`'s D term divides by microseconds, so it does next to nothing, and a D term that works
(0.1 or 0.3 per second) made it worse in the sim by amplifying the distance-sensor noise. A lower gain
is the damping.

**Simulator** (robot-test before it; `weave.py`, 40 comp fields): heading swing during a tile median 7.2 ->
5.3 deg (90%: 14.8 -> 12.1), tiles where it swings back and forth 2+ times 47% -> 22%, 4+ times 8% -> 1%.
Score, 600 comp fields: -1.6 +-3.1 (seeds 1-600), +1.8 +-2.9 (601-1200); time lost +1 / +0 pts, home -0 /
+0 pts: no change. Gain 0.7 weaves even less (11%) but got home 2-3 pts less often, so not taken.

**Depends on:** how fast the real robot answers a steering command and how noisy its side sensors are.

**Tests on the real robot** (cameras off):

a. **Corridor with walls on both sides,** 4-5 tiles: it should go straight, no visible left-right swing.
b. **One wall only:** same; start 3-4 cm off the 80 mm target gap and check it settles within about a tile
   without swinging past it. In the log, the `[CENTER] ... err=` values should shrink without changing sign
   back and forth.
c. If it now drifts toward a wall and corrects too slowly, try 1.5 (between the old 2 and this 1).

## 2h. Turns that finish: minimum power 45, gain 8

**What changed** (`Main/movement.cpp`, `absoluteturn()`): `TURN_MIN_PWM` 20 -> 45, turn PID gain 4.5 -> 8.

**Why:** the robot only starts turning on the spot at PWM 25-30 (bench test 2), and turns about 5 deg/s
there. With power = 4.5 x the angle left, never under 20, every turn crawled or stalled a few degrees short
of the 2 deg finish window and waited out its time limit: in the sim 90 deg turns took exactly 2.00 s and
180 deg turns 4.00 s, 26% of the exploring time (on 2026-10-07 small turns often didn't move the robot at
all). Now: full power until about 19 deg before the target, then 45, which finishes.

**Simulator** (600 comp fields vs robot-test before it; the sim's turning is fitted to bench tests 2 and 3):
turning time 111 -> 68 s per run, moves starting more than 5 deg off 10% -> 6%. Score +12.2 +-3.5 (seeds
1-600), +9.0 +-3.3 (601-1200); time lost +2 +-2 / +2 +-2 pts, home -1 +-3 / -4 +-3 pts (it explores
79% -> 85% of the field, so more moves and more chances to hit something; see 2j). Gain 12 or 10 with
minimum 50 gave the same score.

**Depends on:** how far the real robot coasts after a turn at full power (overshoot) and its real minimum.

**Tests on the real robot** (cameras off, Serial log):

a. **90 and 180 deg turns** in the middle of a tile, 5 each way: each should take about 1-1.5 s (90) or 2-2.5 s
   (180), from `[TURN] target=` to `finished turning`, not 2.0 / 4.0 s, and end within 2-3 deg (`turn target=...
   err=`). It mustn't swing past the target and back more than once.
b. **Small turns** of 5-15 deg (start the robot a little crooked before a move): they should move it.
c. If it overshoots, lower the gain (6) before the minimum; if small turns still stall, raise the minimum.

## 2i. Shorter waits between moves

**What changed** (`Main/main.cpp`): the fixed waits on every tile.
- SENSE_TILE: the 200 ms wait after reading the walls is gone (nothing reads anything after it).
- EXECUTE_MOVE, after the turn: 200 / 100 ms around `parallel()` -> 80 / 30 ms (a fresh distance reading takes
  ~33 ms; 80 ms lets the robot settle after the turn).
- EXECUTE_MOVE, after the move: `delay(200); parallel(); delay(100);` -> `delay(80);`. SENSE_TILE's
  `centreAlong()` squares up with `parallel()` first thing, so it was done twice.

**Why:** about 0.8 s of waiting per tile, 14% of the exploring time in the sim.

**Simulator** (600 comp fields vs robot-test before it): +10.7 +-3.3 (seeds 1-600), +8.7 +-3.2 (601-1200);
time lost +0 +-2 / +3 +-2 pts, home -2 +-3 / -4 +-3 pts (explores 79% -> 84%; more moves, see 2j). Keeping the
second `parallel()` with the short waits: +9.7, home -4: worse.

**Depends on:** how long the real robot takes to stop rocking after a turn or a move (the sim's motors stop in
~50 ms, an assumed value).

**Tests on the real robot** (cameras off, Serial log): a few full runs. The `reading walls` values and the walls
in the map should be as reliable as before: watch for `tile mismatch` and `position corrected` lines and walls
missed or invented on tiles you know. If walls get misread, raise the 80 ms waits back toward 200.

## 2j. No-progress check: stuck against something the front sensors can't see

**What changed** (`Main/movement.cpp`, `fwd()`): at the start of a move it notes a wall square ahead (both
front sensors 150-400 mm and within 30 mm of each other: at most the end of the next tile) and/or a wall
behind (back sensor 20-400 mm). From 160 mm of encoder travel on, those readings must have changed by at least
35% of what the encoders say. If none has, three loops in a row, the robot is stuck: it stops, backs up until
the reading is back where it started (by the sensors: the encoders over-counted), and the edge is treated
like an obstacle (blocked the first time, as in section 1). Not after a tilt (ramp, bump) in that move.

**Why:** the biggest way the robot gets lost in the sim is driving into something the front sensors can't see
(an obstacle between their beams, or a corner on a wall): the wheels slip, the encoders count a tile it never
drove. In 100 sim runs the check fired 54 times: 48 with the robot really stuck (37 on a wall, 11 on an
obstacle), 6 while it was moving.

**Simulator** (600 comp fields): on robot-test +5.2 +-3.1 / -1.4 +-2.9 (seeds 1-600 / 601-1200), time lost
-1 / -1 pts. On top of 2h and 2i: +0.8 +-3.2 / +3.2 +-3.1, time lost -2 +-2 / -4 +-2 pts, home -1 / +2 pts.
**2h + 2i + 2j together vs robot-test before them:** +18.9 +-3.7 / +16.9 +-3.4, time lost +1 +-2 / +1 +-2 pts,
home -3 +-3 / -2 +-3 pts, explored 79% -> 89%; with turns that jam on walls (`wallNudgeMm=0`) +9.5 +-2.9, time
lost -3 pts, home +6 pts; with motor A like the others +13.5 +-3.4.

**Depends on:** how the VL53L0X sees a wall 150-400 mm ahead while driving, and what the wheels do when the
robot is blocked (in the sim they keep turning and the encoders count).

**Tests on the real robot** (cameras off, Serial log):

a. **Blocked.** Put a heavy box (or tape one down) in the middle of the next tile, narrower than the gap
   between the two front sensors (~16 cm) so neither sees it, with a wall at the far end of that tile. Start a
   move toward it: expect `[FWD] no progress: ...` within about 20 cm of travel, `[FWD] blocked, backing up by
   the distance sensors`, the robot back near the middle of its tile, and `obstacle ahead: blocking edge`.
b. **No false alarms.** A few normal runs with nothing in the way: `no progress` must not appear while the
   robot is moving freely. If it does, note the readings in the log line and the place.
c. **If (b) shows false alarms,** the ready fix: check only between 160 and 250 mm of encoder travel and while the
   robot points within 6 deg of the tile direction (in `fwd()`, add `&& avgEncoder() <= pulsesForDistanceMm(250)
   && fabs(yawNow) <= 6.0` to the `if(!climbtoggle && !noProgressOff ...` line). In the sim the false alarms all
   came near the end of moves with the robot turned (8 in 150 runs -> 0), at the cost of catching fewer real
   jams: +1.0 +-3.0 / +0.2 +-3.0 (seeds 1-600 / 601-1200), and -2.9 +-2.6 if turns jam on walls. So it waits for
   what the real robot does.

## 2o. The no-progress check is no longer switched off after two stops at one edge (2026-10-18)

**What changed** (`Main/main.cpp` `repeatedFailure()`, from 2k): the 2nd no-progress stop at the same edge used to turn the
check off for the next try there (two in a row were thought more likely false). The "wall after 4 failures" part stays.

**Why:** with 2n most stops are real. In the simulator (obstacle beside the path, seed 3): stopped twice correctly, "map
looks finished early" cleared the block at once, and on the 3rd try, with the check off, the robot drove into the obstacle
and counted a tile it never drove.

**Simulator** (600 comp fields, seeds 1-600 / 601-1200): -0.6 +-1.0 / -0.5 +-0.7, time lost -1 / -1 pts, runs ever lost
55% -> 47% / 54% -> 47%. With 15-20 cm obstacles: +0.1 +-1.6, ever lost 79% -> 72%.

**Robot test:** with M8 (box 15 cm ahead): run fwd() at it 3 times in a row (or let the maze run hit the same box): every
try should stop with `[FWD] no progress`, never `exit=normal` against the box.

**Revert 2o if:** the maze run shows repeated `[FWD] no progress` stops at one edge with nothing there. That is what
happened on 2026-10-17 (logs/2026-10-17 partB8.txt, 15:44:57 on: no-progress again and again on the move home from 26,22
to 26,21, distance sensors 29-70 mm for ~200 mm of encoders), and why the "check off after 2 stops" rule existed. With
2o, the 4th failure writes a wall there, which can cut the way home ("no path found"). Count the stops per edge in the
log (test 2n a) before trusting 2o.

## Run clock (2026-10-18, also on main)

`mazeTime` (RUN_TIME_S, the way-home timing) now starts at the first switch flip (run start), not at power-on: a robot
switched on minutes early during setup thought those minutes were used and headed home early. Restarts after a lack of
progress must use the pause switch (the clock keeps running); cutting the power resets the clock and loses the map.
**Robot test:** switch on, wait 2 minutes, start the run: the LCD run time should start from 0 at the switch flip.

## 2n. No-progress check: the 80 mm window only when the whole move is behind too (2026-10-18)

**Robot, 2026-10-18 (logs/2026-10-18):** M6 (box 10 cm) caught. M8 (box 15 cm) drove into the box and reported a normal
move, the gap in c below, so **the end-of-move check from c is now in** (robot-test): M8, M6, M8 all caught and backed up
to the start. Still to check in the maze: false `no progress at the end` stops in normal runs (sim 20 -> 37 per 80 runs).
Same session: M7 wheels in the air at fw(255) 118/139/150 RPM (A/B/D; sim fit ~158), fw(150) 67/66/89 (sim ~90); M1 turns
0.75-0.97 s within 2 deg; M3 move 1 285 mm; M0 back sensor saw nothing with nothing behind.

**What changed** (`Main/movement.cpp` `fwd()`, no-progress check from 2j/2k): the check over each 80 mm of travel now
stops the move only if the distance sensors also show less than 60% of the encoder distance for the whole move so far.

**Why:** in the simulator with the sensors seeing the floor (fitted to move test M0), 120 of 161 no-progress stops came
while the robot was driving normally: one noisy reading at either end of an 80 mm window was enough. Each false stop
blocks an edge, the map "looks finished" sooner and the robot explores less. With 2x sensor noise or stronger floor
reflections, 90-97% of the stops were false. Now: 20 false, 44 real (was 41 real). The robot showed many no-progress stops
too (7 in run B8, ~15 on one move home on 2026-10-17).

**Simulator** (600 comp fields, seeds 1-600 / 601-1200): +6.5 +-2.4 / +9.3 +-3.0, explored 87% -> 91%, time lost +0 +-1 /
+1 +-1 pts, home -0 / -1 pts, worst 10% of runs 53 -> 62 / 42 -> 61. Without the window at all: +7.1 / +9.2 but runs ever
lost 50% -> 55%.

**Robot test:**
- a. A few minutes of normal exploring in the maze with no obstacles: count `[FWD] no progress` in the log
  (`python tools/robot_log.py log.txt`). It should be 0 or close to it.
- b. Move tests M6 (box 10 cm ahead) and M8 (box 15 cm ahead): it should stop (`[FWD] no progress`) and back up.
- c. **Known gap:** in the simulator (selftest "M8 box"), with 2n a box 200 mm ahead is missed: the robot pushes into it and
  reports a normal move, ~100 mm short (before 2n it was caught; 250 mm is missed by both). If M8 on the robot also drives
  into the box without `[FWD] no progress`, add this end-of-move check in `fwd()` just before
  `if(noProgress && moveInterrupted == false){` (sim: catches 100/150/200 mm, false stops 20 -> 37 per 80 runs, score
  +0.1 +-3.7 / -1.8 +-3.8, home +4 / +2 pts: held until the robot shows the gap):

  ```cpp
  if(!noProgress && !noProgressOff && !climbed && !black && moveInterrupted == false && !obstacleAhead && frontRef && !frontLost){
    double encEnd = avgEncoder() / pulsesForDistanceMm(1);
    int r = measure(1), l = measure(7);
    if(encEnd >= 200 && r > 0 && l > 0 && abs(r - l) <= 40){
      double p = (prog0R + prog0L) / 2.0 - (r + l) / 2.0;   // how much nearer the wall ahead came
      if(p < 0.8 * encEnd){ Serial.println("[FWD] no progress at the end"); fwdExit = "no-progress"; noProgress = true; lastMoveNoProgress = true; }
    }
  }
  ```

## 2m. Centring along the tile: back sensor only for this tile's own wall (2026-10-18)

**What changed** (`Main/Distance.cpp` `centreAlong()`): the back sensor is used only when it reads 150 mm or less (the
wall at the back of this tile). Before, anything up to 450 mm counted, taken as "a wall one tile behind".

**Why:** the back sensor is 43 mm above the floor and its beam reaches the floor: with nothing behind it read 170-540 mm
(move tests M0, 2026-10-16/17), the same as a wall one tile behind. centreAlong() then "corrected" the robot by tens of mm.
In the maze logs, back-wall centring said +24 mm past the middle on average and moved the robot backwards: the robot
ending near the back edge of tiles (video, 2026-10-17). In the simulator with the floor modelled (fitted to M0), this
was the main way runs got lost.

**Simulator** (600 comp fields, seeds 1-600 / 601-1200): +69.8 +-5.1 / +70.5 +-5.2, home 54% -> 82%, time lost 30% -> 10%,
runs ever lost 87% -> 50%. On a sim where sensors don't see the floor: -2.7 +-3.1 (no harm).

**Robot test:** cameras off, logging, a few minutes of exploring in the maze (also part of the tile accuracy check).
- Watch: the robot should end in the middle of tiles, not near the back edge.
- Log: `centring along: ... (back wall)` lines should be rare and small now; `python tools/robot_log.py log.txt`, the
  "stops along the tile" line.

## 2l. After a ramp: drive half a tile from the ramp's end, no timed push (2026-10-18)

**What changed** (`Main/movement.cpp` `fwd()`): after the climb loop, the move used to finish the tile from the encoder
count before the ramp and then push `fw(200)` for 300 ms ("compensating"). Now it drives exactly `RAMP_END_TO_MIDDLE_MM`
(155 mm) from the moment the tilt drops back under 12 deg (the robot's middle is then at about the ramp's end) and stops.
The snap turn after it stays.

**Why:** on the robot (logs/2026-10-17 partB3-B8) every "centring along" right after "compensating" said 15-70 mm past
the middle (from the wall ahead, which reads "short" in the practice maze: really further). It worked only because each
ramp exit in the practice maze had a wall within reach for `centreAlong()`. With no wall there, the robot stays past the
middle and the colour sensor (93 mm ahead) reads the next tile. In the simulator fitted to the robot's real speed the push
went a whole tile on; ramps were the most common way to get lost.

**Simulator** (fitted sim, 600 comp fields, seeds 1-600 / 601-1200): +13.1 +-3.4 / +8.9 +-3.6, time lost 15% -> 8% /
15% -> 8%, home 77% -> 86% / 78% -> 86%, worst 10% of runs 50 -> 69 / 43 -> 51. Stop position after a ramp (at the next
colour read): more than 60 mm past the middle 106 of 542 -> 0 of 404, median +3 mm.

**Robot test:** cameras off, logging. Drive over each ramp of the practice maze 3-4 times each way (up and down).
- Watch: the robot stops in the middle of the tile after the ramp on its own, not past it and not on the slope.
- Log: after each `compensating`, a `centring along` line should be small (under ~20 mm) or missing. Before this change
  it was +15 to +70.
- If it stops **short** (still partly on the ramp, or "centring along" clearly negative from a wall ahead): raise
  `RAMP_END_TO_MIDDLE_MM` to 170-180. If it stops **past**: lower it to 140.
- If it gets stuck at the top or bottom of the ramp, set it back (`git revert` this commit) and tell Claude.

## 2k. Fixes from the maze runs of 2026-10-17 (real robot; the last 3 not yet checked in the simulator)

All found on the real robot (logs/2026-10-16 and logs/2026-10-17). Simulator: 600 comp fields, two seed sets, where noted.

- **White reference at the start of the run** (`takeWhiteReference()`, Color.cpp, first time out of PAUSE). The
  reading at power-on was sometimes 66-145 instead of ~3000 on white, and then black and blue were never seen (the
  robot drove over the black tile). Also: init_color() waits and averages 5 readings. Check: `white reference: ...
  start tile now ~3000` in the log.
- **Blue only if blue > 2.5 x red** (read_color). White and grey are bluish too; a dim white spot (ramp, 0.75) and
  the grey tile counted as blue. Blue tile b/r 5.5-6, white/grey under 2.
- **Walls of this tile seen up to 190 mm on all four sides** (detectWall, was 120), centreAlong() corrects up to
  150 mm (was 100). A robot 8+ cm short of the middle no longer misses the wall ahead and rams it. Sim +4.2 +-2.1 /
  +3.6 +-1.8, home +2 / +4 pts, runs ever lost 24% -> 18% / 23% -> 18%.
- **No-progress check (2j):** back sensor only counts as a reference for a wall under 100 mm behind (it read the
  floor or people at 17-42 cm with nothing behind: 4 false stops; sim -0.9 +-1.5 / +0.5 +-1.4); also checked over
  each 80 mm of travel (an obstacle hit after a free start: sim -0.1 +-1.0 / -0.1 +-1.0, ever lost 28% -> 24% / 23%);
  a reference that moves 40 mm the wrong way is dropped (facing a ramp the front sensors read its slope at ~370 mm,
  then ~800 once tipped: false stop at the ramp; sim 0.0, it never happens there).
- **Left-back sensor offset 30 mm** (SENSOR_OFFSET_MM[5]): the replacement sensor read 26-30 mm more than left-front
  between two walls. Re-measure if the sensor is changed again.
- **After a ramp, no centring on the wall behind** (it is the slope: "-107 mm (back wall)" moved the robot ~10 cm).
- **Wall written after the same edge is blocked twice** (handleShortMove): ~40 tries at one wall at the top of a ramp
  otherwise (the way home falls back to blocked edges).
- **Black found during obstacle avoidance is marked in the map** (it went round the same detour onto it 4 times).
- **Repeated failures at one edge** (replaces the "wall after two blocks" guard): the 2nd no-progress stop at the same
  edge turns the check off for the next try there; the 4th failure of any kind writes a wall (~15 no-progress stops
  on one move home otherwise, 2026-10-17).
- **LCD status display** (display.cpp): line 1 tile from the start, floor, facing, run time (`2,3 F1 N   85s`); line 2
  the last event (OBSTACLE, NO PROGRESS, BLACK, BLUE 5s, RAMP UP/DOWN, CHECKPOINT, POS CORRECTED, WALL WRITTEN,
  EXPLORED, back to start, PAUSED, RESTART, `NO GYRO! check` at power-on, `WHITE REF LOW!` at the start). No waiting
  (the old lcdPrint messages for explored/home/no path paused 1 s each). The simulator ends runs on "back to start" /
  "no path found", so those keep their text. LCD test: BENCH_MODE 3. The new LCD's garbage was a loose GND, D5 bridged
  on the perfboard and a shaky contact, not the code. **Power-on check:** line 2 shows `READY`, or `!` and everything not
  answering (`!GYRO COLR LB FR`: gyro, colour sensor, distance sensors by short name FR RF RB B LB LF FL); it stays until
  power-off. Tested on the robot 2026-10-17 (gyro and sensors unplugged).
- Logging: `[POS] x,y floor f facing d` every tile, `[RET] ...` every step home.
- Move tests: `BENCH_MODE 2` in main.cpp (bench.cpp, `MOVE_TESTS`): M0 all sensors, M1/M2 turns, M3 one-tile moves,
  M4 along a wall, M5 wall ahead, M6 blocked by an obstacle.

Later (2026-10-17): B1 from a second start corner: home in 2:28. Obstacle runs (1b/1c): it blocks the edge the first
time, but "map looks finished early" cleared the block within seconds (only 4 tiles explored, position off), so it
drove around it; an obstacle against a corridor wall passed outside both front sensors and the robot scraped past
on its right corner (right-front side sensor 18 mm), knocked 15-35 deg off. Blue tiles entered 5 times in one run
(2 blue tiles; one in a dead end).

**To do (simulator, then real robot):** don't clear a block made in the last minute / early in the run; treat a side-front
reading under ~30 mm with no wall there as an obstacle; blue revisits on through-routes; sim checks for the 2k fixes not
yet run (ramp centring, repeated-failure guard, black during avoidance), then the logic bugs to main.

**Next tests:** relocalize (2a); pause/restart; cameras on; 1a/1d false obstacle counts.

## 2d. Gyro in IMUPLUS mode (no magnetometer)

**What changed** (`Main/gyro.cpp`): `bno.begin()` used the library default, NDOF. The BNO055 datasheet
(3.3.3.5) defines NDOF as absolute orientation: heading from magnetic north once the magnetometer has
calibrated. The code assumes heading 0 = the direction the robot faced at power-on, and the wall re-sync
only fixes up to 10 deg, so a switch to magnetic north mid-run (or bends from motors and steel near the
field) would throw off every turn after it. IMUPLUS uses only the gyro and accelerometer: always relative
to power-on, slow drift instead, which the wall re-sync removes. Pitch (ramps) works the same.

**Simulator** (200 comp fields, robot-test with NDOF vs IMUPLUS): can't say how the real sensor
behaves. With the sim's default (magnetometer never calibrates in a run) both are identical (133.9). If
it calibrates 60 s into a run: NDOF 43.2, IMUPLUS 133.9 (+90.7 +-8.0), back home 41% -> 78%.

**Tests on the real robot:**

a. **Heading at power-on.** Bench mode prints `heading at power-on` first. Switch on facing four
   different directions: it should read about 0 every time.
b. **No jumps.** A few full runs; watch the `[FWD] entry hdg=` lines. They should stay within a few
   degrees of 0/90/180/270 the whole run, with no sudden change of tens of degrees.
c. **Drift.** Bench test 6 (2 min standing still): a few degrees at most.

## 3. Logic fixes, now on main

- **Ramp up or down from one pitch reading.** The up/down decision read the gyro a second time; with the
  pitch just past 12 deg that reading could miss, so an up-ramp was mapped as a down-ramp (wrong floor).
  Test: drive up a ramp and down it again, 3 times each way. After each, the Serial log shows the tilt
  value printed after `climbing` (positive going up), and `adding ramp to map`; the floor in the log /
  LCD must change the right way (up = floor + 1).
- **Backing into a blue tile: stop 5 s again.** Test: start a move from a blue tile toward something
  that makes it back up (a box ahead, so it stops short). Expect `back on a blue tile: 5 s stop` when it
  had got more than half way, and no lack of progress for leaving the blue tile early.
- **parallel() turns back if it squared up off the grid.** In a corner the two side sensors can see two
  different walls and read the same with the robot turned ~45 deg; it used to stop there. Test: put the
  robot in a corner tile turned about 40 deg toward the corner and call a move (or let it explore from
  there). Expect `parallel: ended off the grid, turning back` instead of the robot staying turned.
- **Bench mode**: see above. With `BENCH_MODE 0` the firmware is unchanged. Test 7 prints what the
  colour sensor reads on white, blue, silver and black tiles: if a white tile reads as silver (r > 800)
  or silver reads as white, set `SILVER_THRESHOLD` / `WHITE_THRESHOLD` in `Main/Globals.h` from those
  numbers (a wrong checkpoint puts the robot in the wrong place after a lack-of-progress restart).

## Already on main, worth a check: steering for the whole tile (`cc0fa71`)

`fwd()` used to clip both sides to 150 PWM, so the gyro and side-wall correction did nothing until the
last ~60 mm of a tile. Now it steers all the way, with the existing gains. Drive a few tiles along a
corridor with a wall on one side, then with walls on both sides, and watch the `[CENTER] ... err= adj=`
lines. The robot should hold a straight line without weaving. If it swings side to side, lower the
`center_PID` gain in `fwd()`.
