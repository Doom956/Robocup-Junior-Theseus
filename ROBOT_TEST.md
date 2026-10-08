# Robot tests

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
| 2026-10-07: `can't find gyro` at one start-up | heading then stuck at 0.00, turns never finish. Check the gyro cable, connector and power |
| 1. Obstacles, 2. relocalize, full runs | not yet |

**Suggested order for a test session**

1. **Bench mode first** (`BENCH_MODE 1` in `Main/main.cpp`, see `Main/bench.cpp`): ~15 minutes, gives the
   sensor offsets, turning power, gyro drift and floor colours. No LCD or USB cable needed: the LED blinks
   slowly while it waits for the pause switch, stays on during a test and blinks fast when done; then plug
   in USB and flip the switch to print every result. Copy the `[BENCH]` lines; they tell us how far the
   simulator is from this robot. Set `BENCH_MODE` back to 0. New test 11 (at the end, one switch flip per
   sensor): a flat wall exactly 100 mm in front of each distance sensor (a printed 100 mm spacer); its last
   line is `SENSOR_OFFSET_MM` for `Main/Distance.cpp`, ready to paste (the sensors read long, the right ones
   about 20 mm more than the left: probably why it hugs right-hand walls). Tests 1-10 can be skipped by
   flipping through them if they were done already.
2. **Steering** (last section, 2e and 2f): a few tiles along a corridor; no weaving, no big turns off a wall, each move ends in the middle of the tile.
3. **Ramp up and down** (section 3): the floor change.
4. **Obstacles** (section 1, tests a-e), then **relocalize** (section 2).
5. A few full runs on a practice field with an obstacle or two; save the Serial logs.

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
