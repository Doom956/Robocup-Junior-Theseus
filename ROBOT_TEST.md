# Robot tests

Changes on this branch depend on how real the simulator's motors, grip, turning, gyro or sensors are,
so they stay off `main` until they pass on the real robot. Each one has a short test below: what to
set up, what to run, and what to watch for in the Serial monitor.

This branch also has everything on `overnight-fixes` (logic fixes: ramp up/down, 5 s stop when backing
into a blue tile, bench mode), so one session tests it all. Whole branch vs main in the simulator (600
comp fields): score +13.9 +-4.6, time lost 44% -> 17%, back home 48% -> 74%.

**Suggested order for a test session**

1. **Bench mode first** (`BENCH_MODE 1` in `Main/main.cpp`, see `Main/bench.cpp`): ~10 minutes, gives the
   sensor offsets, turning power and gyro drift. Copy the `[BENCH]` lines; they tell us how far the
   simulator is from this robot. Set `BENCH_MODE` back to 0.
2. **Steering** (last section): a few tiles along a corridor; no weaving.
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

## 3. From overnight-fixes (logic fixes; branch overnight-fixes, one commit each)

- **Ramp up or down from one pitch reading.** The up/down decision read the gyro a second time; with the
  pitch just past 12 deg that reading could miss, so an up-ramp was mapped as a down-ramp (wrong floor).
  Test: drive up a ramp and down it again, 3 times each way. After each, the Serial log shows the tilt
  value printed after `climbing` (positive going up), and `adding ramp to map`; the floor in the log /
  LCD must change the right way (up = floor + 1).
- **Backing into a blue tile: stop 5 s again.** Test: start a move from a blue tile toward something
  that makes it back up (a box ahead, so it stops short). Expect `back on a blue tile: 5 s stop` when it
  had got more than half way, and no lack of progress for leaving the blue tile early.
- **Bench mode**: see above. With `BENCH_MODE 0` the firmware is unchanged.

## Already on main, worth a check: steering for the whole tile (`cc0fa71`)

`fwd()` used to clip both sides to 150 PWM, so the gyro and side-wall correction did nothing until the
last ~60 mm of a tile. Now it steers all the way, with the existing gains. Drive a few tiles along a
corridor with a wall on one side, then with walls on both sides, and watch the `[CENTER] ... err= adj=`
lines. The robot should hold a straight line without weaving. If it swings side to side, lower the
`center_PID` gain in `fwd()`.
