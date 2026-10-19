# Next steps (updated 2026-10-17, evening)

**Fast PC: the simulator to-do list is now [SIM_WORKLIST.md](SIM_WORKLIST.md)** (say "work through SIM_WORKLIST.md").

## 0. On the other PC: get today's work

First time on that PC only (skip if it already has the project):
```
git clone https://github.com/Doom956/Robocup-Junior-Theseus.git
cd Robocup-Junior-Theseus
pio pkg install -g --tool platformio/toolchain-gccmingw32
```
(also needs PlatformIO and Python 3; see section 1 below)

Every time:
```
cd Robocup-Junior-Theseus
git checkout main
git pull
git fetch
```
Check: `git log --oneline -1` shows `972a272` (LCD one-line check) or newer, and `git log --oneline -1 origin/robot-test`
shows `22c43c5` or newer. Then open Claude Code in that folder: it reads `CLAUDE.md` and this file.

**The robot runs robot-test.** To flash it:
```
git checkout robot-test
git pull
pio run -t upload
git checkout main
```
(robot plugged in by USB, battery ON; check `#define BENCH_MODE 0` in `Main/main.cpp` first). At power-on the LCD should
say `READY`; `!GYRO`, `!COLR` or sensor names (FR RF RB B LB LF FL) mean something isn't plugged in.

What was done on 2026-10-17: `ROBOT_TEST.md` section 2k (on robot-test) and `CLAUDE.md` ("Real robot 2026-10-17").
Raw logs: `logs/2026-10-16`, `logs/2026-10-17`.

## 1. Simulator jobs (run on the faster PC)

Each command builds both versions from git and runs 600 comp fields; do both seed sets (the second line of each pair).
They take a while: start them and leave the PC. Or ask Claude: "run the simulator jobs in NEXT_STEPS.md".

**a. The three robot-test fixes of 2026-10-17 not yet checked** (each is one commit, compared with the one before it):
```
python tools/sim/physics/ab.py 407753c f6e2fde --runs 600
python tools/sim/physics/ab.py 407753c f6e2fde --runs 600 --start 601
python tools/sim/physics/ab.py f6e2fde bc10a83 --runs 600
python tools/sim/physics/ab.py f6e2fde bc10a83 --runs 600 --start 601
python tools/sim/physics/ab.py 6353e9b a1f6662 --runs 600
python tools/sim/physics/ab.py 6353e9b a1f6662 --runs 600 --start 601
```
(407753c -> f6e2fde: no centring on the slope after a ramp, wall after two blocks; f6e2fde -> bc10a83: black found during
avoidance marked; 6353e9b -> a1f6662: repeated-failure guard.) Keep each if its score interval isn't below zero.

**b. Whole robot-test vs main**, to see where both stand now:
```
python tools/sim/physics/ab.py main origin/robot-test --runs 600
python tools/sim/physics/ab.py main origin/robot-test --runs 600 --start 601
```

**c. Move the logic bugs to main** (they are wrong on any robot): white reference at the start of the run, blue only if
blue > 2.5 x red, black found during avoidance marked, repeated-failure guard. Each needs a main-vs-main+fix A/B (no harm)
and `pio run`. Ask Claude: "move the 2026-10-17 logic bugs to main".

**Results (2026-10-17, done on the fast PC):**
- a. All three kept. 407753c -> f6e2fde +0.3 +-1.2 / -0.3 +-1.0 (time lost -3 +-1 / -3 +-1 pts); f6e2fde -> bc10a83
  -0.2 +-0.4 / +0.0 +-0.1; 6353e9b -> a1f6662 +0.5 +-0.7 / +0.7 +-0.9.
- b. main -> robot-test +9.9 +-5.9 / +9.6 +-6.0, time lost 27% -> 10% / 28% -> 10%, home 64% -> 79% / 65% -> 78%. But
  restarts per run 0.62 -> 2.58 / 0.71 -> 2.53 and the worst 10% of runs 30.8 -> 23.8 / 30.8 -> 23.2: a few lost runs loop
  on "no path found" (a wall written after repeated failures, often while the position was already off, cuts the only way
  home; restart, same again). Removing those walls when there is no path home: -0.6 +-0.6 / -0.7 +-0.6, time lost +6 / +6
  pts: dropped.
- c. Done: on main (white reference, blue rule, black in avoidance marked, wall after 4 failures at one edge; the
  no-progress parts of the guard stay on robot-test, main has no no-progress check). main -> main+fixes +1.7 +-3.8 /
  +1.4 +-3.9, time lost -5 +-3 / -7 +-3 pts, home +1 / +2 pts. pio run builds.
- Robot logs 2026-10-17 (robot_log.py, B6-B9): turns median 0.5-0.7 deg; moves start 0.4-0.7 deg off. centreAlong from a
  wall AHEAD says the robot stopped short (median -31 mm), from a wall BEHIND that it stopped past (median +24 mm): both
  say the wall is ~25-30 mm further than the code's 140 mm from the tile middle (RCJ: 280 mm between wall faces). Measure
  the practice maze (inner faces across a tile, wall thickness): if it is wider, nothing to fix for competition; if it
  is 280, the front/back sensors read long at close range.

## 1c. Simulator fitted to the robot (2026-10-18, fast PC)

- **The sim was 2.5x too slow.** One tile took 2.7 s in the sim, 1.08-1.13 s on the robot (M3). Fitted to the move tests and
  bench tests: motor 170 RPM (was 72, the BOM's #3493; the real motors are geared faster, or a 6 V version on 12 V), stall
  torque 4.2 kg.cm (ramp: up at PWM 180 as slow as down at PWM 120 on the robot), skidFactor 2.5, wheelMu 0.071, no wheel
  slip (tractionMean 1.0). Sim now: one tile 1.13 s / 300 mm (robot 1.1 s / 297-326 mm), turnright(150) 1 s 97.6 deg
  (92-97), 90 deg turn 1.1 s (0.77-1.01), bench tests 2 / 4: 30 / 51 (25-30 / 49-51). `--selftest` repeats M3 and M1.
  Not explained: real ramp climbs take ~38 fwd() loops, the sim's ~23 (a log with ms timestamps would tell).
- **Replacement left-back sensor modelled** (`tofLeftBackExtraMm` 30): robot-test's SENSOR_OFFSET_MM[5] = 30 corrected an
  error the sim's sensor didn't have, so robot-test squared up crooked on left walls (heading 9-14 deg off, 82% of runs
  lost). Every robot-test sim result since 2026-10-17 had this (section 1 above too).
- **main -> robot-test on the fitted sim:** +69.9 +-5.6 / +66.6 +-5.7, home 38% -> 77% / 35% -> 78%, time lost 50% -> 15%.
  main has no correction for the long left-back sensor: don't run main on the robot while that sensor is fitted.
- **robot-test settings at real speed** (600 comp): turns back to main's (20 / 4.5) -5.4 +-3.5 (keep 45 / 8); steering
  gain back to 2 +1.3 +-3.8; no minimum PWM 55 +0.7 +-3.3 (no difference: keep, they work on the robot).
- **Overshoot (1b item 1):** the moves themselves end about right (M3 307 mm, sim 300). The robot ending near the back of
  tiles in the maze is centreAlong(): from a wall behind it reads 24 mm "past", from a wall ahead 31 mm "short", so it
  pulls the robot toward whichever wall it uses. Measure the practice maze (inner faces across a tile, wall thickness).
  Centring by balancing front and back when both walls are in the tile: -0.5 +-1.3 / -0.3 +-1.0, walls 300 apart -0.2
  +-3.5: no change in the sim (held). Stopping 12 mm early / PWM 50 not tried: the moves don't overshoot.

## 1d. Section 1b worked through on the fitted sim (2026-10-18, fast PC; robot-test with the ramp fix = 170.2 / 165.3)

- **Ramp exit (now robot-test 2l):** the timed push after a ramp sent the robot a whole tile on at real speed; on the
  robot every "centring along" after "compensating" said 15-70 mm past (logs 2026-10-17). Drive 155 mm from the ramp's
  end instead: +13.1 +-3.4 / +8.9 +-3.6, time lost 15% -> 8%, home 77% -> 86%. Ramps as first cause of getting lost:
  28% -> 3%. Robot test in ROBOT_TEST.md 2l.
- **2. "no path found" loop:** write the 4-failure wall only when this tile's other walls match the map (or a wall is
  seen ahead): -0.0 +-0.5 / +0.4 +-0.4. Held (no effect).
- **4b. side-front obstacle at the corner** (side-front < 30 mm and its back sensor 60+ mm more): -76.9 / -79.4, 20
  restarts per run: fires at every side wall starting or ending beside the robot. Dropped; needs real readings (M4 + box).
- **Obstacle straight ahead** (both front sensors 70-250): not retried. A 5-9 cm obstacle in the middle of the path sits
  in the gap between the front beams (72-116 mm wide at 100-250 mm ahead), so both sensors agree only on walls.
- **4a. keep obstacle blocks made in the last 60 s when the map looks finished early** (strategy, needs the user's yes):
  always keep them -20.4 +-4.2 / -19.3 +-4.5 but ever lost 50% -> 22%, time lost 8% -> 3%, home 86% -> 92% / 95%,
  explored 92% -> 79% (it goes home early instead of driving round); keep them only while something else is open
  -1.0 +-3.1 / +2.3 +-3.1. Neither recommended.
- **Where runs get lost now** (lostcause, 300 fields): obstacles ~75% (pushed obstacle 40%, real obstacle 18%, false
  alarm 16%), short move 8%, scraped wall 6%, ramp 3%. The obstacles the robot pushes are mostly ones between its front
  beams: a third, middle front sensor would see them, but the mux has no free port (7 distance sensors + the colour sensor on
  port 7). Not pursued (user, 2026-10-18).
- **3. Sensors see the floor (sim):** the VL53L0X cone also reaches the floor and ramp slopes (tofFloorGain 3, fitted to M0:
  back 266-373 mm with nothing behind, front ~690, sides ~670-760). With that, robot-test drops 172 -> 90 in the sim:
  centreAlong() took the back sensor's floor reading (~300-370) for "a wall one tile behind" and moved the robot up to
  150 mm. That matches the robot's back-wall centring saying +24 mm "past" on average and the robot ending near the back
  of tiles. Fix on robot-test (2m): back sensor only for this tile's wall (<= 150 mm): +69.8 +-5.1 / +70.5 +-5.2, home
  54% -> 82%; on the sim without floor -2.7 +-3.1. Wall one tile out on the side still reads right (342-364 vs real peak
  375-400). Item 7 (camera wait): not wanted (user).
- **Ramp count capped at 3 tiles** (a robot stuck tilted on the slope added 13-21 ramp tiles in 12 of 674 crossings):
  +0.1 +-0.6 / -0.2 +-0.5 (those runs were already lost). Held.
- **6. Robustness of robot-test** (300 comp fields each, vs the fitted sim): walls 6 mm (wider path, like the practice
  maze) +11.5 +-5.5; walls 30 mm (narrower) -16.7 +-6.4 (home 84% -> 75%); 3x grip variation +3.4; gyro drift 0.5
  deg/min -1.1; 2x sensor noise -26.3 +-7.0 and 2x floor reflection -32.4 +-6.1, both by exploring less (88% -> 75-78%),
  not by getting lost: the no-progress check's 80 mm window fired on noise (3 of 4 stops false by default, 9 of 10 with
  noise). Fixed on robot-test (2n): the window only counts when the whole move is also < 60% behind: +6.5 +-2.4 /
  +9.3 +-3.0, explored 87% -> 91%. Re-check with 2n: 2x noise +4.3 +-5.3, 2x floor reflection -0.1 +-5.3 (both fixed);
  walls 30 mm (270 mm path) still -17.8 +-6.2: probably the one-wall steering target (80 mm, ROBOT_WIDTH_MM, kept as is by
  the user's rule) leaving the robot close to the far side. Only matters if competition walls are thicker than RCJ's 20 mm.
- **2n gap (sim selftest "M8 box"):** a box between the front beams 200 mm ahead is missed with 2n (caught before);
  checking more often catches it but 213 false stops; an end-of-move check catches it (false 20 -> 37) but +0.1 +-3.7 /
  -1.8 +-3.8: held, written into ROBOT_TEST.md 2n c, to add if the robot's M8 drives into the box. New move tests on
  robot-test: M7 wheels in the air (motor RPM vs the sim's 170) and M8 (box 15 cm ahead).
- **Sim 553492e** (wheel spread 5%, LoP restart facing a random direction): main -> robot-test +93.2 +-5.1 / +96.4 +-5.0,
  home 34% -> 83% / 33% -> 81%, time lost 56% -> 10%. robot-test steering gain 2 instead of 1: +3.1 +-3.8 / +1.4 +-4.0 (keep 1).
- **Whole-branch check (2026-10-18, sim f4ca397):** known-good-2026-10-17 (bc10a83) -> robot-test (2l-2o + run clock):
  +80.5 +-4.9 / +75.4 +-5.0, home 52% -> 83% / 55% -> 81%, time lost 36% -> 10%. The simulator says so; the robot got
  home in 3 of 4 maze runs with known-good, so the sim (floor seen by the back sensor, speed fit) likely exaggerates the
  old version's weakness. Robot-confirmed so far: the run clock only. No sign of a regression.
- Section 1b done (2026-10-18): 1 sim fitted + robot-test settings checked; 2 held; 3 floor/ramp seen (+ fix 2m); 4a/4b
  held/dropped; 5 mostly structural; 6 robustness (+ fix 2n); 7 not wanted. robot-test now 2l + 2m + 2n on top of 2k.

## 1b. Next simulator work (written 2026-10-18), in this order

On the fast PC: `git checkout main`, `git pull`, `git fetch`, open Claude Code in the folder and say
**"work through section 1b of NEXT_STEPS.md"** (or one item at a time). Rules as in CLAUDE.md: A/B on 600 comp fields,
both seed sets (`--start 601`), report score / time lost / home; robot-test changes go on robot-test with a ROBOT_TEST.md
entry; strategy changes wait for my yes.

0. **First: why robot-test's lead over main fell from +50 (2026-10-16, 171.4 vs 120.9) to +10 (2026-10-17, home 89% ->
   79%).** Each 2026-10-17 change passed its own A/B, but they were compared commit to commit. Suspect: `SENSOR_OFFSET_MM[5]
   = 30` (the replacement left-back sensor reads 30 mm long on the REAL robot; the simulated one doesn't, so the
   correction makes the sim's sensor read 30 mm short and squaring up on left walls goes wrong). Check: A/B robot-test vs
   a temporary branch of robot-test with that offset at 0 (ab.py takes any two git refs), and 99ff621 (2026-10-16) vs
   current robot-test. If it is the offset: give the sim the same +30 mm on that sensor (a sim
   parameter, measured on the robot) so sim and robot match, and keep the firmware's 30. Don't change the robot's firmware
   for this.
1. **Moves stop short of the tile middle (real robot).** Corrected 2026-10-18: the video of the 2026-10-17 maze runs
   shows the robot **undershooting** (stopping before the middle of the tile), not overshooting. Conflicting data: move
   test M3 measured 307-326 mm by the back sensor (that sensor reads long, +6-13 mm at the bench, and its long-range
   readings were unreliable), and the maze logs' centring from the wall ahead says median 31 mm short (but the practice
   maze may be wider than 280 mm). Steps:
   a. Get one clean real measurement: M3 with a tape measure or ruler (mark the back of the robot before and after each
      of 3 moves), on a maze tile, and measure the practice maze (inner wall faces across a tile, wall thickness).
   b. Fit the simulator's stopping to it (today the sim stops ~6 mm short, stopcentre.py).
   c. With the fitted sim, A/B on robot-test: minimum drive PWM 60 and 65 instead of 55 (ROBOT_TEST.md 2f: "mostly short:
      raise the 55 to 60-65"); keep the best if its interval isn't below zero. Then M3 + one maze run on the robot.
   d. centreAlong() corrects the stop when there is a wall within ~45 cm ahead or behind; the error adds up in open areas.
2. **"no path found" loop on robot-test** (worst 10% of runs, 2.6 restarts per run): the wall written after repeated
   failures can cut the only way home. Removing those walls when there is no path made it worse. Try: only write the
   wall when the position looks right (walls of this tile match the map), or keep the guard's walls out of the way-home
   planner only after a "no path found".
3. **Make the sim see what the real sensors saw:** (a) front sensors read a ramp's slope as a wall at ~370 mm from the
   tile in front of it (logs/2026-10-17 ramp_sensors.txt) and ~800 once the robot tips onto it; (b) the back sensor
   sometimes reads the floor at 170-420 mm with nothing behind (logs/2026-10-16, 2026-10-17 partA). Then re-run robot-test
   vs main: results that depend on these get more trustworthy.
4. **Obstacles:** (a) don't let "map looks finished early" clear an obstacle block made in the last minute (strategy:
   numbers first, then my yes); (b) a side-front sensor under ~30 mm where the map has no wall = obstacle at the corner
   (logs/2026-10-17 partB9.txt: right-front read 48, 23, 18 mm scraping past one).
5. **Blue revisits:** route around blue tiles on through-routes when another way exists.
6. **Robustness of robot-test:** sweep walls wider/narrower (after the maze measurement), more wheel slip, noisier
   sensors; then `lostcause.py` on robot-test to find what now loses most runs.
7. **Optional, for realistic times:** a simulator option for the camera's wait after moves (up to 4 s per move next to a
   wall, from the 2026-10-07 robot runs), so time-based results (way home, explored) match runs with the cameras on.

## 2. Open problems (code, then simulator, then robot)

1. "map looks finished early" clears an obstacle block seconds after it was made, so the robot drives around obstacles
   too soon (idea: don't clear a block made in the last minute). Strategy change: needs your yes after the sim numbers.
2. Obstacles against a wall pass outside both front sensors; the robot scrapes past on its corner (side-front sensor read
   ~18 mm). Idea: a side-front reading under ~30 mm with no wall there = obstacle. Needs real readings first (robot test 3b).
3. Blue tiles entered up to 5 times a run (2 blue tiles): try routing around blue on through-routes.
4. Camera 4 s wait after moves (victim code, team decision to leave it): ask the camera code's author about `>= 0`
   instead of `!= -1` in cameraTask.

## 3. Robot tests still to do

No maze needed (move tests: set `#define BENCH_MODE 2` in `Main/main.cpp` and the list in `Main/bench.cpp` `MOVE_TESTS`):
- a. M6 stuck check: box held or taped down in the middle of the path, wall 30-40 cm ahead; it should back up.
- b. M4 + box: drive along a wall with a box against it in the path; gives the side-sensor readings for problem 2.

In the maze (`BENCH_MODE 0`, cameras off, record with `python tools/serial_log.py logs/<date>/run.txt`):
- c. Relocalize: after 3-4 tiles pause, carry it back one tile facing the same way, resume (LCD: `POS CORRECTED`).
- d. Pause and restart from the last checkpoint.
- e. A full run with no obstacles: count false `OBSTACLE` on the LCD / in the log.
- f. Cameras on: 2 full runs with victims; check victims are still found and note the wait after each move.
- g. Obstacle runs again after problem 1 is fixed.

Hardware: secure the gyro cable and the LCD's loose contact (both caused problems); always run with the battery on
(USB alone is too weak for the motors); switch on with the colour sensor over a white tile.

---

# Notes from 2026-10-06

What was done on 2026-10-06, how to set up another PC, and the software checks that are left. The robot
tests that need the robot in person are in `ROBOT_TEST.md` on the `robot-test` branch.

## 1. Get the code on your PC

```
git clone https://github.com/Doom956/Robocup-Junior-Theseus.git      (first time only)
cd Robocup-Junior-Theseus
git pull
```

Needs, once per PC:

- PlatformIO (VS Code extension, or `pip install platformio`)
- the PC compiler for the simulator: `pio pkg install -g --tool platformio/toolchain-gccmingw32`
- Python 3

Check it works: `python tools/sim/physics/run_physics.py --selftest` builds the simulator and prints its
`BENCH` lines (should show 30, about 92 deg and 51 since the robot was weighed on 2026-10-16).

Branches: `main` (tested fixes) and `robot-test` (main + the changes waiting for real-robot tests). The
A/B tool builds any branch straight from git, so you don't need to switch branches to compare them:
`python tools/sim/physics/ab.py main origin/robot-test --runs 600`. To edit robot-test:
`git fetch` then `git checkout robot-test` (and `git checkout main` to go back).

## 2. What we did on 2026-10-06

**Bench mode on the real robot** (3 runs, battery 11.76 V; results in `CLAUDE.md`):

| | Robot | Sim before | Sim now |
|---|---|---|---|
| Lowest PWM that turns it on the spot | 25 / 30 / 25 | 45 | 25 |
| `turnright(150)` for 1 s | 97 / 92 / 95 deg | 70 | 95 |
| Lowest PWM driving all 3 encoder wheels | 51 / 49 (wheel A lags) | 19 | 19 (wheel A not modelled) |
| Gyro standing still 2 min | 0.00 deg (IMUPLUS); once 12.5 deg jump in NDOF | 0.5 deg/min | 0.03 deg/min |
| Distance sensors vs CAD | left+right +20 mm, front+back +13 mm | 0 on average | +8 mm each |
| Colours | white 0.96, blue 0.19, black 0.05 (all right) | | |

- Wheel A (left front) turns only 42-56 encoder counts at PWM ~50 while B and D do 250-340. By hand it
  feels fine, so it's the motor or its driver channel.
- In main's gyro mode (NDOF) the heading jumped 12.5 deg once while the robot stood still: the magnetometer
  taking over. robot-test's IMUPLUS mode stayed at 0.00. Real evidence for IMUPLUS.

**Simulator fitted to those numbers** (`tools/sim/physics/physics.cpp`, README table): wheelMu 0.2,
skidFactor 1.1, tofOffsetMeanMm 8, gyro drift 0.03, battery 11.76. `--selftest` repeats bench tests 2-4
with the same pass rules. On the fitted sim, 600 comp fields: main 122.4 -> robot-test 148.2
(+25.8 +-4.5, time lost 32% -> 10%, home 62% -> 84%).

**Bench mode fixes** (`Main/bench.cpp`): test 4 passes only when all three encoder wheels count forward
(a humming motor made one encoder count noise and the test stopped with the robot still); tests 2-4 print
the encoder counts; result lines are no longer cut off.

**Where the sim loses points** (robot-test, 600 comp fields; `scoregap.py`): best possible 228, scored
141 (before fitting). Unexplored tiles -61 per run (72% of runs head home on time with 75% explored; in a
quarter the code thinks the maze is done with a third left, mostly because of blocked edges), blue tiles
entered again -20, not home -3.5, restarts -2.4.

**Tested and held** (nothing pushed): gyro re-sync taking 30% of the correction (+2.3 +-2.8 on the fitted
sim), routing around blue tiles (+0.6 +-0.6), retrying blocked edges after 330 s (home -6 +-3),
TURN_MIN_PWM 35 (+2.6 +-3.2 / +3.9 +-3.3 on seeds 1-600 / 601-1200).

**Real robot on robot-test:** ramp up and down 6/6. Steering test: it corrects itself but then waits about
4 s, and drives very close to a wall on its right. Repeat with the Serial log (see section 5).

## 3. Software checks to do (no robot needed)

Follow the rules in `CLAUDE.md`: diagnose with traces, A/B on 600 comp fields and again on new seeds
(`--start 601`), `pio run` builds, physics-dependent changes only on robot-test with a test in
`ROBOT_TEST.md`, strategy changes only after the user's yes.

1. **Low-power stall (likely the ~4 s wait).** The robot needs about PWM 50 to keep all wheels turning
   (wheel A), but `fwd()` slows to as little as 20 near the end of a tile and `centreAlong()` drives at 50.
   It probably stops a couple of cm short and waits out the move's time limit (8 s per tile).
   - DONE (2026-10-06, on main): the sim has a weak motor A, `motorAExtraFriction` 0.16 fading to 0 at
     PWM 128; `--selftest` tests 2/3/4 = 30 / 93 deg / 51 (robot 25-30 / 92-97 / 49-51). It makes moves
     slower (median 2.6 -> 2.9 s) but does NOT reproduce a 4 s wait: moves over 4 s stay at 3%. So the wait
     is still unexplained; the steering log will tell.
   - TESTED, NOT PUSHED (waiting for the steering log): a minimum drive power in `fwd()`. PHYSICS-DEPENDENT,
     for robot-test. In `Main/movement.cpp` replace `double base = min(Scale * 120, 150 - fabs(adjustment));`
     with `double base = max(55.0, min(Scale * 120, 150 - fabs(adjustment)));`. Against robot-test, 600 comp:
     seeds 1-600 +7.2 +-3.3, seeds 601-1200 +5.5 +-3.0 (home +2 +-3, time lost -0 +-2, explored 75 -> 78%),
     with motor A fixed (`--set motorAExtraFriction=0`) +6.0 +-3.1, with jamming turns (`wallNudgeMm=0`)
     -0.6 +-2.9. It helps mostly by moving faster near the end of each tile. Robot test: a few tiles in a
     corridor; the end of each move shouldn't creep, and the robot should stop within ~1 cm of the centre.
     `centreAlong()` at PWM 60 instead of 50: +3.0 +-3.1, adds nothing on top (+4.8 +-3.1 together on new
     seeds): held.
2. **Distance sensor offsets.** All sensors read long (sides about +10 mm each).
   - DONE (on main): bench test 11 reads each sensor against a flat wall exactly 100 mm away (one switch
     flip per sensor; print a 100 mm spacer block) and prints `SENSOR_OFFSET_MM` ready to paste into
     `Main/Distance.cpp`. That should also stop it hugging right-hand walls (the right sensors read about
     20 mm more than the left ones). Needs the robot.
3. **Re-check held physics experiments on the fitted sim.** DONE: against robot-test with the weak motor A,
   600 comp fields, seeds 1-600 / 601-1200: TURN_MIN_PWM 35 +4.0 +-3.1 / -0.2 +-3.1 (time lost +4 +-2 on
   the new seeds), TURN_MIN_PWM 45 +4.5 +-3.4 / +2.2 +-3.1, turn stall boost +4.0 +-3.0 / -0.2 +-2.8,
   minimum turn pace +5.3 +-3.1 / +1.9 +-3.0. None holds on the new seeds: all still held. ("Finishing moves
   at minimum power" is the minimum drive power in item 1.)
4. **Exploring faster** (strategy: needs the user's yes before anything goes anywhere). Time runs out in
   72% of runs; turns take 25% of the exploring time (`timeprof.py`); the code thinks it's done too early
   when blocked edges cut the map (`earlydone.py`).
5. **Move robot-test to main** once the in-person tests pass. IMUPLUS already has real evidence (the
   12.5 deg jump in NDOF). The obstacle strategy needs the user's yes.

## 4. Tools

| Command | What it does |
|---|---|
| `python tools/sim/physics/ab.py A B --runs 600 [--start 601]` | paired A/B of two refs (`WORK` = working copy of `Main/`) |
| `python tools/sim/physics/lostcause.py REF --runs 600` | where and why runs first get lost |
| `python tools/sim/physics/scoregap.py REF 600` | where the missing points go (unexplored, not home, restarts, blue revisits) |
| `python tools/sim/physics/earlydone.py REF 300` | why the code says "maze fully explored" too early |
| `python tools/sim/physics/timeprof.py REF 150` | where the exploring time goes (driving, turning, squaring up...) |
| `python tools/sim/physics/hdgerr.py REF 100` | the firmware's heading error and what the wall re-sync does to it |
| `python tools/sim/physics/run_physics.py --selftest` | the sim's version of bench tests 2-4 (`BENCH` lines) |
| `python tools/sim/physics/run_physics.py --scenario comp --seed 12 --view` | watch one run in the browser |
| `python tools/serial_log.py log.txt` | record the robot's Serial output while it's plugged in |
| `python tools/robot_log.py log.txt` | read a real robot's Serial log |

REF is a git branch or commit (`main`, `origin/robot-test`) or `WORK`.

## 5. In-person robot tests left

Full list with what to expect in `ROBOT_TEST.md` (robot-test branch). Keep the cable plugged into a PC
running `tools/serial_log.py` for each. If time is short, do these first:

1. Steering, one wall close (2e a): start 2-3 cm from the wall, turned 5 deg toward it, 3 tiles. Wall on
   the right, then on the left. (Explains the 4 s wait and the right-wall hugging.)
2. Corridor: a few tiles with one wall, then two walls; no weaving.
3. Obstacles a (wall end, no false alarm) and b (obstacle touching a side wall of the next tile).
4. Obstacles d: a full run with no obstacles must not report any.
5. A few full runs with an obstacle or two.

Also: heading at power-on facing 4 directions (2d a), relocalize (2a, 2c), corner squaring, blue back-up,
the 100 mm sensor test and bench test 5 (60 cm of floor and a ruler).

Obstacle to print (RCJ 2026 3.4.3-4 as used in the sim): an upright cylinder at least 15 cm tall,
5-9 cm across (print a 5 cm and a 9 cm one), heavy or taped down; in the open at least 20 cm from every
wall, or touching a wall and reaching at most 10 cm into the tile.
