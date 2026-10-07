# Next steps (written 2026-10-06)

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
`BENCH` lines (should show 25, about 95 deg and 19).

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
   - Make the sim reproduce bench test 4: a weaker or stiffer motor A (e.g. a per-motor friction value)
     until `--selftest` gives about 50 for test 4 with A far behind B and D, while tests 2 and 3 stay 25
     and 95. Mark it as measured from bench test 4.
   - Then try a minimum drive PWM in `fwd()` (and a higher `centreAlong()` speed) on robot-test, A/B
     against robot-test. Confirm with the steering log before pushing.
2. **Distance sensor offsets.** All sensors read long (sides about +10 mm each). Add a bench test that reads
   each sensor against a wall at exactly 100 mm (print a 100 mm spacer block to hold against the wall),
   then put the results in `SENSOR_OFFSET_MM` (`Main/Distance.cpp`). This should also stop it hugging
   right-hand walls (the right sensors read about 20 mm more than the left ones).
3. **Re-check held physics experiments on the fitted sim.** These were judged with the old turning and
   friction: TURN_MIN_PWM 35/45, finishing moves at minimum power, turn power boost, turn stall boost,
   minimum turn pace. A/B each against `origin/robot-test`, 600 fields + `--start 601`.
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
