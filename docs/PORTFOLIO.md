# Theseus: making a maze-rescue robot stop getting lost

**Engineering portfolio write-up, October 2026.** RoboCupJunior Rescue Maze 2026 robot ("Theseus"), team project.

> **Edit before sharing:** the "My role" section is a draft. Check it describes what you did, and add your name, school
> and team. Keep the note on AI assistance; it is honest and it is what reviewers look for.

---

## 1. Summary

At the 2026 competition in Korea our robot could find victims and drop rescue kits, but it **got lost in the maze**: its
idea of which tile it was on drifted away from reality, so it explored badly and often never found its way back to the
start.

In October 2026 I set out to fix the navigation in a measurable way instead of by trial and error:

1. **Built a physics simulator** that runs the robot's *real* firmware against a simulated robot, made from our CAD
   model, the part datasheets and the official 2026 rules (including the exact scoring).
2. **Calibrated it to the real robot** with a bench-test mode I added to the firmware (turning power, driving power,
   sensor offsets, gyro drift, floor colours, weight).
3. **Tested every change statistically:** each change was run on 600 simulated competition fields against the version
   before it, on the same fields, with 95% confidence intervals, and repeated on a second independent set of 600 fields.
4. **Confirmed on the real robot** with test modes, Serial logs and maze runs, and fed what the real robot did back into
   the simulator.

**Result:** in simulation, the improved firmware scores about **+50 points** more than the original per run (171 vs 121
on 600 fields), the share of runs where the robot loses its position fell from **28% to 8%**, and runs that end back home
rose from **64% to 89%** (2026-10-16 numbers). On the real robot, the morning of 2026-10-17 it got lost in every maze run;
after that day's fixes it **got home in 3 of the last 4 full runs** (about 2.5 minutes each, crossing two ramps).

---

## 2. My role and tools (draft: edit to match)

- Team robot: the mechanical design, electronics and the original firmware (Arduino, multi-core) were built by the team;
  victim detection (cameras) and the kit dispenser are teammates' work and I deliberately did not change them.
- **My part (October 2026):** porting the firmware to PlatformIO/C++, designing the testing approach, the simulator and
  analysis tools, all navigation changes, running the real-robot tests, collecting and reading the logs, debugging the
  hardware faults, and deciding what was accepted or rejected.
- **AI assistance:** much of the code and analysis was written with an AI coding assistant (Claude Code), working under
  rules I set (see section 6). I directed the work, ran the robot, supplied the measurements and made the decisions;
  the assistant wrote and ran code and simulations.
- Tools: C++ (PlatformIO, Arduino GIGA R1 / mbed RTOS), Python (analysis, A/B testing), Git/GitHub, a multimeter.

---

## 3. The robot

| Part | What it does |
|---|---|
| Arduino GIGA R1 (STM32H747) | Main controller |
| 4 DC gearmotors (Pololu 195:1) with encoders, skid steering | Driving and turning on the spot |
| 7 VL53L0X laser distance sensors (2 front, 2 per side, 1 back) | Walls, centring, obstacles |
| BNO055 IMU | Heading and pitch (ramps) |
| TCS34725 colour sensor | Black, blue and silver floor tiles |
| 2 OpenMV cameras, stepper dispenser | Victims and rescue kits (team code) |
| 16x2 LCD | Status display (added in this project) |
| 3S LiPo, weighed total **1.955 kg** | |

The maze: 30 cm tiles, walls, two floors joined by ramps, black tiles (no-go), blue tiles (5 s stop), silver checkpoints,
obstacles, an 8-minute run, points for reaching tiles and for getting home.

---

## 4. What I did, in order

### 4.1 Port and a first simulator (1 Oct)
- Ported the firmware from Arduino sketches to a PlatformIO C++ project so it could be built, versioned and tested.
- Wrote a tile-level **logic simulator** that runs the real navigation code on generated mazes. It found map and
  exploration bugs immediately (black tiles, ramps, the route home).

### 4.2 Physics simulator (1-2 Oct)
- A second simulator runs the **unchanged firmware** against a physics model: motors from their datasheet torque curves,
  skid-steer turning with wheel friction, encoders that count wheel slip, the VL53L0X beam cone and minimum range, gyro
  drift, the colour sensor, ramps and obstacles. Sensor positions come from the CAD file.
- It plays a full RCJ 2026 run with an automatic referee: the real scoring, lack-of-progress restarts, the time limit.
- **Rule: no guessing.** Every number comes from CAD, a datasheet, the rules or a measurement on our robot; anything else is
  marked "assumed" and tested over a range.
- A **paired A/B tool**: two firmware versions on the same 600 random competition-sized fields, reporting the score
  difference, the share of runs where the robot got lost, and runs ending home, each with a 95% confidence interval.

### 4.3 Fixing what the simulator showed (2-4 Oct)
Analysis scripts traced each lost run back to the first moment the robot's map position went wrong. That pointed to
specific bugs, for example:
- the steering only worked in the last 6 cm of each tile (a speed limit clipped it),
- turning used the wrong sign for the left wall,
- the robot didn't check where a wall ended, so it took wall ends for obstacles (fix: **+19.6 points**),
- ramps could be recorded as going down when they went up (one pitch reading for both decisions),
- one distance sensor that stopped answering could freeze the whole robot (added a read timeout).

Each fix was labelled either **logic bug** (wrong on any robot; allowed into the main branch after an A/B showing no
harm) or **physics-dependent** (depends on how realistic the simulator is; kept on a separate `robot-test` branch until
it passed a real-robot test).

### 4.4 Calibrating the simulator to the real robot (3-6 Oct)
- Added a **bench mode** to the firmware: a sequence of tests run with the pause switch and an LED, no laptop needed:
  lowest power that turns or drives the robot, degrees turned per second, gyro drift, sensor readings against walls,
  floor colours.
- The real robot needed **PWM 25-30** to turn on the spot where the simulator had assumed 45, and one wheel (motor A) was
  much weaker at low power. Refitting the model to these measurements changed which fixes looked good, which is exactly
  why calibration mattered.

### 4.5 First real-robot findings (6-7 Oct)
- **Pause switch** read "pause" almost all the time: its pull-down resistor had come loose. Fixed in code with the
  board's internal pull-down.
- **4-second stops** after moves came from the camera thread, not navigation (left to the camera code's author).
- The robot **stopped at the back of tiles** (motor power ramped down below what it needs to move) and **weaved** along
  walls (steering gain too high): both fixed on `robot-test` and checked in simulation.

### 4.6 Turning, waiting and getting stuck (8-12 Oct)
- Turns crawled and waited out their time limit: minimum turning power raised to match the bench measurement.
  Turning time fell from 111 s to 68 s per simulated run.
- Removed about 0.8 s of fixed waiting per tile.
- A **no-progress check**: if the wheels turn but the walls ahead or behind don't get closer, the robot is stuck
  (e.g. on an obstacle the sensors can't see); it backs up and avoids that edge.
- Together: **+18.9 / +16.9 points** on the two field sets; the robot explored 89% of the field instead of 79%.
- Just as important, many ideas were **tested and rejected** when they didn't help on both field sets (see
  `CLAUDE.md`, "Tested and held"), for example faster exploring orders, alternative relocalization and power boosts.

### 4.7 Move tests and weighing the robot (16 Oct)
- A second test mode (**move tests**) checks turns, one-tile moves, wall following and the stuck check on any floor.
  Turns: 90 deg in 0.8-1.0 s, all within 2 deg. One-tile moves: 297-326 mm (target 300).
- Weighed the robot: **1.955 kg**, against an estimate of 1.15 kg from the parts list. The simulator was refitted; the
  improved firmware's lead held (+50.4 points).
- The move tests caught false "stuck" alarms: the back sensor sometimes sees the floor or people 17-42 cm away. Fixed by
  only trusting a wall right behind the robot.

### 4.8 Maze day (17 Oct)
The morning's runs failed in several different ways. Working from the logs one problem at a time:

| Symptom | Cause found | Fix |
|---|---|---|
| Drove onto the black tile | Colour sensor's white reference taken at power-on was ~30x too dark | Take it on the start tile when the run starts |
| White and grey tiles taken for blue | "Blue" meant "blue is the strongest colour", which white is too | Blue only if blue > 2.5x red (blue tile ~6x, white < 2x) |
| Rammed walls, map drifted | A wall was only "seen" under 12 cm; a robot 8+ cm out of place missed it | Up to 19 cm on all sides (sim +4.2 / +3.6 points) |
| False "stuck" at ramps | The slope looks like a wall at 37 cm, then jumps away as the robot tips | Ignore a reading that moves the wrong way |
| Lost after ramps | Re-centred on the ramp's slope behind it | No back-wall centring right after a ramp |
| Loops at one spot, dozens of retries | Way home fell back to a blocked edge | Guard: after repeated failures, mark a wall |
| Detoured onto a black tile 4 times | Black found during obstacle avoidance wasn't recorded | Record it |
| Squared up crooked on left walls | A distance sensor had died; its replacement read 30 mm long | Replaced it, corrected its offset |
| Spun in circles | Gyro cable came loose | Reconnected; added a power-on check (below) |

By the end of the day: **home in 3 of the last 4 runs**, the last runs clean.

### 4.9 Diagnostics: the LCD (17 Oct)
- A status display: tile position, floor, heading and run time on line 1; the last event (OBSTACLE, NO PROGRESS, BLACK,
  BLUE 5s, RAMP UP, ...) on line 2. A test no longer needs a laptop to see what the robot thinks.
- A **power-on self-check**: gyro, colour sensor and all 7 distance sensors; anything missing stays on screen
  (`!GYRO COLR LB`). Two of the day's failures (dead sensor, loose gyro) would have been caught before the run.
- Bringing up the new LCD was its own debugging exercise: a row of blocks meant no initialisation; I wrote a firmware
  test that drives each LCD wire high and low and measured them with a multimeter, which found one data line bridged to
  its neighbour on the perfboard.

---

## 5. Results

**Simulation (600 competition fields, same fields for both, 95% intervals):**

| Date | Original firmware | Improved firmware (`robot-test`) |
|---|---|---|
| 2 Oct (first fixes) | 107.6 | 124.1 |
| 6 Oct (calibrated to the bench) | 122.4 | 148.2 (+25.8 +-4.5) |
| 16 Oct (real weight) | 120.9 | **171.4 (+50.4 +-4.7)** |

On 16 Oct: runs where the robot lost its position 28% -> **8%**; runs ending home 64% -> **89%**.

**Real robot (17 Oct):** every run lost in the morning -> home in **3 of the last 4** runs, about 2.5 min each, two ramps
up and down, black and blue tiles handled. Turns within 2 deg; one-tile moves within about 2.5 cm.

> Note: after the 17 Oct fixes the simulator shows a smaller lead (+10). The prime suspect is a calibration for the real
> robot's replacement sensor that the simulator doesn't share; checking that is the next job (NEXT_STEPS.md, 1b item 0).

---

## 6. How I worked (the part worth talking about)

- **Measure, don't guess.** Every simulator value has a source; the bench and move tests exist to replace guesses with
  measurements, and the weighing changed the model.
- **Paired A/B tests with intervals, on two independent field sets.** A +4-point gain on one set of fields can be luck;
  it has to hold on new fields too.
- **Separate "always wrong" from "depends on the physics".** Logic bugs went to the main branch; anything depending on
  the simulator's realism stayed on a test branch until the real robot confirmed it.
- **Reject things.** A long list of reasonable-sounding changes was tested and not kept.
- **Simulation to reality and back.** Where the real robot disagreed with the simulator (floor readings, ramp slopes,
  overshoot), that became a fix and a simulator improvement.
- **Make the robot explain itself.** Serial logs with the map position on every tile, log-analysis scripts, an on-robot
  display and a power-on self-check.
- **Written rules and notes** (`CLAUDE.md`, `ROBOT_TEST.md`, `NEXT_STEPS.md`) so the work could continue across PCs and
  sessions, and the AI assistant followed the same rules.

## 7. What I learned
- A simulator is only as good as its calibration: one bench test (turning power) changed which fixes looked best.
- Most "navigation" failures on the real robot were sensing problems: a wrong reference value, a dead sensor, a slope
  that looks like a wall.
- Small hardware faults (a loose ground, a bridged pin, a loose cable) cost more time than any algorithm; a self-check
  at power-on is worth adding early.
- Statistics matter: many changes that looked like improvements on 200 fields disappeared on 600 or on new fields.

## 8. Still to do
Overshoot at the end of moves, the "no path found" loop in badly-lost runs, obstacles that touch the robot's corner,
fewer blue-tile revisits, and runs with the cameras on. See `NEXT_STEPS.md`.

---

## For the portfolio page (to collect)
- [ ] Before/after video: a lost run from the morning of 17 Oct and a clean run home.
- [ ] Photo of the robot and of the LCD showing position / a power-on warning.
- [ ] Chart: simulated score and "lost runs" per milestone (section 5 table).
- [ ] Chart: real vs simulated turning power before and after calibration.
- [ ] One example of a log trace showing where the robot got lost and the fix.

Sources in this repo: `CLAUDE.md` (all results and decisions), `ROBOT_TEST.md` on `robot-test` (each change and its
real-robot test), `NEXT_STEPS.md`, `logs/` (raw robot logs), `tools/sim/physics/README.md` (every simulator parameter and
where it comes from), git history from 2026-10-01.
