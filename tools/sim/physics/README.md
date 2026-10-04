# Physics simulator

Runs the robot's **real firmware** (every `.cpp` in `Main/`, unchanged) against a simulated robot
on random RCJ Rescue Maze fields. Instead of a model of the robot's behaviour, the Arduino and
hardware libraries are replaced (`hw/`) by simulated ones, so `setup()` and `loop()`, the PID
loops, `fwd()`, `absoluteturn()`, `parallel()`, `center()`, the timeouts and the 8-minute clock all
run exactly as written.

The tile simulator (`../run_sim.bat`) tests the navigation logic. This one tests what the code does
with motors, sensors and time.

## Run

Needs Python and the PC compiler (`pio pkg install -g --tool platformio/toolchain-gccmingw32`).
From the repo folder:

```
python tools/sim/physics/run_physics.py                       # 100 fields of each type, realistic robot
python tools/sim/physics/run_physics.py --scenario ramp --runs 300
python tools/sim/physics/run_physics.py --ideal               # no noise, drift or motor differences
python tools/sim/physics/run_physics.py --random              # a new block of fields
python tools/sim/physics/run_physics.py --scenario loops --seed 7 --verbose   # one field + the robot's Serial output
python tools/sim/physics/run_physics.py --scenario loops --seed 7 --view      # one field, watch it in the browser
python tools/sim/physics/run_physics.py --selftest            # calibration experiments (see below)
python tools/sim/physics/run_physics.py --live                # several robots running live, in their own window (below)
python tools/sim/physics/run_physics.py --serve               # the batch dashboard in the browser (below)
```

It rebuilds from the current `Main/` every time. One 8-minute match takes a few seconds; fields
run in parallel (about 4 minutes for 1000 runs on 14 cores). Each result line shows: back on the start tile, explored everything, average
coverage, map wall accuracy, how often the code's map position was wrong, lack-of-progress restarts,
time spent pressed against walls, and how runs ended (`home`, `time` = 8:00 ran out, `no path`).
`--trace FILE` (with `--seed`) writes a recording of the run (see below).

## Live view

```
python tools/sim/physics/run_physics.py --live
```

Runs several robots at once, live, in their own window (Edge or Chrome in app mode; the normal
browser if neither is found). Each robot is a separate simulator process on its own random field,
paced to the real clock, so you watch the firmware drive as it happens. It is the same simulation as
the batch runs: the same field and seed give the same run, frame for frame, until you press a button.

- **Robots**: 1, 4, 6, 9, 12 or 16. **Fields**: mixed or one type. **Robot**: realistic or ideal.
- **New random fields** starts every robot on a new field; **Same fields again** reruns the same ones.
- **Speed**: 1× (real time), 2×, 5×, 10×, 30× or Max, for all robots, while they run.
  **Pause all** (Space) freezes the simulation itself, not just the picture.
- **Keep going**: when a robot finishes it starts a new random field 4 s later, so the view runs as long
  as you like. **Finished runs** totals the results.
- Click a robot (or ← →) to see it in detail on the right: bigger field with sensor beams, state,
  heading vs gyro, real tile vs the code's tile, distance readings, the robot's own map and its Serial
  output. Buttons for that robot: **Pause**, **Bump** (shoves it up to 40 mm and 15°, like a knock or a
  wheel slipping), **Restart at checkpoint** (a lack-of-progress restart), **Same field again**,
  **New field**, and **Open replay** (what it has done so far, in the 2D/3D replay viewer).
- **Simulator settings and code changes** (`batteryVoltage=12.4`, `ROBOT_WIDTH_MM=183`, ...) apply from
  the next "New random fields".

`physics.exe --live` is what each robot runs: it writes the recording to stdout as it goes and reads
`pause`, `resume`, `speed X` (0 = as fast as possible), `lop`, `nudge` and `stop` on stdin.

## Batch dashboard

```
python tools/sim/physics/run_physics.py --serve
```

Opens a page in your browser for running batches, like the batch test on the logic simulator's page:
pick the field type, how many fields (20 / 200 / 500 / 2000 or any number), the same fields every time
or new random ones, realistic or ideal robot, any simulator setting, and optional `#define` changes to try.
"Run batch" runs them on all CPU cores and shows, while it runs:

- per field type: back home (with the ± you'd expect from chance), "wrong tile" (it said it was home but
  wasn't), explored everything, average explored, map walls right, lost >20% of the time, restarts, time
  pressed on walls
- how runs ended, as a bar per field type, and a chart of how much of the field each run explored
- every run in a sortable, filterable table, each with a **Watch** button that records that run and opens
  it in the replay viewer (below)
- all batches run since the server started, so you can compare: same fields + one change = a fair test

The server only listens on this PC and rebuilds the firmware whenever a file in `Main/` changes, so you
can edit code in VS Code and press "Run batch" again. Ctrl+C in the terminal stops it.

## Watching a run

`--view` (with `--seed`) records the run and opens it in your browser (the dashboard's **Watch** button
does the same). The page is saved as `tools/sim/physics/replays/<field>-seed<N>.html` and works on its
own: double-click it later or send it to a teammate. It shows:

- **Field**, in **2D** or **3D**: the real field with the robot drawn to scale from the CAD, its wheels,
  the 7 distance sensor beams (25° cone and the reading each sensor returned), the colour sensor, the path
  it drove, and a box on the tile the code *thinks* it is on (green when right, red when wrong). The robot
  outline turns red while it is pressed on a wall. In 2D, "Zoom on robot" follows it up close. 3D shows
  the walls (150 mm), ramps and upper levels, the robot tilting on the ramp, and the sensors at their CAD
  heights; cameras: whole field, around the robot, behind the robot, from above. Drag to turn, scroll to
  zoom, right-drag to move. 3D loads the three.js library from the internet the first time.
- **Now**: the firmware's state (`SENSE_TILE`, `EXECUTE_MOVE`, `RETURN`...), true heading vs what the gyro
  reports, pitch, the real tile vs the code's tile, PWM on all 4 motors, every distance reading.
- **Robot's map**: the map the code has built so far, per floor (m1 / m2 / m3). Walls it imagined are solid
  red, real walls it missed are dashed red, and edges it blocked after two failed moves are amber.
- **Serial monitor**: everything the firmware printed, in step with the replay.
- **Timeline**: exploring, heading home (`RETURN`) and paused stretches; amber where the code's position
  was wrong for 4 s or more; red marks at lack-of-progress restarts. Click to jump; "Next problem" jumps to
  just before the next restart or lost-position moment.

Keys: Space play/pause, ← → 1 s, Shift+← → 10 s. A batch run lists failed fields under `replay:`; add
`--view` to any of them. `tools/sim/physics/viewer.html` can also open a file made with `--trace`.

## What is simulated

| Real hardware | Simulated as |
|---|---|
| Clock | `millis()`/`micros()`/`delay()` run on simulated time; each hardware call costs about its real time (I2C transfers, a VL53L0X waits for its next 33 ms measurement, the TCS34725 waits its 24 ms integration) |
| 4 motors on the motor shield | Pololu 195.3125:1 20D 12 V gearmotors (#3493) on a Carobot V3 shield (TB6612FNG + PCA9685, 0.5 ohm in series) from a 3S LiPo. DC motor model from the datasheet: speed = no-load speed x (PWM duty - load / stall torque), scaled to battery voltage; load = gearbox friction (more from standstill) + sideways wheel drag when turning on the spot + gravity on ramps. A/C are the left side, B/D the right, D mounted reversed (from `motors.cpp`) |
| Driving | skid steering; walls and obstacles block the robot (the wheels keep spinning, so the encoders keep counting); a turning robot scraping a wall gets shoved sideways. The robot's shape is its real outline from the CAD (`outline_mm` in `../cad/robot_geometry.json`: 209 x 183 mm, corners up to 133.5 mm from the centre), so in a closed tile it can only turn on the spot within about 6 mm of the centre |
| Encoders A, B, D | count actual wheel rotation: 5 counts per motor turn x 195.3125 = 976.6 per wheel turn (the code assumes 975); wheel slip shows up as encoder error |
| 7 x VL53L0X | positions and directions from `../cad/robot_geometry.json`; a 25 degree cone, signal-weighted distance (nearer, face-on surfaces count more), noise, a fixed per-sensor offset, out of range = 8190 |
| BNO055 | the frame the firmware asks for: `bno.begin()` (NDOF, the library default) gives heading from magnetic north, as the real sensor does, at a random maze angle with +-2.5 deg local distortion; IMUPLUS gives heading from the start direction, with drift. Noise either way; pitch from the ramp under the two axles |
| TCS34725 | raw r/g/b/clear of the tile under its CAD position |
| Pause switch / referee | the RCJ 2026 rules: a tile is visited when more than half of the robot is on it (5.4.4); lack of progress when it visits a black tile, or leaves a blue tile without first standing still 5 s (5.5.1); and (standing in for the team captain) when it hasn't reached a new tile for 60 s, or 2 s after it stops for good away from home ("no path found", or "back to start" on the wrong tile). Then: switch HIGH for 3 s, robot placed on the last checkpoint it visited, facing the start direction. If the code doesn't respond to that last kind of restart, the run ends where it stopped and the restart isn't counted |
| Score | the RCJ 2026 navigation points (5.6): blue tiles 30 each (10 less per revisit), checkpoints 10, ramps 10 each (once), speed-bump tiles 5, reliability bonus = blue tiles x 10 - lack of progress x 15 (never below 0), exit bonus (back on the start tile) = blue tiles x 10 + ramps x 5. Victims and rescue kits aren't simulated, so a real score adds those |
| Fields | the same generator as the tile simulator (black tiles never wall anything off), plus real ramp slopes of 15-25 degrees, walls 20 mm thick with posts at their ends (280 mm path, RCJ 3.3.3), obstacles (RCJ 3.4.3-4: upright, 5-9 cm across, either in the open at least 20 cm from every wall or touching a wall and reaching at most 10 cm in; `obstacleRate` per plain tile, default 0.03; fixed unless `obstacleMovableFrac` > 0, see below) and speed bumps (1 cm, across the middle of a tile, not on ramps; `bumpRate`, default 0.05) |

Field types: `comp` is the one to judge changes by. It is shaped like the RoboCup 2025 international fields (rescue.rcj.cloud: 48 tiles on two levels joined by a ramp, 2-4 checkpoints, 0-3 black and 2-4 blue tiles, about 6 speed-bump tiles; here 40-60 tiles, speed bumps at 12% of plain tiles). `flat`, `loops` and `ramp` are smaller single features; `big` (up to 256 tiles) and `bigramp` are much larger than any real field and are only stress tests.

Scoring checked against the 2026 rules (RCJRescueMaze2026-final.pdf, 5.6): blue tiles, checkpoints, ramps, speed bumps, the reliability bonus and the exit bonus are counted exactly as written. Victims are not simulated, so their points (5/10/15/30 per identification, 10/30 per rescue kit) and their share of the reliability bonus (+10 per identification and per kit) and the exit bonus (+10 per identification) are missing; stairs (10 + 5 exit) too. A real score is higher, and getting home and avoiding lack-of-progress restarts are worth more than the sim shows.

Obstacles that move (RCJ 3.4.3: "large, heavy items" that "may be fixed to the floor"; 3.4.5: one that is
moved stays where it ends up): with `--set obstacleMovableFrac=0.5` half of them are loose. The robot pushes
a loose one along when its wheels' grip (`wheelMu` x weight) beats the obstacle's floor friction
(`obstacleMu` x its mass), slowing down by the share the friction takes; a wall or another obstacle stops
it, and it stays where it ends up (the distance sensors see it there, and the replay shows it moving). The
result line gives `obstacle_pushed_mm`. Knocking one over isn't simulated. Default 0: all fixed, as before.

Not simulated yet: stairs, bridges (tiles over tiles), the dangerous zone (red tile, debris), floor height steps between tiles (up to 3 mm), victims and the cameras (victim code never
fires), battery sag during a run (a 2200 mAh pack uses only about 6% in 8 minutes).

To check whether a code change helps, compare the two versions on the same fields: `python tools/sim/physics/ab.py main WORK` (the last commit on main against your working copy of `Main/`; any two branches or commits work too, `--types comp,flat`, `--runs`). It prints the score change, and the change in time spent lost and in runs that get back home, each with a 95% interval; only an interval entirely above zero (below zero for time lost) is a clear improvement. Use 600 comp fields (`--runs 600`) for a final decision: comp scores vary too much for 200 to show a gain of about 4 points.

`--set oracleTile=1` is for analysis only, not something the real robot can do: each time the code starts reading a tile, its map position is set to the tile the robot is really on. Comparing a batch with and without it shows how many points getting lost costs. `--set oracleCentre=1` (also analysis only) puts the robot itself in the middle of its tile at the same moments; `=2` only along the way it faces, `=3` only sideways. These showed that stopping in the middle of the tile was the biggest thing to fix (it led to `centreAlong()`).

## Where the numbers come from, and what to measure

All of them are in `Params` at the top of `physics.cpp` and can be changed per run with
`--set name=value`. Values marked **datasheet** come from the parts in the team BOM; the ones marked
**assumed** or **estimate** have no datasheet and are worth measuring once you have the robot.

| Setting | Default | Source | How to check on the robot |
|---|---|---|---|
| `obstacleMovableFrac`, `obstacleMassMinKg/MaxKg`, `obstacleMu` | 0, 0.5-2 kg, 0.4 | **assumed** (the rules only say "large, heavy items" that may be fixed) | the competition's obstacles: weigh one, and find the force that slides it (a luggage scale) |
| `noLoadRpmAt12V`, `stallKgcmAt12V`, `frictionFracAt12V` | 72 RPM, 10 kg.cm, 0.05 | **datasheet**: Pololu #3493 (72 RPM, 80 mA no-load, 1.6 A / 10 kg.cm stall at 12 V; 46 g) | lowest `drivetrain.fw(pwm)` that moves the robot (model: about 13-17) |
| `driverOhms` | 0.5 ohm | **datasheet**: TB6612FNG output ON resistance, upper + lower, typical (the motor itself is 12 V / 1.6 A = 7.5 ohm, so stall torque is about 6% lower than the motor alone) | - |
| encoder counts | 5 per motor turn | **datasheet**: Pololu #3499 encoder, 20 counts per motor turn counting both edges of both channels; the code counts rising edges of one channel = 5 | - |
| `batteryVoltage` | 11.8 V | 3S LiPo (Zeee 2200 mAh in the team BOM; cell count read from the photo) | multimeter on the pack before a run; check it is 3S |
| `robotMassKg` | 1.15 | **estimate**: parts from their datasheets + printed parts, see "Robot mass" below | weigh it |
| `wheelMu` | 0.8 | **assumed** (silicone on the field floor; no datasheet) | lowest `turnright(pwm)` that turns it on the spot (model: about 43) |
| `skidFactor`, `trackWidth` | 1.3, 156 mm | **assumed**, CAD | degrees turned by `turnright(150)` in 1 s (model: about 77) |
| `motorGainSigma` | 0.03 | **assumed** (Pololu gives no motor-to-motor spread) | how far `fw(150)` drifts sideways over 1 m with no walls |
| `tractionMean/Sigma` | 0.97 / 0.02 | **assumed** | encoder counts vs real distance over 2 m |
| `tofNoiseMm/Pct`, `tofOffsetSigma` | 1.5 mm + 3%, 5 mm | **datasheet**: VL53L0X table 12, standard deviation 4% at 33 ms (white target, including part-to-part); table 14, offset drift < 3% | repeated `measure()` at known distances |
| `tofConeDeg`, `tofMaxRange`, `tofPeriodUs` | 25 deg, 1200 mm, 33 ms | **datasheet**: VL53L0X field of view 25 deg; 120 cm minimum on white at 33 ms (table 11); default timing budget | - |
| `tofMinReliableMm` | 30 | Adafruit #3317 page ("approximately 30 mm to 1.2 m") | `measure()` with a wall 10-40 mm away |
| `gyroMagnetic` | -1 = follow the firmware | **datasheet + code**: `bno.begin()` defaults to NDOF, which the BNO055 datasheet (3.3.3.5) defines as absolute orientation, i.e. heading from magnetic north. `IMUPLUS` would be relative to the start. `--set gyroMagnetic=0` / `=1` forces one or the other | does `heading()` read 0 at power-on whichever way the robot faces? |
| `gyroMagneticAfterS` | never (1e9) | NDOF heading only turns magnetic once the magnetometer is calibrated (needs the robot turned through many orientations); the real robot explored like a relative heading, so by default that never happens in a run. `0` = calibrated from the start, `60` = switches 60 s in | watch `bno.getCalibration()` during a run |
| `magErrorDeg` | 2.5 deg | **datasheet**: BNO055 magnetometer heading accuracy +-2.5 deg, fully calibrated (real rooms with motors and steel are usually worse) | heading at the same spot facing the same way in different parts of the field |
| `gyroDriftSigmaDegPerMin` | 0.5 | **assumed** (the datasheet gives the raw gyro offset, which the fusion removes; no figure for fused drift) | heading change while standing still for 5 min (IMUPLUS mode) |
| colour raw values | in `getRawData()` | **assumed**; counts clip at (256 - ATIME) x 1024 = 10240 at 24 ms (**datasheet**, TCS34725) | `read_color()` printout on each tile type of your field |
| time costs | in each simulated library call | library behaviour | time `loop()` iterations with `micros()` |

### Robot mass

From the parts in the BOM and the CAD (`V2.step`):

| Part | Each | Qty | Total | Source |
|---|---|---|---|---|
| Pololu 195:1 20D 12 V gearmotor (#3493) | 46 g | 4 | 184 g | Pololu |
| Encoder boards + magnets (#3499) | ~1 g | 4 | ~4 g | estimate |
| Arduino GIGA R1 WiFi | 63 g | 1 | 63 g | Arduino |
| OpenMV Cam H7 Plus | 17 g | 2 | 34 g | OpenMV |
| Adafruit VL53L0X (#3317) | 1.3 g | 7 | 9 g | Adafruit |
| Adafruit TCS34725 (#1334) | 3.2 g | 1 | 3 g | Adafruit |
| Adafruit BNO055 (#2472) | 3 g | 1 | 3 g | Adafruit |
| 28BYJ-48 stepper (kit dispenser) | 37 g | 1 | 37 g | Adafruit #858 |
| Zeee 3S 2200 mAh LiPo | 137-185 g | 1 | ~160 g | Zeee (50C shorty 137 g, 120C pack 185 g) |
| Carobot motor shield V3, Qwiic mux, 5 V regulator, protoboard | | | ~50 g | estimate |
| 16x2 LCD | ~35 g | 1 | ~35 g | typical 1602 module |
| Wiring, Qwiic cables, LED strips, breaker, screws, bearings | | | ~80 g | estimate |
| Wheels | ~20 g | 4 | ~80 g | estimate |
| 3D-printed chassis, plates, spacers, chutes, camera mount, suspension | | | ~350 g (250-450) | estimate (depends on infill) |
| **Total** | | | **~1.15 kg (1.0-1.3)** | |

`--set robotMassKg=1.0` and `=1.3` bracket it; weigh the robot to replace the estimate.

### Bench mode on the real robot

`Main/bench.cpp` measures most of the **assumed** values above on the robot itself: set `BENCH_MODE` to 1
in `Main/main.cpp`, upload, and follow the Serial monitor (the pause switch starts each test). It prints
the distance sensor readings in a closed tile against what the CAD says, the lowest PWM that turns and
that drives the robot, degrees turned by `turnright(150)` in 1 s, encoder distance and heading change
over a 2 s straight drive, and the gyro's drift standing still. Put the numbers into `--set` (or the
defaults above) so the simulator matches the robot. `BENCH_MODE 0` (the default) leaves the firmware as it is.

`--selftest` runs these experiments on the simulated robot (sensor readings centred in a tile, speed
and turn rate at several PWM values, the lowest PWM that moves or turns it, colour values). Run the
same ones on the real robot and change the defaults until they match.

## Trying code changes without editing Main/

```
python tools/sim/physics/run_physics.py --fw ROBOT_WIDTH_MM=183 --fw ROBOT_LENGTH_MM=198
python tools/sim/physics/run_physics.py --patch "Distance.cpp|old text|new text"
```

`--fw` changes a `#define`, `--patch` replaces a piece of text (it must appear exactly once). Both
work on a temporary copy; `Main/` is never touched. Compare against a plain run on the same seeds.
