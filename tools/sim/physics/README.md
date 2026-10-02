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
python tools/sim/physics/run_physics.py --serve               # the batch dashboard in the browser (below)
```

It rebuilds from the current `Main/` every time. One 8-minute match takes a few seconds; fields
run in parallel (about 4 minutes for 1000 runs on 14 cores). Each result line shows: back on the start tile, explored everything, average
coverage, map wall accuracy, how often the code's map position was wrong, lack-of-progress restarts,
time spent pressed against walls, and how runs ended (`home`, `time` = 8:00 ran out, `no path`).
`--trace FILE` (with `--seed`) writes a recording of the run (see below).

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
| 4 motors on the motor shield | Pololu 195.3125:1 20D 12 V gearmotors (#3493) on a Carobot V3 shield (TB6612FNG + PCA9685) from a 3S LiPo. DC motor model from the datasheet: speed = no-load speed x (PWM duty - load / stall torque), scaled to battery voltage; load = gearbox friction (more from standstill) + sideways wheel drag when turning on the spot + gravity on ramps. A/C are the left side, B/D the right, D mounted reversed (from `motors.cpp`) |
| Driving | skid steering; walls block the robot (the wheels keep spinning, so the encoders keep counting); a turning robot scraping a wall gets shoved sideways |
| Encoders A, B, D | count actual wheel rotation: 5 counts per motor turn x 195.3125 = 976.6 per wheel turn (the code assumes 975); wheel slip shows up as encoder error |
| 7 x VL53L0X | positions and directions from `../cad/robot_geometry.json`; a 25 degree cone, signal-weighted distance (nearer, face-on surfaces count more), noise, a fixed per-sensor offset, out of range = 8190 |
| BNO055 | heading clockwise from the start direction, with drift and noise; pitch from the ramp under the two axles. Option `gyroMagnetic=1`: heading from magnetic north instead (team notes say "the heading is always global"), at a random maze angle, optionally only after `gyroMagneticAfterS` seconds |
| TCS34725 | raw r/g/b/clear of the tile under its CAD position |
| Pause switch / referee | a lack-of-progress restart when the robot drives onto black or doesn't change tile for 60 s: switch HIGH for 3 s, robot placed on the last silver tile it really visited, facing the start direction |
| Fields | the same generator as the tile simulator (black tiles never wall anything off), plus real ramp slopes of 15-25 degrees |

Not simulated: victims and the cameras (victim code never fires), obstacles, speed bumps, debris,
battery sag during a run (a 2200 mAh pack uses only about 6% in 8 minutes), wall thickness.

## Where the numbers come from, and what to measure

All of them are in `Params` at the top of `physics.cpp` and can be changed per run with
`--set name=value`. **Some change the results a lot**, so the ones marked "assumed" are worth measuring.

| Setting | Default | Source | How to check on the robot |
|---|---|---|---|
| `noLoadRpmAt12V`, `stallKgcmAt12V`, `frictionFracAt12V` | 72 RPM, 10 kg.cm, 0.05 | Pololu #3493 datasheet (80 mA no-load / 1.6 A stall) | lowest `drivetrain.fw(pwm)` that moves the robot (model: about 13-17) |
| `batteryVoltage` | 11.8 V | 3S LiPo (Zeee 2200 mAh in the team BOM; cell count read from the photo) | multimeter on the pack before a run; check it is 3S |
| `robotMassKg` | 1.3 | **assumed** | weigh it |
| `wheelMu` | 0.8 | **assumed** (silicone on the field floor) | lowest `turnright(pwm)` that turns it on the spot (model: about 43) |
| `skidFactor`, `trackWidth` | 1.3, 156 mm | **assumed**, CAD | degrees turned by `turnright(150)` in 1 s (model: about 77) |
| `motorGainSigma` | 0.03 | **assumed** | how far `fw(150)` drifts sideways over 1 m with no walls |
| `tractionMean/Sigma` | 0.97 / 0.02 | **assumed** | encoder counts vs real distance over 2 m |
| `tofNoiseMm/Pct`, `tofOffsetSigma` | 1.5 mm + 1.5%, 5 mm | VL53L0X datasheet range | repeated `measure()` at known distances |
| `tofMinReliableMm` | 30 | team BOM notes | `measure()` with a wall 10-40 mm away |
| `gyroDriftSigmaDegPerMin` | 0.5 | **assumed** | heading change while standing still for 5 min |
| `gyroMagnetic` | 0 (relative) | team BOM notes suggest it may be 1 | does `heading()` start at 0 when powered on facing different ways? |
| colour raw values | in `getRawData()` | **assumed** | `read_color()` printout on each tile type of your field |
| time costs | in each simulated library call | library behaviour | time `loop()` iterations with `micros()` |

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
