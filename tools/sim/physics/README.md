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
python tools/sim/physics/run_physics.py --selftest            # calibration experiments (see below)
```

It rebuilds from the current `Main/` every time. One 8-minute match takes about 1-2 seconds; fields
run in parallel. Each result line shows: back on the start tile, explored everything, average
coverage, map wall accuracy, how often the code's map position was wrong, lack-of-progress restarts,
time spent pressed against walls, and how runs ended (`home`, `time` = 8:00 ran out, `no path`).
`--trace FILE` (with `--seed`) writes the robot's real and believed position every 50 ms.

## What is simulated

| Real hardware | Simulated as |
|---|---|
| Clock | `millis()`/`micros()`/`delay()` run on simulated time; each hardware call costs about its real time (I2C transfers, a VL53L0X waits for its next 33 ms measurement, the TCS34725 waits its 24 ms integration) |
| 4 motors on the motor shield | PWM -> wheel speed with a dead band, a 0.08 s lag and a per-motor gain difference. A/C are the left side, B/D the right, D mounted reversed (from `motors.cpp`) |
| Driving | skid steering; ramps slow it uphill; walls block the robot (the wheels keep spinning, so the encoders keep counting); a turning robot scraping a wall gets shoved sideways |
| Encoders A, B, D | count actual wheel rotation, 975 per turn as the code assumes; wheel slip shows up as encoder error |
| 7 x VL53L0X | positions and directions from `../cad/robot_geometry.json`; a 25 degree cone, signal-weighted distance (nearer, face-on surfaces count more), noise, a fixed per-sensor offset, out of range = 8190 |
| BNO055 | heading clockwise from the start direction, with drift and noise; pitch from the ramp under the two axles |
| TCS34725 | raw r/g/b/clear of the tile under its CAD position |
| Pause switch / referee | a lack-of-progress restart when the robot drives onto black or doesn't change tile for 60 s: switch HIGH for 3 s, robot placed on the last silver tile it really visited, facing the start direction |
| Fields | the same generator as the tile simulator (black tiles never wall anything off), plus real ramp slopes of 15-25 degrees |

Not simulated: victims and the cameras (victim code never fires), obstacles, speed bumps, debris,
the BNO055 compass jumping to magnetic north, battery sag, wall thickness.

## Assumptions to calibrate

These come from datasheets and guesses, not from your robot, and **some change the results a lot**
(at a 5% dead band the same code gets home far more often than at 10%). All of them are in `Params`
at the top of `physics.cpp` and can be changed per run with `--set name=value`.

| Setting | Default | How to measure it on the robot |
|---|---|---|
| `motorDeadband` | 0.10 (PWM about 25) | lowest `drivetrain.fw(pwm)` that still moves the robot |
| `motorMaxSpeed` | 250 mm/s at PWM 255 | time `drivetrain.fw(150)` over 1 m, compare with `--selftest` |
| `skidFactor`, `trackWidth` | 1.3, 156 mm | degrees turned by `turnright(150)` in 1 s |
| `motorGainSigma` | 0.04 | how far `fw(150)` drifts sideways over 1 m with no walls |
| `tractionMean/Sigma` | 0.97 / 0.02 | encoder counts vs real distance over 2 m |
| `tofNoiseMm/Pct`, `tofOffsetSigma` | 1.5 mm + 1.5%, 5 mm | repeated `measure()` at known distances |
| `gyroDriftSigmaDegPerMin` | 0.5 | heading change while standing still for 5 min |
| colour raw values | in `getRawData()` | `read_color()` printout on each tile type |
| time costs | in each simulated library call | time `loop()` iterations with `micros()` |

`--selftest` runs the experiments in this table on the simulated robot (sensor readings centred in
a tile, speed at several PWM values, turn rate, colour values). Run the same ones on the real robot
and change the defaults until they match.

## Trying code changes without editing Main/

```
python tools/sim/physics/run_physics.py --fw ROBOT_WIDTH_MM=183 --fw ROBOT_LENGTH_MM=198
python tools/sim/physics/run_physics.py --patch "Distance.cpp|old text|new text"
```

`--fw` changes a `#define`, `--patch` replaces a piece of text (it must appear exactly once). Both
work on a temporary copy; `Main/` is never touched. Compare against a plain run on the same seeds.
