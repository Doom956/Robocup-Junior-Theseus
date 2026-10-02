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
