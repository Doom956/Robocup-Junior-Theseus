# Maze simulator

Two tools:

- **`run_sim.bat`**: runs the robot's **real** navigation code (`Main/navigation.cpp`,
  `Main/MazeTile.cpp`) on your PC against thousands of random RCJ-style fields. Use it to check a
  navigation change before uploading to the robot.
- **`maze_sim.html`**: open it in a browser to watch the planner explore a field step by step,
  edit the field (walls, black/blue/silver tiles, start), see the robot's map for each floor, and
  batch-test 200 or 2000 fields. It is a JavaScript port of the same logic, so it does **not**
  pick up changes to `Main/` automatically. Trust `run_sim.bat` for the real code.

## Setup (once)

```
pio pkg install -g --tool platformio/toolchain-gccmingw32
```

## Run

From the repo folder in the VS Code terminal:

```
tools\sim\run_sim.bat                                   (logic test: perfect robot, must pass 100%)
tools\sim\run_sim.bat --robot realistic --runs 2000     (with faults: compare the numbers)
tools\sim\run_sim.bat --robot realistic --scenario ramp --seed 2 --show
```

| Option | |
|---|---|
| `--robot NAME` | `perfect` (default), `realistic` or `harsh`, see below |
| `--runs N` | fields per scenario (default 500) |
| `--scenario NAME` | `flat`, `loops`, `big`, `ramp`, `bigramp` or `all` |
| `--random` | use a new random block of seeds (printed at the top, so any field can be replayed) |
| `--start S` | use seeds S to S+N-1 |
| `--seed S` | replay one field (the output prints a replay command for the first failure) |
| `--show` | with `--seed`: draw the real field and the robot's map for each floor |
| `--verbose` | also print the robot code's `Serial` output |

### Fields

| Scenario | Fields |
|---|---|
| `flat`    | 5x5 to 8x8, one path between any two tiles (no loops) |
| `loops`   | 5x5 to 9x9 with extra openings (loops / islands) |
| `big`     | 12x12 to 16x16 with loops |
| `ramp`    | two areas on different levels joined by a 1 to 3 tile ramp |
| `bigramp` | two large areas (7-8 x 10-14) on different levels joined by **two** ramps |

All include black, blue and silver tiles. Black tiles are only placed where they don't wall off
any part of the field, so everything is always explorable.

### Robot realism

| | Wall missed | Phantom wall | Stall | Wheel slip | Ramp count off by 1 | Black missed |
|---|---|---|---|---|---|---|
| | per reading | per reading | per move | per move | per ramp | per approach |
| `perfect`   | 0 | 0 | 0 | 0 | 0 | 0 |
| `realistic` | 1% | 1% | 3% | 0.3% | 10% | 2% |
| `harsh`     | 3% | 3% | 8% | 1% | 25% | 6% |

Each fault goes through the same code the robot uses to recover:

- a missed wall makes it drive at a real wall: emergency stop, short move, `handleShortMove()`
  (two failures in a row block that edge)
- a stall is a short move too
- wheel slip moves the map position without moving the robot (it is now lost)
- a miscounted ramp shifts everything mapped past it
- a missed black tile is a lack-of-progress: the robot is restarted at the last checkpoint it
  really visited, and the code restores `x_checkpoint`/`y_checkpoint`/`floor_checkpoint`
- four short moves in a row also count as lack of progress

An 8-minute clock runs in every mode. Time per action (edit the `T_*` constants in `sim.cpp` and
the `T` object in `maze_sim.html` to match your robot): forward 2.5 s, 90° turn 1.2 s, 180° turn
2 s, wall reading 0.8 s, blue tile 5 s, ramp 3 s per tile, failed move 3.5 s, restart 10 s.
`timeToReturn()` decides when to head home, exactly as on the robot.

With `perfect`, every field is run twice and both must pass, or there is a bug in the navigation
logic:

1. **no clock**: it must explore every reachable tile, store every wall exactly, and get home
   (with the clock on, an early trip home would hide a planner that skips parts of the field)
2. **8-minute clock**: it must get home in time, which checks `timeToReturn()`

The test catches deliberate bugs: a planner that can't see far enough fails 2091 of 5000
fields, the old ramp bug fails every ramp field, and the old return-time estimate fails 3.
With faults on, failures are expected; use the numbers to compare code changes:

| Number | Meaning |
|---|---|
| passed | home before 8:00, and either explored everything or headed home early for time |
| home in time | back on the real start tile before 8:00 |
| avg coverage | share of reachable tiles visited (large fields can't be finished in 8 min) |
| map walls | share of stored walls that match the real field on visited tiles |
| lost | map position or floor stopped matching the real robot at some point |

### Same fields or new fields?

By default `run_sim.bat` uses seeds 1 to N, so the same fields come up every time. Use that to
compare two versions of the code fairly. Add `--random` (or untick "Same fields every time" on the
web page, which is the default there) to test on new fields. Between random batches the pass rate
moves a little just by chance: about ±2 points for 2000 fields, ±6.5 points for 200.

## How it works

It is a **tile-level** simulation. The robot is always centred in a tile, facing exactly N, E, S
or W, and a move is either one whole tile or a failure. A wall reading is either the true answer
or flipped by chance. Nothing about where the sensors are mounted, beam angles, distances,
`MIN_DIST`, the robot sitting off-centre, motors or PID is modelled. So it tests the navigation
**decisions** (planning, map bookkeeping, recovery, getting home) when things go wrong, not how
often things go wrong on the real robot.

## What it does NOT test

Motors, sensor hardware, PID tuning, victims and obstacle avoidance. The fault rates are
estimates, not measurements: once you log real runs, set them to what you actually see.

Most of the code under test is the real code. The few parts that live in the hardware files are
copied into `sim.cpp` (`moveForward()` = `fwd()` + `finishTileMove()`, `senseTile()`,
`handleShortMove()`, `blockEdge()`, `timeToReturn()`/`homePath()`, the RETURN loop and the PAUSE
restart). **If you change those parts of `main.cpp`/`movement.cpp`, update `sim.cpp` to match.**

`physics/` is a second simulator that runs the real firmware (all of `Main/`) against a simulated
robot with motors, encoders, distance sensors placed from the CAD, gyro and colour sensor. See
[physics/README.md](physics/README.md).

`cad/` holds the robot's sensor positions taken from the CAD (`robot_geometry.md` / `.json`) and
the script that extracts them from a STEP export. The physics simulator reads them.

`stubs/` holds empty stand-ins for the Arduino and hardware library headers so the robot code
compiles on a PC. They are never used for the real robot build.
