# Maze simulator

Two tools:

- **`run_sim.bat`**: runs the robot's **real** navigation code (`Main/navigation.cpp`,
  `Main/MazeTile.cpp`) on your PC against thousands of random RCJ-style mazes. Use it to check a
  navigation change before uploading to the robot.
- **`maze_sim.html`**: open it in a browser to watch the planner explore a field step by step,
  edit the field (walls, black/blue/silver tiles, start), see the robot's map for each floor, and
  batch-test 200 fields. It is a JavaScript port of the same logic, so it does **not** pick up
  changes to `Main/` automatically. Trust `run_sim.bat` for the real code.

## Setup (once)

```
pio pkg install -g --tool platformio/toolchain-gccmingw32
```

## Run

From the repo folder in the VS Code terminal:

```
tools\sim\run_sim.bat
```

It rebuilds against whatever is in `Main/` right now, then prints one line per scenario:

```
flat    500/500  passed   avg 46.6 moves/run
loops   500/500  passed   avg 67.3 moves/run
big     500/500  passed   avg 231.6 moves/run
ramp    500/500  passed   avg 76.6 moves/run   ramp crossed in 382 runs

ALL PASSED
```

| Scenario | Mazes |
|---|---|
| `flat`  | 5x5 to 8x8, one path between any two tiles (no loops) |
| `loops` | 5x5 to 9x9 with extra openings (loops / islands) |
| `big`   | 12x12 to 16x16 with loops |
| `ramp`  | two areas on different levels joined by a 1 to 3 tile ramp |

All scenarios include black, blue and silver tiles.

### Options

| Option | |
|---|---|
| `--runs N` | mazes per scenario (default 500) |
| `--scenario NAME` | `flat`, `loops`, `big`, `ramp` or `all` |
| `--seed S` | replay one maze (the failure line tells you which seed) |
| `--show` | with `--seed`: draw the real maze and the robot's map for each floor |
| `--verbose` | also print the robot code's `Serial` output |

Replay a failure: `tools\sim\run_sim.bat --scenario ramp --seed 7 --show`

## What each run checks

- every tile reachable without crossing black was visited
- the planner never chose a direction with a real wall
- `x_pos`/`y_pos` and `currentFloor` always match where the robot really is
- every wall stored in the map matches the real maze
- the robot gets back to the start tile, and it is really there

## What it does NOT test

The simulated robot drives perfectly and its sensors never lie. It tests the navigation
**logic** only: not motors, sensors, PID tuning, timing, victims or obstacles.

Most of the code under test is the real code. The few lines of map bookkeeping that live in the
hardware files are copied into `sim.cpp` (`moveForward()` = `finishTileMove()` in `main.cpp` plus the
ramp loop at the end of `fwd()` in `movement.cpp`; `senseTile()` = `SENSE_TILE`/`UPDATE_MAP`; the
RETURN loop). **If you change those parts of `main.cpp`/`movement.cpp`, update `sim.cpp` to match.**

`stubs/` holds empty stand-ins for the Arduino and hardware library headers so the robot code
compiles on a PC. They are never used for the real robot build.
