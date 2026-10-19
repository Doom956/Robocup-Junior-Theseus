# Robot test priority list (written 2026-10-19)

Firmware: **robot-test** (`git checkout robot-test`, `git pull`, `BENCH_MODE 0`). Do the tests top to bottom; stop
wherever time runs out. Details for each test: ROBOT_TEST.md on robot-test (section named in brackets).

## Before every session

- Battery on (USB alone: motors too weak). Write down the battery voltage at the start and end.
- Gyro cable seated. LCD says `READY` (or lists what's missing).
- Log every test: `python tools/serial_log.py logs/<date>/<name>.txt`, started before switching on. The first lines must
  show `firmware <commit>`.
- Restart only with the pause switch, never by cutting power.
- One line per test: what you saw, and roughly when.

## A. No maze needed (5 min)

| # | Test | Log | Pass | Fail |
|---|---|---|---|---|
| 1 | **Ramp guard**: start a move, lift the robot off the floor, put it back | `lift.txt` | log says `pitch over 35 deg: not a ramp`; no long run of `1 section of the ramp climbed` | dozens of `ramp climbed` lines, or the map position jumps a floor |

## B. Maze, cameras OFF (most important first)

| # | Test | Log | Pass | Fail |
|---|---|---|---|---|
| 2 | **Obstacle touching a side wall** (heavy printed obstacle, up to ~9 cm into the path), at the start, middle and far end of the next tile (section 1 b) | `obst_b.txt` | `obstacle ... blocking edge` or `backing up to start tile`; ends in the middle of its own tile, goes elsewhere, never pushes the obstacle | drives into it, pushes it, or ends off-centre / lost |
| 3 | **Wall end, no obstacle** (section 1 a): robot 4-5 cm off-centre toward an open side, then again turned ~10 deg | `obst_a.txt` | `squaring up and looking again`, then drives on | `blocking edge` with nothing there (note the two front readings) |
| 4 | **Relocalize** (section 2 a): let it explore 3-4 tiles, pause, carry it back one tile (same heading), resume | `reloc.txt` | `position corrected`, LCD position right again | no correction, or a wrong one |
| 5 | **Home check** (2c): on the way home, pause, carry it one tile away from the start, resume | `home2c.txt` | `walls don't match the start tile: not home yet`, then drives to the start | stops on the wrong tile |
| 6 | **Dead end behind an obstacle** (section 1 c): the only way out passes the obstacle | `obst_c.txt` | blocks it first; once nothing else is left: `cleared blocked edges`, drives around it, map position right afterwards | lost after driving around, or never goes back |
| 7 | **Obstacle just past a blue tile** (section 1 e) | `obst_e.txt` | `[FWD] on blue: waiting 5 s first` before backing up | backs off the blue tile at once |

## C. Maze, cameras ON (competition conditions)

| # | Test | Log | Pass | Fail |
|---|---|---|---|---|
| 8 | **2-3 full runs with victims and one obstacle**, as at the competition | `cam1.txt`... | gets home; count victim stops and how many were false (for Baichen) | lost, or many false victim stops / long waits |

## D. Move tests (`BENCH_MODE 2`, no maze; set back to 0 afterwards)

| # | Test | Log | Why |
|---|---|---|---|
| 9 | **M4 with a box against the wall** (obstacle at the robot's corner) | `m4box.txt` | sim item B3 needs the real readings |
| 10 | **2 cm step**: M8 with a 2 cm board taped across the path | `step.txt` | stairs (B12): stops and backs up, climbs, or gets stuck? |
| 11 | **Motor swap**: swap motor A's plug with B's, run M7 (robot lifted), swap back | `m7swap.txt` | is the weak wheel the motor or the driver channel? |

## Hardware (any time)

- LCD: re-solder, 100 nF capacitor across pins 1-2.
- Gyro cable: secure it (came loose twice).
- Pause switch: re-solder the ~10 k resistor pin 22 -> GND.

## After the session

Push the logs and notes. If 1-7 pass: move the `known-good` tag to the firmware that ran, and decide whether robot-test
becomes main.
