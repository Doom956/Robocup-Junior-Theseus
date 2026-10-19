# Test plan, 2026-10-19 (maze session on the laptop)

Goal: confirm the four robot-test changes still on the "Not yet confirmed" list (ROBOT_TEST.md on robot-test: 2l, 2m,
2n, 2o), then measure what the simulator still guesses. If they pass, the next change (colour read every 3rd loop)
can go on robot-test and the known-good tag moves to this firmware.

## Before starting

- `git checkout robot-test`, `git pull`. Check `#define BENCH_MODE 0` in `Main/main.cpp`. Upload with the battery on.
- Battery charged. **Write down the battery voltage** (and again at the end).
- Gyro cable seated, **cameras unplugged** (navigation only; the camera wait hides everything else).
- At power-on the LCD says `READY`. The Serial log's first lines must show `firmware <commit>` (expected `9205f12` or
  newer): note it.
- Every test is logged: `python tools/serial_log.py logs/2026-10-19/<name>.txt` (it stamps milliseconds now). Start the
  log before switching on, stop it with Ctrl+C after the run.
- Restarts only with the pause switch, never by cutting the power (that resets the run clock and loses the map).
- After each test, write one line of what you saw (and roughly when: "run 2, ~1:30, hit the obstacle").

## Part A: normal runs (2-3 runs, ~10 min each)

Logs: `run1.txt`, `run2.txt`, `run3.txt`. Start on the start tile, as on 2026-10-17. Put an obstacle in at least one run
(touching a wall, reaching at most ~9 cm into the path, as rule 3.4.4 allows).

| Watch | Change | Pass | Fail |
|---|---|---|---|
| Where it stops in each tile | 2m (back sensor only for this tile's wall) | ends in the middle of tiles | ends near the back edge again |
| `[FWD] no progress` with nothing there | 2n | 0-1 per run | several per run |
| Repeated `no progress` at the **same** edge with nothing there | 2o | never | happens: **2o must be reverted** (as 2026-10-17 partB8, 15:44) |
| Obstacle | 2n/2o | stops, backs up, goes elsewhere; later drives around it | drives into it and carries on |
| Home | all | stops on the start tile, LED blinks | wrong tile / "no path found" |

## Part B: ramp (2l)

Log: `ramp.txt`. Drive it over each ramp 3-4 times **each way** (a short run that crosses the ramp, or place it on the
tile before the ramp).

- Pass: after the ramp it stops in the middle of the next tile by itself; in the log, after each `compensating`, any
  `centring along` is small (under ~20 mm) or missing. (Before 2l: +15 to +70 mm past.)
- Watch **coming down** most closely: the weakest case in the simulator (a 1-tile ramp, 7% stopped 30+ mm short).
- If it stops partly on the slope (short): `RAMP_END_TO_MIDDLE_MM` 155 -> 170-180. Past: -> 140. Stuck on the ramp: tell
  me (revert 2l).

## Part C: black tile (for the next change)

Log: `black.txt`. From two tiles away, let it drive straight at a black tile, 3-4 times.

- Note each time: did it see `black` and back off, and **roughly how far onto the black tile** the robot got (none, a
  little, half).
- In the simulator the colour sensor is read only every ~120 mm of driving, so it can cross onto black before reading it
  (SIM_WORKLIST B8). If the robot gets half onto the tile even once, the colour-read change (every 3rd loop) goes on
  robot-test next.

## Part D: measurements (no log)

- One tile: distance between the two inner wall faces, and the wall thickness (A3: the centring numbers suggested the
  walls are 25-30 mm further apart than RCJ's 280 mm).
- Battery voltage at the end.

## Part E: if there is time (move tests, `BENCH_MODE 2`, then back to 0)

| # | Test | Setup | Why |
|---|---|---|---|
| E1 | M3 three moves (`m3.txt`) | wall behind the robot, 1.2 m clear ahead | with millisecond timestamps: the speed through a move (A1, the M7 vs tile-time conflict) |
| E2 | Step (`step.txt`): M8 with a 2 cm board taped across the path instead of the box | as M8 | B12 stairs: does it stop with `no progress` and back up, climb, or get stuck? |
| E3 | Motor swap: swap motor A's plug with motor B's, run M7 (`m7swap.txt`), swap back | robot lifted | does the weak wheel move with the motor? |

`BENCH_MODE` back to 0 before pushing anything.

## After the session

Push the logs (`logs/2026-10-19/`) and the one-line notes. Then:
- update the "Not yet confirmed" list in ROBOT_TEST.md (what passed, what didn't);
- if Part A-B passed: tag the firmware that ran as the new known-good;
- if 2o failed: revert it on robot-test.
