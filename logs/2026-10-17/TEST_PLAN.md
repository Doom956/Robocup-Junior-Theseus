# Test plan, 2026-10-17

Before starting: battery charged, gyro cable seated, USB in (for the log), cameras off until Part C.
**Before each part, tell me "ready for Part X"**: I flash the right firmware and start a new log.
After each test, tell me what you saw (one line is enough); I read the log.

## Part A: move tests (no maze needed, ~40 min)

**I upload:** move-test firmware. LED blinks slowly = waiting. Place the robot, flip the pause switch on
and off, hands off (the test starts 1 s later). LED on = test running. Point the robot along the line it
should drive before each test.

| # | Test | What to do | What to tell me |
|---|---|---|---|
| A1 | M0 floor readings (10 s) | Robot on a **white maze tile**, nothing within 50 cm, step away. LED goes off when done | Anything near it? |
| A2 | M1 turns | Open floor, room to turn. 4 right, 4 left, then 180 and back | Did it swing past and come back? |
| A3 | M2 small turns | Same spot | Did every small turn move it? |
| A4 | M3 three moves | **Flat wall** right behind the robot, 1.2 m clear ahead | Anything odd (it measures itself) |
| A5 | M4 along a wall | Straight wall on one side, robot parallel, side 4-5 cm from the wall | Did it swing left-right? |
| A6 | M5 wall ahead | Robot facing a wall, nothing behind it. Slide it until the **LED comes on**, then flip | Did it reverse fast? |
| A7 | M6 obstacle | As A6, plus the obstacle **taped down** in the middle, a hand-width ahead; LED must be on | Did the wheels spin or stall? Did it back up? |
| A8 | Stairs (by hand) | Push the robot up a step backwards, then forwards | Does it go up backwards without scraping? |

When the LED blinks fast, all done: flip the switch once more.

## Part B: maze (cameras off)

**I upload:** normal robot-test firmware. **Send me the maze layout first** so I can say which tile to
start on for each test. Start each run from the start tile unless it says otherwise.

| # | Test | What to do | What to watch / tell me |
|---|---|---|---|
| B1 | Free run (5 min) | Normal start, let it explore | Did it get lost? (I check stop positions, turns, straight starts) |
| B2 | Corridor | Start at the end of a 3+ tile straight | Stops in the middle of each tile? Weaving? |
| B3 | One wall | Stretch with a wall on one side only | Holds its distance, no turning into the wall? |
| B4 | Parallel | Start ~10 deg crooked next to a wall | Is the first move straight? |
| B5 | Colour | Normal run | Any stops on white tiles (false blue)? |
| B6 | Floor tiles | Black / blue / silver tile laid on a white one | Backs off black? 5 s on blue? Checkpoint on silver? |
| B7 | Wall end (1a) | No obstacle; robot 4-5 cm off-centre toward an open side, next tile has a wall | Any "obstacle" stop with nothing there? |
| B8 | Obstacle by a wall (1b) | Obstacle touching a wall of the next tile: start, middle, far end (3 runs) | Stops without touching it, goes elsewhere? |
| B9 | Dead end (1c) | Obstacle in front of the only way out of an area | Drives around it later, ends on the right tile? |
| B10 | Relocalize (2a) | After 3-4 tiles: pause, carry it back one tile (same direction), resume | Did it notice? (I check the log) |
| B11 | Home (2c) | Near the end, on its way home: pause, carry it one tile off, resume | Stops on the right tile? |
| B12 | Pause/restart | Pause mid-run, put it on the last checkpoint, restart | Carries on correctly? |
| B13 | Ramp | If the layout has one: up and down | Floor change right? (I check the log) |
| B14 | Full runs x3-5 | Different start tiles and directions, obstacle in different places | Lost? Home? Time? |
| B15 | Display | After the new LCD is in | Does it show the position? |

## Part C: cameras on

**I upload:** same robot-test firmware (cameras plugged in).

| # | Test | What to do | What to tell me |
|---|---|---|---|
| C1 | Full runs x2 | Victims placed as in a competition | Victims found, kits dropped? How long it waits after moves |
