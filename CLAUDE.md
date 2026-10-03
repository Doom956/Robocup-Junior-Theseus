# Theseus — RoboCupJunior Rescue Maze 2026

## 1. Project
- Firmware: Main/ (PlatformIO C++ port, Arduino GIGA R1 M7). Victim detection and dispensing work; the problem at competition (Korea) was the robot getting lost.
- Simulators: tools/sim (logic sim, run_sim.bat) and tools/sim/physics (real firmware against a simulated robot built from our CAD, datasheets and the RCJ 2026 rules; see its README for every parameter and its source).
- Judge changes on the `comp` field type (like the RoboCup 2025 international fields: ~48 tiles, two levels, a ramp). big/bigramp are oversized stress tests only.
- Score = exact RCJ 2026 navigation points; victims and stairs are deliberately not simulated.
- A/B tool: python tools/sim/physics/ab.py <refA> <refB> [--types comp] [--runs 200] [--set name=value]; WORK = working copy of Main/. Same seeds for both; +- is a 95% interval.
- Branches: main = all tested fixes; robot-test = main + obstacle strategy (treat an obstacle like a wall first, drive around only on a second try) + relocalize + side wall one tile away as a steering reference + ROBOT_TEST.md, waiting for real-robot tests (sim, 600 comp: +13.9 +-4.6, time lost 44% -> 17%, home 48% -> 74%); relocalize = the original relocalize branch (now also on robot-test).
- Results on 600 comp fields (seeds 1-600): old main 107.6 -> steering fix 110.2 -> + ramp/blue fixes 111.9 -> robot-test 124.1. Tools: tools/sim/physics/lostcause.py shows where/why sim runs first get lost; tools/robot_log.py reads a real robot's Serial log.
- On main: left-wall sign fix, avoidance recursion guard, map-edge guards, black check in avoidance, turn stops at target, re-read walls on the way home, retry when the map looks finished early, two-wall centring, restart revives a stopped robot, centreAlong (stop at tile centre), 100 ms sensor timeout, wall-end check (+19.6), gyro re-sync on walls, ramp wall-centring, steering for the whole tile (was clipped at 150) + 30 mm wall-end check, ramp up/down from one pitch reading, 5 s stop when backing into a blue tile, parallel() turns back if it squared up off the grid (corner), bench mode (BENCH_MODE in main.cpp, off by default; measures what the sim guesses).
- Tested and held (no clear gain): self-timed return estimate, wedged-turn back-off, relocalize on the way home, map-based wall-end check, second-try drive-on / mid-move avoidance, cascade wall follower, turn stall boost, stall check, finishing moves at minimum power, centring after turns, longer centring range, turn power boost, TURN_MIN_PWM 45, 49/59 mm one-wall gap, move/avoidance progress checks.
- Gyro: the BNO055's real heading drift isn't measured yet (sim assumes 0.5 deg/min); main re-syncs the heading on walls, so drift matters little.

## 2. Rules
- Start every session with git pull.
- Navigation only; don't touch victim detection or dispensing.
- No guessing in the sim: values from CAD, datasheets, the 2026 rules or my bench measurements; mark anything else as assumed.
- Before changing firmware: diagnose with traces (run_physics.py --trace), try it on a temporary copy, A/B on 200 comp fields with ab.py, check pio run.
- Push to main only if it clearly helps (interval above zero, or a definite logic/geometry bug). Otherwise keep it on a branch or uncommitted and tell me.
- Final keep/hold decisions: A/B on 600 comp fields (200 can't show a +4 gain). Report the 95% intervals for score, time lost and back home (no victims in the sim, so staying on track and getting home matter more in a real run than the score shows).
- Label every change: LOGIC BUG (wrong on any robot regardless of physics) -> main only if the 600-field A/B shows no harm (score interval not below zero) and pio run builds. PHYSICS-DEPENDENT (depends on how real the sim's motors, grip, turning, gyro or sensors are: gains, power boosts, avoidance strategy) -> never main; branch robot-test (from main) with ROBOT_TEST.md listing each change and a short real-robot test (what to run, what to watch in the Serial log).
- Strategy changes (e.g. treating an obstacle like a wall) need my yes after I've seen the 600-field numbers, before they go anywhere.
- The goal is the real robot; the sim is a tool and isn't perfect. Avoid real-life regressions: don't fully trust sim results unless they are conclusive or don't depend only on assumed values.
- Keep ROBOT_WIDTH_MM as is (80 mm one-wall gap beat 49/59 mm).
- Never add, pull from or push to my teammate's repo (Chrisyyyys/Robocup-Junoir-Theseus). My repo stays private.
- Answer simply and directly.
