# Theseus — RoboCupJunior Rescue Maze 2026

## 1. Project
- Firmware: Main/ (PlatformIO C++ port, Arduino GIGA R1 M7). Victim detection and dispensing work; the problem at competition (Korea) was the robot getting lost.
- Simulators: tools/sim (logic sim, run_sim.bat) and tools/sim/physics (real firmware against a simulated robot built from our CAD, datasheets and the RCJ 2026 rules; see its README for every parameter and its source).
- Judge changes on the `comp` field type (like the RoboCup 2025 international fields: ~48 tiles, two levels, a ramp). big/bigramp are oversized stress tests only.
- Score = exact RCJ 2026 navigation points; victims and stairs are deliberately not simulated.
- A/B tool: python tools/sim/physics/ab.py <refA> <refB> [--types comp] [--runs 200] [--set name=value]; WORK = working copy of Main/. Same seeds for both; +- is a 95% interval.
- Branches: main = all tested fixes; relocalize = shift position one tile when walls don't match the map (+3.9 on comp), waiting for a real-robot test.
- Results on 200 comp fields: original 57.4 -> main 110.7 -> relocalize 114.5.
- On main: left-wall sign fix, avoidance recursion guard, map-edge guards, black check in avoidance, turn stops at target, re-read walls on the way home, retry when the map looks finished early, two-wall centring, restart revives a stopped robot, centreAlong (stop at tile centre), 100 ms sensor timeout, wall-end check (+19.6), gyro re-sync on walls, ramp wall-centring.
- Tested and held (no clear gain): stall check, finishing moves at minimum power, centring after turns, longer centring range, turn power boost, TURN_MIN_PWM 45, 49/59 mm one-wall gap, move/avoidance progress checks.
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
