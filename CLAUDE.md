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

## 2. Rules
- Navigation only; don't touch victim detection or dispensing.
- No guessing in the sim: values from CAD, datasheets, the 2026 rules or my bench measurements; mark anything else as assumed.
- Before changing firmware: diagnose with traces (run_physics.py --trace), try it on a temporary copy, A/B on 200 comp fields with ab.py, check pio run.
- Push to main only if it clearly helps (interval above zero, or a definite logic/geometry bug). Otherwise keep it on a branch or uncommitted and tell me.
- Keep ROBOT_WIDTH_MM as is (80 mm one-wall gap beat 49/59 mm).
- Never add, pull from or push to my teammate's repo (Chrisyyyys/Robocup-Junoir-Theseus). My repo stays private.
- Answer simply and directly.
