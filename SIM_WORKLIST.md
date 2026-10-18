# Simulator work list for the fast PC (written 2026-10-18)

**How to start:** on the fast PC, `git checkout main`, `git pull`, `git fetch`, open Claude Code in this folder and say
**"work through SIM_WORKLIST.md"** (or name one item). Work top to bottom unless I say otherwise. Tick items off in this
file (`[x]` plus a one-line result) and push, so both PCs see the progress.

## Rules for every item (from CLAUDE.md)

- A/B with `tools/sim/physics/ab.py` on **600 comp fields, both seed sets** (`--start 1` and `--start 601`). Report score,
  time lost and back home with their 95% intervals, plus restarts per run and the worst-10% line.
- Label each change. **Logic bug** (wrong on any robot): main, if the A/B shows no harm and `pio run` builds; also merge
  it into robot-test. **Physics-dependent** (gains, powers, thresholds, avoidance behaviour): robot-test only, with a
  ROBOT_TEST.md entry: what changed, why, the numbers, and a short real-robot test (what to run, what to watch in the log).
- **Strategy changes need my yes** after I've seen the 600-field numbers (e.g. when to drive around obstacles, when to
  go home, exploring order). Write them up and hold them.
- Simulator parameters only from CAD, datasheets, the 2026 rules or measurements on the robot; mark anything else
  "assumed" and test a range. Don't push a sim fit that isn't backed by a measurement.
- After any change to the simulator itself, re-run main vs robot-test (both seed sets) and note it in NEXT_STEPS.md.
- The robot runs **robot-test**. Never run main on the robot while the replacement left-back sensor is fitted.
- Rules: `RCJRescueMaze2026-final.pdf` (junior.robocup.org, last updated 2026-03-29). Key ones below are quoted by number.

---

## A. Make the simulator match the robot better (do first: every later result depends on it)

- [x] **A1 result (2026-10-18):** M7 can't be the motor speed fit: fitting it (166 RPM, friction 0.11) breaks M3 (1.4 s
  vs 1.1), M1 (1.7 s vs 0.8-1.0) and bench tests 2/3 (50 / 84 vs 25-30 / 92-97). Free wheels can't be slower than on the
  floor at full power. Correction (user): the first M7 (36-38) was with the battery OFF (USB only); the second had the
  battery on, voltage unknown, so "low battery" is a guess. fw(150) (66/67/89 RPM = 276-373 mm/s) is consistent with a
  1.1 s tile at PWM <= 150: check the fit against fw(150). Wheel spread: most of it is motor A (already modelled by
  motorAExtraFriction); B vs D 4-8% at full power. 12% stacked on top of motor A cost -15.3 +-6.3 but double-counts A:
  0.05 vs 0.03: -2.5 +-3.6 (no clear effect): default 0.05. Speed fit vs fw(150): matching M7 at 150 (145 RPM: 73-80 vs real
  66-89) makes a tile 1.31-1.36 s (real 1.08-1.13) and turns 1.33 s; with 170 RPM the tile matches but M7 at 150 reads
  88-94. One speed can't match both: the sim's fwd() dynamics differ somewhere (motor response, slow-down near the end).
  Kept the tile-time fit (that's what navigation timing depends on); open until M7 is repeated with the voltage. Next robot session: charged battery, note the voltage, redo M7, and swap
  motor A's plug with another motor's (if the weakness moves with the motor, it's the motor). Selftest prints M7.
- [ ] **A1. Motor speed from M7.** Move test M7 (2026-10-18, logs/2026-10-18/move_tests2.txt), wheels in the air:
  fw(255) **118 / 139 / 150 RPM** (wheels A / B / D), fw(150) **67 / 66 / 89**. The sim's fit is ~158 / ~90. Refit the
  motor model to these (per wheel if the model allows: wheel A is the weak one, as in bench test 4), then re-check that
  M3 (one tile ~1.1 s, ~300 mm), M1 (90 deg in 0.75-1.0 s) and bench tests 2-4 still match (`--selftest`).
- [ ] **A2. Where moves stop.** The robot was seen stopping short of the tile middle (video 2026-10-17); robot-test 2m
  (back-wall centring only for this tile's wall) should have fixed most of it. When the next maze log comes in, run
  `tools/robot_log.py` ("stops along the tile") and compare with `stopcentre.py` on the sim. Fit if they differ.
- [ ] **A3. Practice maze size.** When I measure it (inner wall faces across a tile, wall thickness), add it as a field
  option and re-run robot-test on it. The robot_log centring numbers suggested walls ~25-30 mm further than RCJ's.
- [ ] **A4. Ramp climb timing.** Real ramp climbs take ~38 fwd() loops, the sim's ~23 (NEXT_STEPS 1c). Find out why
  (motor torque on the slope, grip, loop timing) once a log with millisecond timestamps exists.
- [ ] **A5. Speed bumps** (rules 3.4, 5.6.8: 5 points per tile crossed). Model them (height from the rules) and check the
  robot doesn't mistake one for a ramp (RAMP_PITCH_DEG 12; a bump tilts this robot ~7 deg) or lose its position on one.
- [x] **A6 result (2026-10-18, rules PDF of 2026-03-29):** 5.6.6 blue 30, -10 per revisit, never below 0 per tile; 5.6.7
  reliability non-negative, 10 x SBV - 15 x LoP; 5.6.8 bumps 5 per tile; 5.6.9 ramps 10 once per ramp; 5.6.11 checkpoints
  10; 5.6.12 exit +10 per SBV, +5 per ramp; 5.6.13 no duplicates; total never below 0: all as the sim scores them
  (victims, stairs, misidentification not simulated). Exit bonus needs >= 10 s blinking 1 s on / 1 s off with the
  victim LED: the code blinks LEDPIN (51, the victim LED) 1 s on/off at home until paused. 5.5.2: after a LoP the
  robot may be put down facing any direction: the code turns to gyro 0 (the gyro tracks while carried); the sim now
  does that (lopAnyDirection 1, was always the start direction). 5.3.1: the 8 min include calibration, but the code's
  clock starts at power-on (RUN_TIME_S 480) and the sim's at the run start: if the team spends time on the field before
  powering on, lower RUN_TIME_S (team procedure, not changed).
- [ ] **A6. Scoring check against the 2026 rules:** blue tile revisits -10 each (5.6.6), reliability bonus with -15 per
  LoP (5.6.7), exit bonus (5.6.12: +10 per victim and blue tile, +5 per ramp), no duplicate rewards (5.6.13), the 8 min
  including calibration (5.3). Make sure the sim's score does all of these.

## B. Navigation: points still being lost (biggest first)

- [ ] **B1. Blue tile revisits.** Rule 5.6.6: every revisit of the same blue tile costs 10 points, and each entry is a
  5 s stop. Real run 2026-10-17: 5 blue stops with 2 blue tiles. Measure how often the sim re-enters blue (scoregap.py)
  and try: (a) the planner prefers routes that avoid visited blue tiles when another way exists (the earlier try routed
  around *all* blue: +0.6; try only visited ones, and on the way home); (b) when exploring, visit a blue dead end last.
  Strategy-ish: report before keeping.
- [ ] **B2. Obstacles between the front beams** (lostcause 2026-10-18: obstacles cause ~75% of lost runs; most are
  obstacles in the gap between the two front sensors, pushed by the robot). Ideas to test, in this order:
  (a) the BNO055's accelerometer: a sharp deceleration when the bumper hits something (does the sim model it? needs a
  real measurement first: add a move test that logs acceleration while hitting the box); (b) heading disturbance: a
  robot pushing an off-centre obstacle gets turned (gyro yaw rate while the encoders say straight); (c) encoder speed
  dropping while PWM is unchanged; (d) side sensors' change vs encoders. Each as a no-progress-style check; count false
  stops per 100 runs as well as the score.
- [ ] **B3. Obstacles at the robot's corner.** The simple side-front < 30 mm rule fired at every wall start/end
  (-77, dropped). Needs real readings: move test M4 + box against the wall (robot test, see NEXT_STEPS section 3).
  Then: only when the map says there is no wall on that side at that point along the tile.
- [ ] **B4. Obstacle blocks cleared too early** ("map looks finished early" clears a block made seconds before). Tried:
  keep blocks made in the last 60 s (-20, explores less) and keep while something else is open (-1.0 / +2.3). Try:
  keep a block until the robot has explored N more tiles, or clear only blocks made more than 2 min ago. **Strategy:
  report, don't keep without my yes.**
- [ ] **B5. "no path found" loop** in badly-lost runs (robot-test: 2.6 restarts per run at one point; worst 10% of
  runs). Tried: drop guard walls when no path (-0.6/-0.7), write the wall only when the tile matches (no effect). Try:
  after "no path found", relocalize from the last checkpoint's map, or head for the last checkpoint and ask for a LoP
  restart deliberately (rules 5.5) instead of sitting still.
- [ ] **B6. Time management.** The 8 minutes include calibration; the exit bonus pays +10 per victim and per blue tile
  and +5 per ramp only if the robot gets home (5.6.12). Re-check `timeToReturn()` on the fitted (faster) sim:
  RETURN_SEC_PER_TILE and the margin were set on the slow sim. Measure real seconds per tile on the way home.
- [ ] **B7. Exploring order** (which open edge first) on the fitted sim. Earlier tries (left/straight/right,
  side-openings first, time-aware planner) gained nothing on the slow sim; the faster robot changes the trade-offs.
  **Strategy: report first.**
- [ ] **B8. Lack of progress costs:** each LoP is -15 in the reliability bonus (5.6.7) plus the restart time. Count LoP
  causes (lostcause) and see which are avoidable: e.g. stopping 5 s on blue before backing off, black tile handling,
  stuck on ramps.
- [ ] **B9. Relocalize** (robot-test 2): measure how often it fires in the sim and how often it's right; tune or extend
  (e.g. check two tiles back) only if it's often wrong or often missing a real one-tile error.
- [ ] **B10. Ramps:** a robot stuck tilted on the slope added 13-21 ramp tiles (12 of 674 crossings); the cap was held
  because those runs were lost anyway. Look for what gets it stuck (A4) and whether 2l (drive 155 mm after the ramp)
  holds for both ramp directions and lengths.
- [ ] **B11. Speed-sensitive settings** after A1: turn power 45 / gain 8, steering gain 1, minimum drive PWM 55,
  MAX_STEER_DEG 10, the no-progress thresholds (2j/2n): a quick sweep of each on the refitted sim. Keep only clear gains;
  these were checked once on the 2026-10-18 fit (keep all), redo only if A1 changes the motor model a lot.
- [ ] **B12. Stairs** (rules 3.4 and 5.6.10: 10 points up or down; not simulated). The real robot can be pushed up the competition-like
  step backwards and forwards (2026-10-17). Add stairs to the sim (dimensions from the rules) and see whether the current
  code climbs them, treats them as a wall, or gets lost. Only then think about a stair-climbing routine (strategy).

## C. Victims: not simulated (decided 2026-10-18)

Simulating victims would rest on guesses (camera range, angle, miss rate, false circles), none of them measured. The
code-level facts don't need a sim (a second victim on the same tile is ignored; each `c` with no letter costs up to 4 s).
Instead, measure on the robot in the cameras-on maze runs, from the Serial log: victim stops with no victim (the `c`
waits) per run, victims driven past without a stop, and seconds spent in victim stops per run. Those go to Baichen.

## D. Robustness and reporting

- [ ] **D1.** After A1-A3: robustness sweep of robot-test (sensor noise x2, floor reflection x2, grip x3, gyro drift
  0.5 deg/min, walls 6 / 30 mm) and main vs robot-test on both seed sets. Write the table into NEXT_STEPS.md.
- [ ] **D2.** `lostcause.py` and `scoregap.py` on robot-test after each kept change: what loses most points now.
- [ ] **D3.** Update `docs/PORTFOLIO.md` section 5 (results table) with the newest main vs robot-test numbers.

## Not on this list (decided)

- Camera 4 s wait as a sim option, and victims in the sim: not wanted (user, 2026-10-18): measure on the robot instead (section C).
- A third, middle front distance sensor: no free mux port (user, 2026-10-18).
- Changing ROBOT_WIDTH_MM / the 80 mm one-wall gap: keep as is (CLAUDE.md rule).
