# Competition checklist (RCJ Rescue Maze, calibration day Thursday 2026-10-22)

Competition firmware: branch `robot-test`, tag `known-good-2026-10-20` (2026-10-20 maze: home 3 of 3, 2:09 / 2:06 / 1:59,
ramps 10/10). Details: ROBOT_TEST.md (robot-test). Camera scripts: `Camera_models/` on robot-test (Baichen).

## For the actual runs (values as of calibration day, 2026-10-22)

**Robot firmware:** robot-test `de988e4` with BENCH_MODE 0 (navigation = `known-good-2026-10-20`); the start-up line
must say `firmware de988e4 robot-test`. Switch to Baichen's `Vision-branch` only if it passed cameras-on practice runs
today; then write its commit here. Never run with a test mode (BENCH_MODE 2/4/5/6) on the robot.

**Camera files, ready to copy:** `Documents\OpenMV\COMPETITION_2026-10-22\left\` and `...\right\`
(from robot-test `5d575cb`). On each camera's drive: `main.py`, `labels.txt`, `trained.tflite` from its own folder
(left folder -> left camera). Eject before unplugging. Check: `CAMERA_NAME` in main.py matches the camera.

| Cognitive-target setting | Left camera | Right camera |
|---|---|---|
| black | 35, 2, 11 | 23, -11, 7 |
| red | 62, 45, 23 | 72, 36, 15 |
| yellow | 95, 0, 49 | 93, 3, 65 |
| green | 60, -28, 25 | 51, -45, 35 |
| blue | 84, -9, -19 | 85, -18, -17 |
| `WALL_L_MIN` | 48 | 65 (not recalibrated) |
| `WALL_CHROMA_MAX` | 42 | 35 (not recalibrated) |
| `CIRCLE_THRESHOLD` | 1000 | 1600 (not recalibrated) |

Letters (H/S/U): unchanged, work as before. Ring values: black -2, red -1, yellow 0, green +1, blue +2;
sum 2 = H (2 kits), 1 = S (1 kit), 0 = U (none), anything else = fake (ignored).

## Before every run
- Battery charged (write down the voltage). LED strips orange = battery low.
- Gyro cable seated. Cameras plugged in (competition runs) and their LEDs blinking blue.
- Power on with the pause switch on **pause**; check the start-up (`READY` or the log: all sensors found).
- Start: robot on the start tile, flip the switch to run.

## During a run (captain)
- Restarts only with the pause switch, with the robot placed on the **last checkpoint** (any direction). Never cut the
  power (that resets the run clock and loses the map).
- Never flip the switch just to "pause" mid-run: resuming puts the code on the last checkpoint.
- **If it rams an obstacle, spins its wheels or a turn stalls against something: call lack of progress straight away.**
  Driving around obstacles is the robot's known weak point; waiting only lets it get lost.

## Calibration day (venue)
1. Start-up check with the laptop (all 7 distance sensors, colour sensor, motor shield, gyro, `firmware <commit>`).
2. Measure their tiles (RCJ: 300 mm; our practice maze was 320 mm) and a ramp (height, length).
3. Colours on their tiles with the bench colour test (BENCH_MODE 1): white, silver, black, blue. `SILVER_THRESHOLD 800`
   in Globals.h is an absolute value: check silver reads clearly above it and white clearly below.
4. Cameras: `calibrate_colors.py` per camera under their lights (Documents\OpenMV\Camera_models); paste `COLOR_CENTRES`
   into both scripts (Baichen). Check in the OpenMV IDE: H/S/U letters and targets give `SEND`, plain walls nothing.
5. Practice runs on their fields, with cameras, **at least one without the laptop**. Plug in after a run to read the log.
6. Dispenser with all 8 kits (BENCH_MODE 4): right count, right side, the last kits still drop.

## Freeze (end of calibration day)
- Only fixes for things that failed on the robot after this point.
- Make robot-test the new main; tag the frozen commit; flash it with `BENCH_MODE 0`; check the start-up line shows that
  commit. Save both cameras' files (main.py, trained.tflite, labels.txt) into the repo.

## OpenMV notes
- IDE 4.8.11; cameras on firmware 4.8.1 (`Downloads\openmv_v4.8.1\unzipped\OPENMV4P\firmware.bin`).
  Decline IDE updates, "new resources", firmware updates and registration.
- Each camera runs its own `main.py` at power-on (left script on the left camera, right on the right), with
  `trained.tflite` and `labels.txt` next to it. Don't copy `labels.txt` from a Windows git checkout (CRLF line endings
  make the letters disappear); keep the cameras' own copy.
- Eject the camera's drive in Windows before unplugging.

## Pack
Both batteries + charger, laptop + charger, USB cable (+ spare), victims and kits, spare distance sensor, screwdriver,
hot glue, zip ties, multimeter, tape. A new screen only on **3.3 V** (measure first; never 5 V or the LED strip wire).
