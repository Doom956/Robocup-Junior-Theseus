# Robot logs, 2026-10-06

Bench mode (`BENCH_MODE 1`, `Main/bench.cpp`) on the real robot, battery 11.76 V, recorded with
`tools/serial_log.py` (time of the printout in front of each line). Read them with `python tools/robot_log.py FILE`.

- `bench_run1_main_NDOF.txt`: main, gyro in NDOF. Test 4 here used the old pass rule (any one encoder
  over 10 counts) and stopped on encoder noise at PWM 11 with the robot still: ignore that line.
- `bench_run2_main_NDOF.txt`: main, NDOF, fixed test 4. Test 6: heading jumped 12.5 deg standing still.
- `bench_run3_robot-test_IMUPLUS.txt`: robot-test, gyro in IMUPLUS. Test 6: 0.00 deg.

Test 9 ("silver") was a grey tile, not a field tile: ignore it. Test 5 was skipped in runs 1-3.
The robot was centred by hand for test 1, so single sensors are +-5-10 mm; sums of opposite sensors aren't.
