# Robot logs, 2026-10-07

Recorded with `tools/serial_log.py`; read with `python tools/robot_log.py FILE`.

- `steering_attempt1.txt`, `steering.txt`: robot-test, steering test (wall on the right, started 2-3 cm from
  it). Right side sensors read 0-31 mm (under the VL53L0X's ~30 mm minimum); moves started 12-18 deg off;
  at 14:19:04 a reboot printed `can't find gyro` (0x28 missing from the I2C scan) and the heading stayed 0.00.
  Many short pause/resume cycles: the pause switch's pull-down had come loose.
- `switch_test.txt`: pin 22 read every 200 ms with a plain INPUT: HIGH 483 of 485 times in both switch positions.
- `switch_test2.txt`: pin 22 with pull-down / pull-up / plain: pause position 1/1/1, run position 0/1/1
  (open pin floating). Fixed on main with INPUT_PULLDOWN.
- `main_run.txt`: main after the pull-down fix (nothing run).
