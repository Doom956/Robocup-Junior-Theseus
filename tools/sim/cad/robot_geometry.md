# Robot sensor geometry

Sensor positions taken from the Fusion 360 assembly (`V2.f3z` in
[Arsur24/Theseus](https://github.com/Arsur24/Theseus), exported to STEP), for the sensor-level
simulator. The same numbers are in `robot_geometry.json`.

**Frame:** origin on the floor under the middle of the wheelbase (halfway between the front and
back axles). Forward +, left +, height above the floor, all in mm. "Code sensor" is the number
used by `measure()` in `Main/Distance.cpp`.

| Code sensor | Where | CAD part | Forward | Left | Height | Faces |
|---|---|---|---|---|---|---|
| `measure(1)` | front-right | VL53L0X v1:7 | 98.4 | -80.2 | 106.3 | forward |
| `measure(7)` | front-left | VL53L0X v1:6 | 98.4 | 80.2 | 106.3 | forward |
| `measure(2)` | right-front | VL53L0X v1:2 | 88.3 | -90.8 | 106.3 | right |
| `measure(3)` | right-back | VL53L0X v1:3 | -87.7 | -90.8 | 106.3 | right |
| `measure(6)` | left-front | VL53L0X v1:4 | 88.3 | 90.8 | 106.3 | left |
| `measure(5)` | left-back | VL53L0X v1:5 | -87.7 | 90.8 | 106.3 | left |
| `measure(4)` | back | VL53L0X v1:1 | -102.8 | 0.0 | 42.7 | backward |
| Colour sensor | front, underneath | TCS Color Sensor v1:1 | 92.8 | 0.0 | 23.7 | down |

Other measurements:

- front axle 57.8 mm ahead of the centre, back axle 57.8 mm behind (115.6 mm apart)
- outer faces of the distance sensors: 202.5 mm long, 182.9 mm wide
- side sensor pairs are 176 mm apart along the robot

## How it was worked out

- In the CAD the robot faces -Y and its right side is -X. The end with two forward-facing boards
  (and the colour sensor) is the front; the single board at the other end is the back sensor.
- The VL53L0X model is a bare 25.4 x 17.8 mm board with no chip drawn, so each sensor is assumed
  to face outward from the robot. Position = the centre of the board.
- CAD part numbers (v1:1 to v1:7) are Fusion's instance numbers and don't match the code's
  `measure()` numbers; the mapping above is by position.
- The Arduino Mega in the assembly is only a placeholder for the mounting holes.

## To check on the real robot (not done yet)

With the robot centred in a tile, the CAD predicts a side wall reading of about 59 mm and a front
wall reading of about 51 mm. The code's centring constants assume 80 mm (`ROBOT_WIDTH_MM 140`) and
65 mm (`ROBOT_LENGTH_MM 170`). Measure `measure(2)`, `measure(6)` and `measure(1)` with the robot
centred by hand before changing either side.

## Updating

If the robot changes, export the whole assembly from Fusion 360 as STEP (File > Export > STEP)
and run:

```
python tools\sim\cad\extract_sensors.py path\to\V2.step
```

It rewrites `robot_geometry.json` and prints the table. Plain Python, no extra packages.
