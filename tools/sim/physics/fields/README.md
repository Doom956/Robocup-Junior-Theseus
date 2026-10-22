# Real fields for the physics simulator

A field drawn as text runs like any other field type: give the file name as the scenario.

```
python tools/sim/physics/run_physics.py --scenario fields/event.txt --runs 100
python tools/sim/physics/run_physics.py --scenario fields/event.txt --seed 3 --view
python tools/sim/physics/ab.py origin/main robot-test --types fields/event.txt --runs 200
```

The file is looked for as given, then in this folder. The walls, colours and ramp stay the same on every seed. The seed
changes how the robot is placed, the sensor and motor noise, the size and place of any `o` obstacle, and the ramp angle
if the file doesn't give one.

## Format

```
# lines starting with # are notes
tile_mm 320        # optional: tile pitch (RCJ 300; the team's practice maze measured 320)
ramp_deg 20        # optional: ramp angle (otherwise rampMinDeg..rampMaxDeg, 15-25)
+---+---+---+---+---+
|S ^ C  |X  | 1   1 |
+   +---+---+   +---+
|B       R   C1   1 |
+---+---+---+---+---+
```

The top of the drawing is north. Each tile is 3 characters between wall marks: `-` on a `+---+` line is a wall to the
north or south, `|` is a wall to the west or east, and a space means open. A wall drawn on one side counts on both.

Each tile's 3 characters are: type, level, extra.

- **Type:** a space or `.` is a plain tile, `S` the start, `B` blue, `K` black, `C` a checkpoint, `R` a ramp, `X` not
  part of the field.
- **Level:** `1` puts the tile on the upper level; a space means level 0.
- **Extra:** after `S`, the start facing: `^` north, `>` east, `v` south, `<` west. On other tiles, `o` is an obstacle in
  the open (size from `obstacleRMinMm..obstacleRMaxMm`), `=` a speed bump running east-west, `I` one running north-south.

A ramp is one tile long and can run any way. It needs a level-0 tile at one end and a level-1 tile at the other, and it
rises toward the level-1 tile. A field can have more than one ramp (`event.txt` has two).

`--set greyCheckpoints=1` makes checkpoints read like the team's grey tile (bench test 9, 2026-10-06: 0.45 of white). `read_color()`
takes that for a plain tile, so the robot records no checkpoint, but the referee still restarts it on the last one it
visited.
