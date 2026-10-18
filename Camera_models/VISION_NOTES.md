# Victim vision: what changed (2026-10-17) and how to tune it

Files: `ei_object_detection_PhiOmegaPsi -left.py`, `... -right.py` (identical except the
PER-CAMERA SETTINGS block: name and FOMO confidences), `calibrate_colors.py`.
Plan it follows: `VISION_FIX_PROMPT.md`. **Not tested on a camera yet.**

## What changed
| Problem | Change |
|---|---|
| One frame with one FOMO blob sent a letter | Persistence: same letter/sum in 3 of the last 4 frames, close to the same place (`Track`) |
| Edges/seams read as letters | Shape check at every FOMO hit: one dark blob (Otsu threshold of the box), letter size, aspect 0.6-1.6, density 0.12-0.65, doesn't run across the box |
| Hough threshold: too high misses, too low false hits | Lenient Hough (2500, top 3 circles) + strict checks: white wall around, every ring one clear colour, sum 0/1/2 |
| `c` on every raw circle stopped the robot | `c` only for a circle that passed every check |
| Ring samples near ring edges (0.175·i R) | Samples at ring middles 0.1/0.3/0.5/0.7/0.9 R, median patches |
| Unclassified ring counted as black | Unknown ring -> no target |
| Threshold boxes (yellow box took in white) | Nearest calibrated colour with a margin test, after normalising to the wall around the target |
| Colours drift | Gain and white balance locked after start-up |
| Overlays drawn on the frame being measured | Drawn last |
| `y_offset` used `ow`, `get_pixel` crash at frame edge, `range(1,36)` | Fixed |
| FOMO sometimes got colour frames with circles drawn on | FOMO always gets the clean grayscale frame |
| Left skipped letters when any circle was seen | Both checked every frame |

## Values to measure on the robot (marked ASSUMED in the scripts)
Leave `DEBUG_PRINT = True` and watch the OpenMV IDE terminal. Every candidate prints a line:
`left f123 letter H x120 y96 size74 ACCEPT: ok score 0.81`.

| Setting | How |
|---|---|
| `ROW_MIN` / `ROW_MAX` | Robot beside a wall with letters and targets at the rule height (7 cm). Note `y` of accepted hits, add ~25 px each side |
| `LETTER_PX_MIN/MAX` | `size` of accepted letters stopped and driving past; use min-25% / max+25% |
| `R_MIN` / `R_MAX` | `calibrate_colors.py` prints the radius; min-15% / max+15% |
| `POS_TOL_PX` | Drive past a victim at normal speed; how far `x` moves between frames. Make it ~1.5x that |
| `WALL_L_MIN`, `WALL_CHROMA_MAX`, `COLOR_CENTRES` | `calibrate_colors.py` |
| `CIRCLE_THRESHOLD` | Lower until targets are found while driving; the checks reject the rest. Watch the fps line |

## Recalibrate at a venue (5 minutes per camera)
1. Open `calibrate_colors.py` in the OpenMV IDE, connect one camera.
2. Robot beside a wall with the example target (yellow, blue, red, black, green), normal distance.
3. Run, hold still, copy the printed `COLOR_CENTRES` into **both** camera scripts (and the
   wall / radius values if they changed).
4. Repeat for the other camera if its numbers differ a lot (then keep them in the per-camera block).

## Leniency vs accuracy
- `MARGIN_RATIO` (0.75): higher = a patch counts as a colour even when it's between two
  colours (more lenient); lower = stricter.
- `RING_AGREE` (0.6): share of a ring's patches that must agree.
- `CONFIRM_HITS` / `TRACK_WINDOW` (3 of 4): more = fewer false victims, needs more frames.
- `HINT_ON_LETTER` (off): turn on if letters are missed while driving; the robot then stops
  on a shape-checked letter and confirms it while still.

## Building a test set
Set `DEBUG_SAVE = True` (SD card in the camera). Each power-up writes `dbg_left_000/` etc.:
JPEGs of frames with candidates (`_c` colour for circles, `_l` grayscale for letters) and
`log.csv` (frame, kind, label, x, y, size, accepted, reason). Record driving past every
letter and target type, decoys, empty walls, corners, seams, black/blue tiles. Label which
frames really have a victim; count misses and false hits before/after each change.
Turn `DEBUG_SAVE` off for runs (saving is slow).

## Firmware ideas (NOT applied; need your yes)
1. `cameraTask()` stops for any byte. That now works as intended (`c` = verified candidate),
   but `detectCam1/2()` still waits up to 4 s when nothing confirms. With persistence on the
   camera, ~1.5 s would do.
2. `detectCam1/2()` votes over 5 bytes. The camera already needs 3 matching frames, so
   3 bytes would be enough and stop sooner.
