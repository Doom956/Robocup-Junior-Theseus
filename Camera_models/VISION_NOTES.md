# Victim vision: what changed (2026-10-17) and how to tune it

Files: `ei_object_detection_PhiOmegaPsi -left.py`, `... -right.py` (identical except the
PER-CAMERA SETTINGS block: name and FOMO confidences), `calibrate_colors.py`,
`measure_letters.py` and `measure_cognitive.py` (camera: print what the checks see).
Plan it follows: `VISION_FIX_PROMPT.md`. **Not tested on a camera yet.**

## What changed
| Problem | Change |
|---|---|
| One frame with one FOMO blob sent a letter | Persistence: same letter/sum in 3 of the last 4 frames, close to the same place (`Track`) |
| Edges/seams read as letters | Shape check at every FOMO hit: one dark blob (Otsu threshold of the box), letter size, aspect 0.6-1.6, density 0.12-0.65, doesn't run across the box |
| Hough threshold: too high misses, too low false hits | Lenient Hough (1600, radius 25-65 px, top 3 circles; measured 2026-10-21) + strict checks: white wall around, every ring one clear colour, sum 0/1/2 |
| Hough's circle a few px off the target, so ring samples landed in the wrong rings (right camera, 2026-10-21) | `refine_circle()` (2026-10-21): 16 rays start on the wall at 1.5 x Hough's radius and walk inward to the target's outer edge; a circle is fitted through the edge points (radius may change up to 35%); a fit more than 20% bigger gets a second pass (Hough was on an inner ring edge); circles that refine onto one already checked are skipped. Also in `calibrate_colors.py` (frames where it fails are skipped). Fake targets: good targets accepted 45 -> 384 of 400 (hard scenes 51 -> 364), Hough on the black/green edge (0.8 R) 297 of 300, calibrated colours within 1 (without: up to 57 off), about 370-840 pixel reads per circle. Known: a dark rim around a small target is counted in the radius, so ring 5 can read black (1 wrong label in 300, 1 fake accepted in 150) |
| `c` on every raw circle stopped the robot | `c` only for a circle that passed every check |
| Ring samples near ring edges (0.175·i R) | Samples at ring middles 0.1/0.3/0.5/0.7/0.9 R, median patches |
| Unclassified ring counted as black | Unknown ring -> no target |
| Threshold boxes (yellow box took in white) | Nearest calibrated colour with a margin test, after normalising to the wall around the target |
| Colours drift | Gain and white balance locked after start-up |
| Overlays drawn on the frame being measured | Drawn last |
| `y_offset` used `ow`, `get_pixel` crash at frame edge, `range(1,36)` | Fixed |
| FOMO sometimes got colour frames with circles drawn on | FOMO always gets the clean grayscale frame |
| Left skipped letters when any circle was seen | Both checked every frame |

## Measuring the ASSUMED values (measure_letters.py + measure_cognitive.py)
Both run the camera scripts' checks but reject nothing, and print one line per detection
to the OpenMV IDE terminal, with every limit it breaks (`fails`), then a `#SUM` summary.
Copy the terminal from `#CFG` to `#END` and paste it to Claude.
- `measure_letters.py`: FOMO hits (score, peak, position, box contrast, Otsu threshold,
  dark blob size, aspect, density, border touches). Needs trained.tflite + labels.txt.
- `measure_cognitive.py` (2026-10-21, replaces `measure_vision.py`): Hough circles (radius,
  strength, the wall around them: white share, L, chroma), and each ring's colour, agreement,
  wall-normalised LAB and colour margin. No model needed. Circles are searched more leniently
  than the camera script does (`mag` / `radius` / `rank` = the camera script wouldn't check
  that circle). Set `RINGS` to the target's colours to see which rings read wrong. Each line
  shows the refined circle that is checked, `fit` (edge points, 0 = Hough's circle kept) and
  Hough's own circle; `dup` = refined onto a circle already checked. In the IDE the refined
  circle is green/red and Hough's is blue.

1. Run `calibrate_colors.py` first and paste `COLOR_CENTRES` into **both** camera scripts
   **and** `measure_cognitive.py` (ring colours depend on them).
2. Set `CAMERA_NAME`, `SCENE`, `NOTE`, `MOTION` at the top, then run.
3. Record, still and driving past, at the closest and furthest the robot really is from a wall:
   - `measure_letters.py`: `H`, `S`, `U` (also rotated); `none` for decoys
   - `measure_cognitive.py`: `H`, `S`, `U` targets (also off-centre); `F` a fake target
   - `none` (both): empty wall, corner, wall seam, black/blue/silver tile, obstacle, decoy
     letters/symbols (set `NOTE` to say which)

KEEP IN SYNC: both copy the camera scripts' settings and checks. If you change a check or a
value in the camera scripts, change it there too. On synthetic targets
`measure_cognitive.py` gave the camera script's verdict and label on 600/600 circles (2026-10-21).

What it can't measure: in a target scene the circle closest to passing in each frame is taken
as the target (check the green circle in the IDE).

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
