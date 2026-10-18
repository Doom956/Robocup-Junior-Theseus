# Theseus - cognitive target colour calibration (OpenMV). NEW 2026-10-17.
#
# Run this from the OpenMV IDE on ONE camera, on the robot, at the venue lighting:
#   1. Tape a printed cognitive target to a wall at victim height, robot beside it
#      as it would be in a run (camera at its normal distance).
#   2. Set TARGET_COLOURS below to that target's rings, centre outwards.
#      The rules' example target "yellow, blue, red, black, green" has all five colours,
#      so one print of it is enough.
#   3. Run. Hold still. After FRAMES good frames it prints:
#        - COLOR_CENTRES to paste into both camera scripts
#        - the wall's raw LAB, to set WALL_L_MIN / WALL_CHROMA_MAX
#        - the target radius in pixels, to set R_MIN / R_MAX
#      Move the robot a little closer/further (as far as it really varies in a run)
#      and run again; use radius min-15% / max+15%.
#
# The sampling and wall normalisation are copied from the camera scripts: keep them the same.

import sensor, image, time, math

TARGET_COLOURS = ("yellow", "blue", "red", "black", "green")  # centre outwards
FRAMES = 40
EXPOSURE_US = 10000      # must match the camera scripts
EXTRA_GAIN_DB = 0        # must match the camera scripts
CIRCLE_THRESHOLD = 2500
R_MIN = 30               # wide on purpose: we're measuring the radius here
R_MAX = 119
RING_FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)
RING_SAMPLES = 12
PATCH_HALF_FRAC = 0.04
WALL_FRAC = 1.25
# Loose wall test, only to find the wall annulus here.
WALL_L_MIN = 35
WALL_CHROMA_MAX = 30

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.skip_frames(time=2000)
try:
    gain = sensor.get_gain_db()
    sensor.set_auto_gain(False, gain_db=gain + EXTRA_GAIN_DB)
    sensor.set_auto_whitebal(False, rgb_gain_db=sensor.get_rgb_gain_db())
except Exception as e:
    print("could not lock gain/white balance:", e)

ANGLES = [(math.cos(2 * math.pi * k / RING_SAMPLES), math.sin(2 * math.pi * k / RING_SAMPLES))
          for k in range(RING_SAMPLES)]

def median(vals):
    s = sorted(vals)
    return s[len(s) // 2]

def patch_lab(img, x, y, half):
    if x - half < 0 or y - half < 0 or x + half >= img.width() or y + half >= img.height():
        return None
    s = img.get_statistics(roi=(x - half, y - half, 2 * half + 1, 2 * half + 1))
    return (s.l_median(), s.a_median(), s.b_median())

def normalise(lab, wall):
    L = lab[0] * 100.0 / max(wall[0], 1)
    return (min(L, 100.0), lab[1] - wall[1], lab[2] - wall[2])

samples = {}      # colour -> list of normalised LAB (one per ring per frame)
wall_raw = []     # raw LAB of the wall annulus
radii = []
good = 0
clock = time.clock()
while good < FRAMES:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    circles = img.find_circles(threshold=CIRCLE_THRESHOLD, r_min=R_MIN, r_max=R_MAX)
    if not circles:
        print("no circle - point the camera at the target")
        continue
    c = max(circles, key=lambda cc: cc.magnitude())
    x, y, r = c.x(), c.y(), c.r()
    half = max(1, int(r * PATCH_HALF_FRAC + 0.5))

    wall = []
    for ca, sa in ANGLES:
        lab = patch_lab(img, int(x + WALL_FRAC * r * ca + 0.5), int(y - WALL_FRAC * r * sa + 0.5), half)
        if lab is not None and lab[0] >= WALL_L_MIN and lab[1] ** 2 + lab[2] ** 2 <= WALL_CHROMA_MAX ** 2:
            wall.append(lab)
    if len(wall) < RING_SAMPLES // 2:
        print("not enough white wall around the circle (%d) - centre the target" % len(wall))
        continue
    ref = (median([w[0] for w in wall]), median([w[1] for w in wall]), median([w[2] for w in wall]))

    for k, frac in enumerate(RING_FRACS):
        labs = []
        pts = [(x, y)] if k == 0 else []
        for ca, sa in ANGLES:
            pts.append((int(x + frac * r * ca + 0.5), int(y - frac * r * sa + 0.5)))
        for px, py in pts:
            lab = patch_lab(img, px, py, half)
            if lab is not None:
                labs.append(normalise(lab, ref))
        if labs:
            m = (median([l[0] for l in labs]), median([l[1] for l in labs]), median([l[2] for l in labs]))
            samples.setdefault(TARGET_COLOURS[k], []).append(m)
    wall_raw.extend(wall)
    radii.append(r)
    good += 1
    img.draw_circle(x, y, r, color=(0, 255, 0))
    print("frame %d/%d  r=%d  wall L%d a%d b%d" % (good, FRAMES, r, ref[0], ref[1], ref[2]))

print("\n===== paste into both camera scripts =====")
print("COLOR_CENTRES = {")
for name in ("black", "red", "yellow", "green", "blue"):
    if name in samples:
        s = samples[name]
        print('    "%s": (%d, %d, %d),' % (name, median([v[0] for v in s]),
                                           median([v[1] for v in s]), median([v[2] for v in s])))
    else:
        print('    # "%s": not on this target - calibrate with another one' % name)
print('    "white":  (100,  0,   0),')
print("}")

Ls = sorted([w[0] for w in wall_raw])
chroma = sorted([math.sqrt(w[1] ** 2 + w[2] ** 2) for w in wall_raw])
print("\nwall raw L: 5%% %d, median %d   chroma: median %d, 95%% %d" %
      (Ls[len(Ls) // 20], median(Ls), median(chroma), chroma[len(chroma) * 19 // 20]))
print("-> WALL_L_MIN about %d, WALL_CHROMA_MAX about %d" %
      (Ls[len(Ls) // 20] - 10, chroma[len(chroma) * 19 // 20] + 8))
print("radius px: min %d, median %d, max %d" % (min(radii), median(radii), max(radii)))
print("Spread per colour (check they don't overlap):")
for name, s in samples.items():
    print("  %s L %d..%d a %d..%d b %d..%d" % (name,
          min(v[0] for v in s), max(v[0] for v in s),
          min(v[1] for v in s), max(v[1] for v in s),
          min(v[2] for v in s), max(v[2] for v in s)))
