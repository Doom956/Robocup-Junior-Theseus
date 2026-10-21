# Theseus - cognitive target colour calibration (OpenMV). NEW 2026-10-17.
#
# Run this from the OpenMV IDE on ONE camera, on the robot, at the venue lighting:
#   1. Tape a printed cognitive target to a wall at victim height, robot beside it
#      as it would be in a run (camera at its normal distance).
#   2. Set TARGET_COLOURS below to that target's rings, centre outwards.
#      The rules' example target "yellow, blue, red, black, green" has all five colours,
#      so one print of it is enough.
#   3. Run. Hold still. The strongest circle is moved onto the target's outer edge
#      (refine_circle, same as the camera scripts, NEW 2026-10-21): green = the circle used,
#      blue = Hough's circle, red = no edge found, frame skipped. Check the green circle sits on
#      the target's OUTER edge: the rings are sampled at fixed fractions of its radius.
#      After FRAMES good frames it prints:
#        - COLOR_CENTRES to paste into both camera scripts (and measure_cognitive.py)
#        - the wall's raw LAB, to set WALL_L_MIN / WALL_CHROMA_MAX
#        - the target radius and Hough's radius in pixels, to set R_MIN / R_MAX
#      Move the robot a little closer/further (as far as it really varies in a run)
#      and run again; use radius min-15% / max+15%.
#
# The sampling and wall normalisation are copied from the camera scripts: keep them the same.

import sensor, image, time, math

TARGET_COLOURS = ("yellow", "blue", "red", "black", "green")  # centre outwards
FRAMES = 40
EXPOSURE_US = 10000      # must match the camera scripts
EXTRA_GAIN_DB = 0        # must match the camera scripts
CIRCLE_THRESHOLD = 1600  # same as the camera scripts (2026-10-21: the target's circle measured
R_MIN = 25               # 1657-2724 strong, radius 27-52 px; at 2500 it was rarely found)
R_MAX = 65               # The strongest circle is refined onto the target's edge before sampling.
RING_FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)
RING_SAMPLES = 12
PATCH_HALF_FRAC = 0.04
WALL_FRAC = 1.25
# Loose wall test, only to find the wall annulus here: looser than the camera scripts' (45 / 35).
WALL_L_MIN = 35
WALL_CHROMA_MAX = 40     # was 30; the wall around the target measured chroma 17-33 (2026-10-21)
L_WEIGHT = 0.5           # same as the camera scripts (refine_circle's colour distance)

# --- circle refinement (NEW 2026-10-21) ---
# Hough's circle is often a few px off the target (right camera picture, 2026-10-21: one circle
# too big, one too small and off-centre). The rings are only 0.2 R wide, so the ring samples
# landed in the wrong rings. refine_circle() moves each candidate onto the target's outer edge:
# rays start on the wall outside the circle and walk inward to the first pixels that aren't
# wall colour, and a circle is fitted through those edge points.
REFINE = True
REFINE_RAYS = 16         # rays around the circle
REFINE_OUT = 1.5         # rays start at 1.5 x the Hough radius, on the wall even when Hough
                         # locked onto the black/green ring edge at 0.8 R (was 1.35) ...
REFINE_IN = 0.65         # ... and stop at 0.65 x: the target's edge must be in between
REFINE_EDGE_DIST = 20    # ASSUMED: colour distance from the wall (as in classify()) that
                         # counts as the target's edge; 2 pixels in a row must reach it
REFINE_MIN_POINTS = 8    # edge points needed for a fit, else the Hough circle is kept
REFINE_MAX_CHANGE = 0.35 # fitted centre within 0.35 R and radius within 35% of Hough, else
                         # kept (a lock at 0.8 R needs +25%; was 0.25)
REFINE_GROWN = 1.2       # a fit more than 20% bigger than Hough's circle gets a second pass
                         # (an inner ring edge gives 1.25x or more; 1.1 also re-ran small
                         # fits and pushed them onto a dark rim around the target)
DUP_FRAC = 0.15          # refined circles this close (x R) to one already checked: skipped

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

def is_wall(lab):
    return lab[0] >= WALL_L_MIN and lab[1] * lab[1] + lab[2] * lab[2] <= WALL_CHROMA_MAX * WALL_CHROMA_MAX

# ============================== CIRCLE REFINEMENT (NEW 2026-10-21) ======================
RAY_ANGLES = [(math.cos(2 * math.pi * k / REFINE_RAYS), math.sin(2 * math.pi * k / REFINE_RAYS))
              for k in range(REFINE_RAYS)]

def det3(a, b, c, d, e, f, g, h, i):
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

def fit_circle(pts):
    """Least-squares circle x^2 + y^2 + D x + E y + F = 0 through the points.
    Returns (cx, cy, r) or None."""
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = sxz = syz = 0.0
    for px, py in pts:
        z = px * px + py * py
        sx += px; sy += py; sxx += px * px; syy += py * py; sxy += px * py
        sz += z; sxz += px * z; syz += py * z
    det = det3(sxx, sxy, sx, sxy, syy, sy, sx, sy, n)
    if abs(det) < 1e-6:
        return None
    D = det3(-sxz, sxy, sx, -syz, syy, sy, -sz, sy, n) / det
    E = det3(sxx, -sxz, sx, sxy, -syz, sy, sx, -sz, n) / det
    F = det3(sxx, sxy, -sxz, sxy, syy, -syz, sx, sy, -sz) / det
    cx = -D / 2; cy = -E / 2
    rr = cx * cx + cy * cy - F
    if rr <= 0:
        return None
    return cx, cy, math.sqrt(rr)

def refine_pass(img, x, y, r):
    """One pass: rays from REFINE_OUT x r walk inward to the target's outer edge and a circle
    is fitted through the edge points. Returns (x, y, r, edge points used); 0 = no fit."""
    W = img.width(); H = img.height()
    r_out = int(REFINE_OUT * r + 0.5)
    r_in = int(REFINE_IN * r + 0.5)
    lim = REFINE_EDGE_DIST * REFINE_EDGE_DIST
    pts = []
    misses = 0
    for ca, sa in RAY_ANGLES:
        if misses > REFINE_RAYS - REFINE_MIN_POINTS:
            return x, y, r, 0          # can't reach REFINE_MIN_POINTS any more
        # wall reference = the ray's 3 outermost pixels, which must look like wall
        ref = [0.0, 0.0, 0.0]
        inside = True
        for rho in (r_out, r_out - 1, r_out - 2):
            px = int(x + rho * ca + 0.5); py = int(y - rho * sa + 0.5)
            if px < 0 or py < 0 or px >= W or py >= H:
                inside = False
                break
            lab = image.rgb_to_lab(img.get_pixel(px, py))
            ref[0] += lab[0] / 3.0; ref[1] += lab[1] / 3.0; ref[2] += lab[2] / 3.0
        if not inside or not is_wall(ref):
            misses += 1
            continue
        # walk inward to the first of 2 pixels in a row that aren't wall colour
        # (the centre is in the frame, so every pixel between it and the start is too)
        edge = None; run = 0
        for rho in range(r_out - 3, r_in - 1, -1):
            lab = image.rgb_to_lab(img.get_pixel(int(x + rho * ca + 0.5), int(y - rho * sa + 0.5)))
            dL = lab[0] - ref[0]; da = lab[1] - ref[1]; db = lab[2] - ref[2]
            if L_WEIGHT * dL * dL + da * da + db * db > lim:
                run += 1
                if run == 2:
                    edge = rho + 1
                    break
            else:
                run = 0
        if edge is None:
            misses += 1
            continue
        pts.append((edge * ca, -edge * sa))   # relative to the Hough centre
    if len(pts) < REFINE_MIN_POINTS:
        return x, y, r, 0
    fit = fit_circle(pts)
    if fit is None:
        return x, y, r, 0
    # drop edge points that are off the fitted circle (glare, something touching it), refit
    tol = max(2.0, 0.06 * fit[2])
    good = [p for p in pts if abs(math.sqrt((p[0] - fit[0]) ** 2 + (p[1] - fit[1]) ** 2) - fit[2]) <= tol]
    if len(good) < REFINE_MIN_POINTS:
        return x, y, r, 0
    if len(good) < len(pts):
        fit = fit_circle(good)
        if fit is None:
            return x, y, r, 0
    cx, cy, rf = fit
    if math.sqrt(cx * cx + cy * cy) > REFINE_MAX_CHANGE * r or abs(rf - r) > REFINE_MAX_CHANGE * r:
        return x, y, r, 0
    return int(x + cx + 0.5), int(y + cy + 0.5), int(rf + 0.5), len(good)

def refine_circle(img, x, y, r):
    """Move a Hough circle onto the target's outer edge (colour frame).
    Returns (x, y, r, edge points used); 0 points = the Hough circle, unchanged.
    A fit much bigger than Hough's circle means Hough was on an inner ring edge, and the rays
    may have started inside a pale outer ring that passes the wall test (fake targets: blue on
    a tinted wall gave a wrong label). A second pass from the bigger circle starts on the wall."""
    x1, y1, r1, n1 = refine_pass(img, x, y, r)
    if n1 and r1 > REFINE_GROWN * r:
        x2, y2, r2, n2 = refine_pass(img, x1, y1, r1)
        if n2:
            return x2, y2, r2, n2
    return x1, y1, r1, n1

def is_duplicate(done, x, y, r):
    """True if a circle like this was already checked this frame (two Hough circles often
    refine onto the same target)."""
    for px, py, pr in done:
        if abs(x - px) + abs(y - py) <= DUP_FRAC * r and abs(r - pr) <= DUP_FRAC * r:
            return True
    return False

samples = {}      # colour -> list of normalised LAB (one per ring per frame)
wall_raw = []     # raw LAB of the wall annulus
radii = []        # target radius (refined)
hough_radii = []  # Hough's radius for the same frames (what R_MIN / R_MAX must cover)
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
    x, y, r, n_fit = refine_circle(img, c.x(), c.y(), c.r())
    if n_fit == 0:
        img.draw_circle(c.x(), c.y(), c.r(), color=(255, 0, 0))
        print("no target edge found around the circle (x%d y%d r%d) - frame skipped" % (c.x(), c.y(), c.r()))
        continue
    half = max(1, int(r * PATCH_HALF_FRAC + 0.5))

    wall = []
    for ca, sa in ANGLES:
        lab = patch_lab(img, int(x + WALL_FRAC * r * ca + 0.5), int(y - WALL_FRAC * r * sa + 0.5), half)
        if lab is not None and is_wall(lab):
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
    hough_radii.append(c.r())
    good += 1
    img.draw_circle(c.x(), c.y(), c.r(), color=(0, 128, 255))
    img.draw_circle(x, y, r, color=(0, 255, 0))
    print("frame %d/%d  r=%d (hough %d, %d edge pts)  wall L%d a%d b%d" %
          (good, FRAMES, r, c.r(), n_fit, ref[0], ref[1], ref[2]))

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
print("target radius px: min %d, median %d, max %d" % (min(radii), median(radii), max(radii)))
print("Hough radius px:  min %d, median %d, max %d  (R_MIN / R_MAX must cover these)" %
      (min(hough_radii), median(hough_radii), max(hough_radii)))
print("Spread per colour (check they don't overlap):")
for name, s in samples.items():
    print("  %s L %d..%d a %d..%d b %d..%d" % (name,
          min(v[0] for v in s), max(v[0] for v in s),
          min(v[1] for v in s), max(v[1] for v in s),
          min(v[2] for v in s), max(v[2] for v in s)))
