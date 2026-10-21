# Theseus - COGNITIVE TARGET measurement (OpenMV). NEW 2026-10-21 (replaces measure_vision.py;
# letters are measured with measure_letters.py).
#
# Point the camera at a printed cognitive target. It runs the same circle search and the same
# target check as the camera scripts, rejects nothing, and PRINTS every measurement to the
# OpenMV IDE serial terminal. Copy the whole terminal output and paste it to Claude: it shows
# whether the limits in the camera scripts are too strict or too loose.
#
# How to use:
#   1. No model needed (this script doesn't use trained.tflite).
#   2. Set SCENE (what is in front of the camera), NOTE, MOTION and, if you know them, RINGS
#      (the printed target's ring colours, centre outwards) below.
#   3. Run from the OpenMV IDE. It stops after FRAMES frames and prints a summary.
#   4. Copy everything from the terminal (from the #CFG line to #END).
#   Good set of runs: each target (H, S, U) still, off-centre in the picture, at the closest
#   and furthest wall distance, and driving past; a fake target (F); then SCENE = "none" at a
#   corner, a wall seam, a black / blue tile, a letter victim, an obstacle and anything round,
#   so we can see what must still be rejected.
#
# Line format (one line per circle):
#   T,frame,label,x,y,r,mag,fit,hough,wall_n,white_n,wall_L,wall_chroma,ring1,ring2,ring3,ring4,ring5,sum,fails
#   x,y,r = the circle that is checked: Hough's circle moved onto the target's outer edge
#   (refine_circle, NEW 2026-10-21); fit = edge points used (0 = Hough's circle kept);
#   hough = Hough's x:y:r;
#   label = what the ring colours add up to (H/S/U; F = fake sum; - = a ring has no colour);
#   wall_n / white_n = patches in the annulus just outside the circle / how many look like
#   wall; wall_L / wall_chroma = their median raw L and chroma (WALL_L_MIN / WALL_CHROMA_MAX);
#   ringK = colour:share:L:a:b:dist:ratio = the colour most of the ring's patches voted for,
#   the share that voted for it (RING_AGREE), the median wall-normalised LAB of its patches,
#   the median distance to the nearest colour centre (MAX_COLOR_DIST) and the median
#   nearest / 2nd-nearest distance (MARGIN_RATIO).
#   fails = every camera-script limit this circle breaks, joined by "+", or "ok".
#   Circles are searched with a lower threshold and a wider radius range than the camera
#   script, so "mag" / "radius" / "rank" = the camera script would not have checked this circle
#   (approximate: a Hough search with other settings can merge circles differently);
#   "dup" = it refined onto a circle the camera script already checked this frame.
#
# KEEP IN SYNC: the limits, refine_circle() and target_metrics() mirror the camera scripts'
# refine_circle() and check_target() (ei_object_detection_PhiOmegaPsi -left/-right.py; target
# settings are the same in both).

import sensor, image, time, math

# ============================== WHAT IS IN FRONT OF THE CAMERA ==========================
CAMERA_NAME = "left"     # left / right
SCENE = "H"              # H / S / U = a target of that status; F = fake target (sum not 0/1/2);
                         # "none" = nothing that should count
NOTE = "still_close"     # free text, no commas: still_close, far, off_centre, corner, seam ...
MOTION = "still"         # still / moving
RINGS = ""               # ring colours centre outwards, e.g. "yellow,blue,red,black,green";
                         # "" = not known. Only for the summary (which rings read wrong).
FRAMES = 150             # frames, then a summary
DRAW = True              # show each circle (green = accepted, red = rejected) and the wall
                         # annulus (grey) in the IDE

# Circle search for RECORDING: lower threshold and wider radius range than the camera script,
# so circles it just misses are printed too.
MEASURE_CIRCLE_THRESHOLD = 1500
MEASURE_R_MIN = 20
MEASURE_R_MAX = 119
MAX_CIRCLES_RECORDED = 5  # the circles the camera script checks + the strongest others

# ============== COPIED FROM THE CAMERA SCRIPT (keep the same values) ====================
EXPOSURE_US = 10000
EXTRA_GAIN_DB = 0
LOCK_GAIN_AND_WB = True
ROW_MIN = 40             # circle centre must be in this band
ROW_MAX = 165
CIRCLE_THRESHOLD = 1600
R_MIN = 25
R_MAX = 65
MAX_CIRCLE_CANDIDATES = 3
RING_FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)
RING_SAMPLES = 12
PATCH_HALF_FRAC = 0.04
RING_AGREE = 0.6
WALL_FRAC = 1.25
WALL_WHITE_FRAC = 0.6
WALL_L_MIN = 45
WALL_CHROMA_MAX = 35
# --- circle refinement (NEW 2026-10-21) ---
# Hough's circle is often a few px off the target (right camera picture, 2026-10-21: one circle
# too big, one too small and off-centre). The rings are only 0.2 R wide, so the ring samples
# landed in the wrong rings. refine_circle() moves each candidate onto the target's outer edge:
# rays start on the wall outside the circle and walk inward to the first pixels that aren't
# wall colour, and a circle is fitted through those edge points.
REFINE = True
REFINE_RAYS = 16         # rays around the circle
REFINE_OUT = 1.35        # rays start at 1.35 x the Hough radius (on the wall) ...
REFINE_IN = 0.65         # ... and stop at 0.65 x: the target's edge must be in between
REFINE_EDGE_DIST = 20    # ASSUMED: colour distance from the wall (as in classify()) that
                         # counts as the target's edge; 2 pixels in a row must reach it
REFINE_MIN_POINTS = 8    # edge points needed for a fit, else the Hough circle is kept
REFINE_MAX_CHANGE = 0.25 # fitted centre within 0.25 R and radius within 25% of Hough, else kept
DUP_FRAC = 0.15          # refined circles this close (x R) to one already checked: skipped
COLOR_CENTRES = {
    "black":  (15,   0,   0),
    "red":    (55,  55,  35),
    "yellow": (90,  -5,  70),
    "green":  (55, -45,  25),
    "blue":   (60,  -5, -45),
    "white":  (100,  0,   0),
}
COLOR_VALUE = {"black": -2, "red": -1, "yellow": 0, "green": 1, "blue": 2}
SUM_TO_LABEL = {2: "H", 1: "S", 0: "U"}
L_WEIGHT = 0.5
MAX_COLOR_DIST = 45
MARGIN_RATIO = 0.75

# ============================== CAMERA SET-UP (same as the camera scripts) ==============
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.skip_frames(time=2000)
if LOCK_GAIN_AND_WB:
    try:
        gain = sensor.get_gain_db()
        sensor.set_auto_gain(False, gain_db=gain + EXTRA_GAIN_DB)
        sensor.set_auto_whitebal(False, rgb_gain_db=sensor.get_rgb_gain_db())
    except Exception as e:
        print("#WARN could not lock gain/white balance:", e)

ANGLES = [(math.cos(2 * math.pi * k / RING_SAMPLES), math.sin(2 * math.pi * k / RING_SAMPLES))
          for k in range(RING_SAMPLES)]

EXPECT = [s.strip() for s in RINGS.split(",")] if RINGS else []
if EXPECT and len(EXPECT) != len(RING_FRACS):
    print("#WARN RINGS needs %d colours, got %d: ignored" % (len(RING_FRACS), len(EXPECT)))
    EXPECT = []

# ============================== TARGET MEASUREMENTS =====================================
def median(vals):
    s = sorted(vals)
    return s[len(s) // 2]

def patch_lab(img, x, y, half):
    if x - half < 0 or y - half < 0 or x + half >= img.width() or y + half >= img.height():
        return None
    s = img.get_statistics(roi=(x - half, y - half, 2 * half + 1, 2 * half + 1))
    return (s.l_median(), s.a_median(), s.b_median())

def is_wall(lab):
    return lab[0] >= WALL_L_MIN and lab[1] * lab[1] + lab[2] * lab[2] <= WALL_CHROMA_MAX * WALL_CHROMA_MAX

def normalise(lab, wall):
    L = lab[0] * 100.0 / max(wall[0], 1)
    return (min(L, 100.0), lab[1] - wall[1], lab[2] - wall[2])

def classify_detail(lab):
    """Same decision as classify() in the camera scripts, plus the two distances."""
    best = None; d1 = 1e9; d2 = 1e9
    for name, c in COLOR_CENTRES.items():
        dL = lab[0] - c[0]; da = lab[1] - c[1]; db = lab[2] - c[2]
        d = math.sqrt(L_WEIGHT * dL * dL + da * da + db * db)
        if d < d1:
            d2 = d1; d1 = d; best = name
        elif d < d2:
            d2 = d
    if d1 > MAX_COLOR_DIST or d1 > MARGIN_RATIO * d2:
        return "unknown", d1, d2
    return best, d1, d2

def target_metrics(img, x, y, r):
    """What check_target() measures, without stopping at the first failure."""
    W = img.width(); H = img.height()
    m = {"rings_in": not (x - 0.95 * r < 0 or y - 0.95 * r < 0 or x + 0.95 * r >= W or y + 0.95 * r >= H),
         "wall_n": 0, "white_n": 0, "wall_L": -1, "wall_chroma": -1.0, "rings": [], "sum": None}
    half = max(1, int(r * PATCH_HALF_FRAC + 0.5))
    wall = []
    for ca, sa in ANGLES:
        lab = patch_lab(img, int(x + WALL_FRAC * r * ca + 0.5), int(y - WALL_FRAC * r * sa + 0.5), half)
        if lab is not None:
            wall.append(lab)
    white = [w for w in wall if is_wall(w)]
    m["wall_n"] = len(wall); m["white_n"] = len(white)
    if wall:
        m["wall_L"] = median([w[0] for w in wall])
        m["wall_chroma"] = median([math.sqrt(w[1] * w[1] + w[2] * w[2]) for w in wall])
    # Reference = the white patches, like check_target(). With none, all annulus patches
    # (only so the rings can still be measured; the camera script stops at "nowall").
    refs = white if white else wall
    ref = None
    if refs:
        ref = (median([w[0] for w in refs]), median([w[1] for w in refs]), median([w[2] for w in refs]))
    total = 0; all_coloured = True
    for k, frac in enumerate(RING_FRACS):
        pts = [(x, y)] if k == 0 else []   # the centre belongs to the inner circle
        for ca, sa in ANGLES:
            pts.append((int(x + frac * r * ca + 0.5), int(y - frac * r * sa + 0.5)))
        votes = {}; labs = []; d1s = []; ratios = []
        if ref is not None:
            for px, py in pts:
                lab = patch_lab(img, px, py, half)
                if lab is None:
                    continue
                nl = normalise(lab, ref)
                name, d1, d2 = classify_detail(nl)
                votes[name] = votes.get(name, 0) + 1
                labs.append(nl); d1s.append(d1); ratios.append(d1 / max(d2, 0.001))
        if labs:
            col = max(votes, key=votes.get)
            ring = {"col": col, "share": votes[col] / len(labs),
                    "L": median([l[0] for l in labs]), "a": median([l[1] for l in labs]),
                    "b": median([l[2] for l in labs]), "dist": median(d1s), "ratio": median(ratios)}
        else:
            ring = {"col": "none", "share": 0.0, "L": -1, "a": 0, "b": 0, "dist": -1, "ratio": -1}
        m["rings"].append(ring)
        if ring["col"] in COLOR_VALUE:
            total += COLOR_VALUE[ring["col"]]
        else:
            all_coloured = False
    # Sum of the vote winners, even if a ring is unclear: what it would read with a looser RING_AGREE.
    m["sum"] = total if all_coloured else None
    return m

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

def refine_circle(img, x, y, r):
    """Move a Hough circle onto the target's outer edge (colour frame).
    Returns (x, y, r, edge points used); 0 points = the Hough circle, unchanged."""
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

def is_duplicate(done, x, y, r):
    """True if a circle like this was already checked this frame (two Hough circles often
    refine onto the same target)."""
    for px, py, pr in done:
        if abs(x - px) + abs(y - py) <= DUP_FRAC * r and abs(r - pr) <= DUP_FRAC * r:
            return True
    return False

def fails(m, y, hough_r, mag, rank, dup):
    """Every camera-script limit this circle breaks (check_target() stops at the first one)."""
    f = []
    if mag < CIRCLE_THRESHOLD: f.append("mag")
    if hough_r < R_MIN or hough_r > R_MAX: f.append("radius")
    if rank >= MAX_CIRCLE_CANDIDATES: f.append("rank")
    if dup: f.append("dup")
    if y < ROW_MIN or y > ROW_MAX: f.append("row")
    if not m["rings_in"]: f.append("edge")
    if m["wall_n"] < RING_SAMPLES // 2:
        f.append("walledge")
    elif m["white_n"] < WALL_WHITE_FRAC * m["wall_n"]:
        f.append("nowall")
    for k, g in enumerate(m["rings"]):
        if g["col"] in ("white", "unknown", "none"):
            f.append("ring%d%s" % (k + 1, "off" if g["col"] == "none" else g["col"]))
        elif g["share"] < RING_AGREE:
            f.append("ring%dunclear" % (k + 1))
    if m["sum"] is not None and m["sum"] not in SUM_TO_LABEL: f.append("fake")
    return f

def ring_text(g):
    return "%s:%.2f:%.0f:%.0f:%.0f:%.0f:%.2f" % (g["col"], g["share"], g["L"], g["a"], g["b"], g["dist"], g["ratio"])

def spread(vals):
    if not vals:
        return "-"
    s = sorted(vals)
    n = len(s)
    return "min %.2f p10 %.2f med %.2f p90 %.2f max %.2f n %d" % (s[0], s[n // 10], s[n // 2], s[n * 9 // 10], s[-1], n)

# ============================== MAIN LOOP ===============================================
print("#CFG camera=%s scene=%s note=%s motion=%s rings=%s frames=%d circle_thr=%d r=%d-%d candidates=%d "
      "row=%d-%d ring_agree=%.2f wall_white=%.2f wall_L>=%d wall_chroma<=%d max_dist=%d margin=%.2f "
      "measure_thr=%d measure_r=%d-%d refine=%s rays=%d edge_dist=%d" %
      (CAMERA_NAME, SCENE, NOTE, MOTION, "/".join(EXPECT) or "-", FRAMES, CIRCLE_THRESHOLD, R_MIN, R_MAX,
       MAX_CIRCLE_CANDIDATES, ROW_MIN, ROW_MAX, RING_AGREE, WALL_WHITE_FRAC, WALL_L_MIN, WALL_CHROMA_MAX,
       MAX_COLOR_DIST, MARGIN_RATIO, MEASURE_CIRCLE_THRESHOLD, MEASURE_R_MIN, MEASURE_R_MAX,
       REFINE, REFINE_RAYS, REFINE_EDGE_DIST))
print("#CENTRES " + " ".join("%s=%d/%d/%d" % (k, v[0], v[1], v[2]) for k, v in COLOR_CENTRES.items()))
print("#COLS T,frame,label,x,y,r,mag,fit,hough (x:y:r),wall_n,white_n,wall_L,wall_chroma,"
      "ring1,ring2,ring3,ring4,ring5 (colour:share:L:a:b:dist:ratio),sum,fails")

clock = time.clock()
frames_with_circle = 0       # frames with any circle
frames_checked = 0           # frames where the camera script would check at least one circle
frames_pass = 0              # frames where it accepts a circle (any label)
frames_right = 0             # frames where it accepts a circle with the SCENE's label
accepted_labels = {}         # label -> frames it was accepted with
fail_count = {}
vals = {"r": [], "mag": [], "y": [], "white_frac": [], "wall_L": [], "wall_chroma": [],
        "min_share": [], "max_dist": [], "max_ratio": [],
        "fit": [], "shift": [], "r_change": []}   # refinement: edge points, centre move px, r / Hough r
ring_read = [{} for k in RING_FRACS]                   # per ring: colour read -> count
ring_lab = [([], [], []) for k in RING_FRACS]          # per ring: normalised L, a, b
for frame in range(1, FRAMES + 1):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    found = img.find_circles(threshold=MEASURE_CIRCLE_THRESHOLD, r_min=MEASURE_R_MIN, r_max=MEASURE_R_MAX)
    found = sorted(found, key=lambda c: c.magnitude(), reverse=True)
    # rank = place among the circles the camera script would find (-1 = it wouldn't find it)
    cands = []
    n_found = 0
    for c in found:
        if c.magnitude() >= CIRCLE_THRESHOLD and R_MIN <= c.r() <= R_MAX:
            cands.append((c, n_found)); n_found += 1
        else:
            cands.append((c, -1))
    # always record the circles the camera script checks, then the strongest others
    recorded = [t for t in cands if 0 <= t[1] < MAX_CIRCLE_CANDIDATES]
    others = [t for t in cands if not (0 <= t[1] < MAX_CIRCLE_CANDIDATES)]
    recorded += others[:max(0, MAX_CIRCLES_RECORDED - len(recorded))]

    hits = []
    checked = []       # refined circles the camera script checks, in its order (for "dup")
    for c, rank in recorded:
        hx, hy, hr, mag = c.x(), c.y(), c.r(), c.magnitude()
        x, y, r, n_fit = refine_circle(img, hx, hy, hr) if REFINE else (hx, hy, hr, 0)
        dup = False
        if 0 <= rank < MAX_CIRCLE_CANDIDATES:
            dup = is_duplicate(checked, x, y, r)
            if not dup:
                checked.append((x, y, r))
        m = target_metrics(img, x, y, r)
        f = fails(m, y, hr, mag, rank, dup)
        label = "-" if m["sum"] is None else SUM_TO_LABEL.get(m["sum"], "F")
        print("T,%d,%s,%d,%d,%d,%d,%d,%d:%d:%d,%d,%d,%d,%.0f,%s,%s,%s" %
              (frame, label, x, y, r, mag, n_fit, hx, hy, hr, m["wall_n"], m["white_n"], m["wall_L"],
               m["wall_chroma"], ",".join(ring_text(g) for g in m["rings"]),
               "-" if m["sum"] is None else str(m["sum"]), "+".join(f) or "ok"))
        hits.append((x, y, r, mag, m, f, label, n_fit, hx, hy, hr))

    frames_with_circle += 1 if hits else 0
    frames_checked += 1 if n_found else 0
    ok_labels = [h[6] for h in hits if not h[5]]
    if ok_labels:
        frames_pass += 1
        for lb in set(ok_labels):
            accepted_labels[lb] = accepted_labels.get(lb, 0) + 1
    frames_right += 1 if SCENE in ok_labels else 0

    # Statistics: "none" = every circle (each one could be a false victim); a target scene =
    # the circle closest to passing in each frame (fewest failures, then strongest), taken as
    # the target. Check in the IDE that it is the target.
    if SCENE == "none":
        use = hits
    else:
        use = [min(hits, key=lambda h: (len(h[5]), -h[3]))] if hits else []
    for x, y, r, mag, m, f, label, n_fit, hx, hy, hr in use:
        vals["fit"].append(n_fit)
        if n_fit:
            vals["shift"].append(math.sqrt((x - hx) ** 2 + (y - hy) ** 2)); vals["r_change"].append(r / hr)
        for k in f:
            fail_count[k] = fail_count.get(k, 0) + 1
        vals["r"].append(r); vals["mag"].append(mag); vals["y"].append(y)
        vals["white_frac"].append(m["white_n"] / m["wall_n"] if m["wall_n"] else 0.0)
        if m["wall_n"]:
            vals["wall_L"].append(m["wall_L"]); vals["wall_chroma"].append(m["wall_chroma"])
        vals["min_share"].append(min(g["share"] for g in m["rings"]))
        vals["max_dist"].append(max(g["dist"] for g in m["rings"]))
        vals["max_ratio"].append(max(g["ratio"] for g in m["rings"]))
        if SCENE != "none":
            for k, g in enumerate(m["rings"]):
                ring_read[k][g["col"]] = ring_read[k].get(g["col"], 0) + 1
                if g["col"] != "none":
                    ring_lab[k][0].append(g["L"]); ring_lab[k][1].append(g["a"]); ring_lab[k][2].append(g["b"])

    if DRAW:   # last, so it never touches measured pixels
        for x, y, r, mag, m, f, label, n_fit, hx, hy, hr in hits:
            if n_fit:   # Hough's circle before refinement, in blue
                img.draw_circle(hx, hy, hr, color=(0, 128, 255))
            img.draw_circle(x, y, int(WALL_FRAC * r), color=(128, 128, 128))
            img.draw_circle(x, y, r, color=(0, 255, 0) if not f else (255, 0, 0), thickness=2 if not f else 1)

print("#SUM scene=%s note=%s motion=%s frames=%d fps=%.1f" % (SCENE, NOTE, MOTION, FRAMES, clock.fps()))
print("#SUM frames with any circle: %d, with a circle the camera script checks: %d, accepted: %d, "
      "accepted as %s: %d" % (frames_with_circle, frames_checked, frames_pass, SCENE, frames_right))
print("#SUM accepted labels (frames): %s" % str(accepted_labels))
print("#SUM limit failures (%s): %s" % ("all circles" if SCENE == "none" else "best circle per frame", str(fail_count)))
for k in ("r", "mag", "y", "white_frac", "wall_L", "wall_chroma", "min_share", "max_dist", "max_ratio",
          "fit", "shift", "r_change"):
    print("#SUM %-11s %s" % (k, spread(vals[k])))
if SCENE != "none":
    for k in range(len(RING_FRACS)):
        print("#RING %d expect %s read %s L %s | a %s | b %s" %
              (k + 1, EXPECT[k] if EXPECT else "?", str(ring_read[k]),
               spread(ring_lab[k][0]), spread(ring_lab[k][1]), spread(ring_lab[k][2])))
print("#END")
