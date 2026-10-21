# Theseus victim camera - RIGHT side.
# Edge Impulse FOMO letters (Phi/Psi/Omega -> H/S/U) + cognitive targets (5 coloured rings).
#
# Based on the Edge Impulse - OpenMV FOMO Object Detection Example
# (MIT license, Copyright (c) 2013-2024 OpenMV LLC, https://github.com/openmv/openmv/blob/master/LICENSE).
#
# CHANGED 2026-10-17 (see Camera_models/VISION_NOTES.md for the why and how to tune):
#   - Left and right scripts are now identical except the PER-CAMERA SETTINGS block.
#   - Nothing is reported from a single frame any more: a letter or target has to be
#     seen in CONFIRM_HITS of the last TRACK_WINDOW frames, in about the same place,
#     with the same letter / ring sum (Tracker below).
#   - Letters: every FOMO hit is shape-checked (one dark blob of letter size and shape).
#     Wall edges, seams and black tiles fail this check.
#   - Targets: Hough finds candidates leniently, then each candidate must have white wall
#     around it, 5 rings that each have a clear colour, and a sum of 0, 1 or 2.
#   - CHANGED 2026-10-21: each Hough circle is first moved onto the target's outer edge
#     (refine_circle); circles that end up on the same target are checked once.
#   - Colours: rings are sampled at their middles (0.1/0.3/0.5/0.7/0.9 R) with small median
#     patches, normalised to the wall around the target, and matched to the nearest
#     calibrated colour (calibrate_colors.py). Unclear ring -> no target (was: black).
#   - Gain and white balance are locked after start-up.
#   - "c" is only sent for a circle that passed every check (asks the robot to stop and
#     look), not for every raw Hough circle.
#   - Debug overlays are drawn after sampling and inference, not on the frame being measured.
#   - DEBUG_SAVE records frames + a CSV log to the SD card to build a test set.
#
# Values marked ASSUMED are not measured yet. Measure them on the robot (VISION_NOTES.md).
# UART bytes are unchanged: H / S / U for a victim, c for "stop, something is there".

import sensor, image, time, ml, math, uos, gc
import pyb
from pyb import UART

# ============ PER-CAMERA SETTINGS (the only block that differs left / right) ============
CAMERA_NAME = "right"
# FOMO confidence per class. Kept from the old right script (0.9, but 0.7 for S).
CLASS_CONFIDENCE = {"H": 0.9, "S": 0.7, "U": 0.9}
DEFAULT_CONFIDENCE = 0.9

# ============================== SHARED SETTINGS =========================================
# --- camera ---
EXPOSURE_US = 10000      # kept from the old script. Shorter = less blur while moving;
                         # try 5000 with EXTRA_GAIN_DB ~6 and re-run calibrate_colors.py.
EXTRA_GAIN_DB = 0        # added to the gain the camera settled on at start-up.
LOCK_GAIN_AND_WB = True  # NEW: stops colours drifting when a dark tile / bright wall
                         # comes into view. Locks at whatever it sees at power-up,
                         # so start it facing a normal lit wall.

# --- where victims can be in the image ---
# Victims are ~7 cm above the floor, so they appear in a fixed band of rows.
# MEASURED 2026-10-19 (one camera screenshot + measure_letters.py): letters centred at
# y 80-120; from y ~195 down the frame is black (robot / floor in view, always there).
FLOOR_ROW = 190          # the search box never reaches below this row (was: whole frame).
                         # The black band merged with the letter into one big blob.
ROW_MIN = 40             # FOMO centre must be in this band (was 0-239)
ROW_MAX = 165            # = FLOOR_ROW - 25: a centre lower than this is on the black band

# --- persistence (both letters and targets) ---
TRACK_WINDOW = 4         # look at the last 4 frames
CONFIRM_HITS = 3         # need 3 of them with the same letter / ring sum
POS_TOL_PX = 40          # ASSUMED: how far a detection may move between frames
                         # (driving past moves it ~20 px per frame; measure with DEBUG_PRINT)
SIZE_TOL = 0.25          # targets: radius may change by 25% between frames

# --- letters (shape check around each FOMO hit, on the grayscale frame) ---
# MEASURED 2026-10-19 with measure_letters.py on printed letters (Omega/U and Psi/S, still,
# one wall distance; Phi/H not recorded yet). The 30 hits where the whole letter was
# measured: size 64-69 px, aspect 0.97-1.18, density 0.30-0.33, contrast 205-228.
# Limits = that range with a margin (other distances, rotation, blur, Phi not measured).
LETTER_PX_MIN = 50       # measured 64-69 px (was 40, assumed)
LETTER_PX_MAX = 90       # (was 150, assumed)
LETTER_PAD = 15          # search box = LETTER_PX_MAX + 2*pad = 120 px around the FOMO centre.
                         # Was 190 px: it reached the robot's floor / wires in view, and that
                         # dark area was measured instead of the letter in most frames.
LETTER_ASPECT_MIN = 0.7  # measured 0.97-1.18 (was 0.6)
LETTER_ASPECT_MAX = 1.5  # (was 1.6)
LETTER_DENSITY_MIN = 0.20  # measured 0.30-0.33 (was 0.12, assumed)
LETTER_DENSITY_MAX = 0.45  # (was 0.65, assumed)
LETTER_MIN_CONTRAST = 120  # measured 205-228 (was 60, assumed)
LETTER_MIN_PIXELS = 60

# --- cognitive targets ---
# MEASURED 2026-10-21 with measure_cognitive.py (target U = yellow/blue/red/black/green, about
# run distance, 47 frames): the target's circle had strength 1657-2724 (median 1940) and radius
# 27-52 px (median 35). With 2500 / 50-115 it was checked in 3 of 31 frames, now 28 of 31.
# Other circles on the same target (inner ring edges, and 1.4-2.3x bigger ones around it) are
# rejected by the checks below; R_MAX keeps the big ones from taking the 3 candidate places.
CIRCLE_THRESHOLD = 1600  # just under the weakest target circle (was 2500, assumed).
                         # The checks below do the rejecting, not this number.
R_MIN = 25               # smallest target 27 px (was 50)
R_MAX = 65               # largest target 52 px + 25% for distance (was 115, assumed)
MAX_CIRCLE_CANDIDATES = 3  # strongest Hough circles checked per frame (keeps FPS up)
RING_FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)  # middle of each ring (diameters 1..5 cm -> R/5 each)
RING_SAMPLES = 12        # angles per ring (old code: 35, and it skipped 0 deg)
PATCH_HALF_FRAC = 0.04   # median patch half-size = 0.04 R (stays inside a 0.2 R wide ring)
RING_AGREE = 0.6         # a ring needs 60% of its patches to agree on one colour
WALL_FRAC = 1.25         # wall check annulus just outside the target
WALL_WHITE_FRAC = 0.6    # 60% of the annulus must look like wall
WALL_L_MIN = 45          # ASSUMED: raw LAB of the wall (calibrate_colors.py prints it)
WALL_CHROMA_MAX = 35     # MEASURED 2026-10-21: the wall around the target read chroma 17-33
                         # (tinted; normalise() removes the tint). Was 22 (assumed): the target
                         # failed "no white wall" (7/12 patches, needs 8).

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

# --- colours: nearest calibrated centre, in wall-normalised LAB (wall = L 100, a 0, b 0) ---
# ASSUMED until calibrate_colors.py is run on the real printed targets at the venue.
COLOR_CENTRES = {
    "black":  (15,   0,   0),
    "red":    (55,  55,  35),
    "yellow": (90,  -5,  70),
    "green":  (55, -45,  25),
    "blue":   (60,  -5, -45),
    "white":  (100,  0,   0),  # wall / paper: never a ring colour
}
COLOR_VALUE = {"black": -2, "red": -1, "yellow": 0, "green": 1, "blue": 2}
SUM_TO_LABEL = {2: "H", 1: "S", 0: "U"}  # any other sum = fake target, never reported
L_WEIGHT = 0.5           # L counts half: lighting changes L more than hue
MAX_COLOR_DIST = 45      # further than this from every centre -> unknown
MARGIN_RATIO = 0.75      # nearest must be clearly nearer than the 2nd nearest.
                         # THE leniency knob: higher = more lenient, lower = stricter.

# --- stop hints ("c" byte; the robot stops for any byte, then waits for H/S/U) ---
HINT_ON_TARGET = True    # a target that passed every check in one frame -> "c"
HINT_ON_LETTER = False   # turn on if letters are missed while driving past

# --- debugging ---
DEBUG_PRINT = True       # one line per candidate with the reason it was accepted/rejected
DEBUG_SAVE = False       # save frames with candidates + log.csv to the SD card
DEBUG_MAX_FILES = 400
DEBUG_DRAW = True        # overlays for the OpenMV IDE preview (drawn last)

# ============================== CAMERA SET-UP ===========================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.set_windowing((240, 240))
sensor.set_vflip(True)                 # the camera's upside down on the robot.
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)  # short exposure = less blur
sensor.skip_frames(time=2000)          # let gain / white balance settle at this exposure
if LOCK_GAIN_AND_WB:
    # NEW: freeze gain and white balance so colours don't drift frame to frame.
    try:
        gain = sensor.get_gain_db()
        rgb_gain = sensor.get_rgb_gain_db()
        sensor.set_auto_gain(False, gain_db=gain + EXTRA_GAIN_DB)
        sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain)
        print("locked gain %.1f dB, rgb gain %s" % (gain + EXTRA_GAIN_DB, str(rgb_gain)))
    except Exception as e:
        print("could not lock gain/white balance:", e)

uart = UART(1, 115200, timeout_char=100)
p = pyb.Pin("P3", pyb.Pin.OUT_PP)      # victim LED / signal pin

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]

# Angles for ring sampling. range(RING_SAMPLES) includes 0 deg (old range(1, 36) skipped it).
ANGLES = [(math.cos(2 * math.pi * k / RING_SAMPLES), math.sin(2 * math.pi * k / RING_SAMPLES))
          for k in range(RING_SAMPLES)]

# ============================== FOMO ====================================================
model_info_printed = False

def fomo_post_process(model, inputs, outputs):
    global model_info_printed
    if not model_info_printed:
        # NEW 2026-10-20: printed once. Input (1, H, W, 1) = grayscale model; output
        # (1, H/8, W/8, classes) = one FOMO cell per 8 input pixels (Edge Impulse default).
        print("model input", model.input_shape, model.input_dtype,
              "output", model.output_shape, model.output_dtype)
        model_info_printed = True
    ob, oh, ow, oc = model.output_shape[0]

    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh

    scale = min(x_scale, y_scale)

    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]  # FIXED: was ow

    l = [[] for i in range(oc)]

    for i in range(oc):
        # per-class confidence from CLASS_CONFIDENCE (per-camera block)
        name = labels[i] if i < len(labels) else ""
        conf = CLASS_CONFIDENCE.get(name, DEFAULT_CONFIDENCE)
        class_threshold = [(math.ceil(conf * 255), 255)]

        # REVERTED 2026-10-21 to the Edge Impulse example: the callback's output is already
        # 0..1 on this camera's firmware (converting it again made every cell ~0.50).
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(
            class_threshold, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
        )
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = (
                img.get_statistics(thresholds=class_threshold, roi=rect).l_mean() / 255.0
            )
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return l

# ============================== LETTER SHAPE CHECK ======================================
def check_letter(img, cx, cy):
    """NEW. Is there one letter-shaped dark blob around this FOMO hit?
    Returns (ok, reason, size_px). Runs on the grayscale frame."""
    if cy < ROW_MIN or cy > ROW_MAX:
        return False, "outside row band", 0
    W = img.width(); H = img.height()
    half = LETTER_PX_MAX // 2 + LETTER_PAD
    x0 = max(0, cx - half); y0 = max(0, cy - half)
    x1 = min(W, cx + half); y1 = min(H, cy + half, FLOOR_ROW)   # never into the black band
    if x1 - x0 < LETTER_PX_MIN or y1 - y0 < LETTER_PX_MIN:
        return False, "at frame edge", 0
    roi = (x0, y0, x1 - x0, y1 - y0)

    st = img.get_statistics(roi=roi)
    if st.max() - st.min() < LETTER_MIN_CONTRAST:
        return False, "low contrast", 0
    # Otsu threshold of this box: adapts to the lighting instead of a fixed (0, 20).
    thr = img.get_histogram(roi=roi).get_threshold().value()
    # CHANGED 2026-10-19: merge off (Phi, Psi and Omega are each one connected shape; merging
    # joined the letter with the dark floor/wires next to it) and the blob nearest the FOMO
    # centre is measured, not the largest (the largest was usually the floor, not the letter).
    blobs = img.find_blobs([(0, thr)], roi=roi, pixels_threshold=LETTER_MIN_PIXELS,
                           area_threshold=LETTER_MIN_PIXELS, merge=False)
    if not blobs:
        return False, "no dark blob", 0
    b = min(blobs, key=lambda bb: abs(bb.cx() - cx) + abs(bb.cy() - cy))
    w = b.w(); h = b.h(); size = max(w, h)
    if size < LETTER_PX_MIN or size > LETTER_PX_MAX:
        return False, "size %d px" % size, size
    aspect = w / h
    if aspect < LETTER_ASPECT_MIN or aspect > LETTER_ASPECT_MAX:
        return False, "aspect %.2f (edge or seam?)" % aspect, size
    if b.density() < LETTER_DENSITY_MIN or b.density() > LETTER_DENSITY_MAX:
        return False, "density %.2f" % b.density(), size
    # A wall edge, seam or tile border runs right across the search box.
    touches = ((b.x() <= x0) + (b.y() <= y0) +
               (b.x() + b.w() >= x1) + (b.y() + b.h() >= y1))
    if touches >= 2:
        return False, "runs off the box (edge/seam/tile)", size
    return True, "ok", size

# ============================== COLOUR / TARGET CHECK ===================================
def median(vals):
    s = sorted(vals)
    return s[len(s) // 2]

def patch_lab(img, x, y, half):
    """Median LAB of a small square patch, or None if it's off the frame."""
    if x - half < 0 or y - half < 0 or x + half >= img.width() or y + half >= img.height():
        return None
    s = img.get_statistics(roi=(x - half, y - half, 2 * half + 1, 2 * half + 1))
    return (s.l_median(), s.a_median(), s.b_median())

def is_wall(lab):
    return lab[0] >= WALL_L_MIN and lab[1] * lab[1] + lab[2] * lab[2] <= WALL_CHROMA_MAX * WALL_CHROMA_MAX

def normalise(lab, wall):
    """NEW: per-target white reference. Scale L so the wall is 100, shift a/b so the wall is 0."""
    L = lab[0] * 100.0 / max(wall[0], 1)
    return (min(L, 100.0), lab[1] - wall[1], lab[2] - wall[2])

def classify(lab):
    """NEW: nearest calibrated colour with a margin test (replaces the 5 threshold boxes).
    Returns a colour name, or 'unknown' if it isn't clearly one colour."""
    best = None; d1 = 1e9; d2 = 1e9
    for name, c in COLOR_CENTRES.items():
        dL = lab[0] - c[0]; da = lab[1] - c[1]; db = lab[2] - c[2]
        d = math.sqrt(L_WEIGHT * dL * dL + da * da + db * db)
        if d < d1:
            d2 = d1; d1 = d; best = name
        elif d < d2:
            d2 = d
    if d1 > MAX_COLOR_DIST or d1 > MARGIN_RATIO * d2:
        return "unknown"
    return best

def check_target(img, x, y, r):
    """NEW. Confirm a Hough circle is a cognitive target. Runs on the colour frame.
    Returns (label or None, reason, ring colours)."""
    if y < ROW_MIN or y > ROW_MAX:
        return None, "outside row band", None
    W = img.width(); H = img.height()
    if x - 0.95 * r < 0 or y - 0.95 * r < 0 or x + 0.95 * r >= W or y + 0.95 * r >= H:
        return None, "rings off the frame", None   # FIXED: get_pixel used to crash here
    half = max(1, int(r * PATCH_HALF_FRAC + 0.5))

    # 1. White wall just outside the target? Corners, edges and tiles fail this.
    wall = []
    for ca, sa in ANGLES:
        lab = patch_lab(img, int(x + WALL_FRAC * r * ca + 0.5), int(y - WALL_FRAC * r * sa + 0.5), half)
        if lab is not None:
            wall.append(lab)
    if len(wall) < RING_SAMPLES // 2:
        return None, "wall annulus off the frame", None
    white = [w for w in wall if is_wall(w)]
    if len(white) < WALL_WHITE_FRAC * len(wall):
        return None, "no white wall around (%d/%d)" % (len(white), len(wall)), None
    ref = (median([w[0] for w in white]), median([w[1] for w in white]), median([w[2] for w in white]))

    # 2. Each ring must have one clear colour. Sampled at the ring middles.
    colours = []
    total = 0
    for k, frac in enumerate(RING_FRACS):
        pts = [(x, y)] if k == 0 else []   # the centre belongs to the inner circle
        for ca, sa in ANGLES:
            pts.append((int(x + frac * r * ca + 0.5), int(y - frac * r * sa + 0.5)))
        votes = {}
        n = 0
        for px, py in pts:
            lab = patch_lab(img, px, py, half)
            if lab is None:
                continue
            n += 1
            name = classify(normalise(lab, ref))
            votes[name] = votes.get(name, 0) + 1
        if n == 0:
            return None, "ring %d off the frame" % (k + 1), colours
        best = max(votes, key=votes.get)
        if best == "white" or best == "unknown":
            return None, "ring %d is %s" % (k + 1, best), colours   # was: counted as black
        if votes[best] < RING_AGREE * n:
            return None, "ring %d unclear (%s %d/%d)" % (k + 1, best, votes[best], n), colours
        colours.append(best)
        total += COLOR_VALUE[best]

    # 3. Only sums 0/1/2 are victims; anything else is a fake target (rules 3.6.4).
    label = SUM_TO_LABEL.get(total)
    if label is None:
        return None, "sum %d = fake target" % total, colours
    return label, "ok sum %d" % total, colours

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

# ============================== PERSISTENCE =============================================
class Track:
    """NEW: one letter or target followed across frames."""
    def __init__(self, kind, x, y, size, label, frame):
        self.kind = kind; self.x = x; self.y = y; self.size = size
        self.frames = [frame]; self.labels = [label]

    def add(self, x, y, size, label, frame):
        self.x = x; self.y = y; self.size = size
        self.frames.append(frame); self.labels.append(label)

    def last(self):
        return self.frames[-1]

    def top_label(self):
        counts = {}
        for lb in self.labels:
            counts[lb] = counts.get(lb, 0) + 1
        best = max(counts, key=counts.get)
        return best, counts[best]

tracks = []

def update_tracks(dets, frame):
    """Match this frame's accepted detections to tracks (same kind, close by, similar size)."""
    global tracks
    for kind, x, y, size, label in dets:
        best = None; bd = POS_TOL_PX + 1
        for t in tracks:
            if t.kind != kind or t.last() == frame:
                continue
            if kind == "target" and abs(size - t.size) > SIZE_TOL * t.size:
                continue
            d = abs(t.x - x) + abs(t.y - y)
            if d <= POS_TOL_PX and d < bd:
                best = t; bd = d
        if best is not None:
            best.add(x, y, size, label, frame)
        else:
            tracks.append(Track(kind, x, y, size, label, frame))
    # forget frames older than the window, and tracks not seen in it
    oldest = frame - TRACK_WINDOW + 1
    keep = []
    for t in tracks:
        while t.frames and t.frames[0] < oldest:
            t.frames.pop(0); t.labels.pop(0)
        if t.frames:
            keep.append(t)
    tracks = keep

def confirmed_label(frame):
    """Victim byte to send: a track seen this frame whose letter / ring sum
    was the same in at least CONFIRM_HITS of the last TRACK_WINDOW frames."""
    best = None; best_n = 0
    for t in tracks:
        if t.last() != frame:
            continue
        label, n = t.top_label()
        if n >= CONFIRM_HITS and n > best_n:
            best = label; best_n = n
    return best

# ============================== DEBUG RECORDING =========================================
debug_dir = None
debug_count = 0
if DEBUG_SAVE:
    # Records to a new folder each power-up. Use an SD card: internal flash is tiny.
    for n in range(1000):
        name = "dbg_%s_%03d" % (CAMERA_NAME, n)
        try:
            uos.stat(name)
        except OSError:
            uos.mkdir(name)
            debug_dir = name
            break

def debug_log(frame, kind, label, x, y, size, ok, reason):
    if DEBUG_PRINT:
        print("%s f%d %s %s x%d y%d size%d %s: %s" %
              (CAMERA_NAME, frame, kind, label, x, y, size, "ACCEPT" if ok else "reject", reason))
    if debug_dir is not None:
        try:
            with open(debug_dir + "/log.csv", "a") as f:
                f.write("%d,%s,%s,%d,%d,%d,%d,%s\n" % (frame, kind, label, x, y, size, 1 if ok else 0, reason))
        except Exception as e:
            print("log write failed:", e)

def debug_save(img, frame, tag):
    global debug_count
    if debug_dir is None or debug_count >= DEBUG_MAX_FILES:
        return
    try:
        img.save("%s/%05d_%s.jpg" % (debug_dir, frame, tag), quality=80)
        debug_count += 1
    except Exception as e:
        print("frame save failed:", e)

# ============================== MAIN LOOP ===============================================
clock = time.clock()
frame = 0
while True:
    clock.tick()
    frame += 1
    img = sensor.snapshot().lens_corr(1.8)
    p.low()
    dets = []          # accepted detections this frame: (kind, x, y, size, label)
    target_hint = False
    letter_hint = False
    to_draw = []       # overlays, drawn at the end so they never touch measured pixels

    # --- cognitive targets, on the colour frame, before any drawing or grayscale ---
    circles = img.find_circles(threshold=CIRCLE_THRESHOLD, r_min=R_MIN, r_max=R_MAX)
    circles = sorted(circles, key=lambda c: c.magnitude(), reverse=True)[:MAX_CIRCLE_CANDIDATES]
    checked = []       # refined circles already checked this frame
    for c in circles:
        x, y, r = c.x(), c.y(), c.r()
        fit = ""
        if REFINE:   # NEW 2026-10-21: onto the target's outer edge first
            x, y, r, n = refine_circle(img, x, y, r)
            if n:
                fit = " [fit %d pts, hough x%d y%d r%d]" % (n, c.x(), c.y(), c.r())
        if is_duplicate(checked, x, y, r):
            debug_log(frame, "target", "-", x, y, r, False, "same circle as one already checked" + fit)
            continue
        checked.append((x, y, r))
        label, reason, colours = check_target(img, x, y, r)
        debug_log(frame, "target", label or "-", x, y, r, label is not None,
                  reason + (" " + "/".join(colours) if colours else "") + fit)
        if label is not None:
            dets.append(("target", x, y, r, label))
            target_hint = True
        to_draw.append(("circle", x, y, r, label is not None))
    if circles:
        debug_save(img, frame, "c")    # colour frame, for the ring colours

    # --- letters: FOMO on grayscale every frame (old code mixed grayscale and colour) ---
    img.to_grayscale()
    letter_seen = False
    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0:
            continue  # background class
        name = labels[i]
        if name not in ("H", "S", "U"):
            continue
        for x, y, w, h, score in detection_list:
            letter_seen = True
            cx = math.floor(x + (w / 2)); cy = math.floor(y + (h / 2))   # FOMO centre, as in the original script
            ok, reason, size = check_letter(img, cx, cy)
            debug_log(frame, "letter", name, cx, cy, size, ok, reason + " score %.2f" % score)
            if ok:
                dets.append(("letter", cx, cy, size, name))
                letter_hint = True
            to_draw.append(("letter", cx, cy, i, ok))   # i = class index, for its colour
    if letter_seen:
        debug_save(img, frame, "l")    # grayscale frame, what FOMO saw

    # --- persistence, then send at most one byte per frame ---
    update_tracks(dets, frame)
    victim = confirmed_label(frame)
    if victim is not None:
        uart.write(victim)
        p.high()
        if DEBUG_PRINT:
            print("%s f%d SEND %s" % (CAMERA_NAME, frame, victim))
    elif (target_hint and HINT_ON_TARGET) or (letter_hint and HINT_ON_LETTER):
        uart.write("c")   # "stop and look": robot stops, the next frames confirm or not

    # --- overlays for the IDE preview, last ---
    if DEBUG_DRAW:
        for kind, x, y, s, ok in to_draw:
            if kind == "circle":
                img.draw_circle(x, y, s, color=255 if ok else 80)
            else:
                # CHANGED 2026-10-19: Edge Impulse FOMO example method - a circle at the FOMO
                # centre in the class colour (was a square guessed from the blob size, which
                # didn't line up with the letter). Thick = passed the shape check.
                img.draw_circle((x, y, 12), color=colors[s % len(colors)], thickness=2 if ok else 1)

    if DEBUG_PRINT and frame % 30 == 0:
        print("%s fps %.1f" % (CAMERA_NAME, clock.fps()))
