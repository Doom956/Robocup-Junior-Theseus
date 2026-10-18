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
# ASSUMED: whole frame until measured (DEBUG_PRINT prints y of every detection).
ROW_MIN = 0
ROW_MAX = 239

# --- persistence (both letters and targets) ---
TRACK_WINDOW = 4         # look at the last 4 frames
CONFIRM_HITS = 3         # need 3 of them with the same letter / ring sum
POS_TOL_PX = 40          # ASSUMED: how far a detection may move between frames
                         # (driving past moves it ~20 px per frame; measure with DEBUG_PRINT)
SIZE_TOL = 0.25          # targets: radius may change by 25% between frames

# --- letters (shape check around each FOMO hit, on the grayscale frame) ---
LETTER_PX_MIN = 40       # ASSUMED: letter height (4 cm) in pixels at the wall distance.
LETTER_PX_MAX = 150      # ASSUMED. Measure: DEBUG_PRINT shows "size" of accepted letters.
LETTER_PAD = 20          # search box = LETTER_PX_MAX + 2*pad around the FOMO centre
LETTER_ASPECT_MIN = 0.6  # width/height of the dark blob; seams/edges are long and thin
LETTER_ASPECT_MAX = 1.6
LETTER_DENSITY_MIN = 0.12  # dark pixels / bounding box (ASSUMED from Arial glyphs)
LETTER_DENSITY_MAX = 0.65
LETTER_MIN_CONTRAST = 60   # grayscale max-min in the box; a plain wall has less
LETTER_MIN_PIXELS = 60
LETTER_MERGE_MARGIN = 3

# --- cognitive targets ---
CIRCLE_THRESHOLD = 2500  # ASSUMED: lenient on purpose (old: 5000 left / 3000 right).
                         # The checks below do the rejecting, not this number.
R_MIN = 50               # kept from the old script
R_MAX = 115              # ASSUMED: about half the 240 px window
MAX_CIRCLE_CANDIDATES = 3  # strongest Hough circles checked per frame (keeps FPS up)
RING_FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)  # middle of each ring (diameters 1..5 cm -> R/5 each)
RING_SAMPLES = 12        # angles per ring (old code: 35, and it skipped 0 deg)
PATCH_HALF_FRAC = 0.04   # median patch half-size = 0.04 R (stays inside a 0.2 R wide ring)
RING_AGREE = 0.6         # a ring needs 60% of its patches to agree on one colour
WALL_FRAC = 1.25         # wall check annulus just outside the target
WALL_WHITE_FRAC = 0.6    # 60% of the annulus must look like wall
WALL_L_MIN = 45          # ASSUMED: raw LAB of the wall (calibrate_colors.py prints it)
WALL_CHROMA_MAX = 22     # ASSUMED

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

# Angles for ring sampling. range(RING_SAMPLES) includes 0 deg (old range(1, 36) skipped it).
ANGLES = [(math.cos(2 * math.pi * k / RING_SAMPLES), math.sin(2 * math.pi * k / RING_SAMPLES))
          for k in range(RING_SAMPLES)]

# ============================== FOMO ====================================================
def fomo_post_process(model, inputs, outputs):
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
    x1 = min(W, cx + half); y1 = min(H, cy + half)
    if x1 - x0 < LETTER_PX_MIN or y1 - y0 < LETTER_PX_MIN:
        return False, "at frame edge", 0
    roi = (x0, y0, x1 - x0, y1 - y0)

    st = img.get_statistics(roi=roi)
    if st.max() - st.min() < LETTER_MIN_CONTRAST:
        return False, "low contrast", 0
    # Otsu threshold of this box: adapts to the lighting instead of a fixed (0, 20).
    thr = img.get_histogram(roi=roi).get_threshold().value()
    blobs = img.find_blobs([(0, thr)], roi=roi, pixels_threshold=LETTER_MIN_PIXELS,
                           area_threshold=LETTER_MIN_PIXELS, merge=True,
                           margin=LETTER_MERGE_MARGIN)
    if not blobs:
        return False, "no dark blob", 0
    b = max(blobs, key=lambda bb: bb.pixels())
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
    for c in circles:
        label, reason, colours = check_target(img, c.x(), c.y(), c.r())
        debug_log(frame, "target", label or "-", c.x(), c.y(), c.r(), label is not None,
                  reason + (" " + "/".join(colours) if colours else ""))
        if label is not None:
            dets.append(("target", c.x(), c.y(), c.r(), label))
            target_hint = True
        to_draw.append(("circle", c.x(), c.y(), c.r(), label is not None))
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
            cx = int(x + w / 2); cy = int(y + h / 2)
            ok, reason, size = check_letter(img, cx, cy)
            debug_log(frame, "letter", name, cx, cy, size, ok, reason + " score %.2f" % score)
            if ok:
                dets.append(("letter", cx, cy, size, name))
                letter_hint = True
            to_draw.append(("letter", cx, cy, size, ok))
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
            col = 255 if ok else 80
            if kind == "circle":
                img.draw_circle(x, y, s, color=col)
            else:
                img.draw_rectangle(x - s // 2, y - s // 2, max(s, 4), max(s, 4), color=col)

    if DEBUG_PRINT and frame % 30 == 0:
        print("%s fps %.1f" % (CAMERA_NAME, clock.fps()))
