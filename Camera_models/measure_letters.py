# Theseus - LETTER measurement (OpenMV, firmware v4.8.x). NEW 2026-10-19.
#
# Point the camera at your printed letters. It runs the same FOMO model and the same letter
# shape check as the camera scripts, rejects nothing, and PRINTS every measurement to the
# OpenMV IDE serial terminal. Copy the whole terminal output and paste it to Claude: it
# shows whether the limits in the camera scripts are too strict or too loose.
#
# How to use:
#   1. Copy trained.tflite + labels.txt to the camera like for the camera scripts.
#   2. Set SCENE (the letter in front of the camera), NOTE and MOTION below.
#   3. Run from the OpenMV IDE. It stops after FRAMES frames and prints a summary.
#   4. Copy everything from the terminal (from the #CFG line to #END).
#   Good set of runs: each letter (H, S, U) still, rotated, at the closest and furthest wall
#   distance, and driving past; then SCENE = "none" at a corner, a wall seam, a black tile,
#   and a decoy letter, so we can see what must still be rejected.
#
# Line format (one line per FOMO hit):
#   L,frame,label,score,peak,cx,cy,contrast,otsu,n_blobs,w,h,size,aspect,density,elong,touches,fails
#   score = mean probability of the FOMO cells >= RECORD_CONFIDENCE in this hit;
#   peak  = its strongest cell. The camera scripts accept a hit when peak >= their confidence
#   (at their threshold the hit exists only if one cell reaches it), so "conf" uses peak.
#   fails = every camera-script limit this hit breaks, joined by "+", or "ok".
# First line #MODEL = the model's input/output shape and type (grayscale? grid size?) and the
# camera's firmware version.
#
# KEEP IN SYNC: the limits and letter_metrics() mirror check_letter() in
# ei_object_detection_PhiOmegaPsi -left/-right.py.

import sensor, image, time, ml, math, uos, gc, omv
from ulab import numpy as np

# FIXED 2026-10-21: only v4.8+ gives the predict() callback the model's RAW output; v4.5-4.7
# already give 0..1 probabilities, and converting those again made every cell ~0.50.
RAW_CALLBACK_OUTPUT = (omv.version_major, omv.version_minor) >= (4, 8)

# ============================== WHAT IS IN FRONT OF THE CAMERA ==========================
CAMERA_NAME = "left"     # left / right
SCENE = "H"              # H / S / U = that printed letter; "none" = nothing that should count
NOTE = "still_close"     # free text, no commas: still_close, far, rotated_45, corner, seam ...
MOTION = "still"         # still / moving
FRAMES = 150             # frames, then a summary
RECORD_CONFIDENCE = 0.3  # low on purpose so weak hits are printed too
DRAW = True              # show the search box (grey) and measured blob (white) in the IDE

# ============== COPIED FROM THE CAMERA SCRIPT (keep the same values) ====================
CAMERA_CONFIDENCE = {"H": 0.7, "S": 0.7, "U": 0.7}   # right camera: {"H": 0.9, "S": 0.7, "U": 0.9}
EXPOSURE_US = 10000
EXTRA_GAIN_DB = 0
LOCK_GAIN_AND_WB = True
FLOOR_ROW = 190          # search box never below this row (black robot / floor band)
ROW_MIN = 40
ROW_MAX = 165
LETTER_PX_MIN = 50       # measured 2026-10-19 (see the camera scripts)
LETTER_PX_MAX = 90
LETTER_PAD = 15          # search box 120 px (was 190: it reached the floor / wires)
LETTER_ASPECT_MIN = 0.7
LETTER_ASPECT_MAX = 1.5
LETTER_DENSITY_MIN = 0.20
LETTER_DENSITY_MAX = 0.45
LETTER_MIN_CONTRAST = 120
LETTER_MIN_PIXELS = 60

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

net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
labels = [line.rstrip('\n') for line in open("labels.txt")]

model_info_printed = False

def class_map(model, outputs, i):
    # On v4.8+ the callback gets the RAW output: convert it if the model's output is
    # quantized (int8/uint8). Older firmware has already converted it.
    a = outputs[0][0, :, :, i]
    if RAW_CALLBACK_OUTPUT and model.output_dtype[0] != 'f':
        a = (np.array(a, dtype=np.float) - model.output_zero_point[0]) * model.output_scale[0]
    return a

def fomo_post_process(model, inputs, outputs):
    # Same as the camera scripts, with one low threshold for every class.
    global model_info_printed
    if not model_info_printed:
        print("#MODEL input %s %s output %s %s firmware %s raw_callback_output %s" %
              (model.input_shape, model.input_dtype, model.output_shape, model.output_dtype,
               omv.version_string(), RAW_CALLBACK_OUTPUT))
        model_info_printed = True
    ob, oh, ow, oc = model.output_shape[0]
    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (oh * scale)) / 2) + inputs[0].roi[1]
    th = [(math.ceil(RECORD_CONFIDENCE * 255), 255)]
    l = [[] for i in range(oc)]
    for i in range(oc):
        img = image.Image(class_map(model, outputs, i) * 255)
        for b in img.find_blobs(th, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1):
            x, y, w, h = b.rect()
            st = img.get_statistics(thresholds=th, roi=b.rect())
            score = st.l_mean() / 255.0
            peak = st.l_max() / 255.0          # strongest cell: what the camera's threshold tests
            l[i].append((int(x * scale + x_offset), int(y * scale + y_offset), int(w * scale), int(h * scale), score, peak))
    return l

def letter_metrics(img, cx, cy):
    """What check_letter() measures, without stopping at the first failure."""
    m = {"roi": None, "contrast": -1, "otsu": -1, "n_blobs": 0, "w": 0, "h": 0, "size": 0,
         "aspect": 0.0, "density": 0.0, "elong": 0.0, "touches": -1, "blob": None}
    W = img.width(); H = img.height()
    half = LETTER_PX_MAX // 2 + LETTER_PAD
    x0 = max(0, cx - half); y0 = max(0, cy - half)
    x1 = min(W, cx + half); y1 = min(H, cy + half, FLOOR_ROW)
    m["box_w"] = x1 - x0; m["box_h"] = y1 - y0
    if x1 - x0 < 8 or y1 - y0 < 8:
        return m
    roi = (x0, y0, x1 - x0, y1 - y0)
    m["roi"] = roi
    st = img.get_statistics(roi=roi)
    m["contrast"] = st.max() - st.min()
    thr = img.get_histogram(roi=roi).get_threshold().value()
    m["otsu"] = thr
    # Same as check_letter(): no merging, blob nearest the FOMO centre (changed 2026-10-19).
    blobs = img.find_blobs([(0, thr)], roi=roi, pixels_threshold=LETTER_MIN_PIXELS,
                           area_threshold=LETTER_MIN_PIXELS, merge=False)
    m["n_blobs"] = len(blobs)
    if blobs:
        b = min(blobs, key=lambda bb: abs(bb.cx() - cx) + abs(bb.cy() - cy))
        m["blob"] = b.rect()
        m["w"] = b.w(); m["h"] = b.h(); m["size"] = max(b.w(), b.h())
        m["aspect"] = b.w() / b.h(); m["density"] = b.density(); m["elong"] = b.elongation()
        m["touches"] = (b.x() <= x0) + (b.y() <= y0) + (b.x() + b.w() >= x1) + (b.y() + b.h() >= y1)
    return m

def fails(m, cx, cy, label, peak):
    """Every camera-script limit this hit breaks (check_letter() stops at the first one)."""
    f = []
    if peak < CAMERA_CONFIDENCE.get(label, 1.0): f.append("conf")
    if cy < ROW_MIN or cy > ROW_MAX: f.append("row")
    if m["box_w"] < LETTER_PX_MIN or m["box_h"] < LETTER_PX_MIN: f.append("edge")
    if m["contrast"] < LETTER_MIN_CONTRAST: f.append("contrast")
    if m["n_blobs"] == 0:
        f.append("noblob")
    else:
        if m["size"] < LETTER_PX_MIN: f.append("small")
        if m["size"] > LETTER_PX_MAX: f.append("big")
        if m["aspect"] < LETTER_ASPECT_MIN: f.append("narrow")
        if m["aspect"] > LETTER_ASPECT_MAX: f.append("wide")
        if m["density"] < LETTER_DENSITY_MIN: f.append("thin")
        if m["density"] > LETTER_DENSITY_MAX: f.append("dense")
        if m["touches"] >= 2: f.append("touches")
    return f

def spread(vals):
    if not vals:
        return "-"
    s = sorted(vals)
    n = len(s)
    return "min %.2f p10 %.2f med %.2f p90 %.2f max %.2f n %d" % (s[0], s[n // 10], s[n // 2], s[n * 9 // 10], s[-1], n)

# ============================== MAIN LOOP ===============================================
print("#CFG camera=%s scene=%s note=%s motion=%s frames=%d conf=%s size=%d-%d aspect=%.2f-%.2f "
      "density=%.2f-%.2f contrast>=%d row=%d-%d floor=%d box=%d" %
      (CAMERA_NAME, SCENE, NOTE, MOTION, FRAMES, str(CAMERA_CONFIDENCE), LETTER_PX_MIN, LETTER_PX_MAX,
       LETTER_ASPECT_MIN, LETTER_ASPECT_MAX, LETTER_DENSITY_MIN, LETTER_DENSITY_MAX,
       LETTER_MIN_CONTRAST, ROW_MIN, ROW_MAX, FLOOR_ROW, 2 * (LETTER_PX_MAX // 2 + LETTER_PAD)))
print("#COLS L,frame,label,score,peak,cx,cy,contrast,otsu,n_blobs,w,h,size,aspect,density,elong,touches,fails")

clock = time.clock()
frames_with_hit = 0          # frames with any FOMO hit
frames_right = 0             # frames with a hit of the right letter
frames_pass = 0              # frames where the camera script would accept the right letter
fail_count = {}
vals = {"score": [], "peak": [], "contrast": [], "size": [], "aspect": [], "density": [], "cy": []}
for frame in range(1, FRAMES + 1):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    img.to_grayscale()
    hits = []
    for i, dl in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0 or i >= len(labels):
            continue
        for x, y, w, h, score, peak in dl:
            hits.append((labels[i], score, peak, int(x + w / 2), int(y + h / 2)))
    right = False; passed = False
    draw = []
    for label, score, peak, cx, cy in hits:
        m = letter_metrics(img, cx, cy)
        f = fails(m, cx, cy, label, peak)
        print("L,%d,%s,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d,%s" %
              (frame, label, score, peak, cx, cy, m["contrast"], m["otsu"], m["n_blobs"], m["w"], m["h"],
               m["size"], m["aspect"], m["density"], m["elong"], m["touches"], "+".join(f) or "ok"))
        if label == SCENE or SCENE == "none":
            for k in f:
                fail_count[k] = fail_count.get(k, 0) + 1
            if m["n_blobs"]:
                vals["size"].append(m["size"]); vals["aspect"].append(m["aspect"]); vals["density"].append(m["density"])
            vals["score"].append(score); vals["peak"].append(peak); vals["contrast"].append(m["contrast"]); vals["cy"].append(cy)
        if label == SCENE:
            right = True
            passed = passed or not f
        draw.append((m["roi"], m["blob"]))
    frames_with_hit += 1 if hits else 0
    frames_right += 1 if right else 0
    frames_pass += 1 if passed else 0
    if DRAW:
        for roi, blob in draw:
            if roi: img.draw_rectangle(roi, color=128)
            if blob: img.draw_rectangle(blob, color=255)

print("#SUM scene=%s note=%s motion=%s frames=%d fps=%.1f" % (SCENE, NOTE, MOTION, FRAMES, clock.fps()))
print("#SUM frames with any hit: %d, with the right letter: %d, accepted by the camera script: %d"
      % (frames_with_hit, frames_right, frames_pass))
print("#SUM limit failures (%s hits): %s" % ("all" if SCENE == "none" else SCENE, str(fail_count)))
for k in ("score", "peak", "contrast", "size", "aspect", "density", "cy"):
    print("#SUM %-8s %s" % (k, spread(vals[k])))
print("#END")
