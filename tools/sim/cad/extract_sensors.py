"""Extract the robot's sensor positions from a Fusion 360 STEP export.

Usage:  python extract_sensors.py V2.step
Writes robot_geometry.json next to this script and prints a table.
Plain Python, no extra packages.

How it works:
- rebuilds the STEP assembly tree and places every part in the robot's frame
- finds the 7 "VL53L0X" parts and the "TCS Color Sensor"; each VL53L0X is a flat board, and the
  sensor is taken to face outward from the robot along the board's thin axis
- front = the end with two VL53L0X boards (the colour sensor sits there too)
- the robot centre is halfway between the front and back wheel axles, on the floor under it
- floor = the lowest point of the wheels
- code sensor numbers follow measure() in Main/Distance.cpp: 1 front-right, 7 front-left,
  2 right-front, 3 right-back, 6 left-front, 5 left-back, 4 back
"""
import json, math, os, re, sys

# ---------- STEP parsing ----------
txt = open(sys.argv[1], encoding="latin-1").read()
data = txt[txt.index("DATA;") + 5: txt.rindex("ENDSEC;")]
ents = {int(m.group(1)): re.sub(r"\s+", " ", m.group(2).strip())
        for m in re.finditer(r"#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)", data, re.S)}
if not any("SI_UNIT(.MILLI.,.METRE.)" in e for e in ents.values()):
    sys.exit("expected the STEP file to be in millimetres")

def args(i):
    e = ents[i]; s = e[e.index("(") + 1: e.rindex(")")]
    out, depth, cur, q = [], 0, "", False
    for ch in s:
        if ch == "'": q = not q
        if not q and ch == "(": depth += 1
        if not q and ch == ")": depth -= 1
        if ch == "," and depth == 0 and not q: out.append(cur.strip()); cur = ""
        else: cur += ch
    out.append(cur.strip()); return out
ref = lambda s: int(s[1:])
refs = lambda s: [int(x) for x in re.findall(r"#(\d+)", s)]
nums = lambda s: [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?", s)]

# ---------- rigid transforms (4x4 lists) ----------
def mm(A, B): return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
def eye(): return [[float(i == j) for j in range(4)] for i in range(4)]
def rinv(M):
    o = eye()
    for i in range(3):
        for j in range(3): o[i][j] = M[j][i]
        o[i][3] = -sum(M[k][i] * M[k][3] for k in range(3))
    return o
def unit(v): l = math.sqrt(sum(x * x for x in v)); return [x / l for x in v]
def apply(M, p): return [sum(M[r][k] * p[k] for k in range(3)) + M[r][3] for r in range(3)]
def placement(i):
    a = args(i); o = nums(args(ref(a[1]))[1])
    z = unit(nums(args(ref(a[2]))[1])) if a[2] != "$" else [0.0, 0.0, 1.0]
    x = nums(args(ref(a[3]))[1]) if a[3] != "$" else [1.0, 0.0, 0.0]
    d = sum(x[k] * z[k] for k in range(3)); x = unit([x[k] - z[k] * d for k in range(3)])
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    M = eye()
    for r in range(3): M[r][0], M[r][1], M[r][2], M[r][3] = x[r], y[r], z[r], o[r]
    return M

# ---------- assembly tree ----------
pd_name = {}
for i, e in ents.items():
    if e.startswith("PRODUCT_DEFINITION("):
        pdf = ref(args(i)[2]); pd_name[i] = args(ref(args(pdf)[2]))[1].strip("'")
nauo = {i: (args(i)[1].strip("'"), ref(args(i)[3]), ref(args(i)[4]))
        for i, e in ents.items() if e.startswith("NEXT_ASSEMBLY_USAGE_OCCURRENCE(")}
pds_target = {i: ref(args(i)[2]) for i, e in ents.items() if e.startswith("PRODUCT_DEFINITION_SHAPE(")}
local = {}
for i, e in ents.items():
    if e.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION("):
        rr, pds = [ref(x) for x in args(i)]
        idt = [r for r in refs(ents[rr]) if ents[r].startswith("ITEM_DEFINED_TRANSFORMATION")][0]
        a = args(idt); local[pds_target[pds]] = mm(placement(ref(a[3])), rinv(placement(ref(a[2]))))
children = {}
for k, (_, p, _) in nauo.items(): children.setdefault(p, []).append(k)
root = [r for r in set(pd_name) - {c for (_, _, c) in nauo.values()} if r in children][0]
occ = []
def walk(pd, M):
    for k in children.get(pd, []):
        n, _, c = nauo[k]; W = mm(M, local[k]); occ.append((n, pd_name[c], W, c)); walk(c, W)
walk(root, eye())

# ---------- part geometry (all points of a part's own shape) ----------
pds_of_pd = {v: k for k, v in pds_target.items()}
sdr = {ref(args(i)[0]): ref(args(i)[1]) for i, e in ents.items() if e.startswith("SHAPE_DEFINITION_REPRESENTATION(")}
links = {}
for i, e in ents.items():
    if e.startswith("SHAPE_REPRESENTATION_RELATIONSHIP(") and "TRANSFORMATION" not in e:
        a = args(i); r1, r2 = ref(a[2]), ref(a[3]); links.setdefault(r1, []).append(r2); links.setdefault(r2, []).append(r1)
def points(pd):
    rep = sdr.get(pds_of_pd.get(pd))
    if rep is None: return []
    reps, stack = set(), [rep]
    while stack:
        r = stack.pop()
        if r not in reps: reps.add(r); stack += links.get(r, [])
    pts, seen = [], set()
    stack = [x for r in reps for x in refs(args(r)[1])]
    while stack:
        i = stack.pop()
        if i in seen: continue
        seen.add(i); e = ents[i]
        if e.startswith("CARTESIAN_POINT("): pts.append(nums(args(i)[1])); continue
        if "CONTEXT" in e: continue
        stack += refs(e)
    return pts
def placed(o): return [apply(o[2], p) for p in points(o[3])]
def box(pts): return [min(p[k] for p in pts) for k in range(3)], [max(p[k] for p in pts) for k in range(3)]

vl = [o for o in occ if "VL53L0X" in o[1]]
tcs = [o for o in occ if "TCS" in o[1]]
wheels = [o for o in occ if o[1] == "Wheel"]
if len(vl) != 7 or len(tcs) != 1 or len(wheels) != 4:
    sys.exit(f"expected 7 VL53L0X, 1 TCS and 4 Wheel parts, found {len(vl)}, {len(tcs)}, {len(wheels)}")

# ---------- robot frame ----------
wb = [box(placed(w)) for w in wheels]
floor = min(lo[2] for lo, hi in wb)
axle_y = sorted((lo[1] + hi[1]) / 2 for lo, hi in wb)            # wheel centres along the length
front_axle_end = [(axle_y[0] + axle_y[1]) / 2, (axle_y[2] + axle_y[3]) / 2]
vl_boxes = [(o, box(placed(o))) for o in vl]
cy = lambda b: (b[0][1] + b[1][1]) / 2
ends = sorted(vl_boxes, key=lambda ob: cy(ob[1]))
# the end with two boards facing along the length is the front
low_end = [ob for ob in vl_boxes if (ob[1][1][1] - ob[1][0][1]) < 3 and cy(ob[1]) < sum(front_axle_end) / 2]
front_is_low_y = len(low_end) == 2
centre_y = sum(front_axle_end) / 2
def to_robot(p):  # CAD (x, y, z) -> robot (forward, left, height)
    fwd = (centre_y - p[1]) if front_is_low_y else (p[1] - centre_y)
    left = p[0] if front_is_low_y else -p[0]
    return [fwd, left, p[2] - floor]

sensors = []
for o, (lo, hi) in vl_boxes:
    c = to_robot([(lo[k] + hi[k]) / 2 for k in range(3)])
    thin = min(range(3), key=lambda k: hi[k] - lo[k])            # board normal (CAD axis)
    if thin == 2: sys.exit(f"{o[0]} lies flat; can't tell which way it faces")
    face = "forward" if thin == 1 and c[0] > 0 else "backward" if thin == 1 else "left" if c[1] > 0 else "right"
    sensors.append({"cad_part": o[0], "forward_mm": c[0], "left_mm": c[1], "height_mm": c[2], "faces": face})

def code_id(s):
    f, l, face = s["forward_mm"], s["left_mm"], s["faces"]
    if face == "forward": return 1 if l < 0 else 7
    if face == "backward": return 4
    if face == "right": return 2 if f > 0 else 3
    return 6 if f > 0 else 5
for s in sensors: s["code_sensor"] = code_id(s)
sensors.sort(key=lambda s: [1, 7, 2, 3, 6, 5, 4].index(s["code_sensor"]))
if sorted(s["code_sensor"] for s in sensors) != [1, 2, 3, 4, 5, 6, 7]:
    sys.exit("sensor layout doesn't match the expected 2 front / 2 right / 2 left / 1 back")

tl, th = box(placed(tcs[0]))
colour = to_robot([(tl[k] + th[k]) / 2 for k in range(3)])
allpts = [to_robot(p) for o, (lo, hi) in vl_boxes for p in (lo, hi)]

# Outline seen from above: convex hull of every part at heights that can touch a wall (2-160 mm).
# Parts more than 400 mm across can't be part of the robot (the export has four "Component1"
# reference bodies 570 mm wide) and are left out.
def hull2d(P):
    P = sorted(set((round(x, 1), round(y, 1)) for x, y in P))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]
outline_pts, left_out = [], set()
for o in occ:
    ps = [to_robot(apply(o[2], p)) for p in points(o[3])]
    ps = [p for p in ps if 2 <= p[2] <= 160]
    if not ps: continue
    if max(p[0] for p in ps) - min(p[0] for p in ps) > 400 or max(p[1] for p in ps) - min(p[1] for p in ps) > 400:
        left_out.add(o[1]); continue
    outline_pts += [(p[0], p[1]) for p in ps]
outline = hull2d(outline_pts)
geom = {
    "source": os.path.basename(sys.argv[1]),
    "frame": "origin on the floor under the middle of the wheelbase; forward +, left +, height above floor (mm)",
    "sensors": [{k: (round(v, 1) + 0.0 if isinstance(v, float) else v) for k, v in s.items()} for s in sensors],
    "colour_sensor": {"forward_mm": round(colour[0], 1) + 0.0, "left_mm": round(colour[1], 1) + 0.0, "height_mm": round(colour[2], 1) + 0.0, "faces": "down"},
    "axles_forward_mm": [round(centre_y - y, 1) if front_is_low_y else round(y - centre_y, 1) for y in front_axle_end],
    "sensor_span_mm": {"length": round(max(p[0] for p in allpts) - min(p[0] for p in allpts), 1),
                       "width": round(max(p[1] for p in allpts) - min(p[1] for p in allpts), 1)},
    "outline_mm": [[x, y] for x, y in outline],
    "outline_note": "convex hull seen from above, parts 2-160 mm above the floor; left out: " + ", ".join(sorted(left_out)),
}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_geometry.json")
json.dump(geom, open(out, "w"), indent=2)
print(f"{'code':6} {'cad part':14} {'fwd':>7} {'left':>7} {'height':>7}  faces")
for s in geom["sensors"]:
    print(f"{'m(' + str(s['code_sensor']) + ')':6} {s['cad_part']:14} {s['forward_mm']:7.1f} {s['left_mm']:7.1f} {s['height_mm']:7.1f}  {s['faces']}")
c = geom["colour_sensor"]
print(f"{'colour':6} {tcs[0][0]:14} {c['forward_mm']:7.1f} {c['left_mm']:7.1f} {c['height_mm']:7.1f}  down")
print("axles at forward", geom["axles_forward_mm"], "mm; sensor span", geom["sensor_span_mm"])
print("wrote", out)
