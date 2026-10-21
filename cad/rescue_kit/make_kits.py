"""Rescue kits (RCJ Rescue Maze 2026, rule 3.7): cubes with a large closed hollow inside for steel balls, dropped
in during a print pause. No libraries needed:
python cad/rescue_kit/make_kits.py  ->  RescueKit.stl (one kit) and RescueKit_plate10.stl (10 kits: 8 to load + 2 spare)

Rules 3.7.3: "Each rescue kit must have a minimum size of 1 cm in each dimension and have a minimum volume of 1 cm3 after
deployment." 3.7.2: it must stay near the victim (not roll or bounce away): a cube with a steel ball inside doesn't roll.
3.7.4: at most 8 kits on the robot. The kit is a little over 1 cm (KIT) because prints often come out 0.1-0.2 mm small;
rule 3.8.6 gives +-10% on all measurements, but don't rely on it. Measure a printed kit: every side must be >= 10.0 mm.
Check that this size still runs through the dispenser before printing all of them.

Printing: flat on a face, no supports (the hollow's roof is a short bridge). Add a pause (Bambu Studio: right-click the
layer slider -> "Add pause"; PrusaSlicer: "Add pause print") at the first layer ABOVE PAUSE_Z_MM printed below, fill each
hollow (one ball up to hollow - CLEAR, or several smaller ones), resume. The script prints PAUSE_Z_MM for the sizes set here.
"""
import math, os

KIT = 10.4         # outer cube edge, mm (rule minimum 10.0)
WALL = 1.2         # wall, floor and roof thickness: 3 lines of a 0.4 mm nozzle, 6 layers of 0.2 mm
CLEAR = 0.4        # a ball must be this much smaller than the hollow (prints shrink holes)
GAP = 4.0          # between kits on the plate
PLATE = (5, 2)     # 10 kits


def kit_tris(x0, y0):
    tris = []

    def quad(a, b, c, d, out=True):  # counter-clockwise seen from outside (out=False: a hollow, faces point inward)
        if out: tris.append((a, b, c)); tris.append((a, c, d))
        else: tris.append((a, c, b)); tris.append((a, d, c))

    def box(xa, xb, ya, yb, za, zb, out=True):
        P = lambda x, y, z: (x, y, z)
        quad(P(xa, ya, za), P(xa, yb, za), P(xb, yb, za), P(xb, ya, za), out)   # bottom (normal -z)
        quad(P(xa, ya, zb), P(xb, ya, zb), P(xb, yb, zb), P(xa, yb, zb), out)   # top
        quad(P(xa, ya, za), P(xb, ya, za), P(xb, ya, zb), P(xa, ya, zb), out)   # front (-y)
        quad(P(xa, yb, za), P(xa, yb, zb), P(xb, yb, zb), P(xb, yb, za), out)   # back (+y)
        quad(P(xa, ya, za), P(xa, ya, zb), P(xa, yb, zb), P(xa, yb, za), out)   # left (-x)
        quad(P(xb, ya, za), P(xb, yb, za), P(xb, yb, zb), P(xb, ya, zb), out)   # right (+x)

    w = WALL                         # wall thickness
    h = KIT - 2 * WALL               # hollow edge
    box(x0, x0 + KIT, y0, y0 + KIT, 0, KIT)                                  # outside
    box(x0 + w, x0 + w + h, y0 + w, y0 + w + h, w, w + h, out=False)         # closed hollow
    return tris, w, h


def check_closed(tris):
    from collections import Counter
    k = lambda p: tuple(round(c, 4) for c in p)
    cnt = Counter()
    for a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            cnt[(k(p), k(q))] += 1
    return sum(1 for (p, q), n in cnt.items() if cnt.get((q, p), 0) != n)


def volume(tris):
    return sum((a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6 for a, b, c in tris)


def save(tris, path):
    with open(path, "w") as f:
        f.write("solid kit\n")
        for a, b, c in tris:
            ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]; vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
            n = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx); l = math.sqrt(sum(x*x for x in n)) or 1
            f.write(f" facet normal {n[0]/l:.5f} {n[1]/l:.5f} {n[2]/l:.5f}\n  outer loop\n")
            for p in (a, b, c): f.write(f"   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("  endloop\n endfacet\n")
        f.write("endsolid kit\n")


here = os.path.dirname(os.path.abspath(__file__))
one, w, h = kit_tris(0, 0)
plate = []
for i in range(PLATE[0]):
    for j in range(PLATE[1]):
        plate += kit_tris(i * (KIT + GAP), j * (KIT + GAP))[0]
save(one, os.path.join(here, "RescueKit.stl"))
save(plate, os.path.join(here, "RescueKit_plate10.stl"))
print(f"kit {KIT} mm cube, walls {w:.1f} mm, hollow {h:.1f} mm (one ball up to {h - CLEAR:.1f} mm, or several smaller); "
      f"plastic {volume(one)/1000:.2f} cm3 (outside volume {KIT**3/1000:.2f} cm3, rule minimum 1.00); "
      f"open edges {check_closed(one)} / plate {check_closed(plate)}")
print(f"PAUSE_Z_MM = {w + h:.2f}: pause at the first layer above this height, fill each hollow, resume")
