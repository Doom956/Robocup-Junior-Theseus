"""Case for 20 rescue kits (10.0-10.2 mm cubes, make_kits.py) with a sliding lid, tray and lid in one STL to print
together. No libraries needed:  python cad/rescue_kit/make_case.py  ->  KitCase.stl (next to this file)

- Tray: 5 x 4 square pockets. The dividers are lower than the kits (DIV_H), so 1.5 mm of each kit sticks out to pinch it.
- Sliding lid: grooves along both long walls just under the top edge hold the lid. The lid slides in from the +x end, where
  the end wall stops at the groove's floor; the -x end wall is full height and stops it. A ridge near the lid's open end
  is for your thumb.
- Print as saved: tray upright, lid flat beside it (ridge up), no supports. The 1 mm lip over each groove prints as a short
  overhang.

Each part is a set of overlapping closed boxes; slicers merge them into one part when slicing.
"""
import math, os

COLS, ROWS = 5, 4      # 20 pockets
POCKET = 10.6          # pocket width: a 10.2 mm kit has 0.2 mm each side, a 10.0 mm kit 0.3 mm
DIV = 1.2              # divider thickness
WALL = 2.4             # outer wall thickness (1.4 mm left behind the groove)
FLOOR = 1.2
DIV_H = 8.5            # divider height above the floor: kits (10.0-10.2) stick out 1.5-1.7 mm to grab
GROOVE_Z = 10.6        # groove floor above the tray floor: 0.4 mm above a 10.2 mm kit
GROOVE_H = 2.0         # groove height (lid LID_T plus play)
GROOVE_D = 1.0         # groove depth into each long wall
TOP_LIP = 1.2          # wall above the groove (holds the lid down)
LID_T = 1.6            # lid thickness
LID_CLEAR = 0.3        # lid to groove back (each side) and to the stop wall
RIDGE = (1.2, 1.0)     # thumb ridge on the lid: width, height
GAP = 6.0              # between tray and lid on the print bed
E = 0.03               # overlap so the boxes fuse


def boxes_to_tris(boxes):
    tris = []

    def quad(a, b, c, d):
        tris.append((a, b, c)); tris.append((a, c, d))
    for xa, xb, ya, yb, za, zb in boxes:
        P = lambda x, y, z: (x, y, z)
        quad(P(xa, ya, za), P(xa, yb, za), P(xb, yb, za), P(xb, ya, za))
        quad(P(xa, ya, zb), P(xb, ya, zb), P(xb, yb, zb), P(xa, yb, zb))
        quad(P(xa, ya, za), P(xb, ya, za), P(xb, ya, zb), P(xa, ya, zb))
        quad(P(xa, yb, za), P(xa, yb, zb), P(xb, yb, zb), P(xb, yb, za))
        quad(P(xa, ya, za), P(xa, ya, zb), P(xa, yb, zb), P(xa, yb, za))
        quad(P(xb, ya, za), P(xb, yb, za), P(xb, yb, zb), P(xb, ya, zb))
    return tris


IN_X = COLS * POCKET + (COLS - 1) * DIV
IN_Y = ROWS * POCKET + (ROWS - 1) * DIV
OUT_X, OUT_Y = IN_X + 2 * WALL, IN_Y + 2 * WALL
G0 = FLOOR + GROOVE_Z          # groove floor (z)
G1 = G0 + GROOVE_H             # groove top
TOP = G1 + TOP_LIP             # wall top


def tray():
    b = [(0, OUT_X, 0, OUT_Y, 0, FLOOR)]                                            # floor
    for ya, yb, ga, gb in ((0, WALL, 0, WALL - GROOVE_D), (OUT_Y - WALL, OUT_Y, OUT_Y - WALL + GROOVE_D, OUT_Y)):
        b.append((0, OUT_X, ya, yb, FLOOR - E, G0 + E))                              # long wall below the groove
        b.append((0, OUT_X, ga, gb, G0 - E, G1 + E))                                 # behind the groove
        b.append((0, OUT_X, ya, yb, G1 - E, TOP))                                    # lip above the groove
    b.append((0, WALL, WALL - E, OUT_Y - WALL + E, FLOOR - E, TOP))                  # -x end wall: full height (stop)
    b.append((OUT_X - WALL, OUT_X, WALL - E, OUT_Y - WALL + E, FLOOR - E, G0))       # +x end wall: up to the groove (entry)
    for i in range(1, COLS):                                                         # dividers
        x = WALL + i * POCKET + (i - 1) * DIV
        b.append((x, x + DIV, WALL - E, OUT_Y - WALL + E, FLOOR - E, FLOOR + DIV_H))
    for j in range(1, ROWS):
        y = WALL + j * POCKET + (j - 1) * DIV
        b.append((WALL - E, OUT_X - WALL + E, y, y + DIV, FLOOR - E, FLOOR + DIV_H))
    return b


def lid(y0):
    # In place it runs from the stop wall (x = WALL + clearance) to the tray's +x end, and GROOVE_D - clearance into each
    # groove. On the bed it lies beside the tray at y0.
    lx = OUT_X - WALL - LID_CLEAR
    ly = IN_Y + 2 * (GROOVE_D - LID_CLEAR)
    b = [(0, lx, y0, y0 + ly, 0, LID_T)]
    rx = lx - 5.0                                                                    # thumb ridge 5 mm from the open end
    b.append((rx - RIDGE[0], rx, y0 + 6, y0 + ly - 6, LID_T - E, LID_T + RIDGE[1]))
    return b, lx, ly


def check_closed(tris):
    from collections import Counter
    k = lambda p: tuple(round(c, 4) for c in p)
    cnt = Counter()
    for a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            cnt[(k(p), k(q))] += 1
    return sum(1 for (p, q), n in cnt.items() if cnt.get((q, p), 0) != n)


def save(tris, path):
    with open(path, "w") as f:
        f.write("solid case\n")
        for a, b, c in tris:
            ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]; vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
            n = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx); l = math.sqrt(sum(x*x for x in n)) or 1
            f.write(f" facet normal {n[0]/l:.5f} {n[1]/l:.5f} {n[2]/l:.5f}\n  outer loop\n")
            for p in (a, b, c): f.write(f"   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("  endloop\n endfacet\n")
        f.write("endsolid case\n")


here = os.path.dirname(os.path.abspath(__file__))
lid_boxes, lx, ly = lid(OUT_Y + GAP)
tris = boxes_to_tris(tray() + lid_boxes)
save(tris, os.path.join(here, "KitCase.stl"))
print(f"tray {OUT_X:.1f} x {OUT_Y:.1f} x {TOP:.1f} mm, {COLS * ROWS} pockets of {POCKET} mm; "
      f"lid {lx:.1f} x {ly:.1f} x {LID_T} mm (+ thumb ridge) in grooves {GROOVE_H} x {GROOVE_D} mm; open edges {check_closed(tris)}")
print(f"kits top out at {FLOOR + 10.2:.1f} mm, the lid sits at {G0:.1f}-{G0 + LID_T:.1f} mm")
