"""Case for 20 rescue kits (10.0-10.2 mm cubes, make_kits.py): a tray with 5 x 4 square pockets and a press-on lid.
No libraries needed:  python cad/rescue_kit/make_case.py  ->  KitCase_tray.stl, KitCase_lid.stl (next to this file)

- Tray: each kit stands in its own pocket. The dividers are lower than the kits (DIV_H), so 1.5 mm of each kit sticks out
  to pinch it; the outer wall is higher (WALL_H), so the kits are below its top and the lid closes over them.
- Lid: a flat plate with a lip underneath that fits inside the tray's outer wall (friction fit, LID_CLEAR per side) and a
  tab on one short side to pull it off. Print it as saved: plate on the bed, lip up. Tray: as saved, no supports.

Each part is a set of overlapping closed boxes; slicers merge them into one part when slicing.
"""
import math, os

COLS, ROWS = 5, 4      # 20 pockets
POCKET = 10.6          # pocket width: a 10.2 mm kit has 0.2 mm each side, a 10.0 mm kit 0.3 mm
DIV = 1.2              # divider thickness
WALL = 1.6             # outer wall thickness
FLOOR = 1.2
DIV_H = 8.5            # divider height above the floor: kits (10.0-10.2) stick out 1.5-1.7 mm to grab
WALL_H = 12.0          # outer wall height above the floor: 1.8 mm above the kits, room for the lid's lip
LID_T = 1.6            # lid plate
LIP_H = 1.5            # lip height (ends 0.3 mm above a 10.2 mm kit)
LIP_T = 1.2            # lip thickness
LID_CLEAR = 0.25       # gap between the lip and the outer wall's inside, per side
TAB = 4.0              # pull tab: the lid sticks out this far past the tray on one short side
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


def tray():
    b = [(0, OUT_X, 0, OUT_Y, 0, FLOOR)]                                            # floor
    top = FLOOR + WALL_H
    b += [(0, WALL + E, 0, OUT_Y, FLOOR - E, top), (OUT_X - WALL - E, OUT_X, 0, OUT_Y, FLOOR - E, top),   # outer walls
          (0, OUT_X, 0, WALL + E, FLOOR - E, top), (0, OUT_X, OUT_Y - WALL - E, OUT_Y, FLOOR - E, top)]
    for i in range(1, COLS):                                                        # dividers between columns
        x = WALL + i * POCKET + (i - 1) * DIV
        b.append((x, x + DIV, WALL - E, OUT_Y - WALL + E, FLOOR - E, FLOOR + DIV_H))
    for j in range(1, ROWS):                                                        # dividers between rows
        y = WALL + j * POCKET + (j - 1) * DIV
        b.append((WALL - E, OUT_X - WALL + E, y, y + DIV, FLOOR - E, FLOOR + DIV_H))
    return boxes_to_tris(b)


def lid():
    b = [(0, OUT_X + TAB, 0, OUT_Y, 0, LID_T)]                                     # plate (+ tab on the +x side)
    x0, x1 = WALL + LID_CLEAR, OUT_X - WALL - LID_CLEAR                            # lip ring, inside the outer wall
    y0, y1 = WALL + LID_CLEAR, OUT_Y - WALL - LID_CLEAR
    z0, z1 = LID_T - E, LID_T + LIP_H
    b += [(x0, x0 + LIP_T, y0, y1, z0, z1), (x1 - LIP_T, x1, y0, y1, z0, z1),
          (x0, x1, y0, y0 + LIP_T, z0, z1), (x0, x1, y1 - LIP_T, y1, z0, z1)]
    return boxes_to_tris(b)


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
t, l = tray(), lid()
save(t, os.path.join(here, "KitCase_tray.stl"))
save(l, os.path.join(here, "KitCase_lid.stl"))
print(f"tray {OUT_X:.1f} x {OUT_Y:.1f} x {FLOOR + WALL_H:.1f} mm, {COLS * ROWS} pockets of {POCKET} mm, open edges {check_closed(t)}")
print(f"lid {OUT_X + TAB:.1f} x {OUT_Y:.1f} x {LID_T + LIP_H:.1f} mm (lip {LIP_H} mm, {LID_CLEAR} mm clearance), open edges {check_closed(l)}")
