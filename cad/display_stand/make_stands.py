"""Two stands for the Waveshare 2inch LCD Module (ST7789, 240x320), landscape, glued on. No libraries needed:
python cad/display_stand/make_stands.py  ->  DisplayStand_tilted.stl, DisplayStand_flat.stl (next to this file).

Each part is a set of overlapping closed solids (plate pieces, posts, walls, gussets, foot); slicers (Bambu Studio,
PrusaSlicer, Cura) merge them into one part when slicing.

Display dimensions (Waveshare 2inch_LCD_Module_2D_Drawing.pdf): board 58 x 35 mm, mounting holes 53 x 30 mm apart
(2.5 mm from each edge, ~2.8 mm = M2.5); PCB 1.6 mm + display 2.2 mm; on the back the PH2.0 connector is 5.6 mm tall,
7.5-27.4 mm from the top edge and 5-12 mm from one side edge (the side opposite the 8 solder holes, which are 2.4 mm
from the other edge).

Mounting: the display screws onto the four posts from the front (M2.5, e.g. the screws that come with it, without the
brass standoffs). The 2.2 mm holes go right through post and plate, so any screw length works; posts are 8 mm tall, so
the connector clears the plate.
- tilted: wires go out the back through a window in the plate on each side (behind the connector and the solder
  holes), above the foot.
- flat: a tray with a solid bottom (for glue) and a low wall around the board; wires go out through a slot in the
  back wall, raised above the floor.
"""
import math, os

TILT_DEG = 60.0             # tilted stand: screen angle from horizontal (90 = upright)
PLATE_T = 3.0               # plate / floor thickness
BOARD_W, BOARD_H = 58.0, 35.0
HOLE_DX, HOLE_DY = 53.0, 30.0
MARGIN = 1.0                # clearance around the board
POST_H = 8.0                # board sits this far in front of the plate
POST_OD, POST_ID = 5.5, 2.2 # 2.2 mm hole for an M2.5 screw in plastic, right through
WIRE_U0, WIRE_U1 = 15.0, 27.5   # tilted: wire windows, |x| from the centre (connector at 17-24, solder holes ~26.6)
WIRE_HALF_V = 11.5              # tilted: window height +-11.5 mm around the board's middle (posts start at 12.25)
FOOT_T = 3.0
FOOT_W = 64.0               # tilted: foot width (x)
FOOT_Y0, FOOT_Y1 = -20.0, 14.0   # tilted: foot depth, behind / in front of the plate's bottom edge
GUSSET_X, GUSSET_T, GUSSET_BACK_Y = 8.0, 3.0, -16.0   # tilted: two gussets behind the plate, clear of the windows
GUSSET_UP = 0.7             # tilted: gusset reaches this fraction up the plate
WALL_T = 2.0                # flat: wall thickness around the board
WALL_TOP = POST_H + 1.6     # flat: wall height above the floor (= top of the board's PCB)
SLOT_W = 24.0               # flat: wire slot width in the back wall
SLOT_FLOOR = 3.0            # flat: slot starts this far above the floor
SEG = 48


def build(tilt_deg):
    tris = []

    def quad(a, b, c, d):  # counter-clockwise seen from outside
        tris.append((a, b, c)); tris.append((a, c, d))

    def hexa(P):  # 8 corners: P[i] for (x0/x1, y0/y1, z0/z1) as bits (x, y, z)
        c = lambda x, y, z: P[x + 2 * y + 4 * z]
        quad(c(0,0,0), c(0,1,0), c(1,1,0), c(1,0,0)); quad(c(0,0,1), c(1,0,1), c(1,1,1), c(0,1,1))
        quad(c(0,0,0), c(1,0,0), c(1,0,1), c(0,0,1)); quad(c(0,1,0), c(0,1,1), c(1,1,1), c(1,1,0))
        quad(c(0,0,0), c(0,0,1), c(0,1,1), c(0,1,0)); quad(c(1,0,0), c(1,1,0), c(1,1,1), c(1,0,1))

    th = math.radians(tilt_deg)
    tilted = tilt_deg > 0
    V = (0.0, -math.cos(th), math.sin(th))   # up the plate (leans back)
    W = (0.0, math.sin(th), math.cos(th))    # out of the screen face
    O = (0.0, 0.0, FOOT_T if tilted else 0.0)

    def pf(u, v, w):  # plate frame -> world
        return (O[0] + u, O[1] + v * V[1] + w * W[1], O[2] + v * V[2] + w * W[2])

    def pbox(u0, u1, v0, v1, w0, w1):  # box in the plate frame (u x v = -w, so w is listed high-to-low)
        hexa([pf(u, v, w) for w in (w1, w0) for v in (v0, v1) for u in (u0, u1)])

    def wbox(x0, x1, y0, y1, z0, z1):
        hexa([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)])

    def ptube(u, v, w0, w1, ro, ri):
        for i in range(SEG):
            a0, a1 = 2 * math.pi * i / SEG, 2 * math.pi * (i + 1) / SEG
            o0, o1 = (u + ro * math.cos(a0), v + ro * math.sin(a0)), (u + ro * math.cos(a1), v + ro * math.sin(a1))
            i0, i1 = (u + ri * math.cos(a0), v + ri * math.sin(a0)), (u + ri * math.cos(a1), v + ri * math.sin(a1))
            q = lambda p, w: pf(p[0], p[1], w)
            quad(q(o0, w0), q(o0, w1), q(o1, w1), q(o1, w0))   # outer wall
            quad(q(i0, w0), q(i1, w0), q(i1, w1), q(i0, w1))   # inner wall
            quad(q(i0, w1), q(i1, w1), q(o1, w1), q(o0, w1))   # top ring
            quad(q(i0, w0), q(o0, w0), q(o1, w0), q(i1, w0))   # bottom ring

    def prism_x(x0, x1, pts):  # triangle in (y,z), extruded along x
        (ay, az), (by, bz), (cy, cz) = pts
        if (by - ay) * (cz - az) - (bz - az) * (cy - ay) < 0:
            (by, bz), (cy, cz) = (cy, cz), (by, bz)
        A0, B0, C0, A1, B1, C1 = (x0, ay, az), (x0, by, bz), (x0, cy, cz), (x1, ay, az), (x1, by, bz), (x1, cy, cz)
        tris.append((A0, C0, B0)); tris.append((A1, B1, C1))
        quad(A0, B0, B1, A1); quad(B0, C0, C1, B1); quad(C0, A0, A1, C1)

    def plate_with_holes(u0, u1, v0, v1, w0, w1, holes):
        """Plate in the plate frame minus rectangular holes (u0,u1,v0,v1), built from grid cells."""
        us = sorted({u0, u1, *[h[0] for h in holes], *[h[1] for h in holes]})
        vs = sorted({v0, v1, *[h[2] for h in holes], *[h[3] for h in holes]})
        e = 0.03  # cells overlap slightly so they fuse
        for a, b in zip(us, us[1:]):
            for c, d in zip(vs, vs[1:]):
                mu, mv = (a + b) / 2, (c + d) / 2
                if any(h[0] < mu < h[1] and h[2] < mv < h[3] for h in holes):
                    continue
                pbox(max(u0, a - e), min(u1, b + e), max(v0, c - e), min(v1, d + e), w0, w1)

    L = BOARD_H + 2 * MARGIN       # board area (v), board centred at vm
    vm = L / 2
    hw = BOARD_W / 2 + MARGIN
    posts = [(sx * HOLE_DX / 2, vm + sy * HOLE_DY / 2) for sx in (-1, 1) for sy in (-1, 1)]
    sq = 1.9  # square cut under each post (corners at 2.69 mm < post radius 2.75): the post's tube closes it
    post_cuts = [(u - sq, u + sq, v - sq, v + sq) for u, v in posts]

    if tilted:
        v0 = -2.0  # plate sinks 2 mm into the foot
        windows = [(a, b, vm - WIRE_HALF_V, vm + WIRE_HALF_V) for s in (-1, 1)
                   for a, b in [sorted((s * WIRE_U0, s * WIRE_U1))]]
        plate_with_holes(-hw, hw, v0, L, 0, PLATE_T, windows + post_cuts)
        for u, v in posts:
            ptube(u, v, 0.0, PLATE_T + POST_H, POST_OD / 2, POST_ID / 2)
        wbox(-FOOT_W / 2, FOOT_W / 2, FOOT_Y0, FOOT_Y1, 0, FOOT_T)
        B = pf(0, GUSSET_UP * L, 0.5)
        for gx in (-GUSSET_X, GUSSET_X):
            prism_x(gx - GUSSET_T / 2, gx + GUSSET_T / 2,
                    [(0.5, FOOT_T - 1.0), (B[1], B[2]), (GUSSET_BACK_Y, FOOT_T - 1.0)])
    else:
        ow, v0, v1 = hw + WALL_T, -WALL_T, L + WALL_T   # outer footprint
        plate_with_holes(-ow, ow, v0, v1, 0, PLATE_T, post_cuts)                     # floor
        for u, v in posts:
            ptube(u, v, 0.0, PLATE_T + POST_H, POST_OD / 2, POST_ID / 2)
        top = PLATE_T + WALL_TOP
        e = 0.03
        pbox(-ow, -hw + e, v0, v1, PLATE_T - e, top)                                # left wall
        pbox(hw - e, ow, v0, v1, PLATE_T - e, top)                                  # right wall
        pbox(-hw, hw, L - e, v1, PLATE_T - e, top)                                  # front wall
        # back wall (v < 0) with the wire slot, raised SLOT_FLOOR above the floor
        pbox(-hw, -SLOT_W / 2, v0, e, PLATE_T - e, top)
        pbox(SLOT_W / 2, hw, v0, e, PLATE_T - e, top)
        pbox(-SLOT_W / 2 - e, SLOT_W / 2 + e, v0, e, PLATE_T - e, PLATE_T + SLOT_FLOOR)
    return tris


def check_closed(tris):
    """Every edge of every closed piece must be used once in each direction; report how many aren't."""
    from collections import Counter
    k = lambda p: tuple(round(c, 4) for c in p)
    cnt = Counter()
    for a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            cnt[(k(p), k(q))] += 1
    return sum(1 for (p, q), n in cnt.items() if cnt.get((q, p), 0) != n)


def save(tris, path):
    def nrm(a, b, c):
        ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]; vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
        n = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx); l = math.sqrt(sum(x*x for x in n)) or 1
        return tuple(x / l for x in n)
    with open(path, "w") as f:
        f.write("solid stand\n")
        for a, b, c in tris:
            n = nrm(a, b, c)
            f.write(f" facet normal {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n  outer loop\n")
            for p in (a, b, c):
                f.write(f"   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("  endloop\n endfacet\n")
        f.write("endsolid stand\n")


def volume(tris):  # signed volume: positive if the normals point out
    return sum((a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6
               for a, b, c in tris)


here = os.path.dirname(os.path.abspath(__file__))
for name, tilt in (("DisplayStand_tilted", TILT_DEG), ("DisplayStand_flat", 0.0)):
    t = build(tilt)
    save(t, os.path.join(here, name + ".stl"))
    zs = [p[2] for tri in t for p in tri]; ys = [p[1] for tri in t for p in tri]; xs = [p[0] for tri in t for p in tri]
    print(f"{name}.stl: {len(t)} triangles, {max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} x {max(zs)-min(zs):.1f} mm, "
          f"open edges {check_closed(t)}, volume {volume(t)/1000:.2f} cm3")
