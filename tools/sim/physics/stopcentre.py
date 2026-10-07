"""Where along the tile does the robot stop? For every ordinary move: the robot's position along the way it
faces, relative to the middle of the tile it is really on (negative = short of the middle, toward the back),
when fwd() ends ("[FWD] exit=normal") and after centreAlong() ("reading walls").
  python tools/sim/physics/stopcentre.py origin/robot-test 40     # git ref (or WORK), number of comp fields
"""
import concurrent.futures, math, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, lostcause as lc, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def along(f):
    d = round(f["h"] / 90) % 4
    fx, fy = DIRS[d]
    cx, cy = (math.floor(f["x"] / 300) + .5) * 300, (math.floor(f["y"] / 300) + .5) * 300
    return (f["x"] - cx) * fx + (f["y"] - cy) * fy


def one(exe, seed):
    tr = os.path.join(tempfile.gettempdir(), f"sc-{seed}-{os.getpid()}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr], capture_output=True)
    head, frames, logs, events, result = lc.read_trace(tr)
    os.remove(tr)
    out_fwd, out_read = [], []
    k = 0
    after_move = False
    for t, l in logs:
        while k + 1 < len(frames) and frames[k + 1]["t"] <= t:
            k += 1
        if l == "[FWD] exit=normal":
            out_fwd.append(along(frames[k])); after_move = True
        elif l == "reading walls" and after_move:
            out_read.append(along(frames[k])); after_move = False
    return out_fwd, out_read


def summary(name, xs):
    xs = sorted(xs)
    m = len(xs)
    return (f"  {name:28s} {m:5d} stops: median {xs[m // 2]:+5.0f} mm, middle 80% {xs[m // 10]:+5.0f} .. {xs[9 * m // 10]:+5.0f}, "
            f"more than 30 mm short {sum(x < -30 for x in xs) / m:4.0%}, more than 30 mm past {sum(x > 30 for x in xs) / m:4.0%}")


exe = ab.build_ref(ref)
try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        res = list(pool.map(lambda s: one(exe, s), range(1, n + 1)))
finally:
    os.remove(exe)
print(f"{ref}, {n} comp fields (negative = short of the tile's middle)")
print(summary("when fwd() ends", [x for a, b in res for x in a]))
print(summary("after centreAlong()", [x for a, b in res for x in b]))
