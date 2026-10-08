"""Does the robot drive straight within a tile? For every ordinary move: how far the true heading swings
(largest minus smallest heading during the move, after the first 100 mm) and how often it swings back and
forth (direction changes of the heading by more than 1 deg), plus the sideways position at the end.
  python tools/sim/physics/weave.py main 40       # git ref (or WORK), number of comp fields
"""
import concurrent.futures, math, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, lostcause as lc, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
extra = [x for s in sys.argv[3:] for x in ("--set", s)]


def one(exe, seed):
    tr = os.path.join(tempfile.gettempdir(), f"wv-{seed}-{os.getpid()}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr, *extra], capture_output=True)
    head, frames, logs, events, result = lc.read_trace(tr)
    os.remove(tr)
    starts = [t for t, l in logs if l == "forwarding"]
    ends = [t for t, l in logs if l == "[FWD] exit=normal"]
    out = []
    for ms in starts:
        me = next((t for t in ends if t > ms), None)
        if me is None or me - ms > 6000:
            continue
        mv = [f for f in frames if ms <= f["t"] <= me]
        if len(mv) < 10:
            continue
        x0, y0 = mv[0]["x"], mv[0]["y"]
        target = round(mv[0]["h"] / 90) * 90
        hs = [((f["h"] - target + 180) % 360) - 180 for f in mv if math.hypot(f["x"] - x0, f["y"] - y0) > 100]
        if len(hs) < 5:
            continue
        swings, last_ext, direction = 0, hs[0], 0
        for h in hs[1:]:
            if direction >= 0 and h < last_ext - 1:
                if direction > 0: swings += 1
                direction, last_ext = -1, h
            elif direction <= 0 and h > last_ext + 1:
                if direction < 0: swings += 1
                direction, last_ext = 1, h
            elif (direction > 0 and h > last_ext) or (direction < 0 and h < last_ext):
                last_ext = h
        out.append((max(hs) - min(hs), swings))
    return out


exe = ab.build_ref(ref)
try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        moves = [m for r in pool.map(lambda s: one(exe, s), range(1, n + 1)) for m in r]
finally:
    os.remove(exe)
amp = sorted(a for a, s in moves)
N = len(moves)
print(f"{ref} {' '.join(sys.argv[3:])}: {N} moves in {n} comp fields")
print(f"  heading swing during a move: median {amp[N // 2]:.1f} deg, 90% {amp[9 * N // 10]:.1f} deg")
print(f"  moves that swing back and forth 2+ times: {sum(s >= 2 for a, s in moves) / N:.0%}, 4+ times: {sum(s >= 4 for a, s in moves) / N:.0%}")
