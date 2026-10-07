"""The firmware's heading error (its corrected heading minus the true heading), read from the
'[FWD] entry hdg=' and 'turn target=..., actual=' lines. The offset = firmware - raw gyro changes only in
resyncToNearestCardinal() (parallel(), after 'paralleled'); each change is attributed to the parallel()
call it happened in, and judged by whether it made the error smaller or bigger.
  python tools/sim/physics/hdgerr.py origin/robot-test 100      # git ref (or WORK), number of comp fields
"""
import collections, concurrent.futures, os, re, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, lostcause as lc, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 100
exe = ab.build_ref(ref)
wrap = lambda a: (a + 180) % 360 - 180


def one(seed):
    tr = os.path.join(tempfile.gettempdir(), f"he-{seed}-{os.getpid()}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr], capture_output=True)
    head, frames, logs, events, result = lc.read_trace(tr)
    os.remove(tr)
    k = 0
    pts = []  # (t, offset = firmware - raw, error = firmware - true)
    for t, l in logs:
        m = re.match(r"\[FWD\] entry hdg=([\d.]+)", l) or re.match(r"turn target=[\d.]+, actual=([\d.]+)", l)
        if not m:
            continue
        while k + 1 < len(frames) and frames[k + 1]["t"] <= t:
            k += 1
        f = frames[k]
        fw = float(m.group(1))
        pts.append((t, wrap(fw - f["g"]), wrap(fw - f["h"])))
    # offset changes between consecutive points -> what parallel() saw in between
    changes = []
    for (t0, o0, e0), (t1, o1, e1) in zip(pts, pts[1:]):
        if abs(wrap(o1 - o0)) > 0.8:
            between = [l for t, l in logs if t0 < t <= t1 and (l.startswith("parallel") or l.startswith("paralleled"))]
            changes.append((wrap(o1 - o0), e0, e1, "; ".join(dict.fromkeys(between))[:80]))
    errs = [abs(e) for t, o, e in pts]
    return seed, errs, changes


try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        res = list(pool.map(one, range(1, n + 1)))
finally:
    os.remove(exe)
allerr = sorted(e for s, es, c in res for e in es)
N = len(allerr)
print(f"{N} heading readings at move/turn checks in {n} runs: |firmware - true| median {allerr[N//2]:.1f}, 90% {allerr[9*N//10]:.1f}, "
      f"over 5 deg {sum(e > 5 for e in allerr) / N:.0%}, over 8 deg {sum(e > 8 for e in allerr) / N:.0%}")
ch = [c for s, es, cs in res for c in cs]
better = sum(abs(e1) < abs(e0) - 0.5 for d, e0, e1, w in ch)
worse = sum(abs(e1) > abs(e0) + 0.5 for d, e0, e1, w in ch)
print(f"re-syncs: {len(ch)} ({len(ch)/n:.1f}/run): made the error smaller {better}, bigger {worse}")
big = [c for c in ch if abs(c[2]) > 5 and abs(c[2]) > abs(c[1]) + 0.5]
print(f"re-syncs that left the heading more than 5 deg wrong: {len(big)}")
for d, e0, e1, w in big[:12]:
    print(f"   shift {d:+.1f}: error {e0:+.1f} -> {e1:+.1f}   [{w}]")
