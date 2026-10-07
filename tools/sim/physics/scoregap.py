"""Where do the missing points go? For each field: the most a run could score there (RCJ 2026 navigation
points, as physics.cpp scores them) minus what it scored, split into
  unexplored   blue tiles not scored (30 + 10 reliability + 10 exit), checkpoints, ramps, speed bumps not reached
  not home     exit bonus for what it did score, lost because it didn't end on the start tile
  restarts     15 per lack of progress (from the reliability bonus)
  revisits     10 per extra visit of a blue tile
and the unexplored part by how the run ended.
  python tools/sim/physics/scoregap.py origin/robot-test 600        # git ref (or WORK), fields, first seed
"""
import collections, concurrent.futures, json, os, re, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 600
start = int(sys.argv[3]) if len(sys.argv) > 3 else 1
DX, DY = [0, 1, 0, -1], [1, 0, -1, 0]
FLOOR, BLACK, BLUE, SILVER, RAMP, SOLID = range(6)


def one(exe, seed):
    tr = os.path.join(tempfile.gettempdir(), f"sg-{seed}-{os.getpid()}-{ref.replace('/', '_')}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr], capture_output=True)
    with open(tr, encoding="utf-8", errors="replace") as fh:
        lines = fh.readlines()
    os.remove(tr)
    head = json.loads(lines[0])
    res = json.loads(lines[-1])["result"]
    F = head["field"]; W, H = F["W"], F["H"]
    tiles = F["tiles"]
    # reachable tiles from the start (no walls, not black/solid), as world.h reach()
    seen = {F["sy"] * W + F["sx"]}
    q = [(F["sx"], F["sy"])]
    while q:
        x, y = q.pop()
        for d in range(4):
            if tiles[y * W + x][2] >> d & 1:
                continue
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < W and 0 <= ny < H) or (ny * W + nx) in seen:
                continue
            if tiles[ny * W + nx][0] in (BLACK, SOLID):
                continue
            seen.add(ny * W + nx); q.append((nx, ny))
    B = sum(1 for i in seen if tiles[i][0] == BLUE)
    C = sum(1 for i in seen if tiles[i][0] == SILVER)
    R = 1 if any(tiles[i][0] == RAMP for i in seen) else 0
    bumps = {b[0] for b in F["bumps"]}
    S = sum(1 for i in seen if i in bumps)
    m = re.match(r"blue (\d+) \((\d+) tiles\), checkpoints (\d+), ramps (\d+), speed bumps (\d+), reliability (\d+), exit (\d+)", res["score_parts"])
    blue_pts, sbv, cp, rp_, bp, rel, ex = map(int, m.groups())
    cp //= 10; srn = rp_ // 10; bumps_done = bp // 5
    home = res["home"]
    gap = {
        "unexplored": 50 * (B - sbv) + 10 * (C - cp) + 15 * (R - srn) + 5 * (S - bumps_done),
        "not home": 0 if home else 10 * sbv + 5 * srn,
        "restarts": 10 * sbv - rel,
        "revisits": 30 * sbv - blue_pts,
    }
    best = 50 * B + 10 * C + 15 * R + 5 * S  # blue 30 + reliability 10 + exit 10; ramp 10 + exit 5
    return {"seed": seed, "score": res["score"], "best": best, "gap": gap, "end": res["end"], "home": home,
            "coverage": res["coverage"], "lops": res["lops"], "lop_reasons": res.get("lop_reasons", ""),
            "lost": res["lost_fraction"], "check": best - res["score"] - sum(gap.values())}


exe = ab.build_ref(ref)
try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        out = list(pool.map(lambda s: one(exe, s), range(start, start + n)))
finally:
    os.remove(exe)
json.dump(out, open(os.path.join(tempfile.gettempdir(), f"gap_{ref.replace('/', '_')}.json"), "w"))
N = len(out)
mean = lambda xs: sum(xs) / len(xs)
print(f"{ref}: {N} comp fields: best possible {mean([o['best'] for o in out]):.1f}, scored {mean([o['score'] for o in out]):.1f}"
      f"  (bookkeeping check {max(abs(o['check']) for o in out)})")
for k in ["unexplored", "not home", "restarts", "revisits"]:
    print(f"  {k:11s} -{mean([o['gap'][k] for o in out]):5.1f} per run")
print("unexplored points by how the run ended:")
by = collections.defaultdict(list)
for o in out:
    by[o["end"] + (" (home)" if o["home"] else " (wrong tile)" if o["end"] == "home" else "")].append(o)
for k, os_ in sorted(by.items(), key=lambda kv: -sum(o["gap"]["unexplored"] for o in kv[1])):
    print(f"  {k:20s} {len(os_):4d} runs  unexplored -{sum(o['gap']['unexplored'] for o in os_) / N:5.1f}/run  not home -{sum(o['gap']['not home'] for o in os_) / N:5.1f}/run"
          f"  coverage {mean([o['coverage'] for o in os_]):.0%}  lost {mean([o['lost'] for o in os_]):.0%}")
