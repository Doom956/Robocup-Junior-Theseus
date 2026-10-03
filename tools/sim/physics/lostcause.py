"""Where does the robot first get lost? Records many runs and sorts them by what happened just before
the code's map position first went wrong.

  python tools/sim/physics/lostcause.py main                 # 200 comp fields, the code on branch main
  python tools/sim/physics/lostcause.py WORK --runs 600      # the working copy of Main/
  python tools/sim/physics/lostcause.py origin/robot-test --show 5   # also print the log before the first 5 losses
  python tools/sim/physics/lostcause.py WORK --scenario comp --seed 12   # one field, with its log

The position is checked the way the simulator scores "time lost": each time the code starts reading a
tile (SENSE_TILE), its tile must be the tile the robot is really on. The move between the last right
check and the first wrong one is sorted into:

  false obstacle alarm   obstacle avoidance started with no obstacle ahead (usually a wall end seen by
                         one front sensor with the robot turned or off-centre)
  real obstacle          obstacle avoidance around a real obstacle that ended in the wrong tile
  move: pushed obstacle  an ordinary move into an obstacle the code didn't see; the wheels kept turning,
                         so the encoders counted a tile the robot didn't drive
  move: scraped wall     an ordinary move with the robot pressed against a wall (same effect)
  ramp, black tile, short move, restart, ...

It also counts how runs ended, why the referee restarted them, and whether the code's tile stayed right
while it drove home (RETURN never reads walls, so errors there aren't in "time lost").
"""
import argparse, collections, concurrent.futures, json, math, os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_physics as rp
import ab

STATES = ["SENSE_TILE", "CENTERING", "UPDATE_MAP", "PLAN_NEXT", "VICTIM_DETECT", "EXECUTE_MOVE",
          "BOTCHED_TURN_RECOVERY", "BOTCHED_FWD_RECOVERY", "BACKPEDAL", "PAUSE", "RETURN"]
SENSE, RETURN = 0, 10
RAMP = 4
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # N E S W
NOISY = ("distance travelled", "[CENTER]")  # printed every loop of fwd(); not needed here


def read_trace(path):
    head, frames, logs, events, result = None, [], [], [], None
    with open(path, encoding="utf-8", errors="replace") as fh:
        for i, line in enumerate(fh):
            if i == 0:
                head = json.loads(line)
            elif line.startswith('{"t"'):
                frames.append(json.loads(line))
            elif line.startswith('{"log"'):
                o = json.loads(line)
                if not o["log"].startswith(NOISY):
                    logs.append((o["t"], o["log"]))
            elif line.startswith('{"event"'):
                o = json.loads(line)
                events.append((o["t"], o.get("why", o["event"])))
            elif line.startswith('{"result"'):
                result = json.loads(line)["result"]
    return head, frames, logs, events, result


def analyse(path):
    head, frames, logs, events, result = read_trace(path)
    F = head["field"]
    W, H, sx, sy, tiles, obstacles = F["W"], F["H"], F["sx"], F["sy"], F["tiles"], F["obstacles"]
    size = head["map"]["size"]
    level0 = tiles[sy * W + sx][1]

    def truth(fr):  # the code's (x, y, floor) the robot should have, or None on a ramp / off the field
        tx, ty = math.floor(fr["x"] / 300), math.floor(fr["y"] / 300)
        if not (0 <= tx < W and 0 <= ty < H) or tiles[ty * W + tx][0] == RAMP:
            return None
        return tx - sx + size // 2, ty - sy + size // 2, head["map"]["start"] + tiles[ty * W + tx][1] - level0

    def wrong(fr):
        t = truth(fr)
        return t is not None and (fr["mx"], fr["my"], fr["f"]) != t

    def near_obstacle(fr, extra=5):
        return any(math.hypot(ox - fr["x"], oy - fr["y"]) < 140 + r + extra for ox, oy, r in obstacles)

    def obstacle_ahead(fr):
        h = math.radians(fr["h"]); hx, hy = math.sin(h), math.cos(h)
        for ox, oy, r in obstacles:
            vx, vy = ox - fr["x"], oy - fr["y"]
            if 0 < vx * hx + vy * hy < 100 + 90 + r + 40 and abs(-vx * hy + vy * hx) < 150:
                return True
        return False

    def pose(fr):  # sideways and along offset from the tile centre (mm), heading off the nearest compass direction (deg)
        cx, cy = (math.floor(fr["x"] / 300) + .5) * 300, (math.floor(fr["y"] / 300) + .5) * 300
        k = round(fr["h"] / 90) % 4
        (fx, fy), (rx, ry) = DIRS[k], DIRS[(k + 1) % 4]
        return {"side_mm": round((fr["x"] - cx) * rx + (fr["y"] - cy) * ry), "along_mm": round((fr["x"] - cx) * fx + (fr["y"] - cy) * fy),
                "heading_off_deg": round((fr["h"] - k * 90 + 180) % 360 - 180, 1)}

    out = {"seed": head["seed"], "score": result.get("score") if result else None, "end": result.get("end") if result else "crashed",
           "home": result.get("home") if result else False, "lost_fraction": result.get("lost_fraction") if result else None,
           "restarts": [x.strip() for x in (result or {}).get("lop_reasons", "").split(";") if x.strip()]}

    # the way home: is the code's tile right at each stop between RETURN moves (the last frame of each
    # stretch of standing still)? Already wrong at the first stop = lost before it set off for home.
    stops, k = [], 0
    while k < len(frames):
        if frames[k]["s"] == RETURN and not any(frames[k]["pw"]):
            j = k
            while j + 1 < len(frames) and frames[j + 1]["s"] == RETURN and not any(frames[j + 1]["pw"]):
                j += 1
            if truth(frames[j]) is not None:
                stops.append(wrong(frames[j]))
            k = j + 1
        else:
            k += 1
    out["return_stops"] = len(stops)
    out["return"] = "no return" if not stops else "lost before" if stops[0] else "went wrong" if any(stops) else "right"

    # the position checks: last frame of every stretch of SENSE_TILE
    checks = [k for k, f in enumerate(frames) if f["s"] == SENSE and (k + 1 == len(frames) or frames[k + 1]["s"] != SENSE)]
    prev_ok, first_bad = None, None
    for k in checks:
        if truth(frames[k]) is None:
            continue
        if wrong(frames[k]):
            first_bad = k
            break
        prev_ok = k
    if first_bad is None:
        out["cause"] = "never lost"
        return out
    t0 = frames[prev_ok]["t"] if prev_ok is not None else 0
    fr = frames[first_bad]
    t1 = fr["t"]
    ep_logs = [l for l in logs if t0 < l[0] <= t1]
    ep_frames = [f for f in frames if t0 <= f["t"] <= t1]
    texts = [l[1] for l in ep_logs]
    tr = truth(fr)
    d = fr["d"]
    dx, dy = fr["mx"] - tr[0], fr["my"] - tr[1]
    out.update(t_s=t1 / 1000, tiles_ahead=dx * DIRS[d][0] + dy * DIRS[d][1], tiles_right=dx * DIRS[(d + 1) % 4][0] + dy * DIRS[(d + 1) % 4][1],
               floor_off=fr["f"] - tr[2], log=texts[-15:])

    def at(t):
        return min(ep_frames or frames, key=lambda f: abs(f["t"] - t))

    move_start = next((l[0] for l in ep_logs if l[1] == "forwarding"), None)
    if move_start is not None:
        out["start"] = pose(at(move_start))
    if any(e for e in events if t0 < e[0] <= t1):
        out["cause"] = "restart (" + next(e[1] for e in events if t0 < e[0] <= t1) + ")"
    elif any("1 section of the ramp climbed" in s or "adding ramp to map" in s or s == "short tilt, not a ramp (bump/debris)" for s in texts):
        out["cause"] = "ramp"
    elif any(s in ("obstacle avoidance",) for s in texts):
        t = next(l[0] for l in ep_logs if l[1] == "obstacle avoidance")
        out["cause"] = ("real obstacle" if obstacle_ahead(at(t)) else "false obstacle alarm") + \
                       (" (avoidance gave up)" if any("avoidance timeout" in s for s in texts) else "")
    elif any(s == "black" or s.startswith("black during") for s in texts):
        out["cause"] = "black tile"
    elif any("short move" in s for s in texts):
        out["cause"] = "short move"
    elif any("obstacle ahead during the move" in s for s in texts):
        out["cause"] = "stopped for an obstacle during the move"
    elif move_start is None:
        out["cause"] = "no forward move"
    else:
        moving = [f for f in ep_frames if f["t"] >= move_start]
        exits = [s[len("[FWD] exit="):] for s in texts if s.startswith("[FWD] exit=")]
        how = {"emergency-front": "emergency stop", "timeout": "timed out"}.get(exits[-1] if exits else "", "")
        if any(f["c"] and near_obstacle(f) for f in moving):
            out["cause"] = "move: pushed obstacle"
        elif any(f["c"] for f in moving):
            out["cause"] = "move: scraped wall"
        else:
            out["cause"] = "move"
        if how:
            out["cause"] += " (" + how + ")"
    return out


def run_one(exe, scenario, seed, sets, keep):
    folder = keep or tempfile.gettempdir()
    trace = os.path.join(folder, f"lost-{scenario}-{seed}-{os.getpid()}.jsonl")
    extra = [x for s in sets for x in ("--set", s)]
    try:
        subprocess.run([exe, "--scenario", scenario, "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", trace, *extra],
                       capture_output=True, text=True, timeout=600)
        return analyse(trace)
    except Exception as e:  # a crashed or stuck run: count it, don't stop the batch
        return {"seed": seed, "cause": f"simulator problem ({type(e).__name__})", "end": "crashed", "restarts": [], "return": "no return"}
    finally:
        if not keep and os.path.exists(trace):
            os.remove(trace)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ref", help="git branch / commit, or WORK for the working copy of Main/")
    ap.add_argument("--scenario", default="comp")
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--seed", type=int, help="one field: print what happened before it first got lost")
    ap.add_argument("--set", action="append", default=[], help="simulator parameter, e.g. gyroDriftSigmaDegPerMin=2")
    ap.add_argument("--show", type=int, default=0, help="print the Serial log before the first N losses")
    ap.add_argument("--json", help="write every run's result to this file")
    ap.add_argument("--keep", help="keep the recordings in this folder (open them with viewer.html)")
    a = ap.parse_args()
    exe = ab.build_ref(a.ref)
    seeds = [a.seed] if a.seed is not None else list(range(a.start, a.start + a.runs))
    if a.keep:
        os.makedirs(a.keep, exist_ok=True)
    try:
        with concurrent.futures.ThreadPoolExecutor(os.cpu_count() or 4) as pool:
            res = list(pool.map(lambda s: run_one(exe, a.scenario, s, a.set, a.keep), seeds))
    finally:
        os.remove(exe)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)
    n = len(res)
    lost = [r for r in res if r["cause"] != "never lost"]
    scores = [r.get("score") or 0 for r in res]
    print(f"{a.ref}: {n} {a.scenario} field{'s' if n > 1 else ''}, mean score {sum(scores) / n:.1f}")
    print(f"first wrong position, {len(lost)} of {n} runs ({100 * len(lost) / n:.0f}%) got lost at some point:")
    for cause, k in collections.Counter(r["cause"] for r in lost).most_common():
        print(f"  {k:5d}  {100 * k / max(1, len(lost)):4.0f}%  {cause}")
    print("how runs ended: " + ", ".join(f"{k} {v}" for k, v in collections.Counter(r["end"] for r in res).most_common()))
    restarts = collections.Counter(x for r in res for x in r["restarts"])
    print(f"restarts: {sum(restarts.values()) / n:.2f} per run" + ("  (" + ", ".join(f"{k} {v}" for k, v in restarts.most_common()) + ")" if restarts else ""))
    ret = collections.Counter(r["return"] for r in res)
    print(f"driving home (RETURN): {ret['right']} stayed right, {ret['went wrong']} went wrong on the way, "
          f"{ret['lost before']} were already lost when they set off, {ret['no return']} never started home")
    for r in (lost if a.seed is None else res)[:max(a.show, 1 if a.seed is not None else 0)]:
        print(f"\nseed {r['seed']}: {r['cause']} at {r.get('t_s', 0):.1f} s, code {r.get('tiles_ahead')} tile(s) ahead / "
              f"{r.get('tiles_right')} right of the robot; move started {r.get('start')}")
        for line in r.get("log", []):
            print("    | " + line)


if __name__ == "__main__":
    main()
