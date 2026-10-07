"""Where does the time go before RETURN? Splits the exploring time (start to RETURN or end) into activities
from the Serial log: driving (forwarding -> [FWD] exit), turning ([TURN] -> finished turning), squaring up
(paralleling -> paralleled / parallel: ...), centring along, blue 5 s stops, obstacle avoidance, restarts
(PAUSE), and the rest (delays, reading walls, planning). Also new tiles per minute.
  python tools/sim/physics/timeprof.py origin/robot-test 150
"""
import collections, concurrent.futures, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, lostcause as lc, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 100


def one(exe, seed):
    tr = os.path.join(tempfile.gettempdir(), f"tp-{seed}-{os.getpid()}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr], capture_output=True)
    head, frames, logs, events, result = lc.read_trace(tr)
    os.remove(tr)
    t_end = next((f["t"] for f in frames if f["s"] == 10), frames[-1]["t"])  # first RETURN frame
    # activity per frame time from log intervals
    act = {}
    opens = {"forwarding": "driving", "[TURN]": "turning", "paralleling": "squaring up (parallel)", "centring along": "centring along",
             "obstacle avoidance": "obstacle avoidance", "back on a blue tile": "blue 5 s stop", "[FWD] on blue": "blue 5 s stop"}
    intervals = []
    cur = None
    for t, l in logs:
        if t > t_end:
            break
        key = next((v for k, v in opens.items() if l.startswith(k)), None)
        if key == "driving" and cur and cur[0] == "obstacle avoidance":
            continue  # avoidance's own closing drive counts as avoidance
        if key:
            if cur:
                intervals.append((cur[0], cur[1], t))
            cur = (key, t)
            continue
        if cur and ((cur[0] == "driving" and l.startswith("stop- end of fwd")) or (cur[0] == "turning" and l.startswith("finished turning"))
                    or (cur[0] == "squaring up (parallel)" and (l.startswith("paralleled") or l.startswith("parallel:")))
                    or (cur[0] == "centring along" and l == "reading walls") or (cur[0] == "obstacle avoidance" and l.startswith("[FWD] exit"))):
            intervals.append((cur[0], cur[1], t)); cur = None
    tot = collections.Counter()
    for k, a, b in intervals:
        tot[k] += (b - a) / 1000
    # blue stops in finishTileMove: delay(5000) after a blue tile (no log): count frames standing on blue in EXECUTE_MOVE
    paused = sum(50 for f in frames if f["t"] <= t_end and f["s"] == 9) / 1000
    tot["restart (PAUSE)"] = paused
    blue = 0
    F = head["field"]; W = F["W"]
    import math
    still = 0
    for a, b in zip(frames, frames[1:]):
        if b["t"] > t_end:
            break
        tx, ty = math.floor(b["x"] / 300), math.floor(b["y"] / 300)
        if 0 <= tx < W and 0 <= ty < F["H"] and F["tiles"][ty * W + tx][0] == 2 and b["pw"] == [0, 0, 0, 0] and b["s"] == 5:
            blue += (b["t"] - a["t"]) / 1000
    tot["blue 5 s stop"] += blue
    tot["other (delays, reading walls, planning)"] = t_end / 1000 - sum(tot.values())
    return {"t_end": t_end / 1000, "tot": tot, "tiles": result["tiles"], "coverage": result["coverage"]}


exe = ab.build_ref(ref)
try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        out = list(pool.map(lambda s: one(exe, s), range(1, n + 1)))
finally:
    os.remove(exe)
T = sum(o["t_end"] for o in out)
print(f"{ref}, {n} comp fields: exploring {T / n:.0f} s per run on average")
tot = collections.Counter()
for o in out:
    tot.update(o["tot"])
for k, v in tot.most_common():
    print(f"  {v / n:6.1f} s/run  {100 * v / T:4.1f}%  {k}")
