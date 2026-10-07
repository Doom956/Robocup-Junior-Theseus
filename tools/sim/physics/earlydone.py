"""Runs where the code said 'maze fully explored' with tiles left: at that moment, what in the code's map
hides each real way into the unexplored part?
For every real open edge from a visited tile to a reachable unvisited tile, look at the code's map for
that edge (code tile = real tile shifted by the start offset, on the right floor):
  false wall     the code has a wall there
  blocked        the code has the edge blocked (obstacle / failed moves)
  black          the code marked the neighbour BLACK
  code tile not visited / neighbour visited in code: the code's map is shifted (position was wrong)
  ramp           the edge leads onto a ramp tile
  open in code   nothing in the code's map blocks it (then the planner shouldn't have stopped)
  python tools/sim/physics/earlydone.py origin/robot-test 300
"""
import collections, concurrent.futures, json, math, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ab, lostcause as lc, run_physics as rp

ref = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 300
DX, DY = [0, 1, 0, -1], [1, 0, -1, 0]
FLOOR, BLACK, BLUE, SILVER, RAMP, SOLID = range(6)
# tileCode bits (physics.cpp): walls 0-3, visited 4, type 5-6, elevate 7, descend 8, obstacle 9-12, discovered 13
# code types (MazeTile.h) are read from the bits 5-6 (2 bits)


def one(exe, seed):
    tr = os.path.join(tempfile.gettempdir(), f"ed-{seed}-{os.getpid()}.jsonl")
    subprocess.run([exe, "--scenario", "comp", "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", tr], capture_output=True)
    head, frames, logs, events, result = lc.read_trace(tr)
    os.remove(tr)
    t_done = next((t for t, l in logs if l == "maze fully explored"), None)
    if t_done is None or result["coverage"] >= 0.95:
        return None
    F = head["field"]; W, H, sx, sy = F["W"], F["H"], F["sx"], F["sy"]
    tiles = F["tiles"]; size = head["map"]["size"]; start_floor = head["map"]["start"]
    lv0 = tiles[sy * W + sx][1]
    # code map as of t_done
    code = {}
    for f in frames:
        if f["t"] > t_done:
            break
        for fl, x, y, c in f.get("m", []):
            code[(fl, x, y)] = c
    # real visited tiles up to t_done: majority tile is approximated by the centre tile of each frame
    visited = set()
    for f in frames:
        if f["t"] > t_done:
            break
        tx, ty = math.floor(f["x"] / 300), math.floor(f["y"] / 300)
        if 0 <= tx < W and 0 <= ty < H:
            visited.add(ty * W + tx)
    # reachable
    seen = {sy * W + sx}; q = [(sx, sy)]
    while q:
        x, y = q.pop()
        for d in range(4):
            if tiles[y * W + x][2] >> d & 1:
                continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < W and 0 <= ny < H and ny * W + nx not in seen and tiles[ny * W + nx][0] not in (BLACK, SOLID):
                seen.add(ny * W + nx); q.append((nx, ny))
    unvisited = [i for i in seen if i not in visited and tiles[i][0] != RAMP]
    last = max((f for f in frames if f["t"] <= t_done), key=lambda f: f["t"])
    pos = (last["f"], last["mx"], last["my"])

    def reach_in_code(fl, gx, gy):
        # can the planner get from the code position to code tile (fl, gx, gy), through visited tiles,
        # crossing floors on ramp tiles (marked on both floors)?
        def bfs(use_obst):
            s = {pos}; q = [pos]
            while q:
                z, x, y = q.pop()
                if (z, x, y) == (fl, gx, gy):
                    return True
                c = code.get((z, x, y), 0)
                nxt = []
                if c >> 7 & 3:
                    for z2 in (z - 1, z + 1):
                        if code.get((z2, x, y), 0) >> 7 & 3:
                            nxt.append((z2, x, y))
                for d in range(4):
                    nx, ny = x + DX[d], y + DY[d]
                    cn = code.get((z, nx, ny), 0)
                    if c >> d & 1 or cn >> ((d + 2) % 4) & 1 or not (cn >> 4 & 1) or (cn >> 5 & 3) == 3:
                        continue
                    if use_obst and (c >> (9 + d) & 1 or cn >> (9 + (d + 2) % 4) & 1):
                        continue
                    nxt.append((z, nx, ny))
                for nb in nxt:
                    if nb not in s:
                        s.add(nb); q.append(nb)
            return False
        if bfs(True):
            return "reachable in the code's map (?)" if fl == pos[0] else "reachable across the ramp in the code's map (?)"
        if bfs(False):
            return "cut off by blocked edges"
        return "cut off by walls / ramp in the code's map"
    why = collections.Counter()
    for i in visited:
        if i not in seen or tiles[i][0] == RAMP:
            continue
        x, y = i % W, i // W
        for d in range(4):
            if tiles[i][2] >> d & 1:
                continue
            nx, ny = x + DX[d], y + DY[d]
            j = ny * W + nx
            if not (0 <= nx < W and 0 <= ny < H) or j in visited or j not in seen:
                continue
            if tiles[j][0] == RAMP:
                why["leads onto the ramp"] += 1
                continue
            fl = start_floor + tiles[i][1] - lv0
            cx, cy = x - sx + size // 2, y - sy + size // 2
            c = code.get((fl, cx, cy), 0)
            cn = code.get((start_floor + tiles[j][1] - lv0, cx + DX[d], cy + DY[d]), 0)
            if not (c >> 4 & 1):
                why["code's tile not visited (map shifted)"] += 1
            elif c >> d & 1:
                why["false wall in the code's map"] += 1
            elif c >> (9 + d) & 1:
                why["edge blocked (obstacle / failed moves)"] += 1
            elif (cn >> 5 & 3) == 3:
                why["neighbour marked BLACK"] += 1
            elif cn >> 4 & 1:
                why["neighbour visited in code (map shifted)"] += 1
            else:
                why["open, " + reach_in_code(fl, cx, cy)] += 1
    return {"seed": seed, "t": t_done / 1000, "unvisited": len(unvisited), "why": dict(why), "coverage": result["coverage"],
            "retries": sum(1 for t, l in logs if l.startswith("map looks finished early") and t <= t_done)}


exe = ab.build_ref(ref)
try:
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        out = [o for o in pool.map(lambda s: one(exe, s), range(1, n + 1)) if o]
finally:
    os.remove(exe)
print(f"{ref}: {len(out)} of {n} runs said 'maze fully explored' with coverage < 95% (median at {sorted(o['t'] for o in out)[len(out)//2]:.0f} s)")
tot = collections.Counter()
for o in out:
    tot.update(o["why"])
print("real ways into the unexplored part, and what hid each one in the code's map:")
for k, v in tot.most_common():
    print(f"  {v:4d}  {k}")
runs = collections.Counter(max(o["why"], key=o["why"].get) if o["why"] else "no real way in (black / ramp only)" for o in out)
print("main reason per run:", dict(runs))
for o in out:
    if o["why"] and max(o["why"], key=o["why"].get) in ("open, frontier on the other floor", "leads onto the ramp"):
        print("  seed", o["seed"], "at", o["t"], "s", o["why"], "retries", o["retries"])
print("retries already used:", collections.Counter(o["retries"] for o in out))
