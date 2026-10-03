"""Read a Serial log from the real robot and list where it struggled.

  python tools/robot_log.py run1.txt [run2.txt ...]

Copy everything the Serial monitor printed during a run (115200 baud) into a text file. The robot doesn't
know where it really is, so this can't say when it got lost; it counts the moments that usually lead to
it: moves that ended early or against something, obstacle checks, turns that stopped short, walls that
didn't match the map, position corrections, ramps. It also checks the steering for weaving and collects
the [BENCH] lines of bench mode. The physics simulator prints the same lines (run_physics.py --seed N
--verbose), so a real log and a simulated one can be compared.
"""
import collections, re, statistics, sys


# (whole line?, text, what it means)
EVENTS = [
    (True, "obstacle left", "obstacle seen at the start of a move (left)"),
    (True, "obstacle right", "obstacle seen at the start of a move (right)"),
    (False, "obstacle ahead during the move", "obstacle seen during a move"),
    (False, "obstacle ahead: blocking edge", "edge blocked for an obstacle"),
    (False, "one-sided reading while not square", "squared up before deciding about an obstacle"),
    (False, "obstacle avoidance timeout", "obstacle avoidance gave up"),
    (True, "obstacle avoidance", "drove around an obstacle"),
    (False, "tile mismatch detected", "walls didn't match the map"),
    (False, "position corrected", "position corrected (relocalize)"),
    (False, "[FWD] short move", "short move (backed up to the start tile)"),
    (False, "botched turn detected", "turn missed by more than 20 deg"),
    (False, "parallel: timeout", "squaring up on a wall timed out"),
    (True, "black", "black tile seen"),
    (False, "adding ramp to map", "ramp tile added to the map"),
    (False, "short tilt, not a ramp", "tilt that wasn't a ramp (bump?)"),
    (False, "climb timeout", "ramp climb timed out"),
    (False, "map looks finished early", "map looked finished early: blocked edges cleared"),
    (False, "maze fully explored", "exploring finished"),
    (False, "time to return home", "set off home (time)"),
    (False, "back at start", "back at the start tile"),
    (False, "no path found", "no path home"),
    (False, "back on a blue tile", "5 s stop after backing into a blue tile"),
    (False, "checkpoint recorded", "checkpoint recorded"),
]


def summarize(path):
    lines = [l.strip() for l in open(path, encoding="utf-8", errors="replace")]
    c = collections.Counter()
    turn_err, entry_off, tilts, center_err, bench = [], [], [], [], []
    for i, s in enumerate(lines):
        if s == "forwarding":
            c["moves"] += 1
        elif s.startswith("[FWD] exit="):
            c["move ended: " + s[len("[FWD] exit="):]] += 1
        elif s.startswith("[FWD] entry hdg="):
            m = re.search(r"offset=(-?[\d.]+)", s)
            if m:
                off = (float(m.group(1)) + 180) % 360 - 180
                entry_off.append(off)
        elif s.startswith("turn target="):
            m = re.search(r"err=(-?[\d.]+)", s)
            if m:
                turn_err.append(float(m.group(1)))
        elif s.startswith("[CENTER]"):
            m = re.search(r"err=(-?[\d.]+)", s)
            if m:
                center_err.append(float(m.group(1)))
        elif s == "climbing" and i + 1 < len(lines) and re.fullmatch(r"-?\d+", lines[i + 1]):
            tilts.append(int(lines[i + 1]))
        elif s.startswith("[BENCH]"):
            bench.append(s)
        for exact, key, label in EVENTS:
            if (s == key) if exact else (key in s):
                c[label] += 1
                break
    print(f"== {path}")
    for k, v in c.most_common():
        print(f"  {v:5d}  {k}")
    if turn_err:
        big = sum(1 for e in turn_err if e > 5)
        print(f"  turns: {len(turn_err)} checked, median error {statistics.median(turn_err):.1f} deg, {big} more than 5 deg off")
    if entry_off:
        big = sum(1 for e in entry_off if abs(e) > 3)
        print(f"  moves started {statistics.median([abs(e) for e in entry_off]):.1f} deg off the tile direction (median), {big} of {len(entry_off)} more than 3 deg")
    if center_err:
        flips = sum(1 for a, b in zip(center_err, center_err[1:]) if a * b < 0 and abs(a - b) > 20)
        print(f"  steering: {len(center_err)} samples, error median {statistics.median([abs(e) for e in center_err]):.0f}, "
              f"{flips} big swings from one side to the other (many = weaving)")
    if tilts:
        print(f"  ramps: tilt when detected {tilts} (positive = going up)")
    for b in bench:
        print("  " + b)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        summarize(p)
