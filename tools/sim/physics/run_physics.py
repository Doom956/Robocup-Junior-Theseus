"""Build the physics simulator against the current Main/ code and run it on many fields.

Usage (from the repo folder):
  python tools/sim/physics/run_physics.py                          # 100 fields of each type, realistic robot
  python tools/sim/physics/run_physics.py --scenario ramp --runs 300
  python tools/sim/physics/run_physics.py --ideal                  # no noise, drift or motor differences
  python tools/sim/physics/run_physics.py --set motorDeadband=0.05 --set motorMaxSpeed=300
  python tools/sim/physics/run_physics.py --random                 # a new block of fields
  python tools/sim/physics/run_physics.py --scenario loops --seed 7 --verbose   # one field, robot's Serial output
"""
import argparse, collections, concurrent.futures, json, os, random, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
EXE = os.path.join(HERE, "physics.exe" if os.name == "nt" else "physics")
SCENARIOS = ["flat", "loops", "big", "ramp", "bigramp"]


def firmware_copy(defines, patches=()):
    """Copy Main/ to a temp folder with some #define values changed and/or text replaced
    (the real Main/ is untouched)."""
    import re, shutil, tempfile
    src = os.path.join(REPO, "Main")
    if not defines and not patches:
        return src
    dst = os.path.join(tempfile.mkdtemp(prefix="fw_"), "Main")
    shutil.copytree(src, dst)
    for d in defines:
        name, value = d.split("=", 1)
        hits = 0
        for f in os.listdir(dst):
            p = os.path.join(dst, f)
            if not f.endswith((".cpp", ".h")):
                continue
            text = open(p, encoding="utf-8").read()
            new, n = re.subn(r"^(\s*#define\s+" + re.escape(name) + r"\s+)\S+", lambda m: m.group(1) + value, text, flags=re.M)
            if n:
                open(p, "w", encoding="utf-8").write(new); hits += n
        if not hits:
            sys.exit(f"--fw {d}: no '#define {name}' found in Main/")
    for spec in patches:  # FILE|old text|new text
        fname, old, new = spec.split("|", 2)
        p = os.path.join(dst, fname)
        text = open(p, encoding="utf-8").read()
        if text.count(old) != 1:
            sys.exit(f"--patch: '{old}' found {text.count(old)} times in {fname} (needs exactly 1)")
        open(p, "w", encoding="utf-8").write(text.replace(old, new))
    return dst


def build(defines=(), patches=()):
    gpp = "g++"
    tool = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "toolchain-gccmingw32", "bin")
    env = dict(os.environ)
    if os.path.isdir(tool):
        env["PATH"] = tool + os.pathsep + env["PATH"]  # g++ needs its own folder on PATH to find its helpers
        gpp = os.path.join(tool, "g++.exe")             # Windows doesn't search the new PATH for the program itself
    main = firmware_copy(defines, patches)
    srcs = [os.path.join(main, f) for f in sorted(os.listdir(main)) if f.endswith(".cpp")]
    cmd = [gpp, "-std=gnu++14", "-O2", "-static", "-I", os.path.join(HERE, "hw"), "-I", main,
           os.path.join(HERE, "physics.cpp"), *srcs, "-o", EXE]
    r = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("build failed:\n" + r.stderr[-4000:])


def run_one(scenario, seed, extra):
    r = subprocess.run([EXE, "--scenario", scenario, "--seed", str(seed), *extra], capture_output=True, text=True)
    line = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "{}"
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return {"end": "crashed", "scenario": scenario, "seed": seed}


def pct(v):
    return f"{100 * v:.0f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="all")
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--random", action="store_true")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--ideal", action="store_true")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--fw", action="append", default=[], help="try a #define change, e.g. --fw ROBOT_WIDTH_MM=183 (simulation only)")
    ap.add_argument("--patch", action="append", default=[], help="try a code change: 'FILE|old text|new text' (simulation only)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--trace")
    ap.add_argument("--no-build", action="store_true")
    a = ap.parse_args()
    if not a.no_build:
        build(a.fw, a.patch)
    extra = (["--ideal"] if a.ideal else []) + [x for s in a.set for x in ("--set", s)]
    if a.selftest:
        sys.exit(subprocess.run([EXE, "--selftest", *extra]).returncode)

    if a.seed is not None:  # one field
        sc = SCENARIOS[1] if a.scenario == "all" else a.scenario
        cmd = [EXE, "--scenario", sc, "--seed", str(a.seed), *extra] + (["--verbose"] if a.verbose else []) + (["--trace", a.trace] if a.trace else [])
        sys.exit(subprocess.run(cmd).returncode)

    start = random.randint(1, 1_000_000) if a.random else a.start
    print(f"robot: {'ideal' if a.ideal else 'realistic'}{' ' + ' '.join(a.set) if a.set else ''}"
          f"{'   code changes: ' + ' '.join(a.fw) if a.fw else ''}{'   patches: ' + str(len(a.patch)) if a.patch else ''}"
          f"   seeds {start} to {start + a.runs - 1}")
    for sc in (SCENARIOS if a.scenario == "all" else [a.scenario]):
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
            res = list(pool.map(lambda s: run_one(sc, s, extra), range(start, start + a.runs)))
        n = len(res)
        home = sum(r.get("home", False) for r in res)
        full = sum(r.get("coverage", 0) >= 0.999 for r in res)
        cov = sum(r.get("coverage", 0) for r in res) / n
        walls = sum(r.get("map_walls", 0) for r in res) / n
        lost = sum(r.get("lost_fraction", 0) > 0.2 for r in res)
        lops = sum(r.get("lops", 0) for r in res) / n
        contact = sum(r.get("wall_contact_s", 0) for r in res) / n
        ends = collections.Counter(r.get("end", "?") for r in res)
        why = collections.Counter(x.strip() for r in res for x in r.get("lop_reasons", "").split(";") if x.strip())
        print(f"{sc:8s} home {home}/{n} ({pct(home / n)})  explored everything {pct(full / n)}  avg coverage {pct(cov)}  "
              f"map walls {pct(walls)}  lost >20% of the time {pct(lost / n)}  restarts/run {lops:.2f}  wall contact {contact:.0f} s/run")
        print(f"         ended: {dict(ends)}   restarts: {dict(why)}")
        bad = [r for r in res if not r.get("home")][:5]
        if bad:
            print("         replay: " + "  ".join(f"--scenario {sc} --seed {r.get('seed')}" for r in bad))


if __name__ == "__main__":
    main()
