"""Paired A/B test: two versions of the firmware on the same simulated fields.

  python tools/sim/physics/ab.py main relocalize                 # two git branches/commits
  python tools/sim/physics/ab.py HEAD WORK                       # last commit vs the working copy of Main/
  python tools/sim/physics/ab.py main WORK --types comp,flat --runs 100
  python tools/sim/physics/ab.py main WORK --set gyroDriftSigmaDegPerMin=2

Both versions run on the same seeds (same fields, same noise), so the difference per field is
the code's doing. The +- is a 95% interval on the mean difference (score, and percentage points of
time lost and of runs back home): a change is only clearly better when the whole interval is on the
good side of zero. Judge changes on `comp` (competition-size) fields, 600 of them for a final decision.
"""
import argparse, concurrent.futures, io, os, subprocess, sys, tarfile, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_physics as rp


def build_ref(ref):
    exe = os.path.join(HERE, "build", f"ab-{ref.replace('/', '_')}.exe")
    os.makedirs(os.path.dirname(exe), exist_ok=True)
    if ref == "WORK":                       # the working copy of Main/
        rp.build(exe=exe)
        return exe
    data = subprocess.run(["git", "-C", rp.REPO, "archive", "--format=tar", ref, "Main"], capture_output=True, check=True).stdout
    tmp = tempfile.mkdtemp(prefix="ab_")
    tarfile.open(fileobj=io.BytesIO(data)).extractall(tmp)
    repo = rp.REPO
    try:
        rp.REPO = tmp
        rp.build(exe=exe)
    finally:
        rp.REPO = repo
    return exe


def ci(xs):
    n = len(xs); m = sum(xs) / n
    return m, 1.96 * (sum((x - m) ** 2 for x in xs) / (n - 1)) ** .5 / n ** .5


def summary(X, Y):
    """One line: A -> B on the same fields, with the paired change and its 95% interval for the score,
    the time spent lost and getting back home (the sim has no victims, so the last two matter more
    in a real run than the score shows)."""
    mean = lambda xs: sum(xs) / len(xs)
    pair = lambda key: ci([float(y.get(key, 0)) - float(x.get(key, 0)) for x, y in zip(X, Y)])
    (ms, hs), (ml, hl), (mh, hh) = pair("score"), pair("lost_fraction"), pair("home")
    restarts = lambda R: mean([r.get("lops", 0) for r in R])
    return (f"score {mean([x.get('score', 0) for x in X]):6.1f} -> {mean([y.get('score', 0) for y in Y]):6.1f}  ({ms:+.1f} +-{hs:.1f})"
            f"  time lost {mean([x.get('lost_fraction', 0) for x in X]):4.0%} -> {mean([y.get('lost_fraction', 0) for y in Y]):4.0%}  ({100 * ml:+.0f} +-{100 * hl:.0f} pts)"
            f"  home {mean([x.get('home', False) for x in X]):4.0%} -> {mean([y.get('home', False) for y in Y]):4.0%}  ({100 * mh:+.0f} +-{100 * hh:.0f} pts)"
            f"  explored {mean([x.get('coverage', 0) for x in X]):4.0%} -> {mean([y.get('coverage', 0) for y in Y]):4.0%}"
            f"  restarts/run {restarts(X):.2f} -> {restarts(Y):.2f}"
            f"  crashed {sum(1 for r in Y if 'score' not in r)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--types", default="comp", help="field types, comma-separated (default comp)")
    ap.add_argument("--runs", type=int, default=200, help="fields per type (default 200)")
    ap.add_argument("--start", type=int, default=1, help="first seed (default 1); e.g. --start 601 for fields not used before")
    ap.add_argument("--set", action="append", default=[], help="simulator parameter for both, e.g. gyroDriftSigmaDegPerMin=2")
    args = ap.parse_args()
    types = args.types.split(",")
    extra = [x for s in args.set for x in ("--set", s)]
    exes = {args.a: build_ref(args.a), args.b: build_ref(args.b)}
    seeds = range(args.start, args.start + args.runs)
    jobs = [(v, sc, s) for v in (args.a, args.b) for sc in types for s in seeds]
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count() or 4) as pool:
        res = dict(zip(jobs, pool.map(lambda j: rp.run_one(j[1], j[2], extra, exes[j[0]]), jobs)))
    mean = lambda xs: sum(xs) / len(xs)
    print(f"{args.a} -> {args.b}, {args.runs} fields per type (seeds {args.start}-{args.start + args.runs - 1}), same fields for both")
    for sc in types + (["ALL"] if len(types) > 1 else []):
        keys = [(t, s) for t in (types if sc == "ALL" else [sc]) for s in seeds]
        X = [res[(args.a,) + k] for k in keys]; Y = [res[(args.b,) + k] for k in keys]
        print(f"  {sc:8s} " + summary(X, Y))
    for e in exes.values():
        os.remove(e)


if __name__ == "__main__":
    main()
