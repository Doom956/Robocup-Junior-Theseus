"""Batch dashboard for the physics simulator.

  python tools/sim/physics/run_physics.py --serve

Starts a small web server on this PC (only reachable from this PC) and opens dashboard.html in the
browser. The page asks this server to run batches of fields; the server builds the firmware from Main/
(again whenever a file in Main/ changes), runs the fields in parallel and sends the results back. "Watch"
on any run records that run and opens it in the replay viewer. Ctrl+C in the terminal stops the server.
"""
import concurrent.futures, glob, hashlib, http.server, json, os, random, subprocess, sys, tempfile, threading, time, urllib.parse, webbrowser

import run_physics as rp

BUILD_DIR = os.path.join(rp.HERE, "build")
TRACE_DIR = tempfile.mkdtemp(prefix="physics_replays_")
CPUS = os.cpu_count() or 4


def source_stamp():
    """Changes whenever any file the simulator is built from changes."""
    files = sorted(glob.glob(os.path.join(rp.REPO, "Main", "*.*")) + glob.glob(os.path.join(rp.HERE, "hw", "**", "*.h"), recursive=True)
                   + [os.path.join(rp.HERE, "physics.cpp"), os.path.join(rp.HERE, "world.h")])
    h = hashlib.sha1()
    for f in files:
        st = os.stat(f)
        h.update(f"{f}|{st.st_mtime_ns}|{st.st_size}".encode())
    return h.hexdigest()[:10]


class Builder:
    def __init__(self):
        self.lock = threading.Lock()
        self.built = {}       # key -> (exe, time built)
        self.last = None      # what the page shows about the last build

    def exe_for(self, defines):
        """The simulator built from the current Main/ with these #define changes (built once, then reused)."""
        with self.lock:
            stamp = source_stamp()
            key = stamp + "-" + hashlib.sha1("|".join(sorted(defines)).encode()).hexdigest()[:6]
            if key not in self.built:
                os.makedirs(BUILD_DIR, exist_ok=True)
                for old in glob.glob(os.path.join(BUILD_DIR, "physics-*.exe")):  # builds of older code
                    if stamp not in old:
                        try:
                            os.remove(old)
                        except OSError:
                            pass
                exe = os.path.join(BUILD_DIR, f"physics-{key}.exe")
                t0 = time.time()
                rp.build(defines, (), exe=exe)
                self.built[key] = (exe, time.time())
                self.last = {"at": time.strftime("%H:%M:%S"), "seconds": round(time.time() - t0, 1), "defines": list(defines), "stamp": stamp}
            return self.built[key][0]


builder = Builder()


class Batch:
    def __init__(self, bid, cfg):
        self.id, self.cfg = bid, cfg
        self.results, self.lock = [], threading.Lock()
        self.phase, self.error, self.stop = "building", None, False
        self.started, self.ended = time.time(), None
        scs = rp.SCENARIOS if cfg["scenario"] == "all" else [cfg["scenario"]]
        # seed-major order, so a batch stopped early still covers every field type
        self.tasks = [(sc, s) for s in range(cfg["start"], cfg["start"] + cfg["runs"]) for sc in scs]
        self.total = len(self.tasks)

    def extra(self):
        return (["--ideal"] if self.cfg["ideal"] else []) + [x for s in self.cfg["sets"] for x in ("--set", s)]

    def run(self):
        try:
            exe = builder.exe_for(self.cfg["fw"])
            self.phase = "running"
            extra = self.extra()

            def one(task):
                if self.stop:
                    return None
                r = rp.run_one(task[0], task[1], extra, exe)
                r.setdefault("scenario", task[0])
                r.setdefault("seed", task[1])
                return r

            with concurrent.futures.ThreadPoolExecutor(max_workers=CPUS) as pool:
                for fut in concurrent.futures.as_completed([pool.submit(one, t) for t in self.tasks]):
                    r = fut.result()
                    if r is not None:
                        with self.lock:
                            self.results.append(r)
        except rp.BuildError as e:
            self.error = "The firmware didn't compile:\n" + str(e)
        except Exception as e:  # shown on the page instead of a silent stop
            self.error = f"{type(e).__name__}: {e}"
        finally:
            self.phase = "stopped" if self.stop and not self.error else "error" if self.error else "done"
            self.ended = time.time()

    def info(self, since=0):
        with self.lock:
            res = self.results[since:]
            done = len(self.results)
        return {"id": self.id, "cfg": self.cfg, "total": self.total, "done": done, "phase": self.phase, "error": self.error,
                "elapsed": round((self.ended or time.time()) - self.started, 1), "started": time.strftime("%H:%M", time.localtime(self.started)),
                "since": since, "results": res}


batches = []
current = None
replays = {}


def clean_cfg(body):
    scenario = body.get("scenario", "all")
    if scenario != "all" and scenario not in rp.SCENARIOS:
        raise ValueError(f"unknown field type {scenario}")
    runs = max(1, min(5000, int(body.get("runs", 100))))
    start = random.randint(1, 1_000_000) if body.get("random") else max(1, int(body.get("start", 1)))
    sets = [f"{k}={float(v):g}" for k, v in (body.get("sets") or {}).items()]
    fw = [d.strip() for d in body.get("fw", []) if d.strip()]
    for d in fw:
        if "=" not in d:
            raise ValueError(f"'{d}' should look like NAME=VALUE")
    return {"scenario": scenario, "runs": runs, "start": start, "random": bool(body.get("random")), "ideal": bool(body.get("ideal")), "sets": sets, "fw": fw}


def params():
    exe = builder.exe_for([])
    r = subprocess.run([exe, "--params"], capture_output=True, text=True)
    return json.loads(r.stdout)


def replay_page(q):
    scenario, seed = q.get("scenario", ["loops"])[0], int(q.get("seed", ["1"])[0])
    ideal = q.get("ideal", ["0"])[0] == "1"
    sets, fw = q.get("set", []), q.get("fw", [])
    exe = builder.exe_for(fw)
    key = (exe, scenario, seed, ideal, tuple(sets))
    if key not in replays or not os.path.exists(replays[key]):
        trace = os.path.join(TRACE_DIR, f"{len(replays)}.jsonl")
        cmd = [exe, "--scenario", scenario, "--seed", str(seed), "--geometry", rp.GEOMETRY, "--trace", trace] + (["--ideal"] if ideal else []) \
            + [x for s in sets for x in ("--set", s)]
        subprocess.run(cmd, capture_output=True, text=True)
        page = os.path.join(TRACE_DIR, f"{len(replays)}.html")
        rp.write_replay(trace, page)
        replays[key] = page
    return open(replays[key], "rb").read()


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        global current
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        try:
            if u.path in ("/", "/index.html"):
                return self.send(200, open(os.path.join(rp.HERE, "dashboard.html"), "rb").read(), "text/html; charset=utf-8")
            if u.path == "/viewer.html":
                return self.send(200, open(os.path.join(rp.HERE, "viewer.html"), "rb").read(), "text/html; charset=utf-8")
            if u.path == "/replay":
                return self.send(200, replay_page(q), "text/html; charset=utf-8")
            if u.path == "/api/info":
                return self.send(200, {"params": params(), "scenarios": rp.SCENARIOS, "cpus": CPUS, "build": builder.last,
                                       "running": current.id if current and current.phase in ("building", "running") else None,
                                       "batches": [{k: v for k, v in b.info(len(b.results)).items() if k != "results"} for b in batches]})
            if u.path == "/api/batch":
                bid, since = int(q.get("id", ["-1"])[0]), int(q.get("since", ["0"])[0])
                b = next((b for b in batches if b.id == bid), None)
                if not b:
                    return self.send(404, {"error": "no such batch"})
                return self.send(200, dict(b.info(since), build=builder.last))
            return self.send(404, {"error": "not found"})
        except rp.BuildError as e:
            return self.send(500, "The firmware didn't compile:\n" + str(e), "text/plain; charset=utf-8")
        except (ConnectionError, BrokenPipeError):
            pass

    def do_POST(self):
        global current
        u = urllib.parse.urlparse(self.path)
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if u.path == "/api/batch":
                if current and current.phase in ("building", "running"):
                    return self.send(409, {"error": "a batch is already running"})
                try:
                    cfg = clean_cfg(body)
                except (ValueError, TypeError) as e:
                    return self.send(400, {"error": str(e)})
                current = Batch(len(batches) + 1, cfg)
                batches.append(current)
                threading.Thread(target=current.run, daemon=True).start()
                return self.send(200, {"id": current.id})
            if u.path == "/api/stop":
                if current:
                    current.stop = True
                return self.send(200, {"ok": True})
            return self.send(404, {"error": "not found"})
        except (ConnectionError, BrokenPipeError):
            pass


def main(port=8765, open_browser=True):
    print("Building the simulator from Main/ ...", flush=True)
    try:
        builder.exe_for([])
    except rp.BuildError as e:
        print("The firmware didn't compile (the page will show this too):\n" + str(e)[-1500:])
    server = None
    for p in range(port, port + 20):
        try:
            server = http.server.ThreadingHTTPServer(("127.0.0.1", p), Handler)
            break
        except OSError:
            continue
    if not server:
        sys.exit(f"no free port between {port} and {port + 19}")
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"Dashboard: {url}\n(Ctrl+C here to stop it)", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":  # python server.py [port] [--no-open]
    nums = [a for a in sys.argv[1:] if a.isdigit()]
    main(int(nums[0]) if nums else 8765, "--no-open" not in sys.argv)
