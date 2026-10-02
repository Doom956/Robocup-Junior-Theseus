"""Web server for the physics simulator: the batch dashboard and the live view.

  python tools/sim/physics/run_physics.py --serve     batch dashboard (dashboard.html)
  python tools/sim/physics/run_physics.py --live      live view (live.html) in its own window

Only reachable from this PC. It builds the firmware from Main/ (again whenever a file in Main/ changes).
The dashboard runs batches of fields in parallel; "Watch" records a run and opens it in the replay
viewer. The live view runs several robots at once, each its own simulator process paced to the wall
clock, and streams what they do to the page as it happens. Ctrl+C in the terminal stops everything.
"""
import atexit, concurrent.futures, glob, hashlib, http.server, json, os, random, shutil, subprocess, sys, tempfile, threading, time, urllib.parse, webbrowser

import run_physics as rp

BUILD_DIR = os.path.join(rp.HERE, "build")
TRACE_DIR = tempfile.mkdtemp(prefix="physics_replays_")
CPUS = os.cpu_count() or 4


def source_stamp():
    """Changes whenever any file the simulator is built from changes."""
    files = sorted(glob.glob(os.path.join(rp.REPO, "Main", "*.*")) + glob.glob(os.path.join(rp.HERE, "hw", "**", "*.h"), recursive=True)
                   + [os.path.join(rp.HERE, f) for f in ("physics.cpp", "live_io.cpp", "world.h")])
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


# ---------------------------------------------------------------- live view
class Slot:
    """One robot in the live view: a running `physics.exe --live` and everything it has printed this run."""

    def __init__(self, idx):
        self.idx, self.gen, self.lines, self.proc = idx, 0, [], None
        self.scenario, self.seed, self.done = None, None, False


class Live:
    """Robots running live. Each is its own simulator process, paced to the wall clock; the page
    gets their output as it happens (/api/live/stream) and sends commands back (/api/live/cmd)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.session, self.slots, self.cfg, self.exe = 0, [], None, None
        self.results, self.speed, self.paused = [], 1.0, False

    def start(self, cfg, seeds=None):
        exe = builder.exe_for(cfg["fw"])  # raises BuildError for the page to show
        self.stop_all()
        with self.lock:
            self.session += 1
            self.cfg, self.exe, self.results = cfg, exe, []
            self.speed, self.paused = cfg["speed"], False
            self.slots = [Slot(i) for i in range(cfg["count"])]
        for s in self.slots:
            sc, seed = seeds[s.idx] if seeds and s.idx < len(seeds) else self.pick()
            self.launch(s, sc, seed)

    def pick(self):
        sc = self.cfg["scenario"]
        return (random.choice(rp.SCENARIOS) if sc == "mixed" else sc), random.randint(1, 1_000_000)

    def launch(self, slot, sc, seed):
        cmd = [self.exe, "--scenario", sc, "--seed", str(seed), "--geometry", rp.GEOMETRY, "--live", "--live-speed", f"{self.speed:g}"] \
            + (["--ideal"] if self.cfg["ideal"] else []) + [x for s in self.cfg["sets"] for x in ("--set", s)]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, encoding="utf-8",
                                bufsize=1, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        with self.lock:
            slot.gen += 1
            slot.lines, slot.proc, slot.scenario, slot.seed, slot.done = [], proc, sc, seed, False
            gen, session = slot.gen, self.session
        if self.paused:
            self.send(slot, "pause")
        threading.Thread(target=self.read, args=(slot, proc, gen, session), daemon=True).start()

    def read(self, slot, proc, gen, session):
        for line in proc.stdout:
            line = line.strip()
            if not line.startswith("{"):
                continue
            with self.lock:
                if slot.gen != gen or self.session != session:
                    break
                slot.lines.append(line)
                if line.startswith('{"result"'):
                    try:
                        self.results.append(json.loads(line)["result"])
                    except (ValueError, KeyError):
                        pass
        proc.wait()
        with self.lock:
            if slot.gen != gen or self.session != session:
                return
            slot.done = True
        if self.cfg.get("keep"):
            time.sleep(4)  # leave the result on screen for a moment
            with self.lock:
                if slot.gen != gen or self.session != session or not self.cfg.get("keep"):
                    return
            self.launch(slot, *self.pick())

    def send(self, slot, cmd):
        try:
            slot.proc.stdin.write(cmd + "\n")
            slot.proc.stdin.flush()
        except (OSError, ValueError, AttributeError):
            pass

    def command(self, cmd, idx=None):
        if idx is None:
            if cmd.startswith("speed "):
                self.speed = float(cmd[6:])
            elif cmd in ("pause", "resume"):
                self.paused = cmd == "pause"
        for s in list(self.slots):
            if idx is None or s.idx == idx:
                self.send(s, cmd)

    def restart(self, idx, new_field):
        slot = next((s for s in self.slots if s.idx == idx), None)
        if not slot:
            return
        with self.lock:
            slot.gen += 1  # the old reader ignores whatever the old process still prints
            old, sc, seed = slot.proc, slot.scenario, slot.seed
        if old and old.poll() is None:
            old.kill()
        self.launch(slot, *(self.pick() if new_field else (sc, seed)))

    def stop_all(self):
        with self.lock:
            self.session += 1
            procs = [s.proc for s in self.slots if s.proc]
        for p in procs:
            if p.poll() is None:
                p.kill()

    def snapshot(self, cursors, known_session):
        """Everything new for one page since its last call, as SSE data lines."""
        out = []
        with self.lock:
            if known_session != self.session:
                cursors.clear()
                out.append(json.dumps({"type": "session", "session": self.session, "cfg": self.cfg, "speed": self.speed, "paused": self.paused,
                                       "count": len(self.slots)}))
            for s in self.slots:
                g, pos, done = cursors.get(s.idx, (None, 0, False))
                if g != s.gen:
                    g, pos, done = s.gen, 0, False
                    out.append(json.dumps({"type": "run", "slot": s.idx, "gen": g, "scenario": s.scenario, "seed": s.seed}))
                if pos < len(s.lines):
                    out.append('{"type":"lines","slot":%d,"gen":%d,"lines":[%s]}' % (s.idx, g, ",".join(s.lines[pos:])))
                    pos = len(s.lines)
                if s.done and not done:
                    out.append(json.dumps({"type": "done", "slot": s.idx, "gen": g}))
                    done = True
                cursors[s.idx] = (g, pos, done)
            if cursors.get("results") != len(self.results):
                cursors["results"] = len(self.results)
                out.append(json.dumps({"type": "tally", "results": self.results}))
            state = (self.speed, self.paused, bool(self.cfg and self.cfg.get("keep")))
            if cursors.get("state") != state:
                cursors["state"] = state
                out.append(json.dumps({"type": "state", "speed": state[0], "paused": state[1], "keep": state[2]}))
            return out, self.session

    def replay(self, idx):
        slot = next((s for s in self.slots if s.idx == idx), None)
        if not slot:
            return None
        with self.lock:
            text = "\n".join(slot.lines)
        path = os.path.join(TRACE_DIR, f"live-{idx}-{slot.gen}.jsonl")
        open(path, "w", encoding="utf-8").write(text)
        page = path[:-6] + ".html"
        rp.write_replay(path, page)
        return open(page, "rb").read()


live = Live()
atexit.register(live.stop_all)


def clean_live_cfg(body):
    scenario = body.get("scenario", "mixed")
    if scenario != "mixed" and scenario not in rp.SCENARIOS:
        raise ValueError(f"unknown field type {scenario}")
    sets = [s for s in str(body.get("sets", "")).replace(",", " ").split() if s]
    for s in sets:
        k, _, v = s.partition("=")
        float(v)  # ValueError if it isn't name=number
    fw = [d for d in str(body.get("fw", "")).replace(",", " ").split() if d]
    for d in fw:
        if "=" not in d:
            raise ValueError(f"'{d}' should look like NAME=VALUE")
    return {"count": max(1, min(16, int(body.get("count", 6)))), "scenario": scenario, "ideal": bool(body.get("ideal")),
            "sets": sets, "fw": fw, "speed": max(0.0, float(body.get("speed", 1))), "keep": bool(body.get("keep", True))}


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

    def page(self, name):
        return self.send(200, open(os.path.join(rp.HERE, name), "rb").read(), "text/html; charset=utf-8")

    def stream(self):
        """Server-sent events: the live robots' output, about 25 times a second."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        cursors, session, quiet = {}, -1, 0
        while True:
            out, session = live.snapshot(cursors, session)
            if out:
                self.wfile.write("".join(f"data: {m}\n\n" for m in out).encode())
                self.wfile.flush()
                quiet = 0
            else:
                quiet += 1
                if quiet % 375 == 0:  # keep the connection alive every ~15 s
                    self.wfile.write(b": .\n\n")
                    self.wfile.flush()
            time.sleep(0.04)

    def do_GET(self):
        global current
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        try:
            if u.path in ("/", "/index.html"):
                return self.page("dashboard.html")
            if u.path == "/live":
                return self.page("live.html")
            if u.path == "/viewer.html":
                return self.page("viewer.html")
            if u.path == "/replay":
                return self.send(200, replay_page(q), "text/html; charset=utf-8")
            if u.path == "/api/live/stream":
                return self.stream()
            if u.path == "/api/live/replay":
                page = live.replay(int(q.get("slot", ["0"])[0]))
                return self.send(200, page, "text/html; charset=utf-8") if page else self.send(404, {"error": "no such robot"})
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
            if u.path == "/api/live/start":
                try:
                    cfg = clean_live_cfg(body)
                except (ValueError, TypeError) as e:
                    return self.send(400, {"error": str(e)})
                seeds = [(s.scenario, s.seed) for s in live.slots] if body.get("same") else None
                try:
                    live.start(cfg, seeds)
                except rp.BuildError as e:
                    return self.send(500, {"error": "The firmware didn't compile:\n" + str(e)})
                return self.send(200, {"session": live.session})
            if u.path == "/api/live/cmd":
                cmd = str(body.get("cmd", ""))
                if cmd not in ("pause", "resume", "lop", "nudge") and not cmd.startswith("speed "):
                    return self.send(400, {"error": "unknown command"})
                live.command(cmd, body.get("slot"))
                return self.send(200, {"ok": True})
            if u.path == "/api/live/restart":
                live.restart(int(body.get("slot", 0)), bool(body.get("newField")))
                return self.send(200, {"ok": True})
            if u.path == "/api/live/keep":
                if live.cfg:
                    live.cfg["keep"] = bool(body.get("keep"))
                return self.send(200, {"ok": True})
            return self.send(404, {"error": "not found"})
        except (ConnectionError, BrokenPipeError):
            pass


def open_window(url):
    """Edge or Chrome as an app window (no tabs or address bar); the normal browser if neither is found."""
    for exe in (shutil.which("msedge"), r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                r"C:\Program Files\Microsoft\Edge\Application\msedge.exe", shutil.which("chrome"),
                r"C:\Program Files\Google\Chrome\Application\chrome.exe"):
        if exe and os.path.exists(exe):
            subprocess.Popen([exe, f"--app={url}", "--window-size=1500,950"])
            return
    webbrowser.open(url)


def main(port=8765, open_browser=True, page=""):
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
    server.daemon_threads = True
    base = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"Live view: {base}live\nBatch dashboard: {base}\n(Ctrl+C here to stop)", flush=True)
    if open_browser:
        open_window(base + page) if page == "live" else webbrowser.open(base + page)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped")
    finally:
        live.stop_all()


if __name__ == "__main__":  # python server.py [port] [--no-open] [--live]
    nums = [a for a in sys.argv[1:] if a.isdigit()]
    main(int(nums[0]) if nums else 8765, "--no-open" not in sys.argv, "live" if "--live" in sys.argv else "")
