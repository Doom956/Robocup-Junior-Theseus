# How to use the simulators

There are two simulators. Use the logic one for quick checks of navigation changes, and the physics
one to see what the whole robot does.

| | Logic simulator | Physics simulator |
|---|---|---|
| Tests | the navigation decisions (exploring, the map, ramps, getting home) | the whole firmware: motors, encoders, distance sensors, gyro, colour sensor, timing |
| Robot | always centred and square in a tile; faults are fixed chances | moves continuously, sensors at their CAD positions, real motor specs |
| Speed | thousands of fields in seconds | about 4 minutes for 1000 fields (14-core PC) |
| Code it runs | `Main/navigation.cpp` (the web page is a copy of it) | everything in `Main/`, unchanged |
| Open with | `tools\sim\run_sim.bat` or `tools/sim/maze_sim.html` | `python tools/sim/physics/run_physics.py --live` (watch) or `--serve` (batches) |

All commands below are typed in the VS Code terminal (**Terminal → New Terminal**) with the
`Robocup-Junoir-Theseus` folder open. The terminal must be in that folder: if it says
`can't open file ... run_physics.py`, it is somewhere else (see Problems at the end).

No terminal needed for the physics simulator: double-click **`tools\sim\physics\live.bat`** (live view)
or **`tools\sim\physics\dashboard.bat`** (batch dashboard) in File Explorer.

## 1. Setup (once per computer)

1. Install **Python 3** from python.org. On the first installer screen, tick **"Add python.exe to PATH"**.
2. Install **VS Code** with the **PlatformIO** extension (the same setup used to upload to the robot).
3. Open the `Robocup-Junoir-Theseus` folder in VS Code.
4. Install the PC compiler the simulators use:
   ```
   pio pkg install -g --tool platformio/toolchain-gccmingw32
   ```
   If `pio` is not found, open a PlatformIO terminal instead: the PlatformIO icon in the left bar →
   **Quick Access → Miscellaneous → PlatformIO Core CLI**.
5. Check that Python works: `python --version` should print `Python 3.x`.

## 2. Logic simulator

### In the browser

1. Open `tools/sim/maze_sim.html` (double-click it in File Explorer, or right-click it in VS Code →
   **Reveal in File Explorer**).
2. In the top strip, pick a **Field** type and a **Robot** (perfect, realistic or harsh faults), then
   **Load seed** for a particular field or **New random field**.
3. Optional, before starting: click near a tile edge to add or remove a wall; click a tile to change
   it (plain → black → blue → silver); Shift-click to move the start.
4. Press **Play** (or Space), **Step** (→) for one move at a time, or **Run to end**. **Reset run**
   starts the same field again.
5. While it runs:
   - **Real field**: the robot, its path, and a dashed red box when it is lost (where it thinks it is).
   - **This run**: clock, coverage, moves, restarts, plus a checklist of what a full run needs.
   - **Robot's map**: what the code has stored, one map per floor. Red walls are wrong.
6. **Batch test**: **Run 200 fields** or **Run 2000 fields** of the selected type and robot. Tick
   **Same fields every time** when comparing two versions of the code. Click a seed under
   "Replay a failure" to load that field.

The web page is a JavaScript copy of the navigation code. It does **not** change when you edit
`Main/`. To test your edited code, use the command line below.

### On the real code (command line)

1. Logic check, which must pass on every field:
   ```
   tools\sim\run_sim.bat
   ```
2. With faults, to compare versions of the code:
   ```
   tools\sim\run_sim.bat --robot realistic --runs 2000
   ```
3. Look at one field, drawn in the terminal (the output prints the seed of the first failure):
   ```
   tools\sim\run_sim.bat --robot realistic --scenario ramp --seed 2 --show
   ```

Options: `--scenario flat|loops|big|ramp|bigramp|all`, `--runs N`, `--random` (new fields),
`--seed S` (one field), `--show`, `--verbose` (the robot's Serial output). More in
[README.md](README.md).

## 3. Physics simulator

### Live view: watch robots solve mazes

1. Start it: double-click `tools\sim\physics\live.bat`, or in the terminal:
   ```
   python tools/sim/physics/run_physics.py --live
   ```
   After about 10 seconds (building the firmware) a window opens with 6 robots, each on its own random
   field, running in real time. Leave the terminal open; closing it stops the robots.
2. Top strip:
   - **Robots**: how many at once (1 to 16). Changing it starts new fields.
   - **Fields** (mixed or one type) and **Robot** (realistic or ideal), then **New random fields**.
     **Same fields again** reruns the current fields from the start.
   - **Speed**: 1× is real time; 2× to 30× or **Max** to get through runs faster. Works while they run.
   - **Pause all** (or Space) freezes every robot.
   - **Keep going**: a robot that finishes gets a new random field 4 s later.
3. Each robot's card shows its field, the robot to scale with its sensor beams, the path it drove, and a
   box on the tile the code *thinks* it is on (green = right, red = wrong). The label says exploring,
   heading home, lost, home, wrong tile, 8:00 ran out or no path home.
4. Click a robot (or ← →) to see it in detail on the right: state, heading vs gyro, real tile vs the
   code's tile, distance readings, the robot's own map and its Serial output. Buttons for that robot:
   - **Pause** / **Resume**.
   - **Bump**: knocks it up to 40 mm and 15°, to see if the code copes.
   - **Restart at checkpoint**: a lack-of-progress restart, as the referee would do.
   - **Same field again** / **New field**: restart just this robot.
   - **Open replay**: what it has done so far in the replay viewer (2D and 3D, rewind, full Serial log).
5. **Finished runs** (bottom right) totals home / wrong tile / 8:00 / no path for every run since the
   last "New random fields". For proper numbers use the batch dashboard below.
6. Settings and `#define` changes to try go under **Simulator settings and code changes**; they are
   used from the next "New random fields". After editing `Main/`, press "New random fields" to rebuild.

### Batch dashboard

1. Start it:
   ```
   python tools/sim/physics/run_physics.py --serve
   ```
   It builds the firmware from `Main/` (about 10 seconds) and opens the dashboard in your browser.
   Leave the terminal open; it is the server.
2. In **New batch**, choose:
   - **Field type**: one type or all five.
   - **Fields / type**: how many fields of each type (20 for a quick look, 200+ to compare code).
   - **Same each time, from seed**: keep it ticked to compare code changes on the same fields.
   - **Robot**: realistic (noise, drift, wheel slip) or ideal (none).
   - Optional: **Simulator settings** to change things like battery voltage or robot mass;
     **Code change** to try a `#define` value without editing `Main/`.
3. Press **Run batch**. The progress bar shows how far it is and the time left.
4. Read the results:
   - **Home**: runs where it really ended on the start tile. The ± is how much it moves by chance.
   - **Wrong tile**: it said "back to start" but was somewhere else (it lost track of its position).
   - **Explored all / Avg explored**: how much of the field it visited.
   - **Map walls right**: how many walls in its map match the real field.
   - **Lost >20%**: runs where its map position was wrong for more than a fifth of the time.
   - **Restarts / run**: lack-of-progress restarts (stuck for 60 s, or drove onto black).
   - **How runs ended** and **Field explored**: the same results as charts.
5. In **Runs**, filter (Not home, Wrong tile, Had restarts, Lost) or sort by any column. Click
   **Watch** on any run to open its replay in a new tab.
6. Edit the code in `Main/`, save, and press **Run batch** again. The dashboard rebuilds by itself.
   Compare the two batches in **Batches this session**.
7. Press **Ctrl+C** in the terminal to stop the server.

### Watching a run (replay viewer)

Opens from **Watch** on the dashboard, or from the terminal for one field:
```
python tools/sim/physics/run_physics.py --scenario loops --seed 7 --view
```
The replay is also saved in `tools/sim/physics/replays/` and works on its own (double-click it or
send it to a teammate).

- **Play** / Space, **−1 s / +1 s** (← →), **−10 s / +10 s** (Shift + ← →), and the speed menu.
- Click or drag the **timeline** to jump. Orange stretches = heading home, amber line = the code's
  position was wrong for 4 s or more, red marks = restarts. **Next problem** jumps to just before the
  next one.
- **2D / 3D** above the field. In 3D: drag to turn, scroll to zoom, right-drag to move; cameras for the
  whole field, around the robot, behind the robot and from above (3D needs internet the first time).
- The green or red box is the tile the code **thinks** it is on. Red means the code is lost.
- On the right: the firmware state, true heading vs gyro, motor PWM, the 7 distance readings, the
  robot's own map (red = wrong walls, amber = edges it blocked), the LCD and the result.
- **Serial monitor** shows everything the firmware printed, in step with the replay.

### Command line batches

```
python tools/sim/physics/run_physics.py                          100 fields of each type
python tools/sim/physics/run_physics.py --scenario ramp --runs 300
python tools/sim/physics/run_physics.py --random                 new fields instead of seeds 1 to N
python tools/sim/physics/run_physics.py --ideal                  no noise or drift
python tools/sim/physics/run_physics.py --set batteryVoltage=12.4
python tools/sim/physics/run_physics.py --fw ROBOT_WIDTH_MM=183  try a #define change
python tools/sim/physics/run_physics.py --selftest               calibration experiments
```
Each failed field is listed under `replay:`. Add `--view` to watch one.

## 4. Testing a code change

1. Run a batch **before** the change with the same fields every time (the dashboard's tick box, or
   seeds 1 to N on the command line). Note the numbers.
2. Make the change in `Main/`. To try a `#define` value first, use **Code change** in the
   dashboard or `--fw`; `Main/` stays as it is.
3. Run the **same** batch again and compare. If the difference is smaller than the ±, run more fields.
4. **Watch** a few runs that failed to see why.
5. Check the logic still passes: `tools\sim\run_sim.bat` must pass every field with the perfect robot.
6. Then upload to the robot and test on a real field. Use what you see there to correct the
   simulator settings (see `physics/README.md`, "Where the numbers come from").

## 5. Problems

| Problem | Fix |
|---|---|
| `can't open file ... run_physics.py` | The terminal is in a different folder. Double-click `live.bat` / `dashboard.bat` instead, or use the full path: `python "C:\...\Robocup-Junoir-Theseus\tools\sim\physics\run_physics.py" --live`, or first `cd` to the `Robocup-Junoir-Theseus` folder. |
| `python` is not recognized | Install Python with "Add python.exe to PATH" ticked, then reopen VS Code. Or try `py` instead of `python`. |
| `g++ not found` or `build failed` | Run the setup command in step 1.4. If it says the firmware didn't compile, the error is in `Main/`. |
| The dashboard says it needs the simulator server | Run `python tools/sim/physics/run_physics.py --serve` and keep that terminal open. |
| The dashboard opens on another port | Another program is using 8765. That is fine; use the address the terminal prints, or `--port 8800`. |
| The live view opens in a normal browser tab | Edge/Chrome wasn't found for the app window. It works the same in the tab. |
| Robots at Max speed slow the PC down | Each robot is a process; use fewer robots or a lower speed. 1× uses almost no CPU. |
| 3D stays blank | It needs internet the first time (to load three.js) and a browser with WebGL. 2D always works. |
| `maze_sim.html` ignores my code change | Expected: it is a copy. Use `tools\sim\run_sim.bat` or the physics simulator. |
| Two random batches give different numbers | Chance. Compare with the same fields every time. |
