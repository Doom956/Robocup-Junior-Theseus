# Robocup-Junoir-Theseus
Robocup junior 2026
LT fixes

## Robot code

- `Main/`: the robot's code (C++, Arduino GIGA R1 M7)
- `platformio.ini`: build and upload with PlatformIO in VS Code (`pio run -t upload`)

## Maze simulator

`tools/sim/` tests the robot's navigation code on thousands of random RCJ Rescue Maze fields
(walls with loops, black, blue and silver tiles, ramps between two levels) without the robot.

- **`tools\sim\run_sim.bat`** compiles the real `Main/navigation.cpp` on your PC and runs it.
  Run it before uploading a navigation change: with the default perfect robot every field must
  pass, or the change broke the logic. Add `--robot realistic` to include sensor misreads,
  stalls, wheel slip, miscounted ramps, missed black tiles and the 8-minute clock.
- **`tools/sim/maze_sim.html`** opens in a browser: watch the robot explore step by step, edit a
  field, see the robot's map for each floor, and batch-test 200 or 2000 fields.

It works tile by tile and the fault rates are estimates, so it tests the navigation decisions,
not the hardware. Setup, options and how to read the results: [tools/sim/README.md](tools/sim/README.md).
