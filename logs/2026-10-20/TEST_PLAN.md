# Test plan, 2026-10-20 evening (maze session)

Goal: confirm the robot-test changes waiting for the robot (ramp guard on a real climb, blue wait, 2p ramp detection,
and P2/P3 avoidance if the fast PC has pushed them), and run the obstacle / relocalize / home tests that were never done
on purpose. If it goes well, robot-test becomes main and the known-good tag moves to this firmware.

Details for each test: TEST_PRIORITY.md (main) and ROBOT_TEST.md (robot-test).

## Before leaving (at home, ~20 min, no maze needed)

- [ ] Claude flashes the latest robot-test (`BENCH_MODE 0`). Note the commit: it should include P2/P3 if they're pushed.
- [ ] Charge the battery; bring the spare if you have one.
- [ ] Secure the gyro cable.
- [ ] Bring: laptop + USB cable, heavy obstacle, a box (M4), a 2 cm board (step), a speed bump if you have one, victims.
- [ ] Optional if time: bench test 11 (`BENCH_MODE 1`, Claude flashes), the side sensor pairs match each other.

## At the maze, in this order (cameras OFF until part D)

Every test: start the log first (Claude records it), write the battery voltage at the start, one line of what you saw.
Restarts only with the pause switch.

### A. Ramp (~20 min), log `ramp.txt`
1. Up the practice ramp and down, **3 times each**. Pass: it stops in the middle of the tile after the ramp, like
   2026-10-19. (2p and the ramp guard on a real climb.)
2. Drive it over the **2 cm board** and a **speed bump** (or a 1 cm strip). Pass: no `climbing` in the log, nothing added
   to the map. Fail: `climbing` -> tell Claude (2p may be too sensitive).
3. Watch the tile beside the ramp top (the false obstacle from 2026-10-19, N3): note if it happens again.

### B. Obstacles (~40 min)
4. **Test 2** (`obst_b.txt`): heavy obstacle touching a side wall of the tile ahead, at the near end, middle, far end.
   Pass: stops without touching it, back to its tile middle, goes elsewhere.
5. **Test 3** (`obst_a.txt`): no obstacle; robot 4-5 cm off-centre toward an open side at a wall end, then also turned
   ~10 deg. Pass: squares up and drives on. Fail: `blocking edge` with nothing there.
6. **Test 6** (`obst_c.txt`): a dead end whose only way out passes the obstacle. Pass: blocks it first, later drives
   around it and the LCD position is right afterwards. The sim expects this to fail often: watch the 300 ms full-power
   lurch and whether it hits a wall.
7. **Test 7** (`obst_e.txt`): obstacle touching a side wall in the first half of a blue tile. Pass: `waiting 5 s first`
   before backing off.

### C. Position tests (~20 min)
8. **Test 4** (`reloc.txt`): no pause. While it drives back through tiles it already visited, hold it still with your
   hand for one move, then let go. Pass: `position corrected` or `[FWD] no progress`, LCD position right afterwards.
9. **Test 5** (`home2c.txt`): a run that doesn't visit a silver tile. On the way home: pause, carry it to the tile behind
   the start (same heading), resume. Pass: `not home yet`, then drives to the start.

### D. Full runs, cameras ON (~30 min)
10. **2-3 full runs** (`cam1.txt`, ...) with victims and one obstacle, like the competition. Count victim stops and false
    ones (for Baichen). Pass: gets home.

### E. If there's time
11. Stairs up and down with a log (`stairs.txt`): does it print `climbing`, does the `[POS]` floor stay right?
12. M4 with a box against the wall (`BENCH_MODE 2`, Claude flashes, then back to 0): corner obstacle readings for B3.

## After

Push the logs and notes. Claude updates the "Not yet confirmed" list in ROBOT_TEST.md; if A-D pass, tag the firmware as
`known-good-2026-10-20` and make robot-test the new main.
