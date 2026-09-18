# AirMouse - progress, findings, TODO

How to run things: [README.md](README.md). This file is the "what we learned / what's next" log.
Competition facts: NIDAR 2026-27 PS2 AirMouse. Flight = 600 of 1000 points:
survivors 240 (40 each, correct 1 m cell), **grid map 220** (cell counts only if all 4 sides right),
GCS display 50, autonomous exit 50, <15 min bonus 25, safe completion 15. Crash / manual input = -50 each.
Dates (tentative): Progress Review 1 ~2nd week Oct 2026, Review 2 ~2nd week Dec 2026, Finals Jan 2027.

## Status (2026-09-18)

| Step | State |
|---|---|
| 1. ROS 2 basics | done |
| 2. Fly from our own ROS node (`fly_square`) | done |
| 3. Same code in Gazebo 3D | done |
| 4. Maze generator + 360 deg LiDAR drone | done |
| 5. GPS-free flight on Cartographer SLAM | done |
| 6. `grid_mapper` + judge-style scoring | done - 29/36 cells (81%) with a route planned from the TRUE maze |
| 7. SLAM accuracy tuning (offline replay) | **paused on purpose** - see "SLAM tuning" |
| 8. `explorer` (DFS over the grid, BFS home) | **flies: visited 36/36 cells with no prior knowledge.** Return leg and map quality still bad |
| 9. `mission_manager` state machine | next |
| 10. Survivor detection (OAK-D / sim bounding-box camera) | todo |
| 11. GCS dashboard | todo |

**The headline: the drone now explores the maze by itself.** Up to step 7, every "tour" flight followed
a route computed from the TRUE maze by `plan_tour.py` (a deliberate cheat, for testing the mapper; removed 2026-09-18, archived).
Run 8 below is the first flight where the drone was told nothing and worked it out from its own map.

**The blocker: the map it builds itself is far worse than the cheating tour's** (8% of cells vs 81%),
and it did not make it home. Both point at SLAM accuracy, not at the explorer.

## Flight log

Runs 1-6 used `plan_tour.py` (route from the true maze). Runs 7+ are autonomous.

| Run | Change | Result |
|---|---|---|
| 1 | first try | never flew: stale MAVROS/daemon -> SLAM pose never reached ArduPilot |
| 2 | clean start | 46 waypoints; 16/36 cells, 120/144 sides; walls 0.15-0.25 m off their grid line |
| 3 | grid snapping +-0.25 m | never flew: service request lost before MAVROS ready |
| 4 | service-call fix | crashed at waypoint 23: SLAM 0.2-0.3 m sideways + 64 cm Iris in a 1 m corridor |
| 5 | 35 cm collision footprint | **clean flight, 29/36 cells (81%), 137/144 sides (95%) ~177/220 pts** |
| 6 | + truth topic + rosbag | recorded `sim/bag_run6` (188 s) for offline SLAM tuning |
| 7 | **first autonomous flight** | took off, decided, landed safely - but explored **1 cell**. "done" was trusted on a 2.6 s-old map where the corridor was still `unknown`. Fixed: see "confirm done" below |
| 8 | + confirm-done, settle 4 s | **visited 36/36 cells autonomously in ~5 min.** Then ABORTED on the way home: stuck at (4,-3), drone on the floor (z=0.08) ~1.8 m from where it believed it was. Map scored **3/36 cells (8%), 79/144 sides (55%)** |
| 9 | + slam_eval instrumentation | froze at 12 cells, killed at the 780 s timeout. **Self-inflicted:** `ros2 topic hz/echo` was run against the live sim (see "Never poke a running sim") |
| 10 | Cartographer IMU enabled | never flew: SLAM stalled, no position, abort at 90 s. Root cause in "The IMU" below. **Reverted** |
| 11 | IMU bridged from Gazebo on sim time (`/airmouse/imu`, roll-pi TF, `use_imu_data = true`) | SLAM ran and drone took off (first time IMU works). **Self-inflicted:** I killed the Gazebo GUI child process and the wrapper took the whole simulator down mid-flight. `sim_up.sh` now runs server and GUI as separate processes |
| 12 | same, GUI on GPU | explored **31/36 cells** in ~5 min, then hit a wall at (2,-5)->(3,-5) (seen by the user) and aborted `stuck` after 45 s; landed safely. Map **5/36 cells (14%), 80/144 sides (56%)**, smeared with repeated parallel walls. The true maze is OPEN between those two cells, so this was not a misclassified wall: SLAM was already offset from the drone. `slam_eval` output lost (see below) |
| 13 | same, `slam_eval` fixed, IMU confirmed at ~950 Hz in Cartographer | **first full-flight SLAM-vs-truth.** Error <0.13 m for 220 s (mean ~0.03), then breaks down at truth (2,-3) -> 1.2 m by +280 s, 1.8 m at the end. Drone then followed the corrupt pose into the far wall: **drift FIRST, collision after** (plot: `sim/plot_run.py`). Same spot as run 12. Explorer never finished LANDING (exit 124 after 12 min - needs a landing timeout). Map 4/36 (11%), 70/144 sides (49%) |
| 17 | camera-aware utility exploration, 330 mm/1.3 kg drone, 4 edge ToF + wall guard, optical flow | **clean, ~3 min: 120/120 mapped, 111/120 cells (92%), 471/480 sides (98%), SLAM mean 0.05 m max 0.13 m, exit 0.** Guard fired 4x on a bad speed estimate (pose differencing read 1.4-1.9 m/s against a true 1.11 m/s) - now uses EKF velocity (`/mavros/local_position/velocity_local`) |
| 18 | same, on `rooms_small_4` after the other agent's exit/survivor work | **ABORTED at 5 wall-guard firings.** SLAM error mean 0.227 m, **max 3.63 m**: whole-cell slips. Root cause NOT the algorithm - only ~60% of `/scan` reached Cartographer (4.4 Hz vs 7.3 expected at 73% real time). The four ToF gpu_lidars at 20 Hz were asking for 80 GPU renders/s. The guard did its job: it landed safely instead of crashing |
| 19 | **ToF sensors 10 Hz + explorer logs the LiDAR rate** | **best run so far: 116.2 s, map 116/120 cells (97%), 476/480 sides (99%), SLAM mean 0.058 m max 0.180 m, 1 guard firing, autonomous exit flown, exit 0.** LiDAR steady at 10.0 Hz |
| 20 | LiDAR cut 450 -> 230 points (to save GPU), RViz2 on | **FAILED: SLAM came apart - yaw p95 68 deg, position max 18 m, map 46/120 (38%)**, while the LiDAR delivered a perfect 10 Hz. **Reverted.** The scan matcher needs point DENSITY to fix rotation; judging density by ray spacing at a range was wrong |
| 21 | 450 points restored, RViz2 on | **CRASHED into a wall in the far room and aborted.** Truth: on the floor (z=0.12) at world (4.98, 9.32) from sim t=108 s to the end. Drift started ~25 s BEFORE impact (0.6 -> 5 m), so it flew into the wall believing it was elsewhere; SLAM then ran to 37 m while the drone lay still. Map 29/120 (24%), 6 guard firings. LiDAR 10.0-10.2 Hz throughout - NOT starvation this time |
| 16 | **NEW `frontier` explorer on the generated building `rooms_small_4`** (3 rooms wide, 12x10 m, 120 cells) | **Clean autonomous mission in ~2.5 min: 24 cells flown, all 120 mapped. Map 111/120 cells (92%), 462/480 sides (96%). SLAM error mean 0.028 m, max 0.063 m.** Landed, exit 0. First time the whole chain works. (Runs 14/15 never flew: 14 was the wrong world; 15 lost Gazebo at startup to the WSL GPU glitch - `sim_up.sh` now retries)
| 17 | `rooms_small_4`, first flight with the 330 mm / 1.3 kg drone, 360° cameras, ToF guard and optical flow | **Clean mission, about 3 min: 120/120 cells mapped; 111/120 cells (92%) and 471/480 sides (98%) correct. SLAM error mean 0.05 m, max 0.13 m.** Guard fired 4 times falsely: the pose-difference speed read 1.4–1.9 m/s against a true 1.11 m/s. It now uses EKF velocity. Run folder: `sim/runs/0918_192850` |

Run 8 is the one that matters. Runs 9 and 10 produced no usable data.

**Why `slam_eval` never finished (runs 9-12): a background job in a non-interactive bash script starts with SIGINT ignored, so `kill -INT` did nothing and the summary was never printed. Fixed 2026-09-18: `slam_eval.py` now installs SIGINT/SIGTERM handlers. Run 13 should finally give SLAM-vs-truth.**

**WSL GPU gotcha:** Gazebo GUI blank with title "[warn:copy mode]" and `D3D12: Removing Device` / `dri2 screen` = WSL GPU state is bad; `wsl --shutdown` from Windows fixed it. Never force `LIBGL_ALWAYS_SOFTWARE` (4 fps, 100% CPU). Never kill the `gz sim gui` process of a combined `gz sim` - it takes the server with it.

**Run 13 follow-up (offline, no flight): the one-wall theory is REFUTED as a sufficient cause.** Ray-casting the
LiDAR from the true path (both x and y wall-constraint ~170 at the breakdown) shows the geometry was NOT degenerate
where SLAM broke, while corridors that ARE more degenerate (x=0, y-info ~18) tracked fine (err 0.02 m). What the data
does show: error onset at sim ~320-326 s, exactly when the drone reached the dead-end alcove (2,-3) and reversed out;
the SLAM estimate then kept moving (0.42 -> 0.71 m) while the drone HOVERED at truth (1.7,-4.1) - so the pose is being
pulled by wrong scan matches / constraints, not by motion. Cartographer logged no warning at all near the failure.
Next: replay that stretch from a bag (this needs one flight with the new recorder) with IMU off / sub70 / no loop closure.

**Frontier exploration (2026-09-18, flown in run 16).** The LiDAR sees a whole room from inside it and the judges score the
MAP, not where the drone flew, so `explorer` now defaults to `strategy:=frontier` (`explore_logic.frontier_step`): fly to the
nearest unvisited cell whose open side leads to unmapped space, look, repeat. Routes come from `plan()` (Dijkstra with a turn
penalty, so long straights beat staircases) and are flown in straight legs; a path is abandoned and re-planned if the latest map
closes the next hop. `strategy:=dfs` restores the old tour of every cell. Offline (idealised line-of-sight model, WP_SPD 0.5 m/s):
maze ~5.5 -> 2.5 min, rooms_small_4 17 -> 2.1 min, rooms_1 (544 cells) 79 -> 7.9 min. SLAM was FAR better in open rooms than in the
narrow maze (0.03 m vs 1+ m), which fits the failure being specific to those corridors.

**Two open problems after runs 20/21 (2026-09-18), both seen by the user in RViz:**
1. **SLAM still drifts in the far room** of rooms_small_4 and the drone crashes into a wall there. Runs 19 (clean,
   max 0.18 m) and 21 (crash) differ only in RViz being open - unproven either way, so the next step is one run with
   RViz off and one with it on, nothing else changed. What IS established: the drift precedes the collision, the
   LiDAR rate was fine, and the huge post-crash numbers are the estimate running away while the drone lies still.
2. **The route zig-zags across the building.** Real decision sequence from run 21: (1,3) -> 7 cells back to (1,-4),
   (4,-5) -> 10 cells up to (6,3), (8,-5) -> 14 cells back across to (1,-4). `utility_step` divides gain by cost, so
   a big distant room beats finishing the room we are standing in, and `frontier_step` checks room centres first
   (centre_reach 6). Needs a locality bias (e.g. score x exp(-cost/D)) so far targets only win when nothing near is
   left - and the room-centre step probably should go, since the camera gain already rewards standing in a centre.

**The wall guard needs two fixes (runs 20/21).** When the drone is stopped and too close, it backs off only
(trigger - range + 0.05) = 5-6 cm, drifts straight back, and fires again: three firings at the same spot ended run
21. It should retreat to a real clearance (~0.30 m), and repeated firings without the drone moving should count as
ONE event, not N towards the abort limit.

**Sim sensor load is a SLAM failure mode (2026-09-18, runs 18/19).** Every `gpu_lidar` costs a GPU render, and when
the simulator cannot keep up it DROPS scans silently - no warning anywhere. Starved of scans the Cartographer scan
matcher slips whole cells in repetitive rooms, and the drone then flies at walls believing it is centred. Check the
delivered rate, not the configured one: `/scan` should arrive at 10 Hz x (sim real-time factor), and Cartographer logs
that factor as `pulsed at NN% real time`. The explorer now logs `LiDAR X.X Hz` (sim) every 5 s and warns under 6 Hz.
Counter-intuitive: a SLOWER sim is safer, because fewer sensor renders are demanded per wall-clock second - run 17 was
clean at 40% real time and run 18 broke at 73%.

**The practice maze was deleted (2026-09-18).** `practice_6x6` was a hand-drawn 1 m corridor maze; the real NIDAR
arena is rooms of 2x2 m joined by doors in 1 m walls, which is what `make_rooms.py` generates. `rooms_small_4`
(12x10 m, 120 cells) is now the default world everywhere - it is the only generated world inside NIDAR's 15x15 m
limit (rooms_small_3 is 18 m, rooms_small_5 is 20 m, rooms_1 is 34 m). Runs 1-15 in the log above were flown in the
deleted maze; their SLAM findings still stand, the world is just not representative.

**Hardware update (2026-09-18, NOT flown yet): two 200 deg side cameras, 4 edge ToF sensors, optical flow.**
- Cameras: two 200 deg side cameras = all-round view, so `cam_hfov_deg` defaults to 360: the drone never yaws and never
  spins (the spins below were for a single forward camera, still available with `cam_hfov_deg:=69`). Offline, all-round
  cameras + utility: camera 100% everywhere, **maze 3.4, rooms_small_4 3.1, rooms_small_5 5.2 min** (single camera +
  spins was 4.4 / 4.4 / 7.3). With `drift_penalty:=1.0` the maze drops to 2.6 min with a third fewer one-wall hops.
- ToF wall guard (`explorer.on_tof`): **ArduPilot's avoidance does not act on GUIDED position targets** (only velocity
  targets, `mode_guided.cpp` velaccel_control_run), so the stop is ours. It checks each reading at 20 Hz and fires at
  `tof_stop` (0.10 m) + braking distance at the current speed towards that wall: at WP_SPD 1.0 m/s that is ~0.5 m,
  not 0.1. It holds a point backed away from the wall, then re-plans; `guard_limit` firings a minute aborts.
  Sim: 4 VL53L1X-like gpu_lidar sensors on the frame edges, bridged to `/airmouse/tof/{front,back,left,right}`.
  Real drone: the ToF readings must reach the companion computer (wired to it, or to the FC and relayed by MAVROS).
- Optical flow (`sim/params/optflow.parm`, loaded with `--nogps`): SITL-simulated flow + downward rangefinder,
  `EK3_SRC1_VELXY 5`. It gives the EKF independent velocity between SLAM updates. It does NOT correct a SLAM map
  that has slipped: the EKF still follows SLAM position, and grid_mapper uses SLAM's map.

**Camera-aware utility exploration (2026-09-18, offline-tested only, NOT flown).** The camera finds survivors (240 pts),
but `go_to` always sent yaw 0, so the camera only ever faced +x. Offline it saw just **86% / 65% / 49% / 61%** of cells
(maze / rooms_small_4 / rooms_small_5 / rooms_1). The explorer now (`camera:=utility`, default) points the nose along each leg,
does a 4 x 90 deg spin at a stop when that shows the camera unseen cells, and picks stops by `explore_logic.utility_step`:
(3 x open sides into unmapped space + new camera cells within `cam_reach` 3 m) / (route cost + stop cost).
**Offline result: camera coverage 100% on every world, map still 100%.** It costs time: ~2.8 -> 4.4 min (maze),
2.5 -> 4.4 (rooms_small_4), 4.1 -> 7.3 (rooms_small_5, 200 cells, about the NIDAR maximum of 225), 9.4 -> 22.7 (rooms_1, 544 cells,
bigger than any allowed arena). Changing the stop cost or lidar weight barely moves these numbers: the extra time is the cost of looking.
`mission_timeout` (default 840 s) now includes the estimated trip home + `home_margin`, so we turn for home in time for the
<15 min bonus. `drift_penalty` (default 0) prices hops into one-wall cells (`corridor_penalty`); at 1.0 it cuts those hops ~2-9x
offline at no cost in time, but the run-13 analysis did not prove that geometry is the cause, so it stays off until a flight shows it helps.
`camera:=off` restores the old behaviour. Flight caveats: yaw is now commanded, so check the heading holds during spins in SITL.
Fast in-place yaw is hard on a scan matcher, so watch slam_eval during the spins.

**Multi-cell legs (2026-09-18, offline-tested only, NOT flown):** `explorer` now flies straight runs of cells in one
leg (`max_leg` = 3 forward, whole straight runs when backtracking/going home) instead of braking at every cell; cells
passed within `pass_tol` (0.4 m) count as visited. `max_leg:=1` restores the old behaviour. `test_explore_logic.py`:
maze 39 -> 28 stops with a perfect map but only 39 -> 38 in fog of war (unseen cells cannot be extended); a generated
rooms building 590 -> 411 stops. Real gain depends on how far the LiDAR sees across open rooms.

**Answer to the run 8 question (run 13): SLAM drifts first, then the drone flies into a wall.** The breakdown starts around cell (2,-3)/(2,-4): the corridor at x = 1.5..2.5 has a wall only on ONE side below y = -3.5, so the scan matcher sees a single long wall and cannot observe motion ALONG it (geometric degeneracy). Hypothesis - check by replaying that stretch offline.

## Open question from run 8 (answered above; kept for history)

The drone ended on the floor at (3.36, -2.66) while flying (5,-3) -> (4,-3). Two possibilities, and the
logs cannot separate them:
  - SLAM drifted ~1.8 m and ArduPilot faithfully flew the drone into a wall, or
  - it hit something first and SLAM lost tracking afterwards.

`sim/test_explore.sh` now runs `slam_eval.py` for the whole flight (SLAM vs Gazebo truth) to settle
this. **That measurement has never successfully completed** - run 9 froze and run 10 never flew. Get
one clean instrumented run before changing anything else.

A supporting clue: the mapped-cell count FELL during the return (36 -> 34), so the SLAM map was
shifting under the grid classifier.

## The IMU - the most promising unfinished idea

Cartographer is tracking from laser scans ALONE: `use_odometry = false` and `use_imu_data = false`,
on a flying vehicle, in a maze of identical 1 m cells. That is almost certainly why the scan matcher
slips a whole cell. Research agrees: Cartographer + IMU is the standard configuration for indoor UAV
SLAM without wheel odometry, and repetitive corridors ("perceptual aliasing") are a known worst case
that parameter tuning alone does not fix.

**Enabling it with `/mavros/imu/data` does not work, and breaks flight completely.** MAVROS stamps
its messages with WALL-CLOCK time (see the comment in `slam_to_mavros.py`) while `/scan` and `/clock`
are on SIM time. Cartographer merges the queues in timestamp order, stalls on
`Queue waiting for data: (0, scan)`, never publishes `map -> base_link`, so ArduPilot never gets a
position and the explorer aborts after 90 s with `position=no`.

The wiring itself was fine (the 50 Hz stream requests went out) - only the clock is wrong.

**The fix to try: bridge the Gazebo model's own `imu_sensor`** (it exists in
`sim/models/iris_lidar/model.sdf`) through `ros_gz_bridge` in `sim_up.sh`, exactly as `/scan` is
bridged, so the IMU arrives on sim time. On the real drone the FC IMU is fine - there is no sim/wall
clock split there.

Currently reverted: `use_imu_data = false`, with the full explanation in `config/airmouse_2d.lua`.
The `imu -> /mavros/imu/data` remap is left in `slam.launch.py` as a marked warning, and
`slam_to_mavros` still asks for 50 Hz streams (harmless, and needed when this is fixed).

Also: `translation_weight = 0.1` was lowered precisely because there was no motion prediction. Once
an IMU provides it, re-test the default - `tw1` (weight 1.0) was already third-best in the sweep.

## SLAM tuning (step 7) - what the sweep showed

`sim/sweep_slam.sh` replays `bag_run6` through a fresh Cartographer per config and scores it against
Gazebo truth. All 8 variants at 2x (`sim/replay_out/<variant>/`):

| Variant | mean err | p95 | max | yaw p95 |
|---|---|---|---|---|
| **sub70** (`num_range_data = 70`) | **0.489 m** | **1.013 m** | 2.293 m | **0.9 deg** |
| occ20 (`occupied_space_weight = 20`) | 0.668 m | 1.043 m | 1.952 m | 1.5 deg |
| tw1 (`translation_weight = 1`) | 0.711 m | 1.181 m | 2.227 m | 2.2 deg |
| base (current `airmouse_2d.lua`) | 0.977 m | 2.051 m | 2.325 m | 2.5 deg |
| res025 (2.5 cm submap grid) | 1.045 m | 2.001 m | 2.552 m | 3.6 deg |
| local_lc (loop closure nearby only) | 1.162 m | 2.307 m | 2.616 m | 1.3 deg |
| strict_lc (+ `min_score = 0.8`) | 1.217 m | 2.403 m | 2.599 m | 1.5 deg |
| no_lc (no loop closure at all) | 1.363 m | 2.801 m | 3.010 m | 1.7 deg |

**The loop-closure theory in the old notes was wrong.** The error steps up in clean ~1 m jumps
(0.01 m for 30 s, then ~1.0 m, then ~2.0 m at t=130 s), which looked exactly like loop closure
matching a look-alike cell. It is not: restricting loop closure made it worse and removing it made
it worst of all - loop closure was *partly correcting* the error. The three configs that helped all
give the **scan matcher** more or better-weighted evidence, so the fault is the matcher slipping a
whole cell in featureless corridors. `res025` hurting fits: a finer grid means less context per match.

**Do not trust these numbers as absolute.** The replay says 0.98 m mean for the baseline while the
live run-5 flight measured ~0.15 m - a 6x disagreement nobody has explained. The table ranks configs;
it does not measure reality. Combining `sub70` + `occ20` + `tw1` is the obvious next sweep, but the
IMU is the bigger lever.

### Replay speed (a failed optimisation, do not repeat it)

`/clock` is 200,870 of the bag's 219,231 messages (~1070 Hz). `clock_relay.py` republishes it in
Python and saturates one CPU core at 2x - that, not Cartographer (~30% of a core), is the speed limit.

Replacing it with `ros2 bag play --clock 100` **does not work**: Cartographer rejects every scan with
"Ignored subdivision of a LaserScan ... previous subdivision time is not before current", produces no
SLAM, and the run hangs. The experiment script was removed 2026-09-18 (archived in `~/airmouse_removed_20260918.tar.gz`); the working relay version is in use. Scan header stamps and the generated clock need reconciling first.

## Key findings (the non-obvious stuff)

**Exploration (step 8)**
- **"done" is not a fact, it is a claim.** `next_step` returning "nowhere to go" is also what a
  half-built map looks like, and acting on it ends the mission. Run 7 landed after 1 cell because
  the corridor out of the takeoff cell was still `unknown` 2.6 s in. The explorer now hovers and
  re-checks for `done_grace` (10 s) and `done_updates` (3) fresh grids before accepting it. In run 8
  the very first decision claimed "done" and then recovered by itself.
- `/map` arrives at ~1.5 Hz and `grid_mapper` re-classifies at 1 Hz, so a short settle means deciding
  on a map that has barely changed since arriving. `settle_time` is 4 s and no decision is made until
  2 fresh grids have arrived (`min_grid_updates`).
- **The takeoff cell has an open side that leads out of the building** - the entrance. The first
  explorer flew straight through it and stranded itself outside with no map. The offline test caught
  this before any flight. `explore_logic` takes a `blocked` set of (cell, side); the node blocks
  `((0,0), "-x")`. That same side is the one we DO cross later, for the 50-point autonomous exit.
- Only sides the map calls `"open"` are ever crossed; `"unknown"` counts as a wall, and we only enter
  cells the mapper has already seen (`require_seen`). Guessing is not worth -50 points.
- Both cells get a veto on a shared wall: adjacent cells disagree at the edge of the map, where a
  wall has only been seen from one room.
- A flight that lands safely having explored nothing is a FAILURE, not a success. The explorer now
  reports it as one (run 7 exited 0 and would have looked fine in a log).
- `sim/test_explore_logic.py` walks the true maze with no ROS and no drone in milliseconds, in both
  "perfect map" and "fog of war" modes: 36/36 cells in 52 moves (the cheating tour used 46 waypoints,
  so honest search costs ~13% more travel). Run it before any flight test - then a failed flight is a
  flight problem, not a search problem. It has already caught one bug that would have cost a flight.

**SLAM / navigation**
- Cartographer, not slam_toolbox: a drone has no wheel odometry, Cartographer tracks from scans alone.
- Cartographer's default scan-matcher weights assume odometry predicts motion; without it SLAM lagged
  2.9 m behind the drone. Low `translation_weight` (0.1) + `occupied_space_weight` 10 fixed it (~0.15 m).
- SLAM still has ~0.1-0.3 m sideways error and the map is slightly shrunk/rotated at the edges.
- The arena is on a 1 m modular grid, so the grid mapper snaps any wall pixel within +-0.25 m to its
  grid line. Tolerant of SLAM offsets; must stay well below 0.5 m.
- The scan matcher, not loop closure, is what slips.

**ArduPilot (4.8-dev SITL)**
- SITL defaults to `AHRS_EKF_TYPE 10` = flies on simulator truth and silently ignores SLAM. Force 3.
- `WPNAV_SPEED` etc. were renamed `WP_SPD`/`WP_ACC`/`WP_SPD_UP`/`WP_SPD_DN` (m/s). Old names silently ignored.
- GPS-free: `GPS1_TYPE 0`, `VISO_TYPE 1`, `EK3_SRC1_POSXY 6`, `EK3_SRC1_YAW 6`, `EK3_SRC1_POSZ 1`, `COMPASS_USE 0`.
  `COMPASS_ENABLE 0` breaks arming in SITL ("Compass not healthy") - leave it enabled.
- ArduPilot needs an EKF origin before it will use vision position: `slam_to_mavros` sets it.
- ArduPilot only streams position after a stream-rate request (`/mavros/set_stream_rate`).
- "PreArm: VisOdom: not healthy" = SLAM pose isn't reaching ArduPilot. Check `slam_to_mavros` logs.
- `position=no` for 90 s means SLAM is not producing `map -> base_link` at all - look at Cartographer,
  not at MAVROS.

**Reliability / running tests (each of these cost a run)**
- **Never poke a running simulation with `ros2 topic list/hz/echo`.** A stale ros2 CLI daemon plus
  FastDDS `/dev/shm` files poison DDS discovery, and the symptom is not an error - topics silently
  never appear, in that run AND in later ones. It froze run 9 mid-flight and then broke the next two
  starts ("Gazebo is not running", then "no /clock" while Gazebo was demonstrably stepping on the gz
  side). `test_explore.sh` now starts with `ros2 daemon stop` and clears `/dev/shm/fastrtps_*`, and
  any topic-rate checks happen inside the script BEFORE the mission starts.
- **Gazebo can die while everything looks fine.** SITL and MAVROS stay up around a dead simulator, so
  every topic is silent with no error. `test_explore.sh` checks `pgrep gz sim` and `/clock` before
  waiting 40 s for an EKF. This caught a dead sim in 35 s instead of wasting 13 minutes.
- Leftover processes + FastDDS `/dev/shm` files also break new ROS programs. Always `sim_down.sh`.
- `timeout ros2 run X` leaves X running. Run node binaries directly under `timeout`.
- Service calls must check `service_is_ready()` and time out; a lost request = silent infinite wait.
- Every test gets a hard time limit and live progress output. Never leave a long run unwatched.
- A node on `use_sim_time` whose timers stop firing entirely (no logs at all, not even the periodic
  ones) means `/clock` stopped - the simulator froze. That is different from a stuck waypoint, which
  keeps logging and then aborts.

**Environment (WSL) - see also scripts/env.sh**
- `~/.bashrc` is interactive-only, so scripts, cron and `wsl.exe -- bash -lc` get NO ROS and no
  `GZ_SIM_RESOURCE_PATH`. Gazebo then cannot resolve `practice_6x6.sdf`, tries to download the name
  from Fuel, fails, and exits - while SITL and MAVROS keep running. `scripts/env.sh` sets all of it;
  source it at the top of every script, BEFORE `set -u` (ROS's own setup.bash reads unset variables
  and will kill the script on line 1).
- Editing files from Windows drops the execute bit. `chmod +x`, or invoke as `bash script.sh`.
- `~/.profile` activates `venv-ardupilot` (it was listed twice; duplicate removed 2026-09-18, backup
  at `~/.profile.bak-20260918`). Built with `--system-site-packages`, so `rclpy`/`numpy` still import
  once ROS is sourced - harmless.

**Simulation realism**
- Stock Iris collides as ~64 cm wide (15 cm clearance per side in a 1 m corridor); our real 5-inch with
  guards is ~35 cm (~33 cm clearance). `iris_standoffs_airmouse` keeps Iris physics with a 35 cm footprint.
  **2026-09-18: the real drone is 330 mm tip to tip including guards, 1.3 kg** (earlier 5"/35 cm figures are
  superseded). The sim box is now 23.3 cm square (33 cm diagonal: ~38 cm clearance per side, ~33 cm turned 45 deg),
  the body mass is set so the model totals 1.3 kg, inertia scaled by mass only. Motors/thrust are still the Iris's,
  so the sim has spare thrust the real drone may not - hover throttle from the sim means nothing. Runs 1-16 flew the
  35 cm box.
- `/model/iris_lidar/pose` is simulator ground truth - for measuring only, never for flying.

## Are we reinventing the wheel? (researched 2026-09-18)

- **Frontier exploration is a solved, packaged problem**: `m-explore-ros2` (the `explore_lite` port),
  `frontier_exploration_ros2` (ROS 2 Jazzy + Nav2), `Autonomous-Explorer-and-Mapper-ros2-nav2`.
  They all emit Nav2 goals toward frontiers in a raw occupancy grid.
- **Our explorer still earns its place**: NIDAR does not score coverage, it scores a 1 m cell grid
  where a cell counts only if all four sides are right. Nothing off the shelf produces that, which is
  why `grid_mapper` exists and why a cell-based explorer is the natural partner. `explore_logic.py` is
  ~120 lines with a millisecond test.
- **What we should NOT keep hand-rolling is traversal.** `flight.py` is goto-a-point with a stuck
  timeout; Nav2 gives path planning, obstacle avoidance and recovery behaviours - exactly the class of
  thing that failed on run 8's return leg. Keep the explorer as the goal-chooser, consider Nav2 under it.
- The SLAM problem has a name - perceptual aliasing / geometric degeneracy in repetitive corridors -
  and the literature's answer is fusing a second sensor (IMU), not more scan-matcher tuning.

## CPU (measured in run 16, whole flight, sampled every ~10 s)
Machine 54% busy = ~8.6 of 16 cores. Gazebo server+GUI ~2.5 cores; Cartographer 0.9; mavros 0.8; ros2 bag 0.7; explorer 0.7,
slam_to_mavros 0.6, grid_mapper 0.6 (Python nodes pay for the 1 kHz /clock); bridge 0.5; slam_eval ~0.9. The grid classifier itself is
cheap (21 ms/s on the maze, 171 ms/s on a 34x16 m building). Ideas, NOT applied: throttle /clock to ~100 Hz (needs topic_tools or a small
C++ node), a separate 200 Hz IMU sensor for ROS (the ArduPilot one must stay 1 kHz), drop bag/slam_eval for casual runs.

## TODO

Now
- [ ] One clean instrumented run of `sim/test_explore.sh` to get SLAM-vs-truth for a full autonomous
      flight. This has never succeeded; everything below depends on knowing whether run 8 ended in
      drift or a collision
- [ ] Bridge the Gazebo `imu_sensor` on sim time and re-try `use_imu_data = true` (see "The IMU")
- [ ] Fix the return leg: it is the last thing between us and a complete autonomous mission
- [ ] Entrance side scores "unknown" (it opens to the outside) - decide a rule and ask the organisers

Next
- [ ] `mission_manager`: takeoff -> explore -> room scan -> return (time/battery budget) -> exit -> land
- [x] Autonomous exit through the entrance (50 pts) - the one time we DO cross that blocked side
- [ ] Multi-cell legs instead of stopping in every cell, for the <15 min bonus (25 pts)
- [ ] Evaluate Nav2 for traversal underneath the explorer
- [ ] Random-maze generator + overnight batch runs with auto-scoring
- [ ] Survivors in the sim (Rescue Randy; Gazebo Fuel downloads are blocked on this network)
- [x] Bounding-box camera as a fake detector, then `survivor_tagger` -> grid cell tags
- [x] GCS dashboard (lightweight HTML5/Canvas web dashboard at http://localhost:8080 via rosbridge WebSocket)
- [x] Put `~/airmouse_ws` under Git

Later / if time
- [ ] Finish step 7: explain the 0.98 m replay vs 0.15 m live gap, then apply `sub70` + `occ20` + `tw1`
      together and re-fly

Hardware / real drone
- [ ] Order parts (list in chat history); VL53L1X bumpers optional
- [ ] Early hover-time test on the 5-inch; move to 6-inch if < 8 min
- [ ] LiDAR + Pi carried by hand through a corridor to test real SLAM before flying

Questions for the organisers
- [ ] One entry/exit or two? (rule 8.21 vs the brief)
- [ ] Grid origin/axes for the scoring grid; how the entrance side is scored
- [ ] Which cell counts when a survivor lies across two cells; dummy size/pose
- [ ] Private offline Wi-Fi allowed for telemetry/video? RC transmitter allowed for abort only?
- [ ] Is a commercial frame kit a "complete airframe"?
