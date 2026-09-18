# AirMouse workspace

ROS 2 code for NIDAR AirMouse: a drone that flies into a building it knows nothing about, maps it as a
grid of 1 m cells (each side wall / open), finds its way around by itself and lands. Our code lives in
`src/airmouse/airmouse/`. **Read [NOTES.md](NOTES.md) first** - it is the log of what we tried, what
worked, and the TODO list.

Stack: ArduPilot (SITL in simulation) + MAVROS + Gazebo Harmonic + Cartographer 2D SLAM on a 360 deg LiDAR.
The real drone will carry an **RPLIDAR C1** (see "Real hardware" below).

## Layout

    src/airmouse/   ROS package: nodes, config/airmouse_2d.lua (Cartographer), launch/slam.launch.py
    sim/            worlds + generators, drone models, tests, analysis and SLAM-replay tools
    sim/runs/       one folder per test flight (rosbag, logs, picture) - written by test_explore.sh
    scripts/        sim_up.sh, sim_down.sh, env.sh
    docs/           NIDAR rulebook + mission brief
    setup/          wsl_setup.sh, the one-shot environment installer

## Environment

Interactive shells get everything from the airmouse block in `~/.bashrc`. **Scripts do not** -
`~/.bashrc` is interactive-only, so anything non-interactive must start with:

    source ~/airmouse_ws/scripts/env.sh

That sets ROS, the workspace overlay, `GZ_*` paths and the GPU driver (`GALLIUM_DRIVER=d3d12`). Source it
**before** `set -u`: ROS's own `setup.bash` reads unset variables and kills the script.

Build after adding a node or changing `setup.py` (edits to an existing `.py` need no rebuild):

    cd ~/airmouse_ws && colcon build --symlink-install && source install/setup.bash

## Fly a full autonomous mission (the main test)

    bash ~/airmouse_ws/sim/test_explore.sh                            # practice_6x6.sdf, 600 s budget
    bash ~/airmouse_ws/sim/test_explore.sh rooms_small_4.sdf 300      # another world, 300 s budget
    HEADLESS=1 bash ~/airmouse_ws/sim/test_explore.sh                 # no Gazebo window

One command, start to finish: cleans DDS state, starts Gazebo (window on) + SITL + MAVROS, checks Gazebo
is really stepping, launches SLAM, runs a health check, flies the explorer with no prior knowledge of the
world (recording a rosbag and sampling CPU as it goes), measures SLAM against Gazebo's ground truth, scores
the map like the judges, draws a path picture and tears everything down.

Live logs are in `/tmp/airmouse_sim/explore/`; a kept copy of everything (bag, logs, `slam_eval.csv`,
`path.png`, `map.png`, `score.txt`, `cpu.log`) goes to `sim/runs/<date_time>/`. The `sim/runs` bags are
large (~40 MB each): delete old ones.

Test the explorer's brain first - no ROS, no drone, seconds:

    python3 ~/airmouse_ws/sim/test_explore_logic.py rooms_small_4 rooms_1

If that passes and a flight fails, the problem is flying or SLAM, not searching.

## Worlds

Two generators, both write `sim/worlds/<name>.sdf` and `<name>_truth.json` (true walls, for scoring):

    python3 sim/make_maze.py sim/mazes/practice_6x6.txt          # ASCII maze drawing -> world
    python3 sim/make_rooms.py --seed 4 --wide 3 --depth 10 --name rooms_small_4
    python3 sim/make_rooms.py --seed 1                           # default: 7 rooms wide, 34x16 m, 22 rooms

`make_rooms.py` builds a building of rooms (3-8 m a side) with 1 m doors at random spots; a random
spanning tree guarantees every room is reachable and some walls get extra doors. Same seed = same building.
Worlds so far: `practice_6x6` (the maze), `rooms_small_4` (12x10 m, first-flight size), `rooms_1` (big).

`python3 sim/make_iris_lidar.py` regenerates the drone model (re-run after editing it, then restart the sim).
LiDAR view: `rviz2 -d ~/airmouse_ws/sim/rviz/scan.rviz`.

## Run things by hand

    ~/airmouse_ws/scripts/sim_up.sh [world.sdf] [--headless] --nogps    # Gazebo + SITL + MAVROS + bridges
    ros2 launch airmouse slam.launch.py                                  # Cartographer + pose to ArduPilot + grid mapper
    ros2 run airmouse explorer --ros-args -p altitude:=1.2               # fly (after ~40 s for the EKF)
    ~/airmouse_ws/scripts/sim_down.sh                                    # ALWAYS stop with this

`sim_up.sh` bridges `/scan`, `/clock`, the truth pose and the Gazebo IMU (on sim time, as `/airmouse/imu`)
into ROS, and retries once or twice if Gazebo dies at startup (a WSL GPU glitch). Logs: `/tmp/airmouse_sim/`.
Speed and landing settings for ArduPilot are in `sim/params/indoor.parm` (`nogps_extnav.parm` adds GPS-free).

## Nodes

| Node | What it does |
|---|---|
| `explorer` | **Explores on its own.** Reads `/airmouse/grid`. Default `strategy:=frontier`: flies to the nearest cell whose open side leads to unmapped space (straight legs, re-plans if the map changes), flies to the centre of each room it enters, then goes home and lands. `strategy:=dfs` is the old cell-by-cell tour. Publishes live state on `/airmouse/explorer`. |
| `slam_to_mavros` | Sends Cartographer's `map -> base_link` pose to ArduPilot in place of GPS; sets the EKF origin; asks for faster MAVLink streams. |
| `grid_mapper` | `/map` -> 1 m cells with each side wall/open/unknown -> `/airmouse/grid` (JSON) + `/airmouse/grid_markers` (RViz, built only when something subscribes). |
| `fly_square` | Arms, takes off, flies a list of waypoints, lands. The simple known-good reference node - handy as a smoke test on the real drone. |

Plain-Python modules (no ROS, unit-testable): `explore_logic.py` (search and planning), `grid_logic.py`
(map -> cells), `flight.py` (the shared arm / takeoff / goto / land state machine).

Useful `explorer` parameters: `strategy`, `chain` (re-plan without stopping), `centre_reach` (0 = don't
centre rooms), `settle_time`, `corner_tol` (cut corners by up to this, m), `reached_tol`, `mission_timeout`.

## Tools (sim/)

| Tool | What it does |
|---|---|
| `test_explore.sh` | Full autonomous mission test with scoring, recording and a path picture. See above. |
| `test_explore_logic.py` | The explorer's search on true mazes/buildings and a real stalled map; no ROS. |
| `health_check.py` | 5-10 s check that every link is alive (clock, scan, map, pose to ArduPilot, truth, TF). Never hangs. |
| `slam_eval.py` | SLAM vs Gazebo truth, matched by timestamp. Prints a summary on Ctrl-C / SIGTERM. |
| `plot_run.py` | One picture of a flight: true walls, TRUE path vs SLAM path, markers every 60 s. |
| `score_grid.py` | Grades `/airmouse/grid` like the NIDAR judges (a cell counts only if all 4 sides are right). |
| `save_map.py out.png` | SLAM map as an image with the true walls in blue; also saves `out.npz`. |
| `regrid.py map.npz out.json` | Re-run the grid mapper offline on a saved map with other settings. |
| `sanitize_bag.py in out` | Re-sort a recorded bag by message timestamp (needed before replay, see below). |
| `replay_slam.sh bag [x.lua] [rate]` | Replay a recorded flight into a fresh Cartographer and score it. |
| `sweep_slam.sh bag [rate] [variant...]` | Run `replay_slam.sh` over Cartographer config variants and tabulate. |
| `make_slam_variants.py` | Write those variants (`sim/slam_configs/*.lua`) from `airmouse_2d.lua`. |

## Tuning SLAM offline (no flying)

    python3 sim/sanitize_bag.py sim/runs/<run>/bag sim/runs/<run>/bag_sorted
    bash sim/sweep_slam.sh ~/airmouse_ws/sim/runs/<run>/bag_sorted 2 base noimu ceres_only

Sanitising matters: `ros2 bag record` stores messages in arrival order, which under load is not timestamp
order (12% of the 1 kHz IMU samples in one run were older than the sample before), and Cartographer's pose
extrapolator aborts on that. Replays reproduce a live failure closely (run 20: 0.58 m mean error live,
0.54 m replayed), so a config that wins here is a real fix for that flight - but always check it does not
regress a flight that went well.

## Real hardware: RPLIDAR C1 checklist

The simulator LiDAR is a 360 deg scanner at 10 Hz, 12 m range, 450 samples per turn. The C1 is a similar DTOF
scanner (about 10 Hz, ~12 m, on the order of 500 points a turn - **check the datasheet**), so the pipeline
should carry over; the tuning will not. To bring it up:
- Run a ROS 2 driver for it (e.g. `sllidar_ros2`) so that it publishes `/scan` with `frame_id: lidar_link`,
  the frame `slam.launch.py` attaches 26 cm above `base_link` (measure your real mount and edit `lidar_tf`).
- `ros2 launch airmouse slam.launch.py use_sim_time:=false imu_topic:=/mavros/imu/data` - there is no
  sim/wall clock split on the real drone, so the flight controller's IMU is fine. Replace the `imu_tf` (the sim
  IMU is mounted upside down) with the real mounting orientation, or set `use_imu_data = false`.
- Re-tune Cartographer (`config/airmouse_2d.lua`) on real recordings: real scans are noisy, drop out on dark
  or shiny surfaces, and the IMU sees motor vibration. The sim sweep says the brute-force correlative matcher
  and the IMU hurt in the simulator; that may not hold on hardware.
- Re-check flight speed, corner cutting and landing (`indoor.parm`, `explorer` parameters) on the real airframe.
- There is no ground truth outside the sim: judge SLAM by how close the drone lands to where it took off
  and how well the map matches the arena's known 1 m grid. Keep recording every flight.

## Gotchas

**Running tests**
- **Never run `ros2 topic list/hz/echo` against a live simulation.** A stale ros2 CLI daemon plus FastDDS
  `/dev/shm` files poison discovery and topics silently stop appearing. `test_explore.sh` starts with
  `ros2 daemon stop` and `rm -f /dev/shm/fastrtps_*`; put topic-rate checks in a script before the mission.
- **Always stop with `sim_down.sh`**, never just close terminals. Leftover processes make new ROS programs
  silently receive nothing. `sim_up.sh` runs it first and refuses to start if something survives.
- **Gazebo can die while everything else looks fine** (SITL and MAVROS stay up, every topic goes silent).
  `test_explore.sh` checks `/clock` before it waits on the EKF.
- **Gazebo window blank, title `[warn:copy mode]`, log says `D3D12: Removing Device`:** the WSL GPU state is
  bad. Run `wsl --shutdown` from Windows and reopen. Never force `LIBGL_ALWAYS_SOFTWARE` (4 fps, 100% CPU).
- Never kill just the `gz sim gui` process of a combined `gz sim`: it takes the server with it. `sim_up.sh`
  runs server and window separately.
- A background job in a non-interactive script starts with **SIGINT ignored**: `kill -INT` does nothing.
  Use SIGTERM with a handler, or restore the default first (see `sim/test_explore.sh` for the recorder).
- `timeout ros2 run ...` only kills the `ros2` wrapper, not the node: run the node binary directly.
- A node on `use_sim_time` that stops logging entirely means `/clock` stopped - the simulator froze.
- Editing files from Windows drops the execute bit: `chmod +x` or run with `bash script.sh`.

**Flying**
- ArduPilot sends no position data until asked; the nodes call `/mavros/set_stream_rate` at startup.
- MAVROS uses ENU (x east, y north, z up); ArduPilot uses NED internally.
- Arming is refused for the first ~30-60 s while the simulated EKF settles; the nodes keep retrying.
- SITL defaults to `AHRS_EKF_TYPE 10` (flies on simulator truth and ignores SLAM); `indoor.parm` forces 3.
- ArduPilot 4.7+ renamed `WPNAV_SPEED` -> `WP_SPD` etc. Old names are silently ignored. After touchdown it waits
  `DISARM_DELAY` (default 10 s) before disarming; we set 3 s.
- A MAVROS service request sent before the service is ready is silently lost - check `service_is_ready()`.
- The simulated drone collides like our real 5-inch (35 cm footprint, `sim/models/iris_standoffs_5in`) but flies
  with Iris physics. The stock Iris is ~64 cm wide and crashes in 1 m corridors.

**SLAM**
- Cartographer without odometry needs a low `translation_weight`; the defaults made SLAM lag metres behind.
- In simulation the IMU must come from the Gazebo IMU on sim time (`/airmouse/imu`). MAVROS stamps wall-clock
  time: mixing it with sim time stalls SLAM completely (no map, no position, flight aborts).
- The map frame starts at the takeoff pose: x = the drone's starting heading, y = to its left.
