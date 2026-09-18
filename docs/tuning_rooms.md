# Tuning task: make the mission faster on the NIDAR-style arena without making it worse

**Who this is for:** an agent working on its own to tune parameters. Read all of it before you start. Every
rule under "Hard rules" comes from a run we lost or a crash we had (see NOTES.md).

> ## READ THIS FIRST: a run is only valid if the machine is healthy
> On 2026-09-18 a whole evening of flights was wasted on a bug that looked exactly like bad algorithms. Nodes
> leaked from stopped runs (eight of them, the oldest alive for two hours), the load hit 12 of 16 cores, and
> Gazebo silently DROPPED LiDAR SCANS. A starved Cartographer slips whole cells, the drone flies to where it
> wrongly believes the cells are, and it hits a wall. The maps fell from 97% to ~25%, and re-flying the exact
> code of the best run reproduced the failure - which is the only reason we stopped blaming the code.
>
> Before every run: `scripts/sim_down.sh` must print "nothing left running", and `pgrep -f airmouse/lib/airmouse`
> must print nothing. During the run, the health check prints `LiDAR delivering X of 10 Hz (sim)`: it must be 9+.
> The run now refuses to fly below 8. **If you ever see `scans are being dropped`, throw the run away. It measures
> your machine's load, not your change.** Do not tune around it, and do not raise the thresholds to make it pass.

## The goal

Cut the **mission time** on `rooms_small_4.sdf`: 12 x 10 m, 120 cells, big rooms joined by doors in 1 m walls. That
is what the NIDAR arena looks like (max 15 x 15 m, 2 x 2 m rooms, 1 m corridors), and it is the default world of every
script. Keep everything else at least as good as the baseline below.

**Guard against overfitting:** once you have a candidate setting, fly it once on `rooms_small_5.sdf` (20 x 10 m, a
different layout; larger than NIDAR allows, but a fair check). If it only wins on rooms_small_4, it is not a real win.

A run only counts as better if ALL of these hold:

| Metric | Where to read it | Must be |
|---|---|---|
| explorer exit code | `explorer exit code:` in the run output | `0` |
| crashes / aborts | no `ABORT`, `stuck`, `abort` in `explorer.log` | none |
| map | `cells fully correct` in `score.txt` | >= baseline |
| SLAM error | `max` in the `SLAM accuracy` block | < 0.30 m (worse than that and the drone is guessing where walls are) |
| wall guard firings | `WALL GUARD` lines | <= baseline, and none with the ToF reading under 0.20 m |
| camera coverage | `camera_seen` in the last `/airmouse/explorer` state (explorer.log) | not lower than baseline |
| mission time | `mission time:` in the run summary | lower than baseline |

The competition scores the map (220 pts), survivors (240) and a safe exit (50). Finishing under 15 min only earns a
25-point bonus. We are already well under 15 min, so **a run that is faster but maps worse is a regression**. So is
one that is faster but crashes (-50). When in doubt, keep the slower, safer setting.

## Baseline (2026-09-18, all defaults, BEFORE the starvation bug was found)

**Caveat: this baseline was flown on a healthy machine but several later runs were not, so re-fly it first and
use YOUR number.** It should reproduce now that the leak is fixed; if it does not, stop and report rather than
tuning against a moving target.

Run folder: `sim/runs/0918_201734` (defaults, 2026-09-18, ToF at 10 Hz).

| Metric | Value |
|---|---|
| mission time | **116.2 s** (sim, exploring start to landed) |
| explorer exit code | 0, autonomous exit flown (out via -x, landed 1.2 m outside) |
| map | **116/120 cells correct (97%)**, 476/480 sides (99%) |
| SLAM error | mean 0.058 m, p95 0.166 m, max 0.180 m |
| wall guard | 1 firing (front, 0.10 m, drone nearly stationary) |
| LiDAR rate | 10.0 Hz throughout (see below - this matters more than any parameter) |
| cells flown | 31 of 120 |

**Read this before tuning: the LiDAR rate is the thing that breaks runs.** The run before this one aborted with a
SLAM error of 3.6 m, because only ~60% of scans reached Cartographer (4.4 Hz against the 7.3 expected at that sim
speed): starved, the scan matcher slips whole cells in rooms that look alike, the drone flies to wrong cell centres,
and the wall guard aborts the mission. The cause was GPU load from the four ToF sensors at 20 Hz; they are 10 Hz now.
The explorer logs `LiDAR X.X Hz` every 5 s and warns below 6 Hz. **If a run shows that warning, throw the run away -
it measures your machine's load, not your change.** Close other GPU/CPU-heavy programs, or use `HEADLESS=1`.

## How to run one experiment

```bash
cd ~/airmouse_ws
colcon build --symlink-install --packages-select airmouse     # only if you changed Python code
python3 sim/test_explore_logic.py                             # offline check, must end in ALL PASS
EXPLORER_ARGS="-p settle_time:=1.0 -p done_grace:=3.0" \
    sim/test_explore.sh rooms_small_4.sdf 600 > /tmp/run.log 2>&1
grep -E "explorer exit code|mission time|cells fully correct|max |WALL GUARD|ABORT" /tmp/run.log
```

- `EXPLORER_ARGS` passes extra `-p name:=value` explorer parameters without editing any file. **Use this for
  explorer parameters. Do not change the defaults in explorer.py until a setting has passed at least 2 runs.**
- A run takes about 5 min: under 1 min of startup, the flight, then scoring. Everything is kept in
  `sim/runs/<MMDD_HHMMSS>/` (logs, `score.txt`, `path.png` = true path vs SLAM path).
- **Each run varies.** The same settings can differ by 10-20 s. Never judge a change on one run: repeat any
  apparent win once before believing it.

## What to tune, best bets first

Change **ONE thing per run**, and record every run in the results table at the bottom (wins and failures).

### 1. Hover and decision waits (explorer parameters, lowest risk)
| Parameter | Default | Try | What it does | Risk if too low |
|---|---|---|---|---|
| `settle_time` | 2.0 s | 1.5, 1.0 | hover in a new cell before deciding | deciding on a map that has not caught up yet (see run 7 in NOTES) |
| `min_grid_updates` | 2 | 1 | fresh grid maps needed before a decision | same as above |
| `done_grace` | 5.0 s | 3.0 | how long "nothing left to explore" is re-checked before going home | ending the mission early on a half-built map (run 7 landed after 1 cell) |
| `done_updates` | 3 | 2 | fresh grids during that re-check | same |

### 2. Cornering (explorer parameters, medium risk)
| Parameter | Default | Try | What it does | Risk |
|---|---|---|---|---|
| `corner_tol` | 0.25 m | 0.30 | how close to a corner cell before turning (less braking) | cutting corners: the drone is 23 cm wide and SLAM is ~0.1 m off. **Never above 0.30** |
| `pass_tol` | 0.4 m | 0.45 | how close to a cell's centre counts as having passed it mid-leg | marking cells visited that were not really passed |

### 3. Route choice (explorer parameters, low risk, may not help)
| Parameter | Default | Try | Notes |
|---|---|---|---|
| `drift_penalty` | 0.0 | 1.0 | avoids hops into cells with a wall on one side only (where SLAM broke in the old corridor maze). Offline on rooms_small_4: same time, 7 -> 5 such hops. Try it for safety, not speed |
| `centre_reach` | 6 | 3, 0 | how far the drone detours to fly into a room's centre. The LiDAR maps a room from its doorway, so a smaller value may skip detours. Watch the map score: walls seen only from the doorway can come out wrong |
| `cam_reach` | 3.0 m | 4.0 | fewer camera detours. Only if the survivor camera really detects a person at 4 m, otherwise it will miss survivors. Leave it alone unless told otherwise |

Offline estimate for any explorer change (seconds, no sim): `python3 sim/test_explore_logic.py` prints `~X min`
for rooms_small_4 (add `rooms_small_5` as an argument for the second world). It does not model SLAM, so it can only rank routes, not predict real times.

### 4. Flight speed (ArduPilot parameters, HIGHEST risk, do last)
In `sim/params/indoor.parm`: `WP_SPD 1.0` (m/s), `WP_ACC 2.0` (m/s^2).
- Try `WP_SPD 1.25`, then `1.5`. If you change `WP_SPD`, also pass the same value as `-p wp_speed:=...` (it is used
  for the time-home estimate) and `WP_ACC` as `-p brake_acc:=...` (the wall guard's braking distance).
- Narrow corridors are where SLAM failed before (runs 8, 12, 13), and the 1 m corridors here are the same. Faster flight gives the scan matcher less overlap between
  scans. **Stop raising speed at the first run where SLAM max error goes over 0.20 m or the guard fires.**
- `git diff sim/params/indoor.parm` before you finish: leave it at the best setting that passed, or back at 1.0.

## Do NOT touch
- The wall-guard values `tof_stop`, `tof_latency`, `guard_limit`: they are the safety net, not tuning knobs.
- `reached_tol`, `altitude`, `entrance_side`, `auto_exit`, `exit_distance`.
- Cartographer config (`src/airmouse/config/*.lua`) and SLAM launch files: SLAM tuning is separate work (NOTES "SLAM tuning").
- The worlds themselves (`sim/worlds/*`), the scorer (`sim/score_grid.py`), `sim/slam_eval.py`: changing
  how we measure is not the same as getting better.

## Hard rules (each one cost us a run)
1. **Never run `ros2 topic echo/hz/list` or open rviz while a run is flying.** It froze run 9.
2. **Never edit `sim/test_explore.sh` or `scripts/*.sh` while a run is in progress.** Bash reads scripts as it
   goes, so an edit corrupts the running flight.
3. **Never kill the Gazebo GUI process by hand.** If the window is blank with `[warn:copy mode]`, the WSL GPU is in
   a bad state: stop, tell the user, they run `wsl --shutdown` from Windows.
4. One run at a time. `test_explore.sh` shuts the sim down at the end, so the window closing is normal.
5. If a run fails to START (never takes off, `DEAD` in the health check, no `/clock`), that is an
   infrastructure problem, not a result. Rerun once. If it happens again, stop and report, don't tune around it.
6. Git: work on branch `tuning/rooms` (create it from `main`). Commit the results table after every few
   runs. **Never push, never force, never rewrite history.** The user reviews and merges.

## Known-unverified: three changes that never had a fair flight

All three are in the code with their offline evidence, and all three were flown only while the machine was
starved, so their flight results mean nothing. Re-measure them before judging - each is one run:

| Change | Default | Offline evidence | What a fair flight would show |
|---|---|---|---|
| Manhattan map fit (`manhattan:=true`, grid_mapper) | on | `sim/test_grid_fit.py`: injected drifts of 3-9 deg and up to 0.45 m scored 42-83/120 raw and **120/120 fitted** | whether the map score survives real (non-rigid) drift |
| Locality bias (`locality:=5.0`) | 5.0 | rooms_small_4 44 -> 37 moves, longest trip 13 -> 7 cells; rooms_small_5 89 -> 65, 25 -> 11 | flight time down, map unchanged |
| Velocity control (`control:=velocity`) | **off** | none - it has never flown cleanly | it commanded <=1.4 m/s and truth measured 3.3 m/s, but that run was starved. Bring it up in a HOVER first, not a mission |

Also unverified: that cutting the LiDAR from 450 to 230 points destroys SLAM (it did, in a starved run). Until
someone re-measures it on a healthy machine, leave it at 450 - the comment in `make_iris_lidar.py` explains why.

## When you are done
Leave:
1. The results table below, filled in (every run, including failures).
2. At most 3 new defaults changed in `explorer.py` / `indoor.parm`, each backed by at least 2 passing runs, in
   one commit whose message lists the before/after mission time.
3. A 5-line summary at the top of this file: best time vs baseline, what helped, what did nothing, what failed.

## Results

| # | run folder | change (vs defaults) | exit | mission time | cells correct | SLAM max | guard | notes |
|---|---|---|---|---|---|---|---|---|
| 0 | `0918_201734` | none (baseline) | 0 | 116.2 s | 116/120 (97%) | 0.180 m | 1 | autonomous exit flown |
