#!/usr/bin/env bash
# End-to-end test of AUTONOMOUS exploration: the drone is told nothing about the maze.
#
#   sim/test_explore.sh [world.sdf] [mission_timeout_s]     (Gazebo window on; HEADLESS=1 to hide it)
#   EXPLORER_ARGS="-p settle_time:=1.0 -p drift_penalty:=1.0" sim/test_explore.sh    (extra explorer parameters)
#
# Default world: rooms_small_4 (12 x 10 m, big rooms and 1 m walls - the NIDAR arena style, within its 15 x 15 m).
#
# Unlike the plan_tour.py runs, no route is computed in advance - the explorer reads only
# /airmouse/grid, the map it is building as it flies. At the end we score that map like the judges.
#
# Everything here has a hard time limit and prints progress as it goes: an unattended run that
# hangs is how we lost whole evenings before (see NOTES "Reliability").
set -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
# ROS's own setup.bash reads unset variables, so it must be sourced BEFORE `set -u`, or the script
# dies on its first line with "AMENT_TRACE_SETUP_FILES: unbound variable" and nothing else happens.
source "$HERE/../scripts/env.sh"          # ~/.bashrc is interactive-only; see that file
set -u

WORLD=${1:-rooms_small_4.sdf}
EXPLORER_ARGS=${EXPLORER_ARGS:-}
MISSION_TIMEOUT=${2:-600}
OUT=/tmp/airmouse_sim/explore
BIN=$HOME/airmouse_ws/install/airmouse/lib/airmouse
rm -rf "$OUT"; mkdir -p "$OUT"            # stale logs from a failed run are worse than no logs
# Everything worth keeping from a run goes here (bag, logs, picture) - $OUT is wiped by the next run
RUNDIR=$HOME/airmouse_ws/sim/runs/$(date +%m%d_%H%M%S)
mkdir -p "$RUNDIR"

BAG_PID=""; CPU_PID=""
cleanup() {
  # stop the recorder / CPU sampler too if the run is cut short, or they outlive it
  [ -n "$BAG_PID" ] && kill -INT "$BAG_PID" 2>/dev/null     # the recorder was started with SIGINT restored
  [ -n "$CPU_PID" ] && kill "$CPU_PID" 2>/dev/null           # a background shell loop ignores SIGINT
  "$HERE/../scripts/sim_down.sh" > /dev/null 2>&1
}
trap cleanup EXIT

die() { echo; echo "FAILED: $*"; exit 1; }

# A stale ros2 CLI daemon or leftover FastDDS shared memory poisons discovery, and the symptom is
# not an error - topics simply never appear (this cost two runs: "Gazebo not running", then
# "no /clock" while Gazebo was demonstrably stepping on the gz side). Always start from clean.
ros2 daemon stop > /dev/null 2>&1 || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo "=== [1/6] simulation up ($WORLD, no GPS) $(date +%H:%M:%S)"
"$HERE/../scripts/sim_up.sh" "$WORLD" ${HEADLESS:+--headless} --nogps > "$OUT/sim_up.log" 2>&1 \
    || { tail -20 "$OUT/sim_up.log"; die "sim_up.sh returned an error"; }

# Gazebo dying is the failure that wasted a run: SITL and MAVROS stay up, so everything LOOKS
# started while every topic is silent. Check the simulator itself before waiting 40 s for an EKF.
echo "=== [2/6] simulator alive? $(date +%H:%M:%S)"
pgrep -f "gz sim" > /dev/null || { tail -15 /tmp/airmouse_sim/gazebo.log; die "Gazebo is not running"; }
# waits for the FIRST /clock message (up to 60 s) - this replaced a blind `sleep 20` before this step
timeout 60 ros2 topic echo /clock --once > /dev/null 2>&1 || die "no /clock - Gazebo is not stepping"
echo "Gazebo is running and publishing /clock"

echo "=== [3/6] SLAM + grid mapper $(date +%H:%M:%S)"
ros2 launch airmouse slam.launch.py > "$OUT/slam.log" 2>&1 &

# Was a blind `sleep 40` then a 10 s health check. Now: wait until every link (clock, scan, SLAM map and pose,
# ArduPilot, ToF) is actually up, then measure for 5 s. The EKF still needs time before ArduPilot will arm, but
# the explorer keeps retrying the arm, so there is nothing to gain by waiting for it here.
echo "=== [4/6] health check (waits for every link) $(date +%H:%M:%S)"
timeout -k 5 150 python3 "$HERE/health_check.py" --wait 120 --seconds 5 2>&1 | tee "$OUT/health.txt"
grep -q "DEAD" "$OUT/health.txt" && echo "WARNING: something is DEAD above - flying anyway, watch it"

# Is Cartographer actually USING the IMU? use_imu_data can be true while the topic is unmapped or
# too slow, and SLAM then runs exactly as before with no error to say so. Measure it once, here,
# before the flight - never by poking the live sim mid-flight (that froze a whole run).
echo "--- IMU into SLAM ---"
grep -h "asked ArduPilot" "$OUT/slam.log" 2>/dev/null | head -2
# Cartographer logs its measured IMU rate every ~15 s: wait for the first one (was a blind `sleep 16`)
for _ in $(seq 1 40); do grep -q "imu rate" "$OUT/slam.log" 2>/dev/null && break; sleep 0.5; done
grep -h "imu rate" "$OUT/slam.log" 2>/dev/null | tail -1 | sed -E "s/.*(imu rate: [0-9.]+ Hz).*/cartographer \1/" \
    | grep . || echo "WARNING: Cartographer reports NO imu rate - the IMU is not reaching SLAM"
grep -icE "imu.*(missing|dropp|older|unable)" "$OUT/slam.log" 2>/dev/null \
    | sed "s/^/cartographer IMU complaints: /"

# Record the whole flight so any stretch can be replayed offline through a fresh Cartographer
# (sim/replay_slam.sh). A background job in a non-interactive script starts with SIGINT IGNORED, so
# `kill -INT` would do nothing and the bag would never be finalised: restore the default first.
python3 -c 'import signal,os,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    ros2 bag record -o "$RUNDIR/bag" /scan /clock /tf /tf_static /airmouse/imu \
    /model/iris_lidar/pose /mavros/local_position/pose /mavros/vision_pose/pose /airmouse/tof/front \
    /airmouse/tof/back /airmouse/tof/left /airmouse/tof/right \
    > "$OUT/bag.log" 2>&1 &
BAG_PID=$!

# Where does the CPU go during a real flight? top's SECOND iteration is the live rate (the first is
# the average since the process started), so sample twice and keep the second. See $OUT/cpu.log.
{ while true; do
    echo "--- $(date +%H:%M:%S)"
    top -b -n 2 -d 2 -o %CPU -w 160 | awk '/^top -/{n++} n==2' | head -16
    sleep 8
  done; } > "$OUT/cpu.log" 2>&1 &
CPU_PID=$!

# SLAM error against Gazebo truth, for the whole flight (SIGINT makes it print its summary)
python3 -u "$HERE/slam_eval.py" --idle 100000 --csv "$OUT/slam_eval.csv" > "$OUT/slam_eval.txt" 2>&1 &
EVAL_PID=$!

echo "=== [5/6] EXPLORING (no prior knowledge) $(date +%H:%M:%S)${EXPLORER_ARGS:+  args: $EXPLORER_ARGS}"
timeout -k 15 "$((MISSION_TIMEOUT + 180))" "$BIN/explorer" --ros-args \
    -p use_sim_time:=true -p altitude:=1.2 -p mission_timeout:="$MISSION_TIMEOUT.0" $EXPLORER_ARGS \
    2>&1 | tee "$OUT/explorer.log"
rc=${PIPESTATUS[0]}
echo "explorer exit code: $rc"

kill -INT "$EVAL_PID" 2>/dev/null; wait "$EVAL_PID" 2>/dev/null
kill -INT "$BAG_PID" 2>/dev/null; wait "$BAG_PID" 2>/dev/null
kill "$CPU_PID" 2>/dev/null; wait "$CPU_PID" 2>/dev/null
echo "=== SLAM accuracy during the flight ==="
cat "$OUT/slam_eval.txt"

echo "=== [6/6] scoring the map it built $(date +%H:%M:%S)"
timeout -k 5 60 python3 "$HERE/score_grid.py" --truth "$HERE/worlds/${WORLD%.sdf}_truth.json" 2>&1 | tee "$OUT/score.txt" | tail -25
timeout -k 3 30 python3 "$HERE/save_map.py" "$OUT/map.png" --truth "$HERE/worlds/${WORLD%.sdf}_truth.json" 2>&1 | tail -1

# One picture of the flight (true path vs SLAM path over the true walls), then keep the run
timeout -k 3 60 python3 "$HERE/plot_run.py" "$OUT/slam_eval.csv" "$OUT/path.png" \
    --truth "$HERE/worlds/${WORLD%.sdf}_truth.json" 2>&1 | grep -v -i "warn" | tail -1
cp "$OUT"/*.log "$OUT"/*.txt "$OUT"/*.csv "$OUT"/*.png "$RUNDIR"/ 2>/dev/null

echo
echo "=== summary ==="
grep -E "(step|backtrack|exploration|home again|out of time|ABORT)" "$OUT/explorer.log" | tail -10
grep -m1 "visited .* cells:" "$OUT/explorer.log"
grep -m1 -o "mission time: .*" "$OUT/explorer.log" || echo "mission time: none (never landed normally)"
[ -n "$EXPLORER_ARGS" ] && echo "explorer args: $EXPLORER_ARGS"
echo "WALL GUARD fired: $(grep -c "WALL GUARD:" "$OUT/explorer.log" 2>/dev/null || echo 0) times"
echo "camera decisions: $(grep -c "camera:" "$OUT/explorer.log" 2>/dev/null || echo 0)"
echo "picture: $OUT/path.png"
echo "kept: $RUNDIR/  (bag, logs, picture)"
exit $rc
