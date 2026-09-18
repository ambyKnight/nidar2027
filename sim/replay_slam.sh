#!/usr/bin/env bash
# Offline SLAM tuning: replay a recorded flight (LiDAR + sim clock + truth) into a FRESH Cartographer
# with the given .lua settings, and measure its accuracy against the simulator's ground truth.
#
#   replay_slam.sh <bag_dir> [config.lua] [rate]
#   e.g. replay_slam.sh /tmp/airmouse_sim/bag_tour ~/airmouse_ws/src/airmouse/config/airmouse_2d.lua 2
#
# Needs nothing else running (it refuses to start on top of a live simulation).
BAG=${1:?bag dir}
LUA=${2:-$HOME/airmouse_ws/src/airmouse/config/airmouse_2d.lua}
RATE=${3:-2}
OUT=/tmp/airmouse_sim/replay
mkdir -p "$OUT"
"$(dirname "$0")/../scripts/sim_down.sh" > /dev/null || { echo "old processes still running - aborting"; exit 1; }

P="--ros-args -p use_sim_time:=true"
# The live config uses the IMU (sim time, /airmouse/imu). Replay it too, with the same upside-down mount
# transform slam.launch.py publishes - without it Cartographer waits forever for IMU data and never
# produces a pose (run 10). Variants that switch the IMU off skip all of this.
IMU_TOPICS=""; IMU_REMAP=""
# the LAST assignment wins (variant files append overrides after the base config)
if grep -E 'use_imu_data *=' "$LUA" | tail -1 | grep -q true; then
  IMU_TOPICS="/airmouse/imu"; IMU_REMAP="-r imu:=/airmouse/imu"
  ros2 run tf2_ros static_transform_publisher --roll 3.14159265 --frame-id base_link --child-frame-id imu_link $P \
      > "$OUT/imu_tf.log" 2>&1 &
fi
# bag /clock is remapped to /clock_raw and relayed forward-only (see clock_relay.py)
python3 "$(dirname "$0")/clock_relay.py" > "$OUT/clock.log" 2>&1 &
ros2 run tf2_ros static_transform_publisher --z 0.26 --frame-id base_link --child-frame-id lidar_link $P \
    > "$OUT/tf.log" 2>&1 &
ros2 run cartographer_ros cartographer_node -configuration_directory "$(dirname "$LUA")" \
    -configuration_basename "$(basename "$LUA")" $P $IMU_REMAP > "$OUT/carto.log" 2>&1 &
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0 $P \
    > "$OUT/grid.log" 2>&1 &
sleep 3
python3 -u "$(dirname "$0")/slam_eval.py" --idle 100000 --csv "$OUT/eval.csv" > "$OUT/eval.txt" 2>&1 &
EVAL=$!
sleep 1
# only the raw inputs + truth: SLAM output (/tf) is recomputed, not replayed
ros2 bag play "$BAG" --rate "$RATE" --topics /scan /clock /model/iris_lidar/pose $IMU_TOPICS \
    --remap /clock:=/clock_raw > "$OUT/play.log" 2>&1
echo "time jumps seen by Cartographer: $(grep -c 'jump back in time' "$OUT/carto.log")"
# the bag may start with the drone sitting on the ground (test_explore.sh records from the health check), so
# do not let slam_eval stop on "idle": let it run to the end of the bag, then ask it to report
sleep 4
kill -TERM $EVAL 2>/dev/null
wait $EVAL
cat "$OUT/eval.txt"
timeout -k 3 30 python3 "$(dirname "$0")/save_map.py" "$OUT/map.png" | tail -1
"$(dirname "$0")/../scripts/sim_down.sh" > /dev/null
