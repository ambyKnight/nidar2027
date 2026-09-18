#!/usr/bin/env bash
# Start the whole simulation in the background: Gazebo (with window) + ArduPilot SITL + MAVROS
# + the LiDAR bridge (Gazebo `scan` -> ROS /scan).
# Then run our code in this terminal, e.g.:  ros2 run airmouse fly_square
# Stop everything with:  ~/airmouse_ws/scripts/sim_down.sh
#
# Usage: sim_up.sh [world.sdf] [--headless] [--nogps]     (default: rooms_small_4.sdf)
#   e.g. sim_up.sh iris_runway.sdf   for ArduPilot's open runway (no maze, no LiDAR)
#   --nogps   ArduPilot ignores GPS and waits for SLAM position (then: ros2 launch airmouse slam.launch.py)
set -e
WORLD=rooms_small_4.sdf
GZ_ARGS=""
PARAMS=Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm,$HOME/airmouse_ws/sim/params/indoor.parm
for arg in "$@"; do
  case $arg in
    --headless) GZ_ARGS="-s" ;;
    --nogps) PARAMS=$PARAMS,$HOME/airmouse_ws/sim/params/nogps_extnav.parm,$HOME/airmouse_ws/sim/params/optflow.parm ;;
    *) WORLD=$arg ;;
  esac
done
WORLD_NAME=${WORLD%.sdf}      # make_maze.py names the <world> after the file, and the IMU's gz topic contains it
LOGS=/tmp/airmouse_sim
mkdir -p "$LOGS"

# Old processes from a previous run (e.g. a second MAVROS) silently break everything - clean first
"$(dirname "$0")/sim_down.sh" > /dev/null || { echo "Could not stop old simulation processes - aborting"; exit 1; }

echo "Starting Gazebo with $WORLD ..."
# Server and GUI are separate processes on purpose: with one `gz sim` wrapper, closing/killing the GUI
# takes the whole simulation down with it. Both render on the GPU via Mesa d3d12 (scripts/env.sh);
# do NOT force LIBGL_ALWAYS_SOFTWARE - that is the ~4 fps, 100% CPU path we already fixed once.
# The WSL GPU path occasionally kills a fresh Gazebo at startup ("D3D12: Removing Device" then a GLX
# error) and everything downstream looks started while nothing steps. Check it lived, retry if not.
start_gz() {   # start_gz <log> <what> <seconds to wait before judging it alive> <gz args...>
  local log=$1 what=$2 wait_s=$3; shift 3
  for attempt in 1 2 3; do
    gz sim "$@" > "$log" 2>&1 &
    local pid=$!
    sleep "$wait_s"
    if kill -0 "$pid" 2>/dev/null; then return 0; fi
    echo "Gazebo $what died at startup (attempt $attempt): $(grep -m1 -E 'Removing Device|GLX|rror' "$log" | cut -c1-100) - retrying"
    # clean up only what failed: retrying the window must not take the healthy server down with it
    if [ "$what" = server ]; then "$(dirname "$0")/sim_down.sh" > /dev/null 2>&1 || true
    else pkill -f "gz sim gui" 2>/dev/null || true; fi
    sleep 3
  done
  echo "Gazebo $what would not start after 3 tries - see $log"; return 1
}
start_gz "$LOGS/gazebo.log" server 6 -s -r -v2 "$WORLD" || exit 1
if [ -z "$GZ_ARGS" ]; then
  start_gz "$LOGS/gazebo_gui.log" window 14 -g -v2 || echo "WARNING: no Gazebo window - continuing without it"
fi

echo "Starting ArduPilot SITL (physics from Gazebo) ..."
# -w WIPES the parameter EEPROM first. SITL keeps parameters in ~/ardupilot/eeprom.bin BETWEEN RUNS, and those
# take precedence over --defaults, so without this a flight runs on whatever every previous session left behind
# (2026-09-18: a day of runs accumulated settings, and re-flying the exact code of the best run did not reproduce
# it). Every flight must start from sim/params/*.parm alone, or no result means anything.
(cd ~/ardupilot && build/sitl/bin/arducopter --model JSON -w --defaults "$PARAMS" \
    -I0 > "$LOGS/sitl.log" 2>&1 &)
sleep 3

echo "Starting MAVROS ..."
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 > "$LOGS/mavros.log" 2>&1 &

echo "Starting Gazebo -> ROS bridge (/scan LiDAR, /airmouse/range_front, /clock sim time, /airmouse/imu on SIM time, /model/iris_lidar/pose truth) ..."
ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /model/iris_lidar/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \
    /range_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /world/$WORLD_NAME/model/iris_lidar/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    --ros-args -r /range_front:=/airmouse/range_front -r /world/$WORLD_NAME/model/iris_lidar/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu:=/airmouse/imu \
    > "$LOGS/bridge.log" 2>&1 &

echo
echo "Simulation up. Logs in $LOGS/"
echo "See the LiDAR:  rviz2 -d ~/airmouse_ws/sim/rviz/scan.rviz"
echo "Fly:            ros2 run airmouse fly_square     (open runway only - it will hit maze walls!)"
