#!/usr/bin/env bash
# Stop everything sim_up.sh / slam.launch.py / our nodes / test tools started, then check nothing is left.
# Exit code 1 if something survived (so scripts can refuse to start on top of stale processes).
#
# Matches full command lines (process names are cut to 15 chars, so -x would miss e.g.
# cartographer_occupancy_grid_node). Never kills its own ancestors (the shell/script that called it,
# whose command line may mention these names).
# "airmouse/lib/airmouse" matches EVERY node of our package by its install path, present and future. Listing
# nodes by name is how eight survivor_tagger processes survived every run on 2026-09-18 (that node was added
# after this list was written): they piled up for two hours, ate the CPU, and Gazebo then dropped 90% of the
# LiDAR scans - which starves SLAM, and a starved SLAM flies the drone into walls. Never list our nodes by name.
PATTERNS=("[g]z sim" "[b]in/arducopter" "[m]avros_node" "[r]os2 launch mavros" "[r]os2 launch airmouse"
          "[p]arameter_bridge" "[c]artographer_node" "[c]artographer_occupancy_grid_node"
          "[s]tatic_transform_publisher" "[a]irmouse/lib/airmouse" "[f]ly_square" "[c]ompare_pose.py"
          "[s]lam_eval.py" "[c]lock_relay.py" "[r]viz2"
          "[b]in/ros2 topic" "[b]in/ros2 service" "[b]in/ros2 run" "[t]f2_echo" "[b]in/ros2 bag")

# PIDs of this script and everything above it - never touch these
ancestors=" "
pid=$$
while [ -n "$pid" ] && [ "$pid" -gt 1 ]; do
  ancestors+="$pid "
  pid=$(ps -o ppid= -p "$pid" | tr -d ' ')
done

targets() { for p in "${PATTERNS[@]}"; do pgrep -f "$p"; done | sort -u | while read -r t; do
              [[ "$ancestors" == *" $t "* ]] || echo "$t"; done; }

t=$(targets); [ -n "$t" ] && kill $t 2>/dev/null
sleep 2
t=$(targets); [ -n "$t" ] && kill -9 $t 2>/dev/null   # anything that ignored the polite signal
sleep 1

left=$(targets)
if [ -n "$left" ]; then
  echo "WARNING: still running:"; ps -o pid,etime,args -p $left | cut -c1-120
  exit 1
fi

# Force-killed ROS processes leave FastDDS shared-memory files behind in /dev/shm; new ROS
# programs can then silently stop receiving data. Stop the ros2 CLI daemon and clear them.
ros2 daemon stop > /dev/null 2>&1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "Simulation stopped (nothing left running)."
