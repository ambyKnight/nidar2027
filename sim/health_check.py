#!/usr/bin/env python3
"""Quick health check of the running simulation: is data flowing through every link of the chain?

    python3 health_check.py [--seconds 10] [--wait 120]

--wait N: first wait (up to N s) until EVERY link has delivered at least one message, then measure. This replaces
the fixed sleeps test_explore.sh used to guess with: it returns as soon as the chain is up, and a link that never
comes up is reported DEAD after N s instead of after a sleep that was too short.

It also measures the simulator's real-time factor from /clock and converts the sensor rates into SIM time. That
is the number that matters: the LiDAR is 10 Hz in sim time, so anything much under 10 means Gazebo is DROPPING
scans under load - and a starved Cartographer slips whole cells and wrecks the flight (NOTES "Sim sensor load").

Listens for a few seconds (never hangs) and reports message rates for:
  /clock (Gazebo sim time) -> /scan (LiDAR) -> /map (SLAM) -> TF map->base_link (SLAM pose)
  -> /mavros/vision_pose/pose (pose sent to ArduPilot) -> /mavros/state (ArduPilot link)
Exit code 1 if any link is dead.
"""
import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

CHECKS = [("/clock", Clock), ("/scan", LaserScan), ("/map", OccupancyGrid),
          ("/mavros/vision_pose/pose", PoseStamped), ("/mavros/state", State),
          ("/model/iris_lidar/pose", PoseStamped),  # sim ground truth
          ("/airmouse/tof/front", LaserScan), ("/airmouse/tof/back", LaserScan),   # edge ToF (wall guard)
          ("/airmouse/tof/left", LaserScan), ("/airmouse/tof/right", LaserScan)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--wait", type=float, default=0.0, help="wait up to this long for every link first")
    args = ap.parse_args()

    rclpy.init()
    node = Node("health_check")
    counts = {name: 0 for name, _ in CHECKS}
    last_state = {}
    sim_span = {}
    for name, kind in CHECKS:
        def cb(msg, name=name):
            counts[name] += 1
            if name == "/mavros/state":
                last_state["s"] = msg
            elif name == "/clock":
                t = msg.clock.sec + msg.clock.nanosec * 1e-9
                sim_span.setdefault("first", t)
                sim_span["last"] = t
        qos = qos_profile_sensor_data if name in ("/scan", "/clock", "/model/iris_lidar/pose") \
            or name.startswith("/airmouse/tof/") else 10
        node.create_subscription(kind, name, cb, qos)
    buf = Buffer()
    TransformListener(buf, node)

    def tf_found():
        try:
            buf.lookup_transform("map", "base_link", Time())
            return True
        except TransformException:
            return False

    if args.wait > 0:
        start, deadline, last_print = time.time(), time.time() + args.wait, 0.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = [n for n, c in counts.items() if c == 0] + ([] if tf_found() else ["TF map->base_link"])
            if not missing:
                print(f"  all links up after {time.time() - start:.0f} s")
                break
            if time.time() - last_print > 10:
                last_print = time.time()
                print(f"  waiting ({time.time() - start:.0f} s) for: {', '.join(missing)}", flush=True)
        for name in counts:          # measure rates from here on, not from the wait
            counts[name] = 0

    sim_span.clear()
    tf_ok = 0
    wall_start = time.time()
    end = wall_start + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            buf.lookup_transform("map", "base_link", Time())
            tf_ok += 1
        except TransformException:
            pass

    healthy = True
    wall = max(1e-3, time.time() - wall_start)
    sim = sim_span.get("last", 0.0) - sim_span.get("first", 0.0)
    rtf = sim / wall if sim > 0 else 0.0
    for name, _ in CHECKS:
        rate = counts[name] / wall
        ok = counts[name] > 0
        healthy &= ok
        # sensors are configured in SIM time; dividing by the real-time factor un-does the slow-motion so the
        # number can be compared with the 10 Hz the sensor is supposed to produce
        per_sim = f"  = {rate / rtf:5.1f} /sim-s" if rtf > 0 and name.startswith(("/scan", "/airmouse/tof")) else ""
        print(f"  {'OK  ' if ok else 'DEAD'} {name:<28} {rate:6.1f} msg/s{per_sim}")
    print(f"  {'OK  ' if tf_ok else 'DEAD'} {'TF map -> base_link':<28} {'found' if tf_ok else 'never found'}")
    healthy &= tf_ok > 0
    if rtf > 0:
        scan_sim = counts["/scan"] / wall / rtf
        print(f"  simulator running at {rtf * 100:.0f}% of real time; LiDAR delivering {scan_sim:.1f} of 10 Hz (sim)")
        if scan_sim < 8.0:
            print(f"  WARNING: {(1 - scan_sim / 10) * 100:.0f}% of LiDAR scans are being DROPPED - SLAM will drift "
                  f"and this flight's map cannot be trusted. Close other GPU/CPU load, or run with HEADLESS=1.")
            healthy = False
    try:
        cores = os.cpu_count() or 1
        load = os.getloadavg()[0]
        print(f"  CPU load {load:.1f} over {cores} cores{'  - HEAVILY LOADED' if load > cores * 0.8 else ''}")
    except OSError:
        pass
    if "s" in last_state:
        s = last_state["s"]
        print(f"  ArduPilot: connected={s.connected} armed={s.armed} mode={s.mode}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if healthy else 1)


if __name__ == "__main__":
    main()
