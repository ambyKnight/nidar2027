#!/usr/bin/env python3
"""Quick health check of the running simulation: is data flowing through every link of the chain?

    python3 health_check.py [--seconds 10]

Listens for a few seconds (never hangs) and reports message rates for:
  /clock (Gazebo sim time) -> /scan (LiDAR) -> /map (SLAM) -> TF map->base_link (SLAM pose)
  -> /mavros/vision_pose/pose (pose sent to ArduPilot) -> /mavros/state (ArduPilot link)
Exit code 1 if any link is dead.
"""
import argparse
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
          ("/model/iris_lidar/pose", PoseStamped)]  # sim ground truth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("health_check")
    counts = {name: 0 for name, _ in CHECKS}
    last_state = {}
    for name, kind in CHECKS:
        def cb(msg, name=name):
            counts[name] += 1
            if name == "/mavros/state":
                last_state["s"] = msg
        qos = qos_profile_sensor_data if name in ("/scan", "/clock", "/model/iris_lidar/pose") else 10
        node.create_subscription(kind, name, cb, qos)
    buf = Buffer()
    TransformListener(buf, node)

    tf_ok = 0
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            buf.lookup_transform("map", "base_link", Time())
            tf_ok += 1
        except TransformException:
            pass

    healthy = True
    for name, _ in CHECKS:
        rate = counts[name] / args.seconds
        ok = counts[name] > 0
        healthy &= ok
        print(f"  {'OK  ' if ok else 'DEAD'} {name:<28} {rate:6.1f} msg/s")
    print(f"  {'OK  ' if tf_ok else 'DEAD'} {'TF map -> base_link':<28} {'found' if tf_ok else 'never found'}")
    healthy &= tf_ok > 0
    if "s" in last_state:
        s = last_state["s"]
        print(f"  ArduPilot: connected={s.connected} armed={s.armed} mode={s.mode}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if healthy else 1)


if __name__ == "__main__":
    main()
