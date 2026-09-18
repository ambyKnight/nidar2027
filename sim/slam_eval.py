#!/usr/bin/env python3
"""Measure SLAM accuracy against simulator ground truth, matched by timestamp.

For every truth pose (/model/iris_lidar/pose, sim time) we look up SLAM's map -> base_link at the
SAME sim time, so the number is pure SLAM error (no lag between two separately-sampled readings).
Exits after `--idle` seconds without truth messages (e.g. when a bag replay ends), or on Ctrl-C /
SIGINT (live runs, where the truth never stops) - either way it prints the summary.

    python3 slam_eval.py [--spawn-yaw 90] [--idle 8] [--csv out.csv]
"""
import argparse
import signal
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn-yaw", type=float, default=90.0)
    ap.add_argument("--idle", type=float, default=8.0)
    ap.add_argument("--csv")
    args = ap.parse_args()
    spawn = math.radians(args.spawn_yaw)
    c, s = math.cos(-spawn), math.sin(-spawn)

    # no rclpy signal handler: Ctrl-C/SIGINT must reach our code as KeyboardInterrupt so we can report
    # (rclpy's own handler shuts the context down first and the summary is lost)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # A background job in a non-interactive shell (test_explore.sh) starts with SIGINT IGNORED, so Python
    # never turns it into KeyboardInterrupt and the summary is never printed - which is why this
    # measurement hung and produced nothing in runs 9-12. Install the handlers explicitly.
    def _stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    node = Node("slam_eval", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buf = Buffer(cache_time=Duration(seconds=600))
    TransformListener(buf, node)
    pending = []   # truth messages waiting for SLAM's TF at that time to arrive
    rows = []      # (t, truth_x, truth_y, slam_x, slam_y, err, yaw_err_deg)
    last_msg = [time.time()]

    def on_truth(msg):
        last_msg[0] = time.time()
        pending.append(msg)
    node.create_subscription(PoseStamped, "/model/iris_lidar/pose", on_truth, qos_profile_sensor_data)

    started = time.time()
    try:
        loop(node, buf, pending, rows, last_msg, started, args, c, s, spawn)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):   # a signal landing inside rclpy's C++ call surfaces as RuntimeError
        pass  # Ctrl-C / SIGINT: stop and report what we have
    report(node, rows, args)


def loop(node, buf, pending, rows, last_msg, started, args, c, s, spawn):
    while time.time() - last_msg[0] < args.idle or time.time() - started < 15:
        rclpy.spin_once(node, timeout_sec=0.05)
        keep = []
        for msg in pending:
            try:
                tf = buf.lookup_transform("map", "base_link", Time.from_msg(msg.header.stamp))
            except TransformException:
                # not available (yet): keep for up to 2 s of wall time, then drop
                if time.time() - last_msg[0] < 2 or len(keep) < 50:
                    keep.append(msg)
                continue
            p = msg.pose.position
            tx, ty = c * p.x - s * p.y, s * p.x + c * p.y          # world -> SLAM map frame
            sx, sy = tf.transform.translation.x, tf.transform.translation.y
            yaw_err = math.degrees((yaw_of(tf.transform.rotation) - (yaw_of(msg.pose.orientation) - spawn)
                                    + math.pi) % (2 * math.pi) - math.pi)
            t = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            rows.append((t, tx, ty, sx, sy, math.hypot(sx - tx, sy - ty), yaw_err))
        pending[:] = keep[-200:]


def report(node, rows, args):
    node.destroy_node()
    rclpy.try_shutdown()
    if not rows:
        raise SystemExit("no matched truth/SLAM samples - is SLAM running and the truth topic bridged?")
    a = np.array(rows)
    err = a[:, 5]
    print(f"samples: {len(err)}   position error  mean {err.mean():.3f} m   p95 {np.percentile(err, 95):.3f} m"
          f"   max {err.max():.3f} m   |   yaw error p95 {np.percentile(np.abs(a[:, 6]), 95):.1f} deg")
    worst = a[np.argmax(err)]
    print(f"worst at t={worst[0]:.1f}s: truth ({worst[1]:.2f}, {worst[2]:.2f})  slam ({worst[3]:.2f}, {worst[4]:.2f})")
    if args.csv:
        np.savetxt(args.csv, a, delimiter=",", header="t,truth_x,truth_y,slam_x,slam_y,err,yaw_err_deg", fmt="%.4f")


if __name__ == "__main__":
    main()
