#!/usr/bin/env python3
"""Rewrite a recorded flight so its messages are stored in TIMESTAMP order, for offline SLAM replay.

    python3 sanitize_bag.py runs/<run>/bag runs/<run>/bag_sorted

ros2 bag record stores messages in ARRIVAL order, and under load that is not stamp order: in run 20 about
12% of the 1 kHz IMU samples were older than the sample before them (and a few scans). Cartographer's pose
extrapolator aborts on an IMU sample older than its latest pose ("Check failed: imu_data.time >=
timed_pose_queue_.back().time"), so replaying such a bag crashes. Here every message's record time is set to
its own header stamp (sim time) and the messages are written back sorted by it. Keeps only what replay_slam.sh plays.
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped

TYPES = {"/clock": Clock, "/scan": LaserScan, "/airmouse/imu": Imu, "/model/iris_lidar/pose": PoseStamped}


def stamp_ns(topic, msg):
    st = msg.clock if topic == "/clock" else msg.header.stamp
    return st.sec * 1_000_000_000 + st.nanosec


def main():
    src, dst = sys.argv[1], sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    meta = {t.name: t for t in reader.get_all_topics_and_types() if t.name in TYPES}
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in TYPES:
            rows.append((stamp_ns(topic, deserialize_message(data, TYPES[topic])), len(rows), topic, data))
    rows.sort()          # by stamp; ties keep the recorded order
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for t in meta.values():
        writer.create_topic(t)
    for stamp, _, topic, data in rows:
        writer.write(topic, data, stamp)
    print(f"wrote {len(rows)} messages in stamp order to {dst}")


if __name__ == "__main__":
    main()
