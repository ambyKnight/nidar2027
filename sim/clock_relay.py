#!/usr/bin/env python3
"""Replay helper: republish /clock_raw -> /clock, dropping any message that goes back in time.

A recorded /clock (~800 msg/s) has small out-of-order steps; played back directly, every backward
step makes Cartographer clear its TF buffer ("Detected jump back in time") and ruins the replay.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class ClockRelay(Node):
    def __init__(self):
        super().__init__("clock_relay")
        self.last = -1
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(Clock, "/clock_raw", self.on_clock, qos_profile_sensor_data)

    def on_clock(self, msg):
        t = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if t > self.last:
            self.last = t
            self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ClockRelay())


if __name__ == "__main__":
    main()
