"""ROS node: turn the SLAM /map into NIDAR's scoring format - 1 m cells, each side a wall or an opening.

The logic lives in grid_logic.py (shared with offline tools). This node just runs it on every map
update (at most `rate_hz`) and publishes the result. Because /map fuses every scan so far,
re-classifying each update acts as voting over many scans.

Publishes:
  /airmouse/grid          std_msgs/String, JSON (latched):
      {"cell_size": 1.0, "cells": {"i,j": {"+x": "wall", "-x": "open", "+y": ..., "-y": ..., "seen": true}}}
  /airmouse/grid_markers  visualization_msgs/MarkerArray for RViz (red = wall, green = open, grey = unknown)
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from airmouse.grid_logic import GridParams, classify_map, side_segments

COLOURS = {"wall": (1.0, 0.1, 0.1), "open": (0.1, 0.9, 0.2), "unknown": (0.5, 0.5, 0.5)}


class GridMapper(Node):
    def __init__(self):
        super().__init__("grid_mapper")
        d = GridParams()
        self.params = GridParams(
            cell=self.declare_parameter("cell_size", d.cell).value,
            band=self.declare_parameter("band", d.band).value,
            margin=self.declare_parameter("end_margin", d.margin).value,
            wall_ratio=self.declare_parameter("wall_ratio", d.wall_ratio).value,
            open_ratio=self.declare_parameter("open_ratio", d.open_ratio).value,
        )
        self.period = 1.0 / self.declare_parameter("rate_hz", 2.0).value  # re-classify at most this often
        self.last_update = 0.0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 1)
        self.grid_pub = self.create_publisher(String, "/airmouse/grid", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/airmouse/grid_markers", latched)
        self.updates = 0

    def on_map(self, msg):
        now = time.monotonic()
        if now - self.last_update < self.period:
            return
        self.last_update = now

        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        cells = classify_map(grid, info.origin.position.x, info.origin.position.y, info.resolution, self.params)

        self.grid_pub.publish(String(data=json.dumps({"cell_size": self.params.cell, "frame": "map",
                                                      "cells": cells})))
        # thousands of Python marker objects per update - only worth building if RViz is listening
        if self.marker_pub.get_subscription_count() > 0:
            self.marker_pub.publish(self.markers(cells, msg.header.frame_id or "map"))
        self.updates += 1
        if self.updates == 1 or self.updates % 20 == 0:
            walls = sum(v == "wall" for cell in cells.values() for v in cell.values())
            self.get_logger().info(f"grid: {len(cells)} cells seen, {walls} wall sides")

    def markers(self, cells, frame):
        lines = Marker()
        lines.header.frame_id = frame
        lines.ns, lines.id, lines.type, lines.action = "grid", 0, Marker.LINE_LIST, Marker.ADD
        lines.scale.x = 0.05
        lines.pose.orientation.w = 1.0
        for key, sides in cells.items():
            i, j = map(int, key.split(","))
            for side, (a, b) in side_segments(i, j, self.params.cell).items():
                r, g, bl = COLOURS[sides[side]]
                for x, y in (a, b):
                    lines.points.append(Point(x=float(x), y=float(y), z=0.05))
                    lines.colors.append(ColorRGBA(r=r, g=g, b=bl, a=1.0))
        return MarkerArray(markers=[lines])


def main():
    rclpy.init()
    rclpy.spin(GridMapper())


if __name__ == "__main__":
    main()
