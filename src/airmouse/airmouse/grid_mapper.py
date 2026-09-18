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
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from airmouse.grid_logic import GridParams, classify_map, fit_manhattan, side_segments

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
        # Every wall in the arena is on a 1 m line and square to its neighbours, so the whole map has one rotation
        # and one offset that put the most wall pixels on grid lines. Fitting those and classifying in the
        # corrected frame undoes a rigid SLAM drift, which otherwise costs the map outright: run 22 drifted 0.8 m
        # and scored 26%. 0 turns it off (classify in the raw SLAM frame).
        self.manhattan = self.declare_parameter("manhattan", True).value
        self.fit = (0.0, 0.0, 0.0)
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
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        if self.manhattan:
            self.fit = fit_manhattan(grid, ox, oy, res, self.params.cell)
        cells = classify_map(grid, ox, oy, res, self.params, self.fit if self.manhattan else None)

        # The fit goes out with the grid: anything that converts between cells and map coordinates (the explorer
        # flying to a cell centre, RViz markers) has to use the same frame the cells were classified in.
        self.grid_pub.publish(String(data=json.dumps({"cell_size": self.params.cell, "frame": "map",
                                                      "fit": list(self.fit), "cells": cells})))
        # thousands of Python marker objects per update - only worth building if RViz is listening
        if self.marker_pub.get_subscription_count() > 0:
            self.marker_pub.publish(self.markers(cells, msg.header.frame_id or "map"))
        self.updates += 1
        if self.updates == 1 or self.updates % 20 == 0:
            walls = sum(v == "wall" for cell in cells.values() for v in cell.values())
            th, dx, dy = self.fit
            fit_txt = f", manhattan fit {math.degrees(th):+.1f} deg ({dx:+.2f}, {dy:+.2f}) m" if self.manhattan else ""
            self.get_logger().info(f"grid: {len(cells)} cells seen, {walls} wall sides{fit_txt}")

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
