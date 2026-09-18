#!/usr/bin/env python3
"""Survivor detection, grid localisation and geotagging node for NIDAR AirMouse.

NIDAR Rulebook Section 5 & 6:
  - Detect survivors (humans/dummies) in rooms.
  - Autonomously identify the 1 m grid cell (i, j) where each survivor is located.
  - Tag survivor locations on the 2D map with markers and transmit to GCS in real-time.

Subscribes:
  /mavros/local_position/pose    geometry_msgs/PoseStamped
  /airmouse/grid                 std_msgs/String (JSON)
  /airmouse/camera/detections    std_msgs/String (JSON) - live AI detections from camera
  /airmouse/sim/survivors_truth  std_msgs/String (JSON) - optional ground truth for simulation

Publishes:
  /airmouse/survivors            std_msgs/String (JSON, latched)
  /airmouse/survivor_markers     visualization_msgs/MarkerArray (for RViz)
"""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from airmouse.explore_logic import line_of_sight, parse_grid


class SurvivorTagger(Node):
    def __init__(self):
        super().__init__("survivor_tagger")
        self.cell_size = self.declare_parameter("cell_size", 1.0).value
        self.cluster_radius = self.declare_parameter("cluster_radius", 0.85).value
        self.cam_reach = self.declare_parameter("cam_reach", 3.0).value
        self.sim_detection = self.declare_parameter("sim_detection", True).value
        truth_param = self.declare_parameter("truth_file", "").value

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pose = None
        self.yaw = 0.0
        self.cells = {}
        self.survivors = []  # [{id, tag, cell: [i, j], x, y, confidence, first_seen, last_seen, hits}]

        # Subscriptions
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self.on_pose, 10)
        self.create_subscription(String, "/airmouse/grid", self.on_grid, latched)
        self.create_subscription(String, "/airmouse/camera/detections",
                                 self.on_camera_detections, 10)
        self.create_subscription(String, "/airmouse/sim/survivors_truth",
                                 self.on_truth_override, latched)

        # Publishers
        self.survivor_pub = self.create_publisher(String, "/airmouse/survivors", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "/airmouse/survivor_markers", latched)

        # Sim truth survivors (if file given)
        self.sim_survivor_list = []
        if truth_param and os.path.isfile(truth_param):
            self.load_truth_file(truth_param)

        # Periodic timer (2 Hz) for simulated camera check and state publish
        self.create_timer(0.5, self.tick)
        self.get_logger().info("survivor_tagger initialized and listening for survivor detections")

    def load_truth_file(self, path):
        try:
            data = json.loads(Path(path).read_text())
            if "survivors" in data:
                self.sim_survivor_list = data["survivors"]
                self.get_logger().info(f"loaded {len(self.sim_survivor_list)} sim survivors from {path}")
        except Exception as exc:
            self.get_logger().warn(f"could not load truth file: {exc}")

    def on_truth_override(self, msg):
        try:
            data = json.loads(msg.data)
            self.sim_survivor_list = data.get("survivors", data if isinstance(data, list) else [])
            self.get_logger().info(f"updated sim survivors list ({len(self.sim_survivor_list)} items)")
        except Exception as exc:
            self.get_logger().warn(f"bad truth override: {exc}")

    def on_pose(self, msg):
        self.pose = msg.pose.position
        q = msg.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_grid(self, msg):
        try:
            _, cells = parse_grid(json.loads(msg.data))
            self.cells = cells
        except Exception:
            pass

    def on_camera_detections(self, msg):
        """Process incoming camera AI detections.
        
        Expected JSON:
          {"detections": [{"rel_x": ..., "rel_y": ..., "confidence": 0.95}, ...]}
          or [{"x": world_x, "y": world_y, "confidence": ...}]
        """
        if self.pose is None:
            return
        try:
            payload = json.loads(msg.data)
            items = payload.get("detections", payload if isinstance(payload, list) else [])
            for item in items:
                conf = float(item.get("confidence", 0.9))
                if conf < 0.4:
                    continue
                if "rel_x" in item and "rel_y" in item:
                    rx, ry = float(item["rel_x"]), float(item["rel_y"])
                    # Transform from body/camera frame (rel_x forward, rel_y left) to map frame
                    wx = self.pose.x + rx * math.cos(self.yaw) - ry * math.sin(self.yaw)
                    wy = self.pose.y + rx * math.sin(self.yaw) + ry * math.cos(self.yaw)
                elif "x" in item and "y" in item:
                    wx, wy = float(item["x"]), float(item["y"])
                else:
                    continue
                self.register_survivor(wx, wy, conf)
        except Exception as exc:
            self.get_logger().warn(f"failed to parse detection message: {exc}")

    def tick(self):
        """Simulated camera detection: checks line-of-sight to true survivors in sim mode."""
        if self.sim_detection and self.sim_survivor_list and self.pose is not None and self.cells:
            drone_xy = (self.pose.x, self.pose.y)
            for s in self.sim_survivor_list:
                sx = s.get("x", s.get("world_xy", [0, 0])[0])
                sy = s.get("y", s.get("world_xy", [0, 0])[1])
                dist = math.dist(drone_xy, (sx, sy))
                if dist <= self.cam_reach:
                    # Check visibility via explore_logic line_of_sight
                    drone_cell = (int(round(self.pose.x / self.cell_size)),
                                  int(round(self.pose.y / self.cell_size)))
                    survivor_cell = (int(round(sx / self.cell_size)),
                                     int(round(sy / self.cell_size)))
                    visible = line_of_sight(self.cells, drone_cell, self.cam_reach)
                    if survivor_cell in visible:
                        self.register_survivor(sx, sy, confidence=0.98)

    def register_survivor(self, wx, wy, confidence=0.9):
        """Debounce and cluster detections into unique survivor tags."""
        now = time.monotonic()
        # Check against existing survivors
        for s in self.survivors:
            if math.dist((wx, wy), (s["x"], s["y"])) < self.cluster_radius:
                # Update running average
                s["x"] = (s["x"] * s["hits"] + wx) / (s["hits"] + 1)
                s["y"] = (s["y"] * s["hits"] + wy) / (s["hits"] + 1)
                s["cell"] = [int(round(s["x"] / self.cell_size)),
                             int(round(s["y"] / self.cell_size))]
                s["confidence"] = max(s["confidence"], confidence)
                s["last_seen"] = now
                s["hits"] += 1
                self.publish_survivors()
                return

        # New survivor found!
        sid = len(self.survivors) + 1
        tag = f"S{sid}"
        cell = [int(round(wx / self.cell_size)), int(round(wy / self.cell_size))]
        record = {
            "id": sid,
            "tag": tag,
            "cell": cell,
            "x": round(wx, 2),
            "y": round(wy, 2),
            "confidence": round(confidence, 2),
            "first_seen": now,
            "last_seen": now,
            "hits": 1
        }
        self.survivors.append(record)
        self.get_logger().info(
            f"*** SURVIVOR DETECTED: {tag} at cell ({cell[0]}, {cell[1]}), world ({wx:.2f}, {wy:.2f}) [conf {confidence:.2f}] ***")
        self.publish_survivors()

    def publish_survivors(self):
        msg = {
            "count": len(self.survivors),
            "survivors": [
                {
                    "id": s["id"],
                    "tag": s["tag"],
                    "cell": s["cell"],
                    "x": s["x"],
                    "y": s["y"],
                    "confidence": s["confidence"],
                    "hits": s["hits"]
                }
                for s in self.survivors
            ]
        }
        self.survivor_pub.publish(String(data=json.dumps(msg)))

        # Markers for RViz
        if self.marker_pub.get_subscription_count() > 0:
            markers = []
            for s in self.survivors:
                # Sphere beacon
                m = Marker()
                m.header.frame_id = "map"
                m.ns, m.id = "survivors", s["id"]
                m.type, m.action = Marker.CYLINDER, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = s["x"], s["y"], 0.2
                m.scale.x, m.scale.y, m.scale.z = 0.4, 0.4, 0.4
                m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
                markers.append(m)

                # Text tag S1, S2
                t = Marker()
                t.header.frame_id = "map"
                t.ns, t.id = "survivor_labels", s["id"]
                t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                t.pose.position.x, t.pose.position.y, t.pose.position.z = s["x"], s["y"], 0.6
                t.scale.z = 0.35
                t.text = s["tag"]
                t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                markers.append(t)
            self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorTagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
