#!/usr/bin/env python3
"""Lightweight HTTP server serving the AirMouse GCS Web Dashboard.

Serves static assets (index.html, style.css, app.js) from the package's web directory.
The web frontend connects directly to rosbridge_server (port 9090) via WebSockets.

Run via:
    ros2 run airmouse dashboard_server
    ros2 run airmouse dashboard_server --ros-args -p port:=8080
"""
import functools
import os
import sys
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node


def find_web_dir():
    """Locate the web dashboard assets directory."""
    # 1. Try installed package share directory
    try:
        share_dir = get_package_share_directory("airmouse")
        candidate = Path(share_dir) / "web"
        if candidate.is_dir() and (candidate / "index.html").exists():
            return str(candidate)
    except PackageNotFoundError:
        pass

    # 2. Try source directory relative to this file
    src_candidate = Path(__file__).resolve().parents[1] / "web"
    if src_candidate.is_dir() and (src_candidate / "index.html").exists():
        return str(src_candidate)

    # 3. Workspace fallback
    ws_candidate = Path(__file__).resolve().parents[3] / "src" / "airmouse" / "web"
    if ws_candidate.is_dir() and (ws_candidate / "index.html").exists():
        return str(ws_candidate)

    raise FileNotFoundError("Could not find AirMouse web dashboard directory.")


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__("dashboard_server")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.web_dir = find_web_dir()
        self.get_logger().info(f"Serving GCS dashboard from: {self.web_dir}")

        handler = functools.partial(SimpleHTTPRequestHandler, directory=self.web_dir)
        self.server = ThreadingHTTPServer(("0.0.0.0", self.port), handler)

        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

        self.get_logger().info(f"======================================================")
        self.get_logger().info(f"AirMouse GCS Dashboard is live at: http://localhost:{self.port}")
        self.get_logger().info(f"======================================================")

    def destroy_node(self):
        self.get_logger().info("Shutting down HTTP dashboard server...")
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
