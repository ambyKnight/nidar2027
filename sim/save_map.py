#!/usr/bin/env python3
"""Save the current SLAM /map as a PNG, with the true maze walls drawn on top in blue.

    python3 save_map.py out.png [--truth worlds/rooms_small_4_truth.json] [--spawn-yaw 90]

Black = wall seen by SLAM, white = free space, grey = unknown, blue = real walls (from the truth file).
The SLAM map frame starts at the drone's takeoff pose, so we rotate the truth by the spawn yaw.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageDraw
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

SCALE = 4  # output pixels per map cell (map cells are 5 cm)


def grab_map():
    rclpy.init()
    node = Node("save_map")
    got = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.setdefault("map", m), qos)
    deadline = time.time() + 20
    while rclpy.ok() and "map" not in got and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    if "map" not in got:
        raise SystemExit("No /map message within 20 s - is SLAM running?")
    return got["map"]


def true_walls_in_map_frame(truth, spawn_yaw_deg):
    """Yield wall segments ((x1,y1),(x2,y2)) in the SLAM map frame."""
    half = truth["cell_size_m"] / 2
    # world -> map: map x axis points along the drone's initial heading
    yaw = math.radians(spawn_yaw_deg)
    c, s = math.cos(-yaw), math.sin(-yaw)

    def to_map(x, y):
        return (c * x - s * y, s * x + c * y)

    for cell in truth["cells"].values():
        cx, cy = cell["world_xy"]
        edges = {"N": ((cx - half, cy + half), (cx + half, cy + half)),
                 "S": ((cx - half, cy - half), (cx + half, cy - half)),
                 "W": ((cx - half, cy - half), (cx - half, cy + half)),
                 "E": ((cx + half, cy - half), (cx + half, cy + half))}
        for side, (a, b) in edges.items():
            if cell[side]:
                yield to_map(*a), to_map(*b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--truth", default=str(Path(__file__).parent / "worlds/rooms_small_4_truth.json"))
    ap.add_argument("--spawn-yaw", type=float, default=90.0)
    args = ap.parse_args()

    m = grab_map()
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    grid = np.array(m.data, dtype=np.int16).reshape(h, w)
    # raw copy for offline re-gridding/tuning without flying again (regrid.py)
    np.savez_compressed(Path(args.out).with_suffix(".npz"), grid=grid, origin=[ox, oy], resolution=res)

    img = np.full((h, w), 170, np.uint8)       # unknown
    img[(grid >= 0) & (grid < 35)] = 255       # free
    img[grid >= 65] = 0                         # occupied
    img = np.flipud(img)                        # image rows go down, map y goes up
    pic = Image.fromarray(img).convert("RGB").resize((w * SCALE, h * SCALE), Image.NEAREST)
    draw = ImageDraw.Draw(pic)

    def px(x, y):
        return ((x - ox) / res * SCALE, (h - (y - oy) / res) * SCALE)

    if Path(args.truth).exists():
        for a, b in true_walls_in_map_frame(json.loads(Path(args.truth).read_text()), args.spawn_yaw):
            draw.line([px(*a), px(*b)], fill=(40, 90, 255), width=2)
    # takeoff point
    x0, y0 = px(0, 0)
    draw.ellipse([x0 - 6, y0 - 6, x0 + 6, y0 + 6], outline=(0, 170, 0), width=3)

    pic.save(args.out)
    known = int((grid >= 0).sum())
    print(f"saved {args.out}: {w}x{h} cells @ {res:.2f} m, known area {known * res * res:.1f} m^2")


if __name__ == "__main__":
    main()
