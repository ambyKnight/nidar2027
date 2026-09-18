#!/usr/bin/env python3
"""Score the grid_mapper output against the maze's ground truth, like the NIDAR judges.

NIDAR: a cell counts only if ALL its sides (wall / opening) are mapped correctly.
"unknown" or a missing cell counts as wrong.

    python3 score_grid.py [--truth worlds/practice_6x6_truth.json] [--spawn-yaw 90] [--json saved_grid.json]

Without --json it grabs the latest /airmouse/grid message from ROS.
Prints the true maze and our map side by side in the same ASCII format as the maze drawings
('?' marks sides we got wrong or don't know).
"""
import argparse
import json
import math
import time
from pathlib import Path

WORLD_DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}


def grab_grid():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node("score_grid")
    got = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/airmouse/grid", lambda m: got.setdefault("g", m.data), qos)
    deadline = time.time() + 20
    while rclpy.ok() and "g" not in got and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    if "g" not in got:
        raise SystemExit("No /airmouse/grid message within 20 s - is grid_mapper running?")
    return json.loads(got["g"])


def world_to_map(spawn_yaw_deg):
    """Return functions converting world (x, y) and world directions into the SLAM map frame."""
    yaw = math.radians(spawn_yaw_deg)
    c, s = math.cos(-yaw), math.sin(-yaw)

    def point(x, y):
        return c * x - s * y, s * x + c * y

    def side(world_dir):
        dx, dy = point(*WORLD_DIRS[world_dir])
        return ("+x" if dx > 0 else "-x") if abs(dx) > abs(dy) else ("+y" if dy > 0 else "-y")
    return point, side


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--truth", default=str(Path(__file__).parent / "worlds/practice_6x6_truth.json"))
    ap.add_argument("--spawn-yaw", type=float, default=90.0)
    ap.add_argument("--json", help="score a saved /airmouse/grid JSON file instead of the live topic")
    args = ap.parse_args()

    truth = json.loads(Path(args.truth).read_text())
    grid = json.loads(Path(args.json).read_text()) if args.json else grab_grid()
    ours = grid["cells"]
    to_map, side_in_map = world_to_map(args.spawn_yaw)

    rows, cols = truth["rows"], truth["cols"]
    got = {}          # (r, c, world_dir) -> True if correct
    cells_ok = 0
    for key, cell in truth["cells"].items():
        r, c = map(int, key.split(","))
        mx, my = to_map(*cell["world_xy"])
        mine = ours.get(f"{round(mx)},{round(my)}", {})
        ok_all = True
        for d in "NESW":
            want = "wall" if cell[d] else "open"
            ok = mine.get(side_in_map(d)) == want
            got[(r, c, d)] = ok
            ok_all &= ok
        cells_ok += ok_all

    sides_ok = sum(got.values())
    total_cells = rows * cols

    # ASCII: truth on the left, ours on the right ('?' = wrong or unknown)
    def draw(mark_errors):
        out = []
        for r in range(rows + 1):
            line = "+"
            for c in range(cols):
                cell_r, d = (r, "N") if r < rows else (r - 1, "S")
                wall = truth["cells"][f"{cell_r},{c}"][d]
                bad = mark_errors and not got[(cell_r, c, d)]
                line += ("??" if bad else ("--" if wall else "  ")) + "+"
            out.append(line)
            if r == rows:
                break
            line = ""
            for c in range(cols + 1):
                cell_c, d = (c, "W") if c < cols else (c - 1, "E")
                wall = truth["cells"][f"{r},{cell_c}"][d]
                bad = mark_errors and not got[(r, cell_c, d)]
                line += ("?" if bad else ("|" if wall else " ")) + ("  " if c < cols else "")
            out.append(line)
        return out

    left, right = draw(False), draw(True)
    print(f"{'TRUE MAZE':<{len(left[0]) + 4}}OUR MAP ('?' = wrong or unknown)")
    for a, b in zip(left, right):
        print(f"{a:<{len(left[0]) + 4}}{b}")
    print()
    print(f"cells fully correct: {cells_ok}/{total_cells} = {100 * cells_ok / total_cells:.0f}%   "
          f"(NIDAR mapping score ~ {220 * cells_ok / total_cells:.0f}/220 if the arena were this maze)")
    print(f"sides correct:       {sides_ok}/{len(got)} = {100 * sides_ok / len(got):.0f}%")


if __name__ == "__main__":
    main()
