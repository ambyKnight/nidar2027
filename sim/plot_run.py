#!/usr/bin/env python3
"""One-picture post-mortem of a flight: true walls, the drone's TRUE path and the path SLAM thought it flew.

    python3 plot_run.py [slam_eval.csv] [out.png] [--truth worlds/x_truth.json] [--spawn-yaw 90]

Both paths are in the SLAM map frame (what slam_eval.py writes). Markers every 60 s; the last point
of each path is a star. Where the red line leaves the green one is where SLAM lost the drone.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from score_grid import world_to_map  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="/tmp/airmouse_sim/explore/slam_eval.csv")
    ap.add_argument("out", nargs="?", default="/tmp/airmouse_sim/explore/path.png")
    ap.add_argument("--truth", default=str(Path(__file__).parent / "worlds/rooms_small_4_truth.json"))
    ap.add_argument("--spawn-yaw", type=float, default=90.0)
    args = ap.parse_args()

    to_map, _ = world_to_map(args.spawn_yaw)
    truth = json.loads(Path(args.truth).read_text())
    a = np.loadtxt(args.csv, delimiter=",", skiprows=1)
    t = a[:, 0] - a[0, 0]

    fig, ax = plt.subplots(figsize=(9, 9))
    for cell in truth["cells"].values():
        x, y = cell["world_xy"]
        for wall, (p, q) in {"N": ((x - .5, y + .5), (x + .5, y + .5)), "S": ((x - .5, y - .5), (x + .5, y - .5)),
                             "W": ((x - .5, y - .5), (x - .5, y + .5)), "E": ((x + .5, y - .5), (x + .5, y + .5))}.items():
            if cell[wall]:
                (x0, y0), (x1, y1) = to_map(*p), to_map(*q)
                ax.plot([x0, x1], [y0, y1], color="black", lw=2, solid_capstyle="round")
    ax.plot(a[:, 1], a[:, 2], color="tab:green", lw=1.5, label="TRUE path (Gazebo)")
    ax.plot(a[:, 3], a[:, 4], color="tab:red", lw=1.2, alpha=.8, label="SLAM path")
    for s in range(0, int(t[-1]), 60):
        i = int(np.argmin(abs(t - s)))
        ax.annotate(f"{s}s", (a[i, 1], a[i, 2]), color="tab:green", fontsize=8)
        ax.annotate(f"{s}s", (a[i, 3], a[i, 4]), color="tab:red", fontsize=8)
        ax.plot([a[i, 1], a[i, 3]], [a[i, 2], a[i, 4]], color="gray", lw=.5, ls=":")
    ax.plot(a[-1, 1], a[-1, 2], "*", color="tab:green", ms=14)
    ax.plot(a[-1, 3], a[-1, 4], "*", color="tab:red", ms=14)
    e = a[:, 5]
    ax.set_title(f"mean SLAM error {e.mean():.2f} m, max {e.max():.2f} m, {t[-1]:.0f} s")
    ax.set_xlabel("SLAM x (start heading), m")
    ax.set_ylabel("SLAM y (left), m")
    ax.set_aspect("equal")
    ax.grid(alpha=.3)
    ax.legend(loc="upper right")
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
