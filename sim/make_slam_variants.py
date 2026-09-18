#!/usr/bin/env python3
"""Write Cartographer config variants (base airmouse_2d.lua + a few overrides) for offline tuning.

    python3 make_slam_variants.py        -> sim/slam_configs/<name>.lua
    replay_slam.sh <bag> sim/slam_configs/<name>.lua
"""
from pathlib import Path

BASE = Path.home() / "airmouse_ws/src/airmouse/config/airmouse_2d.lua"
OUT = Path(__file__).parent / "slam_configs"

VARIANTS = {
    "base": "",
    # 2026-09-18 runs 13 and 20: SLAM lost the drone when it entered a small enclosed space and reversed out
    # of it. Is it the IMU, the matcher window, or the matcher itself?
    "noimu": "TRAJECTORY_BUILDER_2D.use_imu_data = false",
    # smaller search window: a wrong 30 cm jump is impossible if the matcher may only move 15 cm
    "narrow": "TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15\n"
              "TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)",
    # combinations of the run-20 sweep winners
    "ceres_noimu": "TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false\n"
                   "TRAJECTORY_BUILDER_2D.use_imu_data = false",
    "ceres_sub70": "TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false\n"
                   "TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70",
    # no brute-force matcher at all: rely on the motion prediction + Ceres refinement
    "ceres_only": "TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false",
    # trust the motion prediction more: less jitter, but risk of lag coming back
    "tw1": "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.",
    # pull scans harder onto the walls
    "occ20": "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.",
    # finer local map (2.5 cm) for sharper matching
    "res025": "TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025",
    # bigger local submaps: more wall context per match
    "sub70": "TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70",
    # The maze is made of identical 1 m cells, so loop closure can match the WRONG look-alike place
    # and make the pose jump by exactly 1-2 m (seen in the run-6 replay at t=60 s and t=160 s).
    # Only close loops near where we think we are, never search the whole map:
    "local_lc": "POSE_GRAPH.constraint_builder.max_constraint_distance = 2.\n"
                "POSE_GRAPH.global_sampling_ratio = 0.",
    # ... and additionally demand a much better match before accepting a closure
    "strict_lc": "POSE_GRAPH.constraint_builder.max_constraint_distance = 2.\n"
                 "POSE_GRAPH.global_sampling_ratio = 0.\n"
                 "POSE_GRAPH.constraint_builder.min_score = 0.8",
    # no loop closure at all: pure scan-to-map tracking (drift only, but no jumps)
    "no_lc": "POSE_GRAPH.optimize_every_n_nodes = 0",
}


def main():
    base = BASE.read_text()
    assert "return options" in base
    OUT.mkdir(exist_ok=True)
    for name, extra in VARIANTS.items():
        text = base.replace("return options", f"-- variant: {name}\n{extra}\n\nreturn options")
        (OUT / f"{name}.lua").write_text(text)
    print(f"wrote {len(VARIANTS)} variants to {OUT}: {', '.join(VARIANTS)}")


if __name__ == "__main__":
    main()
