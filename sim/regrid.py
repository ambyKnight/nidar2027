#!/usr/bin/env python3
"""Offline: re-run the grid mapper on a saved SLAM map (.npz from save_map.py) and write the grid JSON.
Lets you tune grid_logic parameters in seconds instead of re-flying the drone.

    python3 regrid.py tour_map.npz out.json [--band 0.25] [--wall-ratio 0.4] [--open-ratio 0.6]
    python3 score_grid.py --json out.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

from airmouse.grid_logic import GridParams, classify_map


def main():
    d = GridParams()
    ap = argparse.ArgumentParser()
    ap.add_argument("map_npz")
    ap.add_argument("out_json")
    ap.add_argument("--band", type=float, default=d.band)
    ap.add_argument("--margin", type=float, default=d.margin)
    ap.add_argument("--wall-ratio", type=float, default=d.wall_ratio)
    ap.add_argument("--open-ratio", type=float, default=d.open_ratio)
    args = ap.parse_args()

    z = np.load(args.map_npz)
    p = GridParams(band=args.band, margin=args.margin, wall_ratio=args.wall_ratio, open_ratio=args.open_ratio)
    cells = classify_map(z["grid"], float(z["origin"][0]), float(z["origin"][1]), float(z["resolution"]), p)
    Path(args.out_json).write_text(json.dumps({"cell_size": p.cell, "frame": "map", "cells": cells}))
    print(f"{len(cells)} cells -> {args.out_json}  ({p})")


if __name__ == "__main__":
    main()
