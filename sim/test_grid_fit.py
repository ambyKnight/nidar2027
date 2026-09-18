#!/usr/bin/env python3
"""Offline test of the Manhattan grid fit: can it recover a known SLAM drift? No ROS, no drone, milliseconds.

Renders the TRUE arena walls into an occupancy grid, applies a known rotation + offset (what SLAM drift does to
a map), and checks fit_manhattan() finds it back and that classifying in the corrected frame restores the score.

    python3 sim/test_grid_fit.py
"""
import sys, math, json, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/airmouse"))
from airmouse.grid_logic import fit_manhattan, classify_map, GridParams

RES, CELL = 0.05, 1.0
def build(truth, theta, dx, dy):
    """Render the true walls into an occupancy grid, rotated/shifted like a drifted SLAM map."""
    pts = []
    for key, c in truth["cells"].items():
        wx, wy = c["world_xy"]; i, j = int(round(wy)), int(round(-wx))
        for d, side in (("N", "+x"), ("S", "-x"), ("E", "-y"), ("W", "+y")):
            if not c[d]: continue
            xl, xh, yl, yh = (i-.5)*CELL, (i+.5)*CELL, (j-.5)*CELL, (j+.5)*CELL
            p0, p1 = {"+x": ((xh,yl),(xh,yh)), "-x": ((xl,yl),(xl,yh)),
                      "+y": ((xl,yh),(xh,yh)), "-y": ((xl,yl),(xh,yl))}[side]
            for t in np.linspace(0, 1, 40):
                pts.append((p0[0]+t*(p1[0]-p0[0]), p0[1]+t*(p1[1]-p0[1])))
    pts = np.array(pts)
    c, s = math.cos(theta), math.sin(theta)                 # apply the drift we want to recover
    dx_, dy_ = c*pts[:,0]-s*pts[:,1]+dx, s*pts[:,0]+c*pts[:,1]+dy
    ox, oy = dx_.min()-1, dy_.min()-1
    w = int((dx_.max()+1-ox)/RES)+1; h = int((dy_.max()+1-oy)/RES)+1
    grid = np.zeros((h, w), dtype=np.int16)                  # free everywhere we flew
    grid[((dy_-oy)/RES).astype(int), ((dx_-ox)/RES).astype(int)] = 100
    for _ in range(2):                                       # thicken walls a little, like a real map
        grid[1:,:] = np.maximum(grid[1:,:], grid[:-1,:]); grid[:,1:] = np.maximum(grid[:,1:], grid[:,:-1])
    return grid, ox, oy

truth = json.load(open(Path(__file__).parent / "worlds/rooms_small_4_truth.json"))
failures = 0
# Offsets stay under half a cell on purpose: a whole-cell shift is indistinguishable from none (see
# fit_manhattan's LIMIT note), and real drift is anchored at the takeoff point anyway.
for theta_deg, dx, dy in ((0, 0, 0), (3, 0.4, -0.3), (-5, 0.35, 0.45), (7, -0.45, 0.35), (-9, 0.2, -0.4)):
    grid, ox, oy = build(truth, math.radians(theta_deg), dx, dy)
    th, fx, fy = fit_manhattan(grid, ox, oy, RES, CELL)
    # score cells against truth, with and without the fit
    def score(fit):
        cells = classify_map(grid, ox, oy, RES, GridParams(), fit)
        ok = tot = 0
        for key, c in truth["cells"].items():
            wx, wy = c["world_xy"]; k = f"{int(round(wy))},{int(round(-wx))}"
            if k not in cells: continue
            tot += 1
            want = {"+x": c["N"], "-x": c["S"], "-y": c["E"], "+y": c["W"]}
            ok += all((cells[k][s] == "wall") == w for s, w in want.items())
        return ok, tot
    a, tot = score(None); b, _ = score((th, fx, fy))
    ok = b >= max(a, int(tot * 0.95))
    failures += 0 if ok else 1
    print(f"{'PASS' if ok else 'FAIL'}  drift {theta_deg:+.0f} deg ({dx:+.2f},{dy:+.2f}) m -> fit "
          f"{math.degrees(th):+.2f} deg ({fx:+.2f},{fy:+.2f}): cells correct raw {a}/{tot}, fitted {b}/{tot}")
print("ALL PASS" if not failures else f"{failures} FAILURE(S)")
sys.exit(1 if failures else 0)
