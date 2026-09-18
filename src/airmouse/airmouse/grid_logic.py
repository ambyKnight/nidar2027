"""Pure grid-mapping logic (no ROS), shared by the grid_mapper node and offline tools.

Turns an occupancy grid into NIDAR's scoring format: 1 m cells, each side "wall" / "open" / "unknown".

The grid is anchored at the takeoff point: the drone starts in the centre of the entrance cell,
facing along the grid, so cell (i, j) is centred on map point (i, j) * cell_size.

NIDAR arenas are built on a modular grid, so walls can only stand on grid lines. We therefore
"snap": any wall pixel within `band` metres of a grid line counts for that line. This makes the
result tolerant of small SLAM offsets (walls seen 0.15-0.25 m off their true line).
"""
from dataclasses import dataclass

import numpy as np

OCCUPIED, FREE = 65, 35  # occupancy thresholds (0-100)


@dataclass
class GridParams:
    cell: float = 1.0        # cell size (m)
    band: float = 0.25       # snap distance either side of a grid line (m); must stay well below cell/2
    margin: float = 0.15     # ignore the ends of each side, where walls meet (m)
    wall_ratio: float = 0.4  # share of a side that must show wall pixels to call it a wall
    open_ratio: float = 0.6  # share of a side that must be seen free (see-through) to call it open


def make_sampler(grid, ox, oy, res):
    """grid: (height, width) int array of -1 / 0-100. Returns f(xs, ys) -> values at map points."""
    h, w = grid.shape

    def f(xs, ys):
        cols = np.floor((xs - ox) / res).astype(int)
        rows = np.floor((ys - oy) / res).astype(int)
        inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        out = np.full(np.shape(xs), -1, dtype=np.int16)
        out[inside] = grid[rows[inside], cols[inside]]
        return out
    return f


def classify_side(sample, p0, p1, res, p):
    """Classify the straight cell side p0 -> p1 (map frame) as wall / open / unknown."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    length = np.linalg.norm(p1 - p0)
    along = (p1 - p0) / length
    normal = np.array([-along[1], along[0]])
    s = np.arange(p.margin, length - p.margin + 1e-6, res)          # positions along the side
    d = np.arange(-p.band, p.band + 1e-6, res)                      # offsets across it
    pts = p0 + s[:, None, None] * along + d[None, :, None] * normal
    vals = sample(pts[..., 0], pts[..., 1])

    occupied = (vals >= OCCUPIED).any(axis=1)                       # a wall pixel near this spot
    # "open" = the laser saw through the gap: free pixels right on the line and no wall nearby
    centre = np.abs(d) <= res + 1e-6
    free = ((vals[:, centre] >= 0) & (vals[:, centre] < FREE)).all(axis=1) & ~occupied
    if occupied.mean() >= p.wall_ratio:
        return "wall"
    if free.mean() >= p.open_ratio:
        return "open"
    return "unknown"


def cell_seen(sample, i, j, p):
    c, h = p.cell, p.cell * 0.3
    xs, ys = np.meshgrid(np.linspace(i * c - h, i * c + h, 7), np.linspace(j * c - h, j * c + h, 7))
    return bool((sample(xs, ys) >= 0).mean() > 0.6)


def classify_map(grid, ox, oy, res, p=GridParams()):
    """Return {"i,j": {"+x": ..., "-x": ..., "+y": ..., "-y": ..., "seen": True}} for every seen cell."""
    sample = make_sampler(grid, ox, oy, res)
    h, w = grid.shape
    c = p.cell
    x1, y1 = ox + w * res, oy + h * res
    irange = range(int(np.ceil(ox / c - 0.5)), int(np.floor(x1 / c + 0.5)))
    jrange = range(int(np.ceil(oy / c - 0.5)), int(np.floor(y1 / c + 0.5)))

    cells = {}
    for i in irange:
        for j in jrange:
            if not cell_seen(sample, i, j, p):
                continue
            xl, xh, yl, yh = (i - 0.5) * c, (i + 0.5) * c, (j - 0.5) * c, (j + 0.5) * c
            cells[f"{i},{j}"] = {
                "+x": classify_side(sample, (xh, yl), (xh, yh), res, p),
                "-x": classify_side(sample, (xl, yl), (xl, yh), res, p),
                "+y": classify_side(sample, (xl, yh), (xh, yh), res, p),
                "-y": classify_side(sample, (xl, yl), (xh, yl), res, p),
                "seen": True,
            }
    return cells


def side_segments(i, j, cell):
    """Map-frame end points of each side of cell (i, j)."""
    xl, xh, yl, yh = (i - 0.5) * cell, (i + 0.5) * cell, (j - 0.5) * cell, (j + 0.5) * cell
    return {"+x": ((xh, yl), (xh, yh)), "-x": ((xl, yl), (xl, yh)),
            "+y": ((xl, yh), (xh, yh)), "-y": ((xl, yl), (xh, yl))}
