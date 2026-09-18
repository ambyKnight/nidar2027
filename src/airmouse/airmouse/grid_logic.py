"""Pure grid-mapping logic (no ROS), shared by the grid_mapper node and offline tools.

Turns an occupancy grid into NIDAR's scoring format: 1 m cells, each side "wall" / "open" / "unknown".

The grid is anchored at the takeoff point: the drone starts in the centre of the entrance cell,
facing along the grid, so cell (i, j) is centred on map point (i, j) * cell_size.

NIDAR arenas are built on a modular grid, so walls can only stand on grid lines. We therefore
"snap": any wall pixel within `band` metres of a grid line counts for that line. This makes the
result tolerant of small SLAM offsets (walls seen 0.15-0.25 m off their true line).

Snapping only tolerates offsets smaller than `band` (0.25 m). Beyond that the map falls apart - run 22 drifted
0.8 m and scored 26% with a map that LOOKED reasonable. fit_manhattan() buys that back: every wall in the arena
is on a 1 m line and at 90 degrees to its neighbours, so the whole map has one rotation and one offset that put
the most wall pixels on grid lines. Finding those two numbers and classifying in the corrected frame undoes a
rigid SLAM drift completely. It cannot undo a map that is internally smeared (walls doubled at different angles).
"""
import math
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


def wall_points(grid, ox, oy, res, limit=6000):
    """Map-frame (x, y) of the occupied pixels, thinned to at most `limit` of them."""
    rows, cols = np.nonzero(grid >= OCCUPIED)
    if len(rows) > limit:
        step = len(rows) // limit + 1
        rows, cols = rows[::step], cols[::step]
    return ox + (cols + 0.5) * res, oy + (rows + 0.5) * res


def _line_fit(v, cell, band, bins=64):
    """Best offset for one axis: how far to shift `v` so the most values sit on WALL lines.

    Wall lines are the cell boundaries, at (k + 1/2) * cell - cell (0, 0) is centred on the takeoff point, so its
    sides are at +-cell/2. Aligning to whole multiples of `cell` instead shifts the entire map half a cell, which
    scores worse than not fitting at all.

    Returns (score, offset). Works on the residual to the nearest line, histogrammed circularly, so it costs one
    pass over the points per axis rather than a search over offsets.
    """
    resid = v % cell - cell / 2
    counts, edges = np.histogram(resid, bins=bins, range=(-cell / 2, cell / 2))
    width = max(1, int(round(band / (cell / bins))))
    window = np.concatenate([counts, counts])          # circular: a line can wrap past +-cell/2
    sums = np.array([window[k:k + 2 * width + 1].sum() for k in range(bins)])
    k = int(np.argmax(sums))
    centre = edges[k] + (width + 0.5) * (cell / bins)
    return float(sums[k]), float((centre + cell / 2) % cell - cell / 2)


def fit_manhattan(grid, ox, oy, res, cell=1.0, band=0.12, max_rot_deg=12.0, step_deg=0.5):
    """Rotation and offset that put the most wall pixels on grid lines: (theta_rad, dx, dy).

    A point in the corrected frame is p_c = R(-theta) . p_s - (dx, dy), where p_s is in the SLAM frame.
    Rotation is about the SLAM origin, which is the takeoff point, so cell (0, 0) stays put by construction.
    Returns (0, 0, 0) if the map has too few wall pixels to fit anything.

    LIMIT: the offset can only be recovered within half a cell. Shifting a grid by a whole cell looks identical,
    so a drift of 0.8 m aliases to -0.2 m and would re-label every cell. That is the right trade here because
    Cartographer's origin IS the takeoff point: the map is anchored where the drone started and drift grows with
    distance from it, so the offset near the anchor is small. Rotation has no such ambiguity and is recovered
    exactly. What this cannot fix at all is a map that is internally inconsistent (the same wall drawn twice at
    different angles) - there is no single rigid transform for that.
    """
    xs, ys = wall_points(grid, ox, oy, res)
    if len(xs) < 200:
        return 0.0, 0.0, 0.0
    best = (-1.0, 0.0, 0.0, 0.0)
    for deg in np.arange(-max_rot_deg, max_rot_deg + 1e-9, step_deg):
        th = math.radians(deg)
        c, s = math.cos(-th), math.sin(-th)
        rx, ry = c * xs - s * ys, s * xs + c * ys
        sx, offx = _line_fit(rx, cell, band)
        sy, offy = _line_fit(ry, cell, band)
        if sx + sy > best[0]:
            best = (sx + sy, th, offx, offy)
    return best[1], best[2], best[3]


def corrected_sampler(grid, ox, oy, res, fit):
    """Sampler that takes CORRECTED map points and reads the drifted map underneath them."""
    raw = make_sampler(grid, ox, oy, res)
    theta, dx, dy = fit
    c, s = math.cos(theta), math.sin(theta)

    def f(xs, ys):
        px, py = np.asarray(xs) + dx, np.asarray(ys) + dy
        return raw(c * px - s * py, s * px + c * py)
    return f


def classify_map(grid, ox, oy, res, p=GridParams(), fit=None):
    """Return {"i,j": {"+x": ..., "-x": ..., "+y": ..., "-y": ..., "seen": True}} for every seen cell.

    With `fit` (from fit_manhattan) the cells are classified in the corrected frame, so a rigid SLAM drift
    does not smear walls off their grid lines.
    """
    sample = corrected_sampler(grid, ox, oy, res, fit) if fit else make_sampler(grid, ox, oy, res)
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
