#!/usr/bin/env python3
"""Generate a building of rooms (not a maze) as an ASCII drawing, then build the Gazebo world from it.

    python3 make_rooms.py --seed 1                 # 7 rooms wide, rooms 3-8 m, 1 m doors
    python3 make_rooms.py --seed 2 --wide 5 --depth 20 --extra-doors 0.5

Layout: the arena is `--wide` columns of rooms side by side. Each column has a random width of
3-8 m and is split top to bottom into rooms 3-8 m deep, independently of its neighbours, so walls do
not line up in a grid. Every wall between two touching rooms can hold a door. Doors are exactly 1 m
wide (one cell of the 1 m grid) at a random spot along the shared wall, at least 1 m from a corner
when the wall is long enough.

Connectivity: a random spanning tree of the room-touching graph always gets a door, so every room is
reachable; each remaining shared wall gets one with probability `--extra-doors` (loops). One gap in
the bottom outer wall is the entrance, as in the mazes. Same seed -> same building.

Writes mazes/rooms_<seed>.txt, then runs make_maze.py on it (worlds/rooms_<seed>.sdf + _truth.json).
"""
import argparse
import random
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).parent


def compose(total, lo, hi, rng):
    """Random split of `total` into parts in [lo, hi]."""
    for _ in range(1000):
        parts, left = [], total
        while left > 0:
            sizes = [s for s in range(lo, hi + 1) if s <= left and (left - s == 0 or left - s >= lo)]
            if not sizes:
                break
            s = rng.choice(sizes)
            parts.append(s)
            left -= s
        if left == 0:
            return parts
    raise SystemExit(f"cannot split {total} m into rooms of {lo}-{hi} m")


def build(wide, depth, lo, hi, extra, rng):
    widths = [rng.randint(lo, hi) for _ in range(wide)]
    W, H = sum(widths), depth
    room = [[None] * W for _ in range(H)]       # room id of every 1 m cell
    rooms, x = [], 0
    for w in widths:
        y = 0
        for d in compose(H, lo, hi, rng):
            rid = len(rooms)
            rooms.append((x, x + w, y, y + d))  # c0, c1, r0, r1 (half open)
            for r in range(y, y + d):
                for c in range(x, x + w):
                    room[r][c] = rid
            y += d
        x += w

    def at(r, c):
        return room[r][c] if 0 <= r < H and 0 <= c < W else None

    # wall on every boundary between different rooms (or the outside)
    h = [[at(r - 1, c) != at(r, c) for c in range(W)] for r in range(H + 1)]
    v = [[at(r, c - 1) != at(r, c) for c in range(W + 1)] for r in range(H)]

    # every pair of touching rooms and the cells of the wall they share
    edges = {}
    for c in range(1, W):
        for r in range(H):
            a, b = at(r, c - 1), at(r, c)
            if a != b:
                edges.setdefault((min(a, b), max(a, b)), ("v", c, []))[2].append(r)
    for r in range(1, H):
        for c in range(W):
            a, b = at(r - 1, c), at(r, c)
            if a != b:
                edges.setdefault((min(a, b), max(a, b)), ("h", r, []))[2].append(c)

    # random spanning tree (union-find over shuffled edges) + some extra doors
    parent = list(range(len(rooms)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    keys = list(edges)
    rng.shuffle(keys)
    doors = []
    for k in keys:
        ra, rb = find(k[0]), find(k[1])
        if ra != rb:
            parent[ra] = rb
            doors.append(k)
        elif rng.random() < extra:
            doors.append(k)
    for k in doors:
        kind, line, cells = edges[k]
        cells = sorted(cells)
        pick = cells[1:-1] if len(cells) >= 3 else cells      # keep the door off the corners
        d = rng.choice(pick)
        if kind == "v":
            v[d][line] = False
        else:
            h[line][d] = False

    # entrance: one gap in the bottom wall, a metre in from the room's corners
    bottom = [rid for rid, (c0, c1, r0, r1) in enumerate(rooms) if r1 == H]
    c0, c1, _, _ = rooms[rng.choice(bottom)]
    h[H][rng.randint(c0 + 1, c1 - 2)] = False
    return W, H, h, v, rooms, len(doors)


def render(W, H, h, v):
    out = []
    for r in range(H + 1):
        out.append("+" + "+".join("--" if h[r][c] else "  " for c in range(W)) + "+")
        if r < H:
            out.append("".join(("|" if v[r][c] else " ") + ("  " if c < W else "") for c in range(W + 1)))
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--wide", type=int, default=7, help="rooms side by side (default 7)")
    ap.add_argument("--depth", type=int, default=16, help="arena depth in metres (default 16)")
    ap.add_argument("--min", type=int, default=3, dest="lo", help="smallest room side, m")
    ap.add_argument("--max", type=int, default=8, dest="hi", help="largest room side, m")
    ap.add_argument("--extra-doors", type=float, default=0.35,
                    help="chance of a door on each wall the spanning tree did not need (default 0.35)")
    ap.add_argument("--name", help="output name (default rooms_<seed>)")
    args = ap.parse_args()
    if args.depth < args.lo:
        raise SystemExit("--depth must be at least --min")

    rng = random.Random(args.seed)
    W, H, h, v, rooms, ndoors = build(args.wide, args.depth, args.lo, args.hi, args.extra_doors, rng)
    name = args.name or f"rooms_{args.seed}"
    path = HERE / "mazes" / f"{name}.txt"
    path.write_text(render(W, H, h, v))
    sizes = sorted((c1 - c0, r1 - r0) for c0, c1, r0, r1 in rooms)
    print(f"{name}: {W} x {H} m, {len(rooms)} rooms ({sizes[0][0]}x{sizes[0][1]} .. "
          f"{max(sizes)[0]}x{max(sizes)[1]} m), {ndoors} doors of 1 m")
    subprocess.run([sys.executable, str(HERE / "make_maze.py"), str(path)], check=True)


if __name__ == "__main__":
    main()
