"""Pure exploration logic (no ROS), shared by the explorer node and offline tests.

Works entirely in grid cells. Cell (i, j) is centred on map point (i, j) * cell_size, and the drone
takes off in the centre of cell (0, 0), so the grid the mapper publishes and the coordinates we fly
to are the same numbers (see grid_logic.py).

Two rules keep us alive and inside the arena:

1. We only fly through a side the map calls "open". "unknown" counts as "wall". SLAM is good to
   ~0.1-0.3 m and the drone is 33 cm tip to tip in a 1 m gap, so guessing is not worth a crash (-50 pts).
2. We only fly into a cell the mapper has actually seen (require_seen). Looking through a 1 m
   doorway the LiDAR maps most of the next cell, so this costs nothing indoors - but it stops the
   drone wandering into open space it has not mapped.

The entrance is a special case: an "open" side of the takeoff cell that leads OUT of the arena.
The first version of this code cheerfully flew through it and stranded itself outside with no map.
The `blocked` set holds sides we must never cross; the node blocks the entrance by default.
"""
import heapq
import math
from collections import deque

# side name -> step to the neighbouring cell, and the name of that same wall from the other side
NEIGHBOUR = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}
OPPOSITE = {"+x": "-x", "-x": "+x", "+y": "-y", "-y": "+y"}


def parse_grid(payload):
    """/airmouse/grid JSON -> (cell_size, {(i, j): {side: state}})."""
    cell_size = payload.get("cell_size", 1.0)
    cells = {}
    for key, sides in payload.get("cells", {}).items():
        i, j = (int(v) for v in key.split(","))
        cells[(i, j)] = sides
    return cell_size, cells


def side_between(a, b):
    """Name of the side of cell a facing neighbouring cell b, or None if they are not adjacent."""
    for side, (di, dj) in NEIGHBOUR.items():
        if (a[0] + di, a[1] + dj) == b:
            return side
    return None


def passable(cells, a, b, blocked=(), require_seen=True):
    """May we fly from cell a to neighbouring cell b?

    Both cells get a veto. The side must be "open" as seen from a, and if b has been mapped too it
    must not call that same wall a wall - the two disagree at the edge of the map, where a side is
    often seen from one room only.
    """
    side = side_between(a, b)
    if side is None or a not in cells:
        return False
    if (a, side) in blocked:
        return False
    if cells[a].get(side) != "open":
        return False
    if b in cells:
        return cells[b].get(OPPOSITE[side]) != "wall"
    return not require_seen


def open_neighbours(cells, cell, blocked=(), require_seen=True):
    """Cells we may fly to from `cell`, in a fixed order so runs are repeatable."""
    return [(cell[0] + di, cell[1] + dj) for side, (di, dj) in NEIGHBOUR.items()
            if passable(cells, cell, (cell[0] + di, cell[1] + dj), blocked, require_seen)]


def bfs_path(cells, start, goal, allowed=None, blocked=(), require_seen=True):
    """Shortest path start -> goal through passable sides, as a list of cells NOT including start.

    `allowed` restricts which cells may be flown through (we only route back through cells we have
    already been to, never through a shortcut we have not verified in person).
    Returns None if there is no known route.
    """
    if start == goal:
        return []
    seen, queue = {start: None}, deque([start])
    while queue:
        cell = queue.popleft()
        for nxt in open_neighbours(cells, cell, blocked, require_seen):
            if nxt in seen:
                continue
            if nxt != goal and allowed is not None and nxt not in allowed:
                continue
            seen[nxt] = cell
            if nxt == goal:
                path = [goal]
                while seen[path[-1]] != start:
                    path.append(seen[path[-1]])
                return path[::-1]
            queue.append(nxt)
    return None


def frontier_cells(cells, visited, blocked=(), require_seen=True):
    """Visited cells that still have an unvisited neighbour we are allowed to fly to."""
    return [cell for cell in visited
            if any(nxt not in visited
                   for nxt in open_neighbours(cells, cell, blocked, require_seen))]


def straight_run(start, path):
    """How many cells of `path` (which starts next to `start`) continue in ONE straight line.

    The explorer flies that many cells in a single leg instead of braking at every cell centre.
    [(1,0),(2,0),(2,1)] from (0,0) -> 2: the first two cells are collinear, the third turns.
    """
    if not path:
        return 0
    step = (path[0][0] - start[0], path[0][1] - start[1])
    n, prev = 1, path[0]
    for cell in path[1:]:
        if (cell[0] - prev[0], cell[1] - prev[1]) != step:
            break
        n, prev = n + 1, cell
    return n


def extend_forward(cells, current, path, visited, blocked=(), require_seen=True, max_leg=1):
    """Lengthen a one-cell "step" into a straight leg of up to `max_leg` cells.

    Each extra cell must pass the same test as any single step: side open from both cells, cell already
    seen by the mapper, and not somewhere we have been (a visited cell is a backtrack, not a discovery).
    """
    path = list(path)
    if len(path) != 1 or max_leg <= 1:
        return path
    step = (path[0][0] - current[0], path[0][1] - current[1])
    while len(path) < max_leg:
        nxt = (path[-1][0] + step[0], path[-1][1] + step[1])
        if nxt in visited or not passable(cells, path[-1], nxt, blocked, require_seen):
            break
        path.append(nxt)
    return path


def next_step(cells, current, visited, blocked=(), require_seen=True, max_leg=1):
    """Decide where to go next. Returns (kind, path), path being cells to fly through in order.

    kind is one of:
        "step"      a hop into a new cell (depth-first: keep going while there is somewhere to go);
                    up to `max_leg` cells in a straight line when the ones beyond are open and seen
        "backtrack" several hops through known cells, to the nearest cell that still has an exit
        "done"      every reachable cell has been visited
    """
    for nxt in open_neighbours(cells, current, blocked, require_seen):
        if nxt not in visited:
            return "step", extend_forward(cells, current, [nxt], visited, blocked, require_seen, max_leg)

    # dead end: walk back to the nearest cell that still has an unexplored exit
    best = None
    for cell in frontier_cells(cells, visited, blocked, require_seen):
        path = bfs_path(cells, current, cell, visited, blocked, require_seen)
        if path is not None and (best is None or len(path) < len(best)):
            best = path
    if best:
        return "backtrack", best
    return "done", []


def _dijkstra(cells, start, allowed, blocked, require_seen, turn_cost, edge_cost, is_goal=None):
    """Fewest-turns search shared by plan() and route_costs(). Yields (cost, cell, key) in cost order;
    rebuild a route from `key` with _route(prev, start, key)."""
    best = {(start, None): 0.0}
    prev = {}
    heap = [(0.0, 0, start, None)]
    tick = 0
    while heap:
        cost, _, cell, heading = heapq.heappop(heap)
        if cost > best.get((cell, heading), float("inf")):
            continue
        yield cost, cell, (cell, heading), prev
        for side, (di, dj) in NEIGHBOUR.items():
            nxt = (cell[0] + di, cell[1] + dj)
            if not passable(cells, cell, nxt, blocked, require_seen):
                continue
            if allowed is not None and nxt not in allowed and not (is_goal and is_goal(nxt)):
                continue
            step = cost + 1.0 + (turn_cost if heading not in (None, side) else 0.0)
            if edge_cost is not None:
                step += edge_cost(cells, cell, nxt)
            if step < best.get((nxt, side), float("inf")):
                best[(nxt, side)] = step
                prev[(nxt, side)] = (cell, heading)
                tick += 1
                heapq.heappush(heap, (step, tick, nxt, side))


def _route(prev, start, key):
    path = []
    while key[0] != start:
        path.append(key[0])
        key = prev[key]
    return path[::-1]


def plan(cells, start, is_goal, allowed=None, blocked=(), require_seen=True, turn_cost=0.6, edge_cost=None):
    """Cheapest route from `start` to the nearest cell where is_goal(cell) is true.

    Like bfs_path, but a turn costs extra (`turn_cost` cells' worth): the flight controller brakes at
    every corner, so a route with two long straights beats a staircase of the same length. Returns the
    cells to fly through, NOT including start, or None if no goal is reachable. `allowed` limits the
    cells we may fly through (goal cells are exempt). `edge_cost(cells, a, b)` adds to the price of a
    hop (see corridor_penalty).
    """
    for _, cell, key, prev in _dijkstra(cells, start, allowed, blocked, require_seen, turn_cost, edge_cost,
                                        is_goal):
        if cell != start and is_goal(cell):
            return _route(prev, start, key)
    return None


def route_costs(cells, start, blocked=(), require_seen=True, turn_cost=0.6, edge_cost=None):
    """Cost and route (as plan() would fly it) to EVERY reachable cell: {cell: (cost, path)}."""
    out = {}
    for cost, cell, key, prev in _dijkstra(cells, start, None, blocked, require_seen, turn_cost, edge_cost):
        if cell not in out and cell != start:
            out[cell] = (cost, key, prev)
    return {c: (cost, _route(prev, start, key)) for c, (cost, key, prev) in out.items()}


def corridor_penalty(cells, a, b):
    """1 for a hop into a cell with a wall on only ONE side across the direction of travel, else 0.

    That is the geometry where run 13's SLAM broke down ((2,-3), a single long wall: nothing pins motion
    along it). The offline analysis of run 13 did NOT confirm it as sufficient cause, so the explorer
    weights this 0 by default; it is here to test the idea, not because it is proven.
    """
    side = side_between(a, b)
    lateral = ("+y", "-y") if side in ("+x", "-x") else ("+x", "-x")
    return 1.0 if sum(cells.get(b, {}).get(s) == "wall" for s in lateral) == 1 else 0.0


def line_of_sight(cells, cell, reach, sector=None):
    """Cells visible from the centre of `cell` on the map `cells`: a ray to every cell within `reach`,
    stopped by the first side that is not "open". Lenient at exact corners (a ray grazing a corner passes
    if either way round is open). `sector` = (heading_rad, half_fov_rad) limits it to a camera's view.
    """
    def open_between(p, q):
        return p in cells and cells[p].get(side_between(p, q)) == "open"

    seen = {cell}
    r = int(math.ceil(reach))
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            tgt = (cell[0] + dx, cell[1] + dy)
            dist = math.hypot(dx, dy)
            if tgt in seen or tgt not in cells or dist > reach:
                continue
            if sector is not None:
                off = (math.atan2(dy, dx) - sector[0] + math.pi) % (2 * math.pi) - math.pi
                if abs(off) > sector[1]:
                    continue
            steps, at = max(1, int(dist * 20)), cell
            for k in range(1, steps + 1):
                here = (round(cell[0] + dx * k / steps), round(cell[1] + dy * k / steps))
                if here == at:
                    continue
                if abs(here[0] - at[0]) + abs(here[1] - at[1]) == 1:
                    ok = open_between(at, here)
                else:   # diagonal step across a corner
                    m1, m2 = (here[0], at[1]), (at[0], here[1])
                    ok = (open_between(at, m1) and open_between(m1, here)) or \
                         (open_between(at, m2) and open_between(m2, here))
                if not ok:
                    break
                at = here
                seen.add(at)
    return seen


def is_frontier(cells, cell, blocked=(), unknown=False):
    """A seen cell with an open side leading into a cell nobody has mapped yet: going there reveals more.

    With unknown=True a side the mapper could not classify also counts. At the edge of what the LiDAR has
    seen, a doorway viewed from far away or at a grazing angle comes out "unknown", not "open" (run 19
    found NO open frontier on a 71-cell map that had 18 unknown ones and called the mission done).
    """
    good = ("open", "unknown") if unknown else ("open",)
    for side, (di, dj) in NEIGHBOUR.items():
        if (cells[cell].get(side) in good and (cell, side) not in blocked
                and (cell[0] + di, cell[1] + dj) not in cells):
            return True
    return False


def is_door(cells, a, side):
    """Is the open side `side` of cell a a doorway - a gap in a wall - rather than open floor?

    On open floor the cells either side of the crossing also see open sides on that line; in a doorway
    at least one of them sees the WALL that the gap is cut in.
    """
    di, dj = NEIGHBOUR[side]
    lateral = ((0, 1), (0, -1)) if di else ((1, 0), (-1, 0))
    return any(cells.get((a[0] + li, a[1] + lj), {}).get(side) == "wall" for li, lj in lateral)


def room_centres(cells, blocked=(), min_side=3):
    """The middle cell of every room the map shows.

    Rooms are the regions left when every doorway is cut: flood-fill over open sides, but not through a
    door (checked from both ends). A region counts as a room once it is at least `min_side` cells in
    both directions - that also keeps corridors and half-mapped patches out. The centre is the cell
    nearest the middle of the region's bounding box.
    """
    def crossing(a, side):
        b = (a[0] + NEIGHBOUR[side][0], a[1] + NEIGHBOUR[side][1])
        return b if passable(cells, a, b, blocked) and not (is_door(cells, a, side) or
                                                             is_door(cells, b, OPPOSITE[side])) else None

    centres, done = [], set()
    for start in sorted(cells):
        if start in done:
            continue
        region, queue = {start}, deque([start])
        while queue:
            cur = queue.popleft()
            for side in NEIGHBOUR:
                nxt = crossing(cur, side)
                if nxt is not None and nxt not in region:
                    region.add(nxt)
                    queue.append(nxt)
        done |= region
        is_ = [c[0] for c in region]
        js = [c[1] for c in region]
        if max(is_) - min(is_) + 1 < min_side or max(js) - min(js) + 1 < min_side:
            continue
        mid = ((min(is_) + max(is_)) / 2, (min(js) + max(js)) / 2)
        centres.append(min(region, key=lambda c: ((c[0] - mid[0]) ** 2 + (c[1] - mid[1]) ** 2, c)))
    return centres


def camera_gain(cells, cell, cam_seen, cam_reach):
    """Cells a full yaw spin at `cell` would show the camera that it has not seen yet."""
    return line_of_sight(cells, cell, cam_reach) - cam_seen


def utility_step(cells, current, visited, cam_seen, blocked=(), require_seen=True, cam_reach=3.0,
                 lidar_value=3.0, stop_cost=6.0, edge_cost=None):
    """Pick the stop worth the most per unit of flying: (map gain + camera gain) / (route cost + stop cost).

    Map gain = `lidar_value` cells per open side leading into unmapped space (we cannot know what lies
    behind it; ~3 cells is what one doorway shows). Camera gain = cells a spin there would show the camera
    for the first time - survivors (240 pts) are found by the camera, and the LiDAR maps a room from its
    doorway long before the camera has looked into its corners. `stop_cost` (in cells of flight) is the
    hover + spin at every stop, so ten one-cell nibbles lose to one stop that sees the same.
    Returns ("frontier" | "camera", path) or (None, []) when nothing is left to gain.
    """
    best, best_score = None, 0.0
    for cell, (cost, path) in route_costs(cells, current, blocked, require_seen, edge_cost=edge_cost).items():
        if cell not in cells:
            continue
        lidar = 0 if cell in visited else sum(
            cells[cell].get(side) == "open" and (cell, side) not in blocked
            and (cell[0] + di, cell[1] + dj) not in cells
            for side, (di, dj) in NEIGHBOUR.items())
        gain = lidar * lidar_value + len(camera_gain(cells, cell, cam_seen, cam_reach))
        score = gain / (cost + stop_cost)
        if score > best_score + 1e-9:
            best, best_score = ("frontier" if lidar else "camera", path), score
    return best if best else (None, [])


def frontier_step(cells, current, visited, blocked=(), require_seen=True, centre_reach=6,
                  cam_seen=None, cam_reach=3.0, edge_cost=None):
    """Frontier exploration: decide where to fly next.

    1. If we are in or beside a room whose centre we have not been near, go to the centre (within
       `centre_reach` cells): from the middle every wall is seen square-on, which is what the judges'
       map needs. 0 turns this off.
    2. Otherwise fly to the nearest unvisited frontier cell, however far away.

    The LiDAR sees a whole room from anywhere inside it and the judges score the MAP, not where the
    drone has been - so unlike next_step this does not tour every cell. Routes may cross any seen cell
    whose sides are open. Returns ("centre" | "frontier" | "camera" | "look", path) or ("done", []).

    With `cam_seen` (the cells the camera has already looked at) step 2 becomes utility_step: the stop with
    the most new map AND new camera view per unit of flight, not simply the nearest frontier.
    """
    if centre_reach > 0:
        wanted = {c for c in room_centres(cells, blocked)
                  if not any((c[0] + di, c[1] + dj) in visited for di in (-1, 0, 1) for dj in (-1, 0, 1))}
        if wanted:
            path = plan(cells, current, lambda c: c in wanted, None, blocked, require_seen, edge_cost=edge_cost)
            if path and len(path) <= centre_reach:
                return "centre", path
    if cam_seen is not None:
        kind, path = utility_step(cells, current, visited, cam_seen, blocked, require_seen, cam_reach,
                                  edge_cost=edge_cost)
        if path:
            return kind, path
    path = plan(cells, current,
                lambda c: c in cells and c not in visited and is_frontier(cells, c, blocked),
                None, blocked, require_seen, edge_cost=edge_cost)
    if path:
        return "frontier", path
    # nothing certain is left: go and LOOK at cells whose sides facing unmapped space are undecided
    path = plan(cells, current,
                lambda c: c in cells and c not in visited and is_frontier(cells, c, blocked, unknown=True),
                None, blocked, require_seen, edge_cost=edge_cost)
    return ("look", path) if path else ("done", [])


def path_home(cells, current, visited, home=(0, 0), blocked=(), require_seen=True, through_seen=False):
    """Route back to the takeoff cell (fewest turns).

    Normally only through cells we have already flown. With through_seen=True any cell the map has seen
    and calls open will do - the frontier explorer visits few cells, so retracing its exact route
    can be twice as long as the direct way (it flies through unvisited seen cells on the way out too).
    """
    if current == home:
        return []
    return plan(cells, current, lambda c: c == home, None if through_seen else visited, blocked, require_seen)
