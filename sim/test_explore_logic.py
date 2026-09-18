#!/usr/bin/env python3
"""Offline test of the explorer brain: walk a maze with NO drone and NO ROS, in milliseconds.

Feeds explore_logic the TRUE maze and checks it reaches every cell without leaving the arena. This
is not a flight test - it only proves the search is correct, so that a failed flight means a flight
problem. The "fog of war" run is the honest one: the drone starts knowing only the cell it is in and
learns each cell by seeing into it through an opening, which is what happens in the air.

    python3 sim/test_explore_logic.py [rooms_1 ...]     (extra worlds from sim/worlds/<name>_truth.json)
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/airmouse"))
import math

from airmouse.explore_logic import (NEIGHBOUR, camera_gain, corridor_penalty, frontier_step, line_of_sight,
                                    room_centres, next_step, open_neighbours, passable, path_home, side_between,
                                    straight_run)

TRUTH = Path(__file__).parent / "worlds/practice_6x6_truth.json"
HOME = (0, 0)
BLOCKED = {(HOME, "-x")}   # the entrance: open, but it leads out of the arena

# truth uses (row, col) with N/S/E/W and True = wall; the map frame is rotated 90 deg from the world
# (see plan_tour.py): i = wy, j = -wx, so N -> +x, S -> -x, E -> -y, W -> +y
SIDE_OF = {"N": "+x", "S": "-x", "E": "-y", "W": "+y"}


def truth_to_cells(truth):
    cells = {}
    for key, c in truth["cells"].items():
        wx, wy = c["world_xy"]
        cell = (int(round(wy)), int(round(-wx)))
        cells[cell] = {SIDE_OF[d]: ("wall" if c[d] else "open") for d in SIDE_OF}
        cells[cell]["seen"] = True
    return cells


def learn(known, full_map, cell):
    """The drone maps the cell it is in, and sees through its openings into the next cells."""
    if cell in full_map:
        known[cell] = full_map[cell]
    for side, (di, dj) in NEIGHBOUR.items():
        nxt = (cell[0] + di, cell[1] + dj)
        if full_map.get(cell, {}).get(side) == "open" and nxt in full_map:
            known.setdefault(nxt, full_map[nxt])


def walk(full_map, fog, max_leg=1):
    """Run the explorer decisions to completion. Returns (visited, moves, known map, decisions, legs).

    decisions = times the drone stops to settle and re-plan; legs = straight flights between stops
    or corners (a leg is what the flight controller flies without braking).
    """
    known = {}
    if fog:
        learn(known, full_map, HOME)
    else:
        known = dict(full_map)
    current, visited, moves = HOME, {HOME}, 0
    decisions = legs = 0
    for _ in range(10000):
        kind, path = next_step(known, current, visited, BLOCKED, max_leg=max_leg)
        if kind == "done":
            return visited, moves, known, decisions, legs
        decisions += 1
        i, at = 0, current
        while i < len(path):
            n = straight_run(at, path[i:])
            legs += 1
            at, i = path[i + n - 1], i + n
        for cell in path:
            current, moves = cell, moves + 1
            visited.add(cell)
            if fog:
                learn(known, full_map, cell)
    raise SystemExit("FAIL: explorer did not finish in 10000 decisions (loop?)")


def visible_cells(full_map, cell, reach=12):
    """Cells the LiDAR sees from the centre of `cell` (explore_logic.line_of_sight on the TRUE map)."""
    return line_of_sight(full_map, cell, reach)


CAM_REACH = 3.0                      # m at which the camera still recognises a person
CAM_HALF_FOV = math.radians(35)      # OAK-D RGB ~69 deg horizontal
SPIN_TIME = 8.0                      # s for a 360 deg yaw spin at a stop


def camera_pass(full_map, cam_seen, a, b, fixed_yaw):
    """What the camera sees flying from cell a into b: facing the way we fly, or always +x (fixed_yaw)."""
    heading = 0.0 if fixed_yaw else math.atan2(b[1] - a[1], b[0] - a[0])
    cam_seen |= line_of_sight(full_map, b, CAM_REACH, (heading, CAM_HALF_FOV))


def walk_frontier(full_map, turn_cost_time=1.5, settle=4.0, speed=0.5, camera="fixed", drift=0.0):
    """Frontier exploration with a LiDAR that sees along straight lines. Returns metrics and the final map.

    camera: "fixed" = today's flight (yaw always 0, camera stares along +x, choice ignores it);
    "utility" = face the way we fly, spin at stops that show the camera something new, and choose stops by
    utility_step. drift = weight of corridor_penalty in route costs.
    """
    known = {c: full_map[c] for c in visible_cells(full_map, HOME)}
    current, visited = HOME, {HOME}
    fixed = camera == "fixed"
    cam_seen = line_of_sight(full_map, HOME, CAM_REACH, (0.0, CAM_HALF_FOV))
    edge = (lambda cs, x, y: drift * corridor_penalty(cs, x, y)) if drift else None
    moves = decisions = legs = spins = degenerate = 0
    for _ in range(2000):
        if not fixed and camera_gain(full_map, current, cam_seen, CAM_REACH):
            cam_seen |= line_of_sight(full_map, current, CAM_REACH)
            spins += 1
        kind, path = frontier_step(known, current, visited, BLOCKED, cam_seen=None if fixed else cam_seen,
                                   cam_reach=CAM_REACH, edge_cost=edge)
        if kind == "done":
            break
        decisions += 1
        i, at = 0, current
        while i < len(path):
            n = straight_run(at, path[i:])
            legs += 1
            at, i = path[i + n - 1], i + n
        for cell in path:
            degenerate += int(corridor_penalty(full_map, current, cell))
            camera_pass(full_map, cam_seen, current, cell, fixed)
            current, moves = cell, moves + 1
            visited.add(cell)
        for c in visible_cells(full_map, current):
            known.setdefault(c, full_map[c])
    else:
        raise SystemExit("FAIL: frontier explorer did not finish in 2000 decisions (loop?)")
    home = path_home(known, current, visited, HOME, BLOCKED, through_seen=True) or []
    home_legs, i, at = 0, 0, current
    while i < len(home):
        n = straight_run(at, home[i:])
        home_legs, at, i = home_legs + 1, home[i + n - 1], i + n
    est = (moves + len(home)) / speed + decisions * settle + (legs + home_legs) * turn_cost_time + spins * SPIN_TIME
    return dict(visited=visited, known=known, moves=moves, decisions=decisions, legs=legs, est=est,
                cam=len(cam_seen & set(full_map)) / len(full_map), spins=spins, degenerate=degenerate)


def check_frontier(full_map, failures, name):
    runs = {}
    for label, kw in (("fixed-yaw nearest frontier (today)", dict(camera="fixed")),
                      ("utility + camera", dict(camera="utility")),
                      ("utility + camera + drift 1.0", dict(camera="utility", drift=1.0))):
        r = runs[label] = walk_frontier(full_map, **kw)
        visited, known = r["visited"], r["known"]
        missed = sorted(set(full_map) - set(known))
        outside = sorted(c for c in visited if c not in full_map)
        far = max(visited, key=lambda c: len(path_home(known, c, visited, HOME, BLOCKED) or []))
        home = path_home(known, far, visited, HOME, BLOCKED)
        direct = path_home(known, far, visited, HOME, BLOCKED, through_seen=True)
        ok = not missed and not outside and home is not None and direct is not None and len(direct) <= len(home)
        if label != "fixed-yaw nearest frontier (today)":
            ok = ok and r["cam"] >= 0.98     # the point of it: the camera has looked at (nearly) every cell
        print(f"{'PASS' if ok else 'FAIL'}  {name}, FRONTIER {label}: mapped {len(known)}/{len(full_map)}, "
              f"camera saw {r['cam']:.0%}, {r['moves']} moves, {r['decisions']} stops, {r['spins']} spins, "
              f"{r['legs']} legs, {r['degenerate']} one-wall hops, ~{r['est'] / 60:.1f} min incl. exit")
        if missed:
            print(f"      never SEEN: {missed[:8]}{' ...' if len(missed) > 8 else ''}")
        if outside:
            print(f"      LEFT THE ARENA: {outside}")
        if home is None:
            print("      no route home")
        failures += 0 if ok else 1
    centres = room_centres(full_map, BLOCKED)
    got = [c for c in centres if any((c[0] + a, c[1] + b) in runs["utility + camera"]["visited"]
                                     for a in (-1, 0, 1) for b in (-1, 0, 1))]
    print(f"      room centres on the TRUE map: {len(centres)}, flown to (within 1 cell, utility): {len(got)}")
    return failures


def check_coverage(full_map, failures, max_leg=1, name="maze"):
    total = len(full_map)
    for fog in (False, True):
        label = f"{name}, {'fog of war (map learned by looking)' if fog else 'perfect map'}, max_leg={max_leg}"
        visited, moves, known, decisions, legs = walk(full_map, fog, max_leg)
        missed = sorted(set(full_map) - visited)
        outside = sorted(c for c in visited if c not in full_map)
        home = path_home(known, max(visited), visited, HOME, BLOCKED)
        ok = not missed and not outside and home is not None
        est = moves / 0.5 + decisions * 4.0 + legs * 1.5
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {len(visited)}/{total} cells, {moves} moves, "
              f"{decisions} stops, {legs} legs, ~{est / 60:.1f} min")
        if missed:
            print(f"      never reached: {missed}")
        if outside:
            print(f"      LEFT THE ARENA: {outside}")
        if home is None:
            print("      no route home from the far corner")
        failures += 0 if ok else 1
    return failures


def check_entrance(full_map, failures):
    """require_seen already hides the outside, so test the block with that guard OFF - otherwise
    this passes for the wrong reason and stops protecting us the day we relax require_seen."""
    entrance = (HOME[0] - 1, HOME[1])
    if full_map[HOME].get("-x") != "open":
        print("FAIL  test is stale: the takeoff cell has no open entrance side to block")
    elif not passable(full_map, HOME, entrance, (), require_seen=False):
        print("FAIL  entrance unreachable even unblocked - check side_between / the truth map")
    elif passable(full_map, HOME, entrance, BLOCKED, require_seen=False):
        print("FAIL  entrance is NOT blocked - the drone would leave the arena")
    else:
        print("PASS  entrance side is refused, with and without require_seen")
        return failures
    return failures + 1


def check_stalled_map(failures):
    """Regression (run 19): a real map where every frontier was an UNKNOWN side must not read as 'done'."""
    data = json.loads((Path(__file__).parent / "testdata/run19_stalled_map.json").read_text())
    cells = {tuple(int(v) for v in k.split(",")): c for k, c in data["cells"].items()}
    visited = {tuple(v) for v in data["visited"]}
    kind, path = frontier_step(cells, tuple(data["current"]), visited, BLOCKED)
    if kind == "done":
        print("FAIL  run-19 map: frontier_step says done but 18 cells face unmapped space")
        return failures + 1
    print(f"PASS  run-19 map: not done - {kind} route of {len(path)} cells")
    return failures


def main():
    full_map = truth_to_cells(json.loads(TRUTH.read_text()))
    failures = check_coverage(full_map, 0, max_leg=1)
    failures = check_coverage(full_map, failures, max_leg=3)
    failures = check_frontier(full_map, failures, "maze")
    for name in sys.argv[1:]:        # e.g. rooms_1: a generated building of rooms (make_rooms.py)
        other = truth_to_cells(json.loads((TRUTH.parent / f"{name}_truth.json").read_text()))
        failures = check_coverage(other, failures, max_leg=1, name=name)
        failures = check_coverage(other, failures, max_leg=3, name=name)
        failures = check_frontier(other, failures, name)
    failures = check_entrance(full_map, failures)
    failures = check_stalled_map(failures)

    off = [(c, n) for c in full_map for n in open_neighbours(full_map, c, BLOCKED) if n not in full_map]
    if off:
        print(f"FAIL  would fly outside the maze: {off[:3]}")
        failures += 1
    else:
        print("PASS  never routes through a wall or off the map")

    assert side_between((0, 0), (1, 0)) == "+x" and side_between((0, 0), (0, 2)) is None
    assert straight_run((0, 0), [(1, 0), (2, 0), (2, 1)]) == 2 and straight_run((0, 0), [(1, 0)]) == 1
    assert straight_run((0, 0), [(0, 1), (1, 1)]) == 1 and straight_run((0, 0), []) == 0
    print("ALL PASS" if not failures else f"{failures} FAILURE(S)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
