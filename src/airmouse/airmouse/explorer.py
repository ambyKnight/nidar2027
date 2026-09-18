"""Step 8: explore the maze on our own - no route planned in advance, no knowledge of the layout.

This is the node that makes the flight autonomous. plan_tour.py flew a route computed from the TRUE
maze; the explorer only ever looks at /airmouse/grid, the map the drone is building as it flies.

Cell by cell: stop in the centre of each new cell, hover until the map of that cell settles, then
pick the next cell. Depth-first while there is somewhere new to go, shortest known path back when we
hit a dead end, and finally back to the takeoff cell to land. Deliberately unhurried - in a 1 m
corridor with a 33 cm drone and 0.1-0.3 m SLAM error, arriving is worth more than arriving quickly.
The under-15-minute bonus comes later, by flying multi-cell legs once this works end to end.

    ros2 run airmouse explorer
    ros2 run airmouse explorer --ros-args -p altitude:=1.2 -p settle_time:=2.5

Safety rails, all of which have bitten us before:
  - only ever crosses a side the map calls "open" ("unknown" counts as a wall)
  - refuses the entrance side of the takeoff cell, which is open but leads out of the arena
  - hard budget (mission_timeout, including the trip home): we turn for home while we still can
  - stuck_timeout on every leg, so a waypoint we cannot reach lands the drone instead of hovering
"""
import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from airmouse.explore_logic import (camera_gain, corridor_penalty, frontier_step, is_frontier, line_of_sight,
                                    next_step, parse_grid, passable, path_home, straight_run)
from airmouse.flight import CopterNode

HOME = (0, 0)
# ToF range sensors on the 4 edges of the frame: name -> direction relative to the nose (rad)
TOF_SIDES = {"front": 0.0, "left": math.pi / 2, "back": math.pi, "right": -math.pi / 2}


class Explorer(CopterNode):
    def __init__(self):
        super().__init__("explorer", altitude=1.2)
        self.altitude = self.declare_parameter("altitude", 1.2).value
        self.cell_size = self.declare_parameter("cell_size", 1.0).value
        self.reached_tol = self.declare_parameter("reached_tol", 0.15).value
        # hover this long in each new cell so /map (and so the grid) catches up before we decide.
        # /map arrives at ~1.5 Hz and grid_mapper re-classifies at 1 Hz, so a short settle means
        # deciding on a map that has barely changed since we arrived.
        self.settle_time = self.declare_parameter("settle_time", 2.0).value
        # never decide until this many grids have been published since we got here
        self.min_grid_updates = self.declare_parameter("min_grid_updates", 2).value
        # "nowhere left to go" is also what a half-built map looks like, and acting on it ends the
        # mission. The first flight landed after ONE cell because the corridor out of the takeoff
        # cell was still "unknown" 2.6 s in. So treat "done" as a claim and re-check it.
        self.done_grace = self.declare_parameter("done_grace", 5.0).value
        self.done_updates = self.declare_parameter("done_updates", 3).value
        self.stuck_timeout = self.declare_parameter("stuck_timeout", 45.0).value
        # Fly straight runs of cells in ONE leg instead of braking at every cell centre (ArduPilot stops
        # at each position setpoint, so cell-by-cell flight is stop-and-go). Forward exploration is
        # capped at max_leg cells; backtracks and the way home use whole straight runs through cells we
        # have already flown. Cells passed within pass_tol of their centre count as visited.
        # max_leg = 1 restores the old one-cell-at-a-time flight.
        self.max_leg = self.declare_parameter("max_leg", 3).value
        # "frontier": fly to the nearest cell whose open side leads to unmapped space and look from
        # there - the LiDAR sees a whole room from inside it, and the judges score the MAP, not where
        # we flew. "dfs": the old tour of every cell (kept as a fallback; uses max_leg).
        self.strategy = self.declare_parameter("strategy", "frontier").value
        # Do not stop to decide: if the cell we are flying to gets resolved on the way (its open side no longer
        # leads to unmapped space) re-plan the next frontier at once. Stopping to settle then only happens when
        # the target is STILL a frontier on arrival (we have to look) or nothing is left.
        # Off = the old fly, stop, settle, decide cycle.
        self.chain = self.declare_parameter("chain", True).value
        # On entering a room, fly to its centre (within this many cells) before moving on: from the middle
        # every wall is seen square-on. 0 = don't.
        self.centre_reach = self.declare_parameter("centre_reach", 6).value
        self.chain_seq = -1
        self.route_kind = "frontier"
        self.pass_tol = self.declare_parameter("pass_tol", 0.4).value
        # At a corner in the middle of a route, move on when this close instead of braking to a stop at
        # reached_tol. Cuts the corner by at most this much: the drone is 33 cm tip to tip (23 cm square) in 1 m
        # cells, so keep it well under ~0.3. The FINAL target of a route still needs reached_tol.
        self.corner_tol = self.declare_parameter("corner_tol", 0.25).value
        # total flight budget, INCLUDING the flight home: we turn for home once the estimated trip back
        # (home_path cells / wp_speed + turns + home_margin) would overrun it. 840 s = 14 min, inside
        # NIDAR's under-15-minute bonus.
        self.mission_timeout = self.declare_parameter("mission_timeout", 840.0).value
        self.home_margin = self.declare_parameter("home_margin", 60.0).value
        self.wp_speed = self.declare_parameter("wp_speed", 0.5).value      # WP_SPD in indoor.parm
        # Survivors (240 pts) are found by the CAMERA. "utility": pick stops by map + camera gain per unit of
        # flight (explore_logic.utility_step), so every cell gets looked at within cam_reach. "off" = ignore it.
        # cam_hfov_deg is the COMBINED view: 360 for our two 200 deg side cameras - then the drone never yaws
        # (yaw stresses the scan matcher and buys nothing) and never spins. Below 360 (a single forward camera)
        # the nose points along each leg and the drone spins at stops that would show the camera new cells.
        self.camera = self.declare_parameter("camera", "utility").value
        self.cam_reach = self.declare_parameter("cam_reach", 3.0).value
        cam_hfov = self.declare_parameter("cam_hfov_deg", 360.0).value
        self.omni = cam_hfov >= 360.0
        self.cam_half_fov = math.radians(cam_hfov / 2.0)
        # Wall guard on the 4 edge ToF sensors (/airmouse/tof/<side>). ArduPilot's own avoidance does NOT act
        # on the position targets we send in GUIDED (only on velocity targets), so the guard is ours. It fires
        # when a sensor reads less than tof_stop PLUS the distance needed to brake from the current speed
        # towards that wall (v^2 / 2 brake_acc + v * tof_latency): at 1 m/s a 10 cm trigger alone is ~35 cm
        # too late. On firing it holds a point backed away from the wall, then re-plans from where it is.
        self.tof_stop = self.declare_parameter("tof_stop", 0.10).value
        self.brake_acc = self.declare_parameter("brake_acc", 2.0).value        # WP_ACC in indoor.parm
        self.tof_latency = self.declare_parameter("tof_latency", 0.15).value  # sensor + ROS + FC, s
        self.guard_hold = self.declare_parameter("guard_hold", 1.5).value
        self.guard_limit = self.declare_parameter("guard_limit", 5).value      # firings per minute -> abort
        self.spin_hold = self.declare_parameter("spin_hold", 2.0).value   # s at each 90 deg heading of a spin
        # extra route cost per hop into a cell with a wall on one side only (see corridor_penalty). 0 = off:
        # the run-13 analysis did not confirm that geometry as the cause of the SLAM breakdown.
        drift = self.declare_parameter("drift_penalty", 0.0).value
        self.edge_cost = (lambda cells, a, b: drift * corridor_penalty(cells, a, b)) if drift > 0 else None
        # the entrance is an open side of the takeoff cell that leads outside - never cross it while exploring
        entrance = self.declare_parameter("entrance_side", "-x").value
        self.entrance_side = entrance
        self.auto_exit = self.declare_parameter("auto_exit", True).value
        self.exit_distance = self.declare_parameter("exit_distance", 1.2).value
        self.exit_target = None
        self.exit_heading = 0.0
        self.blocked = {(HOME, entrance)} if entrance else set()

        self.cells = {}
        self.grid_stamp = 0.0
        self.grid_seq = 0          # how many grids we have received, ever
        self.arrival_seq = 0       # grid_seq when we arrived in the current cell
        self.done_since = None     # when "done" was first claimed (None = not claimed)
        self.done_seq = 0
        self.current = HOME
        self.visited = {HOME}
        self.queue = []
        self.state = "SETTLE"
        self.settle_until = 0.0
        self.leg_started = 0.0
        self.mission_started = None
        self.going_home = False
        self.last_log = 0.0
        self.cam_seen = set()      # cells the camera has looked at (on our own map)
        self.heading = 0.0         # yaw we hold, rad
        self.spin = []             # headings still to hold in the current spin
        self.spin_until = 0.0
        self.spun_at = None        # one spin per stop: the map growing DURING a spin must not start another
        self.tof = {}              # side -> (range m, monotonic time)
        self.guard_until = 0.0     # holding a backed-off point until then
        self.guard_target = None
        self.guard_events = []     # monotonic times the guard fired
        # velocity from the flight controller's EKF (map frame), NOT from differencing poses: in run 17 the pose
        # difference read 1.4-1.9 m/s when Gazebo truth never exceeded 1.11 m/s, and fired the guard 4 times for nothing
        self.vel = (0.0, 0.0)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.on_velocity,
                                 qos_profile_sensor_data)
        for side in TOF_SIDES:
            self.create_subscription(LaserScan, f"/airmouse/tof/{side}",
                                     lambda msg, side=side: self.on_tof(side, msg), qos_profile_sensor_data)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/airmouse/grid", self.on_grid, latched)
        self.state_pub = self.create_publisher(String, "/airmouse/explorer", latched)

    def on_grid(self, msg):
        try:
            _, cells = parse_grid(json.loads(msg.data))
        except (ValueError, KeyError) as exc:
            self.get_logger().warn(f"bad grid message ignored: {exc}")
            return
        self.cells = cells
        self.grid_stamp = time.monotonic()
        self.grid_seq += 1

    def on_tof(self, side, msg):
        """Check every ToF reading as it arrives (20 Hz) instead of waiting for the 5 Hz mission tick."""
        ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        now = time.monotonic()
        self.tof[side] = (min(ranges) if ranges else float("inf"), now)
        if self.pose is None or self.phase != "MISSION" or self.mission_started is None:
            return
        heading = self.yaw + TOF_SIDES[side]
        towards = max(0.0, self.vel[0] * math.cos(heading) + self.vel[1] * math.sin(heading))
        trigger = self.tof_stop + towards ** 2 / (2 * self.brake_acc) + towards * self.tof_latency
        rng = self.tof[side][0]
        if rng >= trigger:
            return
        back = min(0.3, trigger - rng + 0.05)
        self.guard_target = (self.pose.x - back * math.cos(heading), self.pose.y - back * math.sin(heading))
        self.go_to(*self.guard_target, yaw=self.heading)       # at once, not on the next tick
        if now > self.guard_until:          # a new event, not the same one still being held
            self.guard_events = [t for t in self.guard_events if now - t < 60.0] + [now]
            self.get_logger().warn(
                f"WALL GUARD: {side} ToF {rng:.2f} m < {trigger:.2f} m at {towards:.2f} m/s towards it - "
                f"backing off {back:.2f} m and re-planning ({len(self.guard_events)} in the last minute)")
        self.guard_until = now + self.guard_hold

    def on_velocity(self, msg):
        self.vel = (msg.twist.linear.x, msg.twist.linear.y)

    def tick_guard(self):
        """While the wall guard holds, fly nothing else; afterwards re-plan from the cell we are really in."""
        if time.monotonic() < self.guard_until:
            self.go_to(*self.guard_target, yaw=self.heading)
            return True
        self.guard_target = None
        if len(self.guard_events) >= self.guard_limit:
            self.abort(f"wall guard fired {len(self.guard_events)} times in a minute - "
                       "position estimate cannot be trusted near walls")
            return True
        self.current = self.nearest_cell()
        self.queue = []
        self.spin = []
        if self.going_home:
            self.head_home()
        else:
            self.state = "SETTLE"
            self.done_since = None
            self.arrival_seq = self.grid_seq
            self.settle_until = time.monotonic() + self.settle_time
            self.leg_started = time.monotonic()
        return True

    def nearest_cell(self):
        """The mapped cell the drone is over right now (falls back to the last cell we passed)."""
        p = self.pose
        cell = (round(p.x / self.cell_size), round(p.y / self.cell_size))
        return cell if cell in self.cells else self.current

    def centre_of(self, cell):
        return cell[0] * self.cell_size, cell[1] * self.cell_size

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps({
            "state": self.state, "current": list(self.current), "visited": len(self.visited),
            "mapped": len(self.cells), "going_home": self.going_home,
            "queue": [list(c) for c in self.queue], "camera_seen": len(self.cam_seen),
            "guard_events": len(self.guard_events),
            "tof": {k: round(v[0], 2) for k, v in self.tof.items()},
            "elapsed": round(time.monotonic() - (self.mission_started or time.monotonic()), 1),
        })))

    def budget_left(self):
        return self.mission_timeout - (time.monotonic() - self.mission_started)

    def time_to_go_home(self):
        """Would the trip home (at WP_SPD, 1.5 s per corner, plus home_margin) eat the rest of the budget?"""
        path = path_home(self.cells, self.current, self.visited, HOME, self.blocked,
                         through_seen=self.strategy == "frontier") or []
        legs, i, at = 0, 0, self.current
        while i < len(path):
            n = straight_run(at, path[i:])
            legs, at, i = legs + 1, path[i + n - 1], i + n
        return self.budget_left() <= len(path) * self.cell_size / self.wp_speed + 1.5 * legs + self.home_margin

    def camera_view(self, cell, heading=None):
        """Mark what the camera sees from `cell`: a wedge along `heading`, or everything (a full spin)."""
        sector = None if heading is None else (heading, self.cam_half_fov)
        self.cam_seen |= line_of_sight(self.cells, cell, self.cam_reach, sector)

    def choose(self, here):
        if self.strategy == "frontier":
            return frontier_step(self.cells, here, self.visited, self.blocked, centre_reach=self.centre_reach,
                                 cam_seen=self.cam_seen if self.camera == "utility" else None,
                                 cam_reach=self.cam_reach, edge_cost=self.edge_cost)
        return next_step(self.cells, here, self.visited, self.blocked, max_leg=self.max_leg)

    # --- the mission -----------------------------------------------------------
    def mission_tick(self):
        if self.mission_started is None:
            self.mission_started = time.monotonic()
            self.leg_started = time.monotonic()
            self.settle_until = time.monotonic() + self.settle_time
            self.get_logger().info("exploring - no prior knowledge of the maze")

        self.publish_state()
        if time.monotonic() - self.last_log > 5.0:
            self.last_log = time.monotonic()
            self.get_logger().info(
                f"{self.state} at {self.current}: {len(self.visited)} visited, "
                f"{len(self.cells)} cells mapped, {self.budget_left():.0f} s left")

        if self.guard_target is not None and self.tick_guard():
            return
        if self.state == "SETTLE":
            self.tick_settle()
        elif self.state == "SPIN":
            self.tick_spin()
        elif self.state == "EXIT":
            self.tick_exit()
        else:
            self.tick_travel()

    def tick_settle(self):
        """Hover in the middle of this cell until the map of it settles, then choose the next cell."""
        self.go_to(*self.centre_of(self.current), yaw=self.heading)
        if time.monotonic() < self.settle_until:
            return
        # decide on a map built since we arrived, not on the one we flew in with
        if not self.cells or self.grid_seq - self.arrival_seq < self.min_grid_updates:
            if time.monotonic() - self.leg_started > self.stuck_timeout:
                self.abort("no grid updates while settling - is grid_mapper running?")
            return

        if not self.going_home and self.time_to_go_home():
            self.get_logger().warn(f"{self.budget_left():.0f} s left - only just enough to fly home: heading home")
            self.head_home()
            return

        # look around before deciding, if a spin would show the camera anything new from here
        if (self.camera == "utility" and not self.omni and self.spun_at != self.current
                and camera_gain(self.cells, self.current, self.cam_seen, self.cam_reach)):
            self.spin = [self.heading + k * math.pi / 2 for k in (1, 2, 3, 4)]
            self.spin_until = time.monotonic() + self.spin_hold
            self.state = "SPIN"
            return

        kind, path = self.choose(self.current)

        if kind == "done":
            if not self.confirm_done():
                return
            self.head_home()
            return

        self.done_since = None
        self.queue = list(path)
        self.route_kind = kind
        self.state = "TRAVEL"
        self.leg_started = time.monotonic()
        self.get_logger().info(f"{kind}: {self.current} -> {self.queue}")

    def tick_spin(self):
        """Yaw through four 90 deg headings in place so the camera looks all round, then decide."""
        self.go_to(*self.centre_of(self.current), yaw=self.spin[0])
        if time.monotonic() < self.spin_until:
            return
        self.camera_view(self.current, self.spin[0])
        self.heading = self.spin.pop(0)
        if self.spin:
            self.spin_until = time.monotonic() + self.spin_hold
            return
        self.camera_view(self.current)       # the whole circle has been seen
        self.spun_at = self.current
        self.state = "SETTLE"
        self.settle_until = time.monotonic()
        self.leg_started = time.monotonic()

    def confirm_done(self):
        """Is "nowhere left to go" real, or is the map just not built yet?

        Keep hovering and re-checking for done_grace seconds AND done_updates fresh grids. Any exit
        that appears in that time cancels the claim. Returns True only once it still holds.
        """
        if self.done_since is None:
            self.done_since = time.monotonic()
            self.done_seq = self.grid_seq
            self.get_logger().warn(
                f"no way on from {self.current} after {len(self.visited)} cells - "
                f"waiting {self.done_grace:.0f} s to be sure before ending the mission")
            return False
        waited = time.monotonic() - self.done_since
        fresh = self.grid_seq - self.done_seq
        # "done" before we have moved at all is almost certainly a map that is not built yet (runs 7 and 21),
        # so be far more patient in the takeoff cell than anywhere else
        grace = self.done_grace if len(self.visited) > 1 else max(self.done_grace, 30.0)
        if waited < grace or fresh < self.done_updates:
            return False
        self.get_logger().info(f"still no way on after {waited:.0f} s and {fresh} map updates")
        return True

    def tick_travel(self):
        """Fly the current straight leg: to the far end of the run of collinear cells at the queue's head."""
        if not self.queue:            # nothing to fly to: look again from here
            self.state = "SETTLE"
            self.arrival_seq = self.grid_seq
            self.settle_until = time.monotonic()
            return
        # the map keeps changing while we fly: if the next hop is no longer open, stop and re-plan
        if not passable(self.cells, self.current, self.queue[0], self.blocked):
            self.get_logger().warn(f"path {self.current} -> {self.queue[0]} is no longer open on the "
                                   f"latest map - hovering to re-plan")
            self.queue = []
            if self.going_home:
                self.head_home()
            else:
                self.state = "SETTLE"
                self.done_since = None
                self.arrival_seq = self.grid_seq
                self.settle_until = time.monotonic()
            return
        # Retarget instead of stopping: a frontier we are flying to is often resolved by what the LiDAR sees
        # on the way in (the cell beyond it becomes mapped). Then the trip is pointless - drop it and plan the
        # next frontier from where we are, without hovering. Only when the target is RESOLVED, not merely
        # because something else is now nearer, so two similar frontiers cannot flip-flop.
        if (self.chain and self.strategy == "frontier" and not self.going_home
                and self.route_kind == "frontier" and self.grid_seq != self.chain_seq):
            self.chain_seq = self.grid_seq               # once per new grid, not every tick
            final = self.queue[-1]
            # a target that has dropped out of the latest map counts as resolved (is_frontier needs it in cells)
            if final not in self.cells or not is_frontier(self.cells, final, self.blocked):
                here = self.nearest_cell()     # plan from where the drone IS, not the last cell it passed
                kind, path = self.choose(here)
                if path:
                    self.get_logger().info(f"retarget from {here}: {final} was resolved on the way -> {kind} {path}")
                    self.current, self.queue, self.route_kind = here, list(path), kind
        run = straight_run(self.current, self.queue)
        target = self.queue[run - 1]
        x, y = self.centre_of(target)
        if self.camera == "utility" and not self.omni:      # nose (and camera) along the leg
            self.heading = math.atan2(target[1] - self.current[1], target[0] - self.current[0])
        self.go_to(x, y, yaw=self.heading)
        if time.monotonic() - self.leg_started > self.stuck_timeout + 10.0 * (run - 1):
            p = self.pose
            self.abort(f"stuck: cell {target} at ({x:.1f}, {y:.1f}) not reached in "
                       f"{self.stuck_timeout + 10.0 * (run - 1):.0f} s, drone at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
            return
        # Progress along the leg. Take the FURTHEST cell of this leg we are already at: after a re-plan the drone
        # can be past the cells at the start of the new queue (run 22 sat on its target for 85 s because the cells
        # before it were never "passed" in order). A cell in the middle of a leg needs pass_tol, a corner
        # corner_tol, and the end of the whole route reached_tol.
        for i in range(run - 1, -1, -1):
            cx, cy = self.centre_of(self.queue[i])
            if i == len(self.queue) - 1:
                tol = self.reached_tol
            else:
                tol = self.corner_tol if i == run - 1 else self.pass_tol
            if self.distance_to(cx, cy) < tol:
                arrived = self.queue[i]
                new_cell = arrived not in self.visited
                for c in self.queue[:i + 1]:
                    self.camera_view(c, None if self.omni else self.heading)
                self.visited.update(self.queue[:i + 1])
                self.current = arrived
                del self.queue[:i + 1]
                self.leg_started = time.monotonic()
                if not self.queue:
                    if self.going_home:
                        if self.current == HOME and self.auto_exit and self.entrance_side:
                            self.start_exit()
                        else:
                            self.finish(f"home again: {len(self.visited)} cells visited")
                        return
                    # settle in a cell we have not mapped from the inside yet; fly straight through
                    # the ones we already know
                    self.state = "SETTLE"
                    self.done_since = None
                    self.arrival_seq = self.grid_seq
                    self.settle_until = time.monotonic() + (self.settle_time if new_cell else 0.0)
                return

    def start_exit(self):
        """Cross the entrance opening to the outside of the arena (50 pts) and land."""
        vecs = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, math.pi),
                "+y": (0.0, 1.0, math.pi / 2), "-y": (0.0, -1.0, -math.pi / 2)}
        vec = vecs.get(self.entrance_side, (-1.0, 0.0, math.pi))
        target_x = HOME[0] * self.cell_size + vec[0] * self.exit_distance
        target_y = HOME[1] * self.cell_size + vec[1] * self.exit_distance
        self.exit_target = (target_x, target_y)
        self.exit_heading = vec[2]
        self.state = "EXIT"
        self.leg_started = time.monotonic()
        self.get_logger().info(
            f"AUTONOMOUS EXIT (50 pts): flying out via {self.entrance_side} to ({target_x:.2f}, {target_y:.2f}) "
            f"at yaw {math.degrees(self.exit_heading):.0f} deg")

    def tick_exit(self):
        """Command position setpoint outside the arena until reached, then land."""
        tx, ty = self.exit_target
        self.go_to(tx, ty, yaw=self.exit_heading)
        if self.distance_to(tx, ty) < self.reached_tol:
            self.finish(f"autonomous exit complete via {self.entrance_side} ({self.exit_distance:.1f} m outside) - landing")
            return
        if time.monotonic() - self.leg_started > self.stuck_timeout:
            self.finish(f"exit timeout ({self.stuck_timeout:.0f} s) - landing outside near entrance")

    def head_home(self):
        """Every reachable cell is done (or the clock ran out): fly back the way we came and land."""
        self.going_home = True
        if self.current == HOME:
            if self.auto_exit and self.entrance_side:
                self.start_exit()
            else:
                self.finish(f"exploration complete: {len(self.visited)} cells visited")
            return
        path = path_home(self.cells, self.current, self.visited, HOME, self.blocked,
                        through_seen=self.strategy == "frontier")
        if not path:
            self.abort(f"no known route home from {self.current}")
            return
        self.queue = list(path)
        self.state = "TRAVEL"
        self.leg_started = time.monotonic()
        self.get_logger().info(f"exploration done ({len(self.visited)} cells) - home via {self.queue}")

    def on_landed(self):
        self.get_logger().info(f"visited {len(self.visited)} cells: {sorted(self.visited)}")
        if len(self.visited) <= 1:
            self.get_logger().error(
                "explored NOTHING - the drone never left the takeoff cell. This is a failure even "
                "though it landed safely: check that the grid had open sides by the time we decided "
                "(settle_time, min_grid_updates), not that the search is broken.")
            self.aborted = True


def main():
    rclpy.init()
    node = Explorer()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code or 0
    except KeyboardInterrupt:
        code = 1
    except ExternalShutdownException:
        # killed from outside (the test harness timeout): report it, do not dump a traceback
        node.get_logger().error("shut down externally before the mission finished")
        code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
