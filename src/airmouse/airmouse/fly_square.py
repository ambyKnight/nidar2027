"""Step 2: fly a 2 m square from ROS 2, through MAVROS, with ArduPilot doing the flying.

Needs ArduPilot SITL and MAVROS running (see ~/airmouse_ws/README.md).

The node is a tiny state machine, the same shape the real mission manager will have:
    WAIT_FOR_FCU -> ARM_AND_TAKEOFF -> CLIMBING -> FLY_CORNERS -> LANDING -> DONE

Parameters (optional):
    route     flat list of x,y waypoints in metres, e.g. "[0.0, -2.0, 0.0, 0.0]"  (default: the 2 m square)
    altitude  metres (default 1.5)
    reached_tol    metres; how close counts as "arrived" (default 0.25)
    stuck_timeout  seconds without reaching the next waypoint before giving up and landing (default 90)
Exits with code 1 if the flight was aborted (stuck, or takeoff rejected too many times).
Example:  ros2 run airmouse fly_square --ros-args -p route:="[0.0, -4.0, 0.0, 0.0]" -p altitude:=1.2
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ALTITUDE = 1.5      # metres above takeoff point
SIDE = 2.0          # length of the square's side, metres
REACHED_TOL = 0.25  # how close counts as "arrived", metres

# Corners in the local ENU frame MAVROS uses: x = east, y = north, z = up
CORNERS = [(SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE), (0.0, 0.0)]


class FlySquare(Node):
    def __init__(self):
        super().__init__("fly_square")

        default_route = [v for corner in CORNERS for v in corner]
        route = self.declare_parameter("route", default_route).value
        self.corners = list(zip(route[0::2], route[1::2]))
        self.altitude = self.declare_parameter("altitude", ALTITUDE).value
        # how close counts as "arrived"; smaller = less corner cutting in narrow corridors
        self.reached_tol = self.declare_parameter("reached_tol", REACHED_TOL).value
        # never hover forever: if the next waypoint isn't reached in this time, land
        self.stuck_timeout = self.declare_parameter("stuck_timeout", 90.0).value

        self.last_progress = time.monotonic()
        self.takeoff_attempts = 0
        self.aborted = False

        self.fcu_state = State()
        self.pose = None
        self.phase = "WAIT_FOR_FCU"
        self.corner = 0
        self.pending = None  # a service call still waiting for its reply
        self.pending_since = 0.0
        self.last_wait_log = time.monotonic()
        self.takeoff_reply = None

        # LISTEN
        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self.on_pose, qos_profile_sensor_data)
        # TALK
        self.target_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.stream_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
        # DECIDE, 5 times a second
        self.create_timer(0.2, self.tick)

    def on_state(self, msg):
        self.fcu_state = msg

    def on_pose(self, msg):
        self.pose = msg.pose.position

    def go_to(self, x, y, z):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)

    def distance_to(self, x, y, z):
        return math.dist((self.pose.x, self.pose.y, self.pose.z), (x, y, z))

    def switch(self, phase):
        self.get_logger().info(f"--> {phase}")
        self.phase = phase

    def tick(self):
        # Don't fire a new command while the previous one is still waiting for a reply
        # Don't fire a new command while the previous one is still waiting for a reply -
        # but never wait forever: a request sent before MAVROS was ready is silently lost
        if self.pending is not None and not self.pending.done():
            if time.monotonic() - self.pending_since < 5.0:
                return
            self.get_logger().warn("no reply from MAVROS within 5 s - retrying")
            self.pending.cancel()
        self.pending = None

        if self.phase == "WAIT_FOR_FCU":
            waited = time.monotonic() - self.last_progress
            if waited > 90:
                self.abort(f"no drone data after 90 s (connected={self.fcu_state.connected}, "
                           f"position={'yes' if self.pose else 'no'}) - is MAVROS running?")
                return
            if time.monotonic() - self.last_wait_log > 10:
                self.last_wait_log = time.monotonic()
                self.get_logger().info(f"waiting for drone: connected={self.fcu_state.connected}, "
                                       f"position={'yes' if self.pose else 'no'}")
            if self.fcu_state.connected and self.pose is None:
                # ArduPilot sends no position data until asked, so request all streams at 10 Hz
                self.call(self.stream_rate, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
            elif self.pose is not None:
                self.switch("ARM_AND_TAKEOFF")

        elif self.phase == "ARM_AND_TAKEOFF":
            # Retry until ArduPilot accepts: it refuses to arm until its EKF has settled
            if self.fcu_state.mode != "GUIDED":
                self.call(self.set_mode, SetMode.Request(custom_mode="GUIDED"))
            elif not self.fcu_state.armed:
                self.call(self.arm, CommandBool.Request(value=True))
            else:
                self.takeoff_reply = self.call(self.takeoff, CommandTOL.Request(altitude=self.altitude))
                if self.takeoff_reply is not None:
                    self.switch("CLIMBING")

        elif self.phase == "CLIMBING":
            # ArduPilot can reject a takeoff (e.g. sent a moment too early) and then auto-disarms,
            # so check its answer instead of waiting forever
            reply = self.takeoff_reply.result()
            if reply is None or not reply.success or not self.fcu_state.armed:
                self.takeoff_attempts += 1
                if self.takeoff_attempts >= 5:
                    self.abort("takeoff rejected 5 times")
                    return
                self.get_logger().warn("Takeoff rejected or drone disarmed - retrying")
                self.switch("ARM_AND_TAKEOFF")
            elif self.pose.z > self.altitude * 0.9:
                self.last_progress = time.monotonic()
                self.switch("FLY_CORNERS")

        elif self.phase == "FLY_CORNERS":
            x, y = self.corners[self.corner]
            self.go_to(x, y, self.altitude)
            if self.distance_to(x, y, self.altitude) < self.reached_tol:
                self.get_logger().info(f"Reached corner {self.corner + 1}/{len(self.corners)}: ({x:.1f}, {y:.1f})")
                self.corner += 1
                self.last_progress = time.monotonic()
                if self.corner == len(self.corners):
                    self.switch("LANDING")
            elif time.monotonic() - self.last_progress > self.stuck_timeout:
                p = self.pose
                self.abort(f"stuck: waypoint {self.corner + 1} ({x:.1f}, {y:.1f}) not reached in "
                           f"{self.stuck_timeout:.0f} s, drone at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

        elif self.phase == "LANDING":
            if self.fcu_state.armed and self.fcu_state.mode != "LAND":
                self.call(self.set_mode, SetMode.Request(custom_mode="LAND"))  # (re)send until it takes
            elif not self.fcu_state.armed:
                self.switch("DONE")
                if self.aborted:
                    self.get_logger().error("Landed after ABORT.")
                    raise SystemExit(1)
                self.get_logger().info("Landed and disarmed. Square complete!")
                raise SystemExit(0)

    def call(self, client, request):
        """Send a MAVROS service request if the service is up; returns the future (or None if not ready)."""
        if not client.service_is_ready():
            return None  # try again next tick
        self.pending = client.call_async(request)
        self.pending_since = time.monotonic()
        return self.pending

    def abort(self, reason):
        """Give up safely: land where we are and exit with an error code."""
        self.get_logger().error(f"ABORT - {reason}. Landing.")
        self.aborted = True
        if self.fcu_state.armed:
            self.switch("LANDING")  # LANDING sends the LAND command
        else:
            raise SystemExit(1)


def main():
    rclpy.init()
    node = FlySquare()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code or 0
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
