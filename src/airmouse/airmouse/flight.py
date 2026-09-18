"""Shared flight plumbing: arm, take off, fly to a point, land - with every retry fly_square learned.

The hard-won parts (see README "Gotchas" and NOTES):
  - ArduPilot streams no position until asked, so we request the streams before anything else.
  - Arming is refused for the first 30-60 s while the EKF settles; keep retrying, do not give up.
  - A takeoff can be rejected and the drone then auto-disarms, so check the reply instead of waiting.
  - A service request sent before the service is ready is silently lost - check service_is_ready()
    and re-send after 5 s without a reply, or the node waits for a message that never comes.

A subclass implements mission_tick(), which runs once the drone is hovering at altitude, and calls
finish() or abort() when it is done. fly_square is deliberately left alone as the known-good
reference node; this is the same state machine with a mission hook in place of a fixed route.
"""
import math
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class CopterNode(Node):
    """WAIT_FOR_FCU -> ARM_AND_TAKEOFF -> CLIMBING -> MISSION -> LANDING -> DONE."""

    def __init__(self, name, altitude=1.2, tick_hz=5.0):
        super().__init__(name)
        self.altitude = altitude
        self.fcu_state = State()
        self.pose = None
        self.phase = "WAIT_FOR_FCU"
        self.aborted = False
        self.landing_started = None
        self.landing_timeout = 60.0
        self.takeoff_attempts = 0
        self.takeoff_reply = None
        self.pending = None
        self.pending_since = 0.0
        self.started = time.monotonic()
        self.last_wait_log = time.monotonic()

        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self.on_pose, qos_profile_sensor_data)
        self.target_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.stream_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
        self.create_timer(1.0 / tick_hz, self.tick)

    # --- things a mission uses -------------------------------------------------
    def go_to(self, x, y, z=None, yaw=None):
        """Hold a position target. ArduPilot needs this re-sent continuously, not once.

        yaw (rad, map frame) points the nose - and the camera. None = 0, the old fixed heading."""
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(self.altitude if z is None else z)
        if yaw is None:
            target.pose.orientation.w = 1.0
        else:
            target.pose.orientation.z = math.sin(yaw / 2.0)
            target.pose.orientation.w = math.cos(yaw / 2.0)
        self.target_pub.publish(target)

    def distance_to(self, x, y, z=None):
        z = self.altitude if z is None else z
        return math.dist((self.pose.x, self.pose.y, self.pose.z), (float(x), float(y), float(z)))

    def mission_tick(self):
        raise NotImplementedError

    def finish(self, message="mission complete"):
        self.get_logger().info(f"{message} - landing")
        self.switch("LANDING")

    def abort(self, reason):
        """Give up safely: land where we are and exit with an error code."""
        self.get_logger().error(f"ABORT - {reason}. Landing.")
        self.aborted = True
        if self.fcu_state.armed:
            self.switch("LANDING")
        else:
            raise SystemExit(1)

    def on_landed(self):
        """Hook for a subclass to print a summary. Called once, after disarm."""

    # --- machinery -------------------------------------------------------------
    def on_state(self, msg):
        self.fcu_state = msg

    def on_pose(self, msg):
        self.pose = msg.pose.position

    def switch(self, phase):
        self.get_logger().info(f"--> {phase}")
        self.phase = phase

    def call(self, client, request):
        """Send a MAVROS request if the service is up; returns the future (None if not ready)."""
        if not client.service_is_ready():
            return None
        self.pending = client.call_async(request)
        self.pending_since = time.monotonic()
        return self.pending

    def tick(self):
        # Never wait forever on a reply: a request sent before MAVROS was ready is silently lost
        if self.pending is not None and not self.pending.done():
            if time.monotonic() - self.pending_since < 5.0:
                return
            self.get_logger().warn("no reply from MAVROS within 5 s - retrying")
            self.pending.cancel()
        self.pending = None

        handler = getattr(self, f"phase_{self.phase.lower()}", None)
        if handler:
            handler()

    def phase_wait_for_fcu(self):
        if time.monotonic() - self.started > 90:
            self.abort(f"no drone data after 90 s (connected={self.fcu_state.connected}, "
                       f"position={'yes' if self.pose else 'no'}) - is MAVROS running?")
            return
        if time.monotonic() - self.last_wait_log > 10:
            self.last_wait_log = time.monotonic()
            self.get_logger().info(f"waiting for drone: connected={self.fcu_state.connected}, "
                                   f"position={'yes' if self.pose else 'no'}")
        if self.fcu_state.connected and self.pose is None:
            self.call(self.stream_rate, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
        elif self.pose is not None:
            self.switch("ARM_AND_TAKEOFF")

    def phase_arm_and_takeoff(self):
        if self.fcu_state.mode != "GUIDED":
            self.call(self.set_mode, SetMode.Request(custom_mode="GUIDED"))
        elif not self.fcu_state.armed:
            self.call(self.arm, CommandBool.Request(value=True))
        else:
            self.takeoff_reply = self.call(self.takeoff, CommandTOL.Request(altitude=self.altitude))
            if self.takeoff_reply is not None:
                self.switch("CLIMBING")

    def phase_climbing(self):
        reply = self.takeoff_reply.result()
        if reply is None or not reply.success or not self.fcu_state.armed:
            self.takeoff_attempts += 1
            if self.takeoff_attempts >= 5:
                self.abort("takeoff rejected 5 times")
                return
            self.get_logger().warn("Takeoff rejected or drone disarmed - retrying")
            self.switch("ARM_AND_TAKEOFF")
        elif self.pose.z > self.altitude * 0.9:
            self.switch("MISSION")

    def phase_mission(self):
        self.mission_tick()

    def phase_landing(self):
        # Never wait forever for the disarm (run 13 sat in LANDING for 12 minutes): after landing_timeout
        # give up and exit with an error so the operator / test harness sees it.
        now = time.monotonic()
        if self.landing_started is None:
            self.landing_started = now
        elif self.fcu_state.armed and now - self.landing_started > self.landing_timeout:
            self.get_logger().error(f"still armed {self.landing_timeout:.0f} s after landing was requested - giving up")
            raise SystemExit(1)
        if self.fcu_state.armed and self.fcu_state.mode != "LAND":
            self.call(self.set_mode, SetMode.Request(custom_mode="LAND"))
        elif not self.fcu_state.armed:
            self.switch("DONE")
            self.on_landed()
            if self.aborted:
                self.get_logger().error("Landed after ABORT.")
                raise SystemExit(1)
            self.get_logger().info("Landed and disarmed.")
            raise SystemExit(0)
