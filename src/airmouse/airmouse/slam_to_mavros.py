"""Feed the SLAM position to ArduPilot, replacing GPS.

Reads the map -> base_link transform that Cartographer publishes and sends it to
/mavros/vision_pose/pose (MAVROS turns it into VISION_POSITION_ESTIMATE for ArduPilot's EKF).
Also sets the EKF origin once, which ArduPilot needs before it will use position without GPS.

Parameters:
    map_frame   (str, default "map")
    base_frame  (str, default "base_link")
    rate_hz     (float, default 30.0)
"""
import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import StreamRate
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# Any fixed point works indoors; ArduPilot only needs *an* origin. Same as the SITL home.
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = -35.363262, 149.165237, 584.0


class SlamToMavros(Node):
    def __init__(self):
        super().__init__("slam_to_mavros")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        rate = self.declare_parameter("rate_hz", 30.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "/mavros/vision_pose/pose", 10)
        self.origin_pub = self.create_publisher(GeoPointStamped, "/mavros/global_position/set_gp_origin", 10)

        # SLAM runs on simulation time; MAVROS/ArduPilot want wall-clock stamps
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_stamp = None
        self.origin_sends = 0
        self.sent = 0

        # ArduPilot sends its IMU at 10 Hz by default - about the same as the LiDAR. Cartographer
        # needs IMU samples either side of every scan, so ask for RAW_SENSORS (1) and EXTRA1 (10,
        # attitude) much faster. Lower imu_stream_hz on the real drone if the link cannot take it.
        self.imu_stream_hz = self.declare_parameter("imu_stream_hz", 50).value
        self.streams_todo = [1, 10]
        self.stream_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")

        self.create_timer(1.0 / rate, self.send_pose)
        self.create_timer(1.0, self.send_origin)
        self.stream_timer = self.create_timer(2.0, self.request_streams)

    def request_streams(self):
        """Raise the autopilot's IMU rate, retrying until MAVROS is up.

        A request sent before the service is ready is silently lost (see NOTES), so check first
        and try again on the next tick rather than assuming it landed.
        """
        if not self.streams_todo:
            self.stream_timer.cancel()
            return
        if not self.stream_rate.service_is_ready():
            return
        stream_id = self.streams_todo.pop(0)
        self.stream_rate.call_async(StreamRate.Request(
            stream_id=stream_id, message_rate=self.imu_stream_hz, on_off=True))
        self.get_logger().info(f"asked ArduPilot for stream {stream_id} at {self.imu_stream_hz} Hz")

    def send_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return  # SLAM not running yet
        if tf.header.stamp == self.last_stamp:
            return  # nothing new; don't feed ArduPilot stale positions
        self.last_stamp = tf.header.stamp

        msg = PoseStamped()
        msg.header.stamp = self.wall_clock.now().to_msg()
        msg.header.frame_id = self.map_frame
        t, r = tf.transform.translation, tf.transform.rotation
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t.x, t.y, t.z
        msg.pose.orientation = r
        self.pose_pub.publish(msg)

        self.sent += 1
        if self.sent == 1:
            self.get_logger().info("SLAM pose flowing to ArduPilot")

    def send_origin(self):
        # Send a few times in case MAVROS/ArduPilot wasn't ready for the first one
        if self.origin_sends >= 10:
            return
        msg = GeoPointStamped()
        msg.header.stamp = self.wall_clock.now().to_msg()
        msg.position.latitude, msg.position.longitude, msg.position.altitude = ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT
        self.origin_pub.publish(msg)
        self.origin_sends += 1


def main():
    rclpy.init()
    rclpy.spin(SlamToMavros())


if __name__ == "__main__":
    main()
