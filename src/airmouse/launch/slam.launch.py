"""GPS-free navigation stack: Cartographer SLAM on /scan + map + pose forwarding to ArduPilot + grid mapper.

Run after sim_up.sh --nogps (or on the real drone, after the LiDAR driver and MAVROS):
    ros2 launch airmouse slam.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("airmouse"), "config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = [{"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true in Gazebo, false on the real drone"),
        DeclareLaunchArgument("imu_topic", default_value="/airmouse/imu",
                              description="IMU for Cartographer: the sim-time Gazebo bridge in simulation; on the "
                                          "real drone /mavros/imu/data (no sim/wall clock split there)"),

        # Where the LiDAR sits on the drone: 26 cm above the body centre
        Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_tf",
             arguments=["--z", "0.26", "--frame-id", "base_link", "--child-frame-id", "lidar_link"],
             parameters=params),

        # The Gazebo IMU sensor is mounted upside down (roll 180 deg, gravity reads -z), frame "imu_link".
        # On the real drone replace this with the flight controller's real mounting orientation.
        Node(package="tf2_ros", executable="static_transform_publisher", name="imu_tf",
             arguments=["--roll", "3.14159265", "--frame-id", "base_link", "--child-frame-id", "imu_link"],
             parameters=params),

        Node(package="cartographer_ros", executable="cartographer_node", name="cartographer",
             arguments=["-configuration_directory", config_dir,
                        "-configuration_basename", "airmouse_2d.lua"],
             # Cartographer listens on "imu". In simulation that is the Gazebo IMU bridged by
             # sim_up.sh, on SIM time. Do NOT use /mavros/imu/data there: MAVROS stamps
             # wall-clock time and mixing the clocks stalls SLAM (see config/airmouse_2d.lua).
             remappings=[("imu", LaunchConfiguration("imu_topic"))],
             parameters=params),

        # Turns Cartographer's submaps into a normal /map (OccupancyGrid), 5 cm per pixel
        Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node", name="map_builder",
             arguments=["-resolution", "0.05", "-publish_period_sec", "0.5"],
             parameters=params),

        Node(package="airmouse", executable="slam_to_mavros", name="slam_to_mavros",
             parameters=params),

        # /map -> 1 m cells with wall/open sides (/airmouse/grid, /airmouse/grid_markers)
        Node(package="airmouse", executable="grid_mapper", name="grid_mapper",
             parameters=params),

        # Autonomous survivor localisation and tagging (/airmouse/survivors)
        Node(package="airmouse", executable="survivor_tagger", name="survivor_tagger",
             parameters=params),
    ])
