-- Cartographer 2D SLAM for AirMouse: one 360 deg LiDAR, no wheel odometry, no GPS.
-- Publishes map -> odom -> base_link; slam_to_mavros forwards map -> base_link to ArduPilot.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,  -- ignore roll/pitch: a drone tilts to move
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3,  -- 50 Hz TF is plenty (default 200 Hz cost every listener CPU)
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- IMU ON (2026-09-18). It stops the scan matcher being the only thing tracking the drone (it slips
-- a whole cell in identical 1 m corridors). In simulation it comes from the Gazebo imu_sensor bridged
-- by sim_up.sh to /airmouse/imu, on SIM time. Do NOT wire /mavros/imu/data in simulation: MAVROS
-- stamps WALL-CLOCK time, Cartographer stalls on "Queue waiting for data: (0, scan)", never
-- publishes map -> base_link and the flight aborts with position=no (run 10).
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- With no odometry, matching every scan against the map does ALL the motion tracking.
-- Search +-30 cm per scan (enough for ~3 m/s at 10 Hz) and don't penalise movement:
-- the default weights assume wheel odometry predicts the motion, which we don't have,
-- and made SLAM lag far behind the real drone (tested 2026-09-18: 2.9 m error over 4 m).
-- OFF (2026-09-18). The brute-force matcher searches +-30 cm / +-20 deg per scan and can jump to a wrong
-- look-alike match; with the IMU now supplying motion prediction, Ceres alone tracks better. Offline replays of
-- two recorded flights (sim/replay_out, make_slam_variants.py "ceres_only"): run-20 bag (live failure) mean error
-- 0.54 m -> 0.13 m, max 1.45 -> 0.43 m; run-17 bag (good flight) 0.10 m -> 0.06 m. Do NOT also switch the IMU off
-- ("ceres_noimu": 0.9-2.5 m) or enlarge the submaps ("ceres_sub70": fine on one bag, 15 m jump on the other).
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- Without the correlative matcher the pose is rock steady, so a HOVERING drone looks "not moving" to the motion filter
-- and almost every scan is dropped (default: one per 5 s) - the map barely grows until the drone flies (run 21: explorer
-- saw an empty map at takeoff and called the mission done after one cell). Insert a scan at least every 0.5 s.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- variant: base


return options
