#!/usr/bin/env python3
"""Create models/iris_lidar: ArduPilot's iris_with_ardupilot drone plus a 360 deg 2D LiDAR on top and a
ToF range sensor on each of the 4 edges, built on models/iris_standoffs_5in (Iris motors and flight physics,
but the collision size and mass of our real drone: 330 mm tip to tip including guards, 1.3 kg).

The LiDAR mimics an LDRobot LD19: 10 Hz, 450 points per turn, 0.05-12 m range, ~1 cm noise.
It publishes on the Gazebo topic `scan` (bridged to the ROS topic /scan by sim_up.sh).
The ToF sensors mimic a VL53L1X: 27 deg field of view, 0.04-4 m, 20 Hz, on Gazebo topics tof_front/back/left/right
(bridged to /airmouse/tof/<side>; the explorer's wall guard uses them).
The optical flow sensor is simulated by ArduPilot SITL itself (sim/params/optflow.parm), not by Gazebo.

Usage:  python3 make_iris_lidar.py
"""
from pathlib import Path

SRC = Path.home() / "ardupilot_gazebo/models/iris_with_ardupilot/model.sdf"
OUT = Path(__file__).parent / "models/iris_lidar"

LIDAR_Z = 0.26  # metres above the model origin: just above the props (~0.22), so they don't block the scan

# The real drone. The sim keeps the Iris motors and flight physics (ArduPilot's gains are tuned for them), but
# collides as our frame and weighs what it weighs.
DIAGONAL = 0.33                    # m, tip to tip INCLUDING prop guards (a square frame's diagonal)
FOOTPRINT = DIAGONAL / 2 ** 0.5    # m, side of the square collision box: 0.233
TOTAL_MASS = 1.3                   # kg, all up
LIDAR_MASS = 0.045
TOF_MASS = 0.005
# ToF sensors on the 4 edges: name -> yaw of the sensor relative to the nose (map frame +x at yaw 0)
TOF = {"front": 0.0, "left": 1.5708, "back": 3.14159, "right": -1.5708}
TOF_Z = 0.05       # m above the model origin: on the frame, under the LiDAR

TOF_TEMPLATE = """
    <!-- ToF range sensor ({name}), VL53L1X-like -->
    <link name="tof_{name}_link">
      <pose>{x:.4f} {y:.4f} {z} 0 0 {yaw}</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="tof_{name}" type="gpu_lidar">
        <gz_frame_id>tof_{name}_link</gz_frame_id>
        <topic>tof_{name}</topic>
        <update_rate>20</update_rate>
        <always_on>true</always_on>
        <lidar>
          <scan>
            <horizontal><samples>9</samples><resolution>1</resolution>
              <min_angle>-0.2356</min_angle><max_angle>0.2356</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>0.04</min><max>4.0</max><resolution>0.001</resolution></range>
          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="tof_{name}_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>tof_{name}_link</child>
    </joint>
"""


def tof_sensors():
    """One ToF on each edge, mounted at the edge of the collision box so it reads distance-to-wall directly."""
    import math
    edge = FOOTPRINT / 2
    return "".join(TOF_TEMPLATE.format(name=n, x=edge * math.cos(yaw), y=edge * math.sin(yaw), z=TOF_Z,
                                       yaw=yaw, mass=TOF_MASS) for n, yaw in TOF.items())


LIDAR = f"""
    <!-- ===== AirMouse: 360 deg 2D LiDAR (LD19-like) ===== -->
    <link name="lidar_link">
      <pose>0 0 {LIDAR_Z} 0 0 0</pose>
      <inertial>
        <mass>{LIDAR_MASS}</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="lidar_visual">
        <geometry><cylinder><radius>0.025</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>lidar_link</gz_frame_id>
        <topic>scan</topic>
        <update_rate>10</update_rate>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>450</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.05</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>lidar_link</child>
    </joint>
    <!-- ===== end AirMouse LiDAR ===== -->
{{TOF_SENSORS}}

    <!-- Ground truth for testing only: publish the model's true pose on /model/iris_lidar/pose
         (bridged to ROS by sim_up.sh) so we can measure SLAM error. The real drone has no such thing. -->
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_model_pose>true</publish_model_pose>
      <publish_link_pose>false</publish_link_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>20</update_frequency>
    </plugin>

"""

CONFIG = """<?xml version="1.0"?>
<model>
  <name>Iris with LiDAR (AirMouse)</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>iris_with_ardupilot plus a 360 deg 2D LiDAR. Generated by make_iris_lidar.py.</description>
</model>
"""


FRAME_SRC = Path.home() / "ardupilot_gazebo/models/iris_with_standoffs/model.sdf"
FRAME_OUT = Path(__file__).parent / "models/iris_standoffs_5in"

FRAME_CONFIG = """<?xml version="1.0"?>
<model>
  <name>Iris frame, AirMouse collision footprint and mass</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>iris_with_standoffs with the Iris motors, but the collision box of our drone (330 mm tip to
    tip with guards) and its 1.3 kg all-up mass. Generated by make_iris_lidar.py.</description>
</model>
"""


def make_small_frame():
    """Copy of the Iris frame with the Iris motors (so ArduPilot's tuning still fits), but it COLLIDES like
    our drone and weighs 1.3 kg. Otherwise the 64 cm Iris has ~15 cm clearance per side in a 1 m corridor and
    crashes where the real drone (~38 cm clearance, 33 cm when turned 45 deg) would not.

    Mass: the body link takes whatever the other links leave of TOTAL_MASS; its inertia is scaled by the same
    ratio (not by size - the Iris rate gains would oscillate on a much smaller inertia). Thrust is still the
    Iris's, so the sim has more thrust to spare than the real drone: do not read hover throttle off it."""
    import re
    sdf = FRAME_SRC.read_text()
    masses = [float(m) for m in re.findall(r"<mass>([0-9.e-]+)</mass>", sdf)]
    body_old = masses[0]                                # base_link comes first
    body = TOTAL_MASS - (sum(masses) - body_old) - LIDAR_MASS - len(TOF) * TOF_MASS
    ratio = body / body_old
    base = re.search(r"<link name='base_link'>.*?</inertial>", sdf, flags=re.S)
    block = re.sub(r"<mass>[0-9.e-]+</mass>", f"<mass>{body:.4f}</mass>", base.group(0), count=1)
    block = re.sub(r"<(i[xyz]{2})>([0-9.e-]+)</\1>",
                   lambda m: f"<{m.group(1)}>{float(m.group(2)) * ratio:.6g}</{m.group(1)}>", block)
    sdf = sdf[:base.start()] + block + sdf[base.end():]
    half = FOOTPRINT / 2
    # body: replace the collision mesh (arms out to the motors) with a footprint-sized box
    sdf = re.sub(r"(<collision name='base_collision'>.*?<geometry>).*?(</geometry>)",
                 rf"\1<box><size>{FOOTPRINT} {FOOTPRINT} 0.08</size></box>\2", sdf, count=1, flags=re.S)
    # rotor discs: no collision (the guards are inside the footprint box)
    sdf = re.sub(r"\s*<collision name='collision'>.*?</collision>", "", sdf, flags=re.S)
    # landing legs: pull in under the footprint
    leg = half - 0.05
    for old, sign in {"0.123 0.22": (1, 1), "0.123 -0.22": (1, -1),
                      "-0.140 0.21": (-1, 1), "-0.140 -0.21": (-1, -1)}.items():
        sdf = sdf.replace(f"<pose>{old} -0.11", f"<pose>{sign[0] * leg:.3f} {sign[1] * leg:.3f} -0.11")
    FRAME_OUT.mkdir(parents=True, exist_ok=True)
    (FRAME_OUT / "model.sdf").write_text(sdf)
    (FRAME_OUT / "model.config").write_text(FRAME_CONFIG)
    print(f"wrote {FRAME_OUT}/model.sdf ({FOOTPRINT * 100:.1f} cm square collision box = {DIAGONAL * 100:.0f} cm "
          f"diagonal, body {body:.3f} kg -> {TOTAL_MASS} kg all up)")


def main():
    make_small_frame()
    sdf = SRC.read_text()
    sdf = sdf.replace('<model name="iris_with_ardupilot">', '<model name="iris_lidar">', 1)
    # use the small-footprint frame (its inner model name stays iris_with_standoffs, so plugins still match)
    sdf = sdf.replace("<uri>model://iris_with_standoffs</uri>", "<uri>model://iris_standoffs_5in</uri>")
    first_plugin = sdf.index("<plugin")
    # insert before the first plugin, at the start of that line
    line_start = sdf.rfind("\n", 0, first_plugin) + 1
    sdf = sdf[:line_start] + LIDAR.replace("{TOF_SENSORS}", tof_sensors()) + sdf[line_start:]

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "model.sdf").write_text(sdf)
    (OUT / "model.config").write_text(CONFIG)
    print(f"wrote {OUT}/model.sdf")


if __name__ == "__main__":
    main()
