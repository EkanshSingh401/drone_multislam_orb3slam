#!/usr/bin/env python3
"""Single source of truth for the simulated D455-mirror sensor rig.

Everything downstream is GENERATED from the SPEC below, so the Gazebo SDF, the
calibration document and the OpenVINS/Kalibr YAMLs cannot drift apart:

  docker/sim/models/d455/model.sdf              Gazebo sensor module
  docker/sim/models/x500_d455/model.sdf         x500 carrying the module
  docker/sim/config_sim_only/kalibr_imu_chain.yaml
  docker/sim/config_sim_only/kalibr_imucam_chain.yaml
  docker/sim/config_sim_only/D455_SIM_CALIBRATION.md

The hard part is the IMU noise units, and getting it wrong is silent:

  * Gazebo's <noise><stddev> for an IMU is a PER-SAMPLE (discrete) standard
    deviation, in m/s^2 or rad/s.
  * OpenVINS/Kalibr want CONTINUOUS-TIME noise DENSITIES, in m/s^2/sqrt(Hz) or
    rad/s/sqrt(Hz).

        sigma_discrete = sigma_density * sqrt(rate)
        sigma_density  = sigma_discrete / sqrt(rate)

    At 200 Hz that is a factor of sqrt(200) = 14.14. Feeding a density straight
    into Gazebo (or a discrete sigma straight into OpenVINS) misstates the noise
    by ~14x, and the filter still runs -- it just mis-weights the IMU.

  * Bias is NOT a plain random walk in Gazebo. gz-sim models a first-order
    Gauss-Markov (Ornstein-Uhlenbeck) process via <dynamic_bias_stddev> (the
    stationary standard deviation, sigma_b) and <dynamic_bias_correlation_time>
    (tau). Kalibr's *_random_walk is the driving random-walk density sigma_rw.
    For an OU process with stationary sigma_b and correlation time tau:

        sigma_rw = sigma_b * sqrt(2 / tau)
        sigma_b  = sigma_rw * sqrt(tau / 2)

    We declare the random-walk densities (the physical quantity) and derive
    Gazebo's sigma_b, so the direction of conversion never loses information.

Run with --check to verify the committed files match the spec.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# ============================== SPEC ==========================================
# Declared in the units the REAL sensor datasheets / Kalibr use. Everything
# Gazebo needs is derived from these.

IMU_RATE_HZ = 200.0

# Continuous-time densities (BMI055-class, as found on a D455).
ACC_NOISE_DENSITY = 2.0e-3      # m/s^2/sqrt(Hz)
ACC_RANDOM_WALK   = 3.0e-4      # m/s^3 (i.e. m/s^2 per sqrt(s))
GYR_NOISE_DENSITY = 1.6e-4      # rad/s/sqrt(Hz)
GYR_RANDOM_WALK   = 2.0e-6      # rad/s^2

# OU correlation time for the bias. Long relative to a ~60 s flight so the bias
# behaves like a random walk over the run.
BIAS_CORRELATION_TIME_S = 3600.0

# Imagers. 848x480 @ 30 Hz is the usual D455 VIO mode.
IMG_W, IMG_H = 848, 480
IMG_RATE_HZ = 30.0
IR_HFOV_RAD = math.radians(87.0)     # D455 depth/IR horizontal FOV
STEREO_BASELINE_M = 0.095            # D455 = 95 mm (D435 is 50 mm)

# Geometry, all relative to the d455_link origin (module mounting reference).
# Gazebo/ROS REP-103 body axes: +X forward, +Y left, +Z up.
# These are realistic D455-LIKE values chosen for this rig; they are NOT scraped
# from a vendor datasheet. Because the SDF is generated from them, whatever is
# written here IS the simulation's ground-truth calibration.
P_CAM0 = (0.0,  STEREO_BASELINE_M / 2.0, 0.0)   # left  IR (infra1)
P_CAM1 = (0.0, -STEREO_BASELINE_M / 2.0, 0.0)   # right IR (infra2)
P_DEPTH = P_CAM0                                 # D455 aligns depth to left IR
P_IMU  = (-0.00552, 0.00510, -0.01174)           # BMI055 inside the module

# Where the module sits on the airframe.
P_MODULE_ON_BASE = (0.12, 0.0, 0.242)

DEPTH_NEAR, DEPTH_FAR = 0.2, 20.0

# Runtime names the bridge config is written against. PX4 spawns
# "<PX4_SIM_MODEL without gz_>_<instance>", so gz_x500_d455 -i 1 -> x500_d455_1.
WORLD_NAME = "forest"
MODEL_NAME = "x500_d455_1"

# ROS-side topic names. Deliberately IDENTICAL to the Jetson/D455 stack so that
# OpenVINS configs and launch files transfer unchanged.
ROS_TOPICS = {
    "infra1_image": "/camera/infra1/image_rect_raw",
    "infra1_info":  "/camera/infra1/camera_info",
    "infra2_image": "/camera/infra2/image_rect_raw",
    "infra2_info":  "/camera/infra2/camera_info",
    "depth_image":  "/camera/depth/image_rect_raw",
    "depth_info":   "/camera/depth/camera_info",
    "imu":          "/camera/imu",
}
# ==============================================================================


def derived() -> dict:
    s = math.sqrt(IMU_RATE_HZ)
    tau = BIAS_CORRELATION_TIME_S
    fx = (IMG_W / 2.0) / math.tan(IR_HFOV_RAD / 2.0)
    return {
        "acc_stddev": ACC_NOISE_DENSITY * s,
        "gyr_stddev": GYR_NOISE_DENSITY * s,
        "acc_bias_stddev": ACC_RANDOM_WALK * math.sqrt(tau / 2.0),
        "gyr_bias_stddev": GYR_RANDOM_WALK * math.sqrt(tau / 2.0),
        "sqrt_rate": s,
        "fx": fx, "fy": fx,
        "cx": IMG_W / 2.0, "cy": IMG_H / 2.0,
    }


def t_imu_from_cam(p_cam) -> tuple[float, float, float]:
    """Translation of a camera origin expressed in the IMU body frame."""
    return tuple(c - i for c, i in zip(p_cam, P_IMU))


# Camera OPTICAL frame (z forward, x right, y down) relative to the Gazebo
# sensor/body frame (x forward, y left, z up):
#   optical x = -body y ;  optical y = -body z ;  optical z = body x
R_OPT_FROM_BODY = ((0.0, -1.0, 0.0),
                   (0.0, 0.0, -1.0),
                   (1.0, 0.0, 0.0))


def mat_vec(R, v):
    return tuple(sum(R[r][c] * v[c] for c in range(3)) for r in range(3))


def t_cam_imu(p_cam) -> list[list[float]]:
    """Kalibr T_cam_imu: 4x4 taking points from the IMU frame to the camera
    OPTICAL frame. Rotation is pure axis relabelling here because every sensor
    is mounted axis-aligned with the module."""
    t_body = tuple(-x for x in t_imu_from_cam(p_cam))   # IMU origin seen from cam body
    t_opt = mat_vec(R_OPT_FROM_BODY, t_body)
    R = R_OPT_FROM_BODY
    return [[R[0][0], R[0][1], R[0][2], t_opt[0]],
            [R[1][0], R[1][1], R[1][2], t_opt[1]],
            [R[2][0], R[2][1], R[2][2], t_opt[2]],
            [0.0, 0.0, 0.0, 1.0]]


def fmt_pose(p) -> str:
    return f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} 0 0 0"


def camera_block(name: str, pose, kind: str, d: dict) -> str:
    """One Gazebo camera sensor. kind is 'ir' (mono) or 'depth'."""
    if kind == "ir":
        fmt = "      <format>L8</format>\n"          # monochrome, like a real IR imager
        stype = "camera"
        extra = ""
    else:
        fmt = "      <format>R_FLOAT32</format>\n"
        stype = "depth_camera"
        extra = ""
    return f"""      <sensor name="{name}" type="{stype}">
        <gz_frame_id>d455_link</gz_frame_id>
        <pose>{fmt_pose(pose)}</pose>
        <update_rate>{IMG_RATE_HZ:g}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>{IR_HFOV_RAD:.9f}</horizontal_fov>
          <image>
            <width>{IMG_W}</width>
            <height>{IMG_H}</height>
{fmt}          </image>
          <clip>
            <near>{DEPTH_NEAR}</near>
            <far>{DEPTH_FAR}</far>
          </clip>
        </camera>{extra}
      </sensor>
"""


def gen_d455_model(d: dict) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
     Re-generate with:  python3 docker/sim/tools/gen_d455_sim.py --write

     D455-mirror sensor module: stereo IR pair ({IMG_W}x{IMG_H} @ {IMG_RATE_HZ:g} Hz,
     {STEREO_BASELINE_M*1000:.0f} mm baseline, monochrome), aligned depth camera, and a
     {IMU_RATE_HZ:g} Hz IMU with an explicit noise model.

     NOTE: no <topic> overrides. Gazebo's <topic> element REPLACES the scoped
     sensor topic, and for a camera it also moves camera_info to
     "<parent>/camera_info" -- so three cameras with <topic> set would all
     collide on /camera_info. PX4's own OakD-Lite model sets
     <topic>depth_camera</topic> and that is precisely why the repo's bridge
     config pointed at a topic nothing published (docker/PATCHES.md section 21).
     Leaving it out gives predictable, collision-free scoped names, which the
     bridge then maps onto the Jetson-side names.

     IMU noise below is PER-SAMPLE (discrete), which is what Gazebo expects.
     It is derived from continuous-time densities -- see
     docker/sim/config_sim_only/D455_SIM_CALIBRATION.md for the conversion. -->
<sdf version="1.9">
  <model name="d455">
    <link name="d455_link">
      <inertial>
        <mass>0.072</mass>
        <inertia>
          <ixx>4.0e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1.3e-5</iyy><iyz>0</iyz><izz>4.5e-5</izz>
        </inertia>
      </inertial>
      <visual name="d455_visual">
        <geometry><box><size>0.124 0.026 0.029</size></box></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
      <collision name="d455_collision">
        <geometry><box><size>0.124 0.026 0.029</size></box></geometry>
      </collision>

{camera_block("infra1", P_CAM0, "ir", d)}
{camera_block("infra2", P_CAM1, "ir", d)}
{camera_block("depth", P_DEPTH, "depth", d)}
      <sensor name="d455_imu" type="imu">
        <gz_frame_id>d455_link</gz_frame_id>
        <pose>{fmt_pose(P_IMU)}</pose>
        <update_rate>{IMU_RATE_HZ:g}</update_rate>
        <always_on>1</always_on>
        <imu>
          <angular_velocity>
{_axis_noise(d["gyr_stddev"], d["gyr_bias_stddev"])}          </angular_velocity>
          <linear_acceleration>
{_axis_noise(d["acc_stddev"], d["acc_bias_stddev"])}          </linear_acceleration>
        </imu>
      </sensor>
    </link>
  </model>
</sdf>
"""


def _axis_noise(stddev: float, bias_stddev: float) -> str:
    block = ""
    for axis in ("x", "y", "z"):
        block += f"""            <{axis}>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>{stddev:.9g}</stddev>
                <dynamic_bias_stddev>{bias_stddev:.9g}</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>{BIAS_CORRELATION_TIME_S:g}</dynamic_bias_correlation_time>
              </noise>
            </{axis}>
"""
    return block


def gen_x500_d455() -> str:
    p = P_MODULE_ON_BASE
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
     x500 airframe carrying the D455-mirror module. Mirrors the structure of
     PX4's own x500_depth model so PX4_SIM_MODEL=gz_x500_d455 works the same
     way as gz_x500_depth. -->
<sdf version="1.9">
  <model name="x500_d455">
    <include merge="true">
      <uri>x500</uri>
    </include>
    <include merge="true">
      <uri>model://d455</uri>
      <pose>{fmt_pose(p)}</pose>
    </include>
    <joint name="D455Joint" type="fixed">
      <parent>base_link</parent>
      <child>d455_link</child>
      <pose relative_to="base_link">{fmt_pose(p)}</pose>
    </joint>
  </model>
</sdf>
"""


def gen_model_config(name: str, desc: str) -> str:
    return f"""<?xml version="1.0"?>
<!-- GENERATED by docker/sim/tools/gen_d455_sim.py -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""


def gen_imu_chain(d: dict) -> str:
    return f"""# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# SIMULATION ONLY. These values describe the Gazebo D455-mirror IMU, not a
# physical D455. Never copy this over the real sensor's Kalibr output.
#
# Values are CONTINUOUS-TIME densities, which is what OpenVINS/Kalibr expect.
# Gazebo is configured with the equivalent PER-SAMPLE sigmas at {IMU_RATE_HZ:g} Hz:
#   sigma_discrete = sigma_density * sqrt({IMU_RATE_HZ:g}) = sigma_density * {d['sqrt_rate']:.6f}
# See D455_SIM_CALIBRATION.md for the full derivation.
imu0:
  T_i_b:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: {ACC_NOISE_DENSITY:.8g}
  accelerometer_random_walk: {ACC_RANDOM_WALK:.8g}
  gyroscope_noise_density: {GYR_NOISE_DENSITY:.8g}
  gyroscope_random_walk: {GYR_RANDOM_WALK:.8g}
  model: calibrated
  rostopic: /camera/imu
  time_offset: 0.0
  update_rate: {IMU_RATE_HZ:g}
"""


def _yaml_mat(m) -> str:
    return "\n".join("  - [" + ", ".join(f"{v:.10g}" for v in row) + "]" for row in m)


def gen_imucam_chain(d: dict) -> str:
    c0 = t_cam_imu(P_CAM0)
    c1 = t_cam_imu(P_CAM1)
    return f"""# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# SIMULATION ONLY -- ground-truth calibration of the Gazebo D455-mirror rig.
# These are EXACT: the same spec generates the SDF the simulator loads, so there
# is no calibration error here. That is the point -- it lets a VIO estimator be
# evaluated without conflating estimator error with calibration error.
#
# Intrinsics follow from the SDF directly: a Gazebo pinhole camera with
# horizontal_fov h and width w has fx = (w/2)/tan(h/2), cx = w/2, cy = h/2, and
# zero distortion.
#   h = {math.degrees(IR_HFOV_RAD):.4f} deg, w = {IMG_W} -> fx = fy = {d['fx']:.6f}
#
# T_cam_imu takes points from the IMU frame into the camera OPTICAL frame
# (z forward, x right, y down). The rotation is a pure axis relabelling because
# every sensor is mounted axis-aligned with the module body frame
# (x forward, y left, z up).
cam0:
  T_cam_imu:
{_yaml_mat(c0)}
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{d['fx']:.6f}, {d['fy']:.6f}, {d['cx']:.1f}, {d['cy']:.1f}]
  resolution: [{IMG_W}, {IMG_H}]
  rostopic: /camera/infra1/image_rect_raw
cam1:
  T_cam_imu:
{_yaml_mat(c1)}
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{d['fx']:.6f}, {d['fy']:.6f}, {d['cx']:.1f}, {d['cy']:.1f}]
  resolution: [{IMG_W}, {IMG_H}]
  rostopic: /camera/infra2/image_rect_raw
"""



def _gz_sensor(sensor: str, leaf: str) -> str:
    return (f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/d455_link"
            f"/sensor/{sensor}/{leaf}")


def gen_bridge_yaml(d: dict) -> str:
    e = []

    def entry(ros, gz, ros_t, gz_t, note=""):
        e.append((ros, gz, ros_t, gz_t, note))

    entry(ROS_TOPICS["infra1_image"], _gz_sensor("infra1", "image"),
          "sensor_msgs/msg/Image", "gz.msgs.Image",
          "left IR, mono8, %dx%d @ %g Hz" % (IMG_W, IMG_H, IMG_RATE_HZ))
    entry(ROS_TOPICS["infra1_info"], _gz_sensor("infra1", "camera_info"),
          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    entry(ROS_TOPICS["infra2_image"], _gz_sensor("infra2", "image"),
          "sensor_msgs/msg/Image", "gz.msgs.Image",
          "right IR, mono8, %g mm baseline from infra1" % (STEREO_BASELINE_M * 1000))
    entry(ROS_TOPICS["infra2_info"], _gz_sensor("infra2", "camera_info"),
          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    entry(ROS_TOPICS["depth_image"], _gz_sensor("depth", "depth_image"),
          "sensor_msgs/msg/Image", "gz.msgs.Image",
          "depth, 32FC1 metres, aligned to infra1")
    entry(ROS_TOPICS["depth_info"], _gz_sensor("depth", "camera_info"),
          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    entry(ROS_TOPICS["imu"], _gz_sensor("d455_imu", "imu"),
          "sensor_msgs/msg/Imu", "gz.msgs.IMU",
          "%g Hz, noise model per D455_SIM_CALIBRATION.md" % IMU_RATE_HZ)

    out = f"""# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# ros_gz_bridge config for the D455-mirror rig.
#
# The ROS topic names are deliberately IDENTICAL to the Jetson/D455 stack, so
# OpenVINS configs and launch files transfer unchanged. "image_rect_raw" is
# accurate rather than aspirational: the simulated cameras have zero distortion,
# so the raw images ARE rectified.
#
# The gz-side names are the SCOPED sensor topics. The model SDF deliberately
# sets no <topic> override -- see docker/PATCHES.md section 21 for what happens
# when it does (PX4's OakD-Lite sets <topic>depth_camera</topic>, which both
# moves the image off its scoped name AND relocates camera_info to /camera_info,
# where three cameras would collide).
#
# NOTE: the gz names embed world "{WORLD_NAME}" and model "{MODEL_NAME}".
# PX4 spawns "<PX4_SIM_MODEL minus gz_>_<instance>", so this matches
# PX4_SIM_MODEL=gz_x500_d455 with -i 1 in world {WORLD_NAME}.
"""
    for ros, gz, rt, gt, note in e:
        if note:
            out += f"\n# {note}\n"
        else:
            out += "\n"
        out += (f'- ros_topic_name: "{ros}"\n'
                f'  gz_topic_name: "{gz}"\n'
                f'  ros_type_name: "{rt}"\n'
                f'  gz_type_name: "{gt}"\n'
                f'  direction: GZ_TO_ROS\n')

    out += f"""
# Simulation clock -- required because every ROS 2 node runs use_sim_time:=true.
- ros_topic_name: "/clock"
  gz_topic_name: "/world/{WORLD_NAME}/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# Ground truth for ATE/RPE. Kept OFF /tf so it can never compete with the SLAM
# TF tree. ros_gz_bridge drops the entity names in this conversion, so consumers
# select the model by INDEX (index 0) -- see docker/PATCHES.md section 27.
- ros_topic_name: "ground_truth/pose_info"
  gz_topic_name: "/world/{WORLD_NAME}/dynamic_pose/info"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
"""
    return out



def t_imu_cam(p_cam) -> list[list[float]]:
    """ORB-SLAM3 IMU.T_b_c1: body(IMU) <- camera(optical). Inverse of T_cam_imu."""
    # R_imu_cam = R_opt_from_body^T ; t = camera origin expressed in IMU frame
    R = R_OPT_FROM_BODY
    Rt = [[R[c][r] for c in range(3)] for r in range(3)]
    t = t_imu_from_cam(p_cam)
    return [[Rt[0][0], Rt[0][1], Rt[0][2], t[0]],
            [Rt[1][0], Rt[1][1], Rt[1][2], t[1]],
            [Rt[2][0], Rt[2][1], Rt[2][2], t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def gen_orbslam3_stereo_inertial(d: dict) -> str:
    T = t_imu_cam(P_CAM0)
    rows = ",\n           ".join(
        "[" + ", ".join(f"{v:.10g}" for v in row) + "]" for row in T)
    bf = d["fx"] * STEREO_BASELINE_M
    return f"""%YAML:1.0
# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# ORB-SLAM3 STEREO-INERTIAL settings for the simulated D455-mirror rig.
# SIMULATION ONLY.
#
# Camera.type "Rectified" is correct here and not a shortcut: the simulated
# cameras have zero distortion, so the raw images already are ideal pinhole
# images. ORB-SLAM3 then needs only Camera1 intrinsics plus Stereo.b.
#
# IMU noise below is in ORB-SLAM3's units, which are CONTINUOUS-TIME densities,
# the same convention as Kalibr/OpenVINS -- NOT Gazebo's per-sample sigmas.
# Gazebo is configured with sigma_discrete = sigma_density * sqrt({IMU_RATE_HZ:g}).
# See config_sim_only/D455_SIM_CALIBRATION.md.

File.version: "1.0"

Camera.type: "Rectified"

Camera1.fx: {d['fx']:.6f}
Camera1.fy: {d['fy']:.6f}
Camera1.cx: {d['cx']:.1f}
Camera1.cy: {d['cy']:.1f}

Camera.width: {IMG_W}
Camera.height: {IMG_H}
Camera.fps: {IMG_RATE_HZ:g}
# IR imagers are monochrome, so colour order is irrelevant; 0 keeps it explicit.
Camera.RGB: 0

# Stereo baseline, metres, and fx*baseline.
Stereo.b: {STEREO_BASELINE_M:.6f}
# Close/far threshold in units of the stereo baseline. 40 is the ORB-SLAM3
# default for EuRoC-class rigs; with bf = {bf:.3f} that puts the boundary at
# roughly {40.0 * STEREO_BASELINE_M:.2f} m.
Stereo.ThDepth: 40.0

# --- IMU ---------------------------------------------------------------------
IMU.NoiseGyro: {GYR_NOISE_DENSITY:.8g}    # rad/s/sqrt(Hz)
IMU.NoiseAcc: {ACC_NOISE_DENSITY:.8g}     # m/s^2/sqrt(Hz)
IMU.GyroWalk: {GYR_RANDOM_WALK:.8g}       # rad/s^2
IMU.AccWalk: {ACC_RANDOM_WALK:.8g}        # m/s^3
IMU.Frequency: {IMU_RATE_HZ:g}

# Transform body(IMU) <- camera1(optical). Exact, not estimated: the same spec
# generates the SDF, so this is ground truth by construction.
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{",".join(f"{v:.10g}" for row in T for v in row)}]

# --- ORB extractor -----------------------------------------------------------
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# --- Viewer (disabled; headless) ---------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""


FILES = {
    "docker/sim/models/d455/model.sdf": lambda d: gen_d455_model(d),
    "docker/sim/models/d455/model.config": lambda d: gen_model_config(
        "d455", "D455-mirror stereo-IR + depth + IMU module (simulation)"),
    "docker/sim/models/x500_d455/model.sdf": lambda d: gen_x500_d455(),
    "docker/sim/models/x500_d455/model.config": lambda d: gen_model_config(
        "x500_d455", "x500 carrying the D455-mirror sensor module (simulation)"),
    "docker/sim/config_sim_only/kalibr_imu_chain.yaml": lambda d: gen_imu_chain(d),
    "docker/sim/config_sim_only/kalibr_imucam_chain.yaml": lambda d: gen_imucam_chain(d),
    "docker/sim/config_sim_only/gz_bridge_d455.yaml": lambda d: gen_bridge_yaml(d),
    "docker/sim/config_sim_only/orbslam3_d455_stereo_inertial.yaml":
        lambda d: gen_orbslam3_stereo_inertial(d),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="write the generated files")
    ap.add_argument("--check", action="store_true", help="verify files match the spec")
    ap.add_argument("--root", default=".")
    args = ap.parse_args()
    d = derived()
    root = Path(args.root)

    if not (args.write or args.check):
        print("IMU noise conversion at "
              f"{IMU_RATE_HZ:g} Hz (sqrt(rate) = {d['sqrt_rate']:.6f}):")
        print(f"  accel density {ACC_NOISE_DENSITY:.4g} m/s^2/sqrt(Hz)"
              f"  -> Gazebo stddev {d['acc_stddev']:.6g} m/s^2")
        print(f"  gyro  density {GYR_NOISE_DENSITY:.4g} rad/s/sqrt(Hz)"
              f"  -> Gazebo stddev {d['gyr_stddev']:.6g} rad/s")
        print(f"  accel RW {ACC_RANDOM_WALK:.4g} m/s^3"
              f"  -> Gazebo dynamic_bias_stddev {d['acc_bias_stddev']:.6g} m/s^2 (tau={BIAS_CORRELATION_TIME_S:g}s)")
        print(f"  gyro  RW {GYR_RANDOM_WALK:.4g} rad/s^2"
              f"  -> Gazebo dynamic_bias_stddev {d['gyr_bias_stddev']:.6g} rad/s (tau={BIAS_CORRELATION_TIME_S:g}s)")
        print(f"  camera fx = fy = {d['fx']:.6f}  cx={d['cx']:.1f} cy={d['cy']:.1f}")
        print(f"  T_imu_cam0 translation = {t_imu_from_cam(P_CAM0)}")
        print(f"  T_imu_cam1 translation = {t_imu_from_cam(P_CAM1)}")
        print("\nRun with --write to emit files, --check to verify them.")
        return 0

    bad = 0
    for rel, fn in FILES.items():
        p = root / rel
        content = fn(d)
        if args.write:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content)
            print(f"  wrote {rel}")
        else:
            cur = p.read_text() if p.exists() else ""
            if cur != content:
                print(f"  STALE {rel}")
                bad += 1
            else:
                print(f"  ok    {rel}")
    if args.check:
        print("all generated files match the spec" if bad == 0
              else f"{bad} file(s) differ -- run with --write")
        return 1 if bad else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
