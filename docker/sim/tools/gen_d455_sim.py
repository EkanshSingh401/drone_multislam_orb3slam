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

# Gravity magnitude. 9.81, NOT Gazebo's 9.8 default, because ORB-SLAM3 hardcodes
#   ORB_SLAM3/include/ImuTypes.h:46  const float GRAVITY_VALUE = 9.81;
# with no way to configure it, while OpenVINS reads gravity_mag from YAML. If the
# world ran at 9.8 then ORB-SLAM3 would be handed a 0.01 m/s^2 error and OpenVINS
# would not, which is a difference in the estimators' INPUTS rather than in their
# estimation. docker/sim/Dockerfile patches every PX4 world to match this value,
# and run_experiment.sh records the world's actual gravity in each MANIFEST.txt.
GRAVITY_MAG = 9.81

# Gazebo's physics step, from the PX4 worlds (<max_step_size>0.004</max_step_size>).
# Sensor stamps are quantised to it, so the INSTANTANEOUS inter-frame gap is a
# whole number of steps and is coarser than 1/IMG_RATE_HZ. At 30 Hz the period is
# 33.33 ms, which lands between 8 steps (32 ms) and 9 steps (36 ms), and the
# stream alternates between the two -- measured exactly that way by
# check_stereo_sync.py (min 32.000 ms, median 32.000, max 36.000).
#
# The MINIMUM gap is what OpenVINS's frame throttle has to clear, so it is
# derived here rather than assumed.
PHYSICS_STEP_S = 0.004
MIN_FRAME_GAP_S = math.floor((1.0 / IMG_RATE_HZ) / PHYSICS_STEP_S) * PHYSICS_STEP_S
MIN_TRACK_FREQ_HZ = 1.0 / MIN_FRAME_GAP_S

# Runtime names the bridge config is written against. PX4 spawns
# "<PX4_SIM_MODEL without gz_>_<instance>", so gz_x500_d455 -i 1 -> x500_d455_1.
WORLD_NAME = "forest"
MODEL_NAME = "x500_d455_1"
MODEL_NAME_NODEPTH = "x500_d455_nodepth_1"

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


def gen_d455_model(d: dict, with_depth: bool = True) -> str:
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
  <model name="{"d455" if with_depth else "d455_nodepth"}">
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
{camera_block("depth", P_DEPTH, "depth", d) if with_depth else "      <!-- depth camera omitted in this variant -->"}
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


def gen_x500_d455(with_depth: bool = True) -> str:
    p = P_MODULE_ON_BASE
    mod = "d455" if with_depth else "d455_nodepth"
    top = "x500_d455" if with_depth else "x500_d455_nodepth"
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
     x500 airframe carrying the D455-mirror module. Mirrors the structure of
     PX4's own x500_depth model so PX4_SIM_MODEL=gz_x500_d455 works the same
     way as gz_x500_depth. -->
<sdf version="1.9">
  <model name="{top}">
    <include merge="true">
      <uri>x500</uri>
    </include>
    <include merge="true">
      <uri>model://{mod}</uri>
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
    return f"""%YAML:1.0
# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# SIMULATION ONLY. These values describe the Gazebo D455-mirror IMU, not a
# physical D455. Never copy this over the real sensor's Kalibr output.
#
# The %YAML:1.0 directive on line 1 is REQUIRED, not decoration: OpenVINS reads
# these files with cv::FileStorage via ov_core::YamlParser, and every config
# OpenVINS ships carries it.
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

  # IMU intrinsics: identity scale/misalignment, zero g-sensitivity. The
  # simulated IMU has none of these imperfections, so identity is EXACT here
  # rather than a placeholder.
  #
  # These five blocks must be present even though estimator_config.yaml sets
  # calib_imu_intrinsics=false and calib_imu_g_sensitivity=false.
  # VioManagerOptions.h:307-315 reads Tw, Ta, R_IMUtoACC, R_IMUtoGYRO and Tg
  # through parse_external() with its default required=true; a missing key
  # clears YamlParser::all_params_found_successfully, and
  # run_subscribe_msckf.cpp:96 then refuses to start. It does NOT fall back to
  # the Eigen defaults it was handed.
  Tw:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoGYRO:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Ta:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoACC:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Tg:
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
"""


def _yaml_mat(m, indent: int = 4) -> str:
    """Emit a matrix as a YAML block sequence, indented for cv::FileStorage.

    The rows must be indented DEEPER than their key. Writing

        T_i_b:
        - [1.0, 0.0, 0.0, 0.0]

    is valid YAML and PyYAML accepts it, but OpenVINS parses these files with
    cv::FileStorage, whose YAML reader rejects it:

        OpenCV(4.6.0) persistence_yml.cpp:359: error: (-212:Parsing error)
        skipSpaces ... kalibr_imu_chain.yaml(17): Incorrect indentation

    and the node aborts with a cv::Exception before reading anything else.
    OpenVINS's own shipped configs indent the rows by four spaces, so match that.
    """
    pad = " " * indent
    return "\n".join(pad + "- [" + ", ".join(f"{v:.10g}" for v in row) + "]" for row in m)


def gen_imucam_chain(d: dict) -> str:
    c0 = t_cam_imu(P_CAM0)
    c1 = t_cam_imu(P_CAM1)
    return f"""%YAML:1.0
# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
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
  # Overlap and time offset stated explicitly. timeshift_cam_imu is optional
  # (VioManagerOptions.h passes required=false) and would default to 0, but the
  # simulated cameras and IMU are stamped from the same simulation clock, so 0
  # is a measured fact here rather than an unstated default.
  cam_overlaps: [1]
  timeshift_cam_imu: 0.0
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{d['fx']:.6f}, {d['fy']:.6f}, {d['cx']:.1f}, {d['cy']:.1f}]
  resolution: [{IMG_W}, {IMG_H}]
  rostopic: /camera/infra1/image_rect_raw
cam1:
  T_cam_imu:
{_yaml_mat(c1)}
  cam_overlaps: [0]
  timeshift_cam_imu: 0.0
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [{d['fx']:.6f}, {d['fy']:.6f}, {d['cx']:.1f}, {d['cy']:.1f}]
  resolution: [{IMG_W}, {IMG_H}]
  rostopic: /camera/infra2/image_rect_raw
"""



def _gz_sensor(sensor: str, leaf: str, model: str = None) -> str:
    return (f"/world/{WORLD_NAME}/model/{model or MODEL_NAME}/link/d455_link"
            f"/sensor/{sensor}/{leaf}")


def gen_bridge_yaml(d: dict, with_depth: bool = True) -> str:
    model = MODEL_NAME if with_depth else MODEL_NAME_NODEPTH
    e = []

    def entry(ros, gz, ros_t, gz_t, note=""):
        e.append((ros, gz, ros_t, gz_t, note))

    entry(ROS_TOPICS["infra1_image"], _gz_sensor("infra1", "image", model),
          "sensor_msgs/msg/Image", "gz.msgs.Image",
          "left IR, mono8, %dx%d @ %g Hz" % (IMG_W, IMG_H, IMG_RATE_HZ))
    entry(ROS_TOPICS["infra1_info"], _gz_sensor("infra1", "camera_info", model),
          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    entry(ROS_TOPICS["infra2_image"], _gz_sensor("infra2", "image", model),
          "sensor_msgs/msg/Image", "gz.msgs.Image",
          "right IR, mono8, %g mm baseline from infra1" % (STEREO_BASELINE_M * 1000))
    entry(ROS_TOPICS["infra2_info"], _gz_sensor("infra2", "camera_info", model),
          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    if with_depth:
        entry(ROS_TOPICS["depth_image"], _gz_sensor("depth", "depth_image", model),
              "sensor_msgs/msg/Image", "gz.msgs.Image",
              "depth, 32FC1 metres, aligned to infra1")
        entry(ROS_TOPICS["depth_info"], _gz_sensor("depth", "camera_info", model),
              "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "")
    entry(ROS_TOPICS["imu"], _gz_sensor("d455_imu", "imu", model),
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
# NOTE: the gz names embed world "{WORLD_NAME}" and model "{model}".
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


def gen_orbslam3_stereo_inertial(d: dict, inertial: bool = True) -> str:
    """ORB-SLAM3 settings for the D455-mirror rig.

    inertial=False emits a STEREO-ONLY config, used for the bisection that asks
    whether the IMU is responsible for the error structure seen in the
    stereo-inertial runs. Stereo-only ORB-SLAM3 runs no visual-inertial bundle
    adjustment, so it has no VIBA stage and should show no mid-flight
    discontinuity. Everything else -- intrinsics, baseline, extractor settings --
    is byte-for-byte the same, so the two differ only in the IMU.
    """
    T = t_imu_cam(P_CAM0)
    rows = ",\n           ".join(
        "[" + ", ".join(f"{v:.10g}" for v in row) + "]" for row in T)
    bf = d["fx"] * STEREO_BASELINE_M
    if inertial:
        imu_block = f"""# --- IMU ---------------------------------------------------------------------
# Written in exponent form with an explicit mantissa decimal point. ORB-SLAM3
# reads these with readParameter<float>, which rejects anything OpenCV
# FileStorage does not classify as real: an integer literal aborts startup with
# "<name> parameter must be a real number, aborting...". IMU.Frequency: 200
# failed exactly that way, while Camera.fps genuinely is read as an int.
IMU.NoiseGyro: {GYR_NOISE_DENSITY:.10e}    # rad/s/sqrt(Hz)
IMU.NoiseAcc: {ACC_NOISE_DENSITY:.10e}     # m/s^2/sqrt(Hz)
IMU.GyroWalk: {GYR_RANDOM_WALK:.10e}       # rad/s^2
IMU.AccWalk: {ACC_RANDOM_WALK:.10e}        # m/s^3
IMU.Frequency: {IMU_RATE_HZ:.1f}

# Transform body(IMU) <- camera1(optical). Exact, not estimated: the same spec
# generates the SDF, so this is ground truth by construction.
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{",".join(f"{v:.10g}" for row in T for v in row)}]
"""
    else:
        imu_block = ("# --- IMU ---------------------------------------------------------------------\n"
                     "# DELIBERATELY ABSENT. This is the stereo-only config: ORB-SLAM3 is\n"
                     "# constructed with System::STEREO, consumes no IMU, and runs no\n"
                     "# visual-inertial bundle adjustment. Used as a bisection against the\n"
                     "# stereo-inertial config, which is identical apart from this block.")
    mode = "STEREO-INERTIAL" if inertial else "STEREO-ONLY (no IMU)"
    return f"""%YAML:1.0
# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
#
# ORB-SLAM3 {mode} settings for the simulated D455-mirror rig.
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

{imu_block}

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



def gen_wrapper_ros_params(d: dict, inertial: bool = True) -> str:
    """ROS params for the stereo-inertial (or stereo-only) wrapper node.

    robot_x/y/z is the base_link -> cam0 (left IR) offset, which the wrapper uses
    to refer its published pose back to the robot body. Derived here so it cannot
    drift from the SDF: module-on-base plus cam0-within-module.
    """
    bx = P_MODULE_ON_BASE[0] + P_CAM0[0]
    by = P_MODULE_ON_BASE[1] + P_CAM0[1]
    bz = P_MODULE_ON_BASE[2] + P_CAM0[2]
    # The wrapper reads parameters under its NODE name, so the stereo-only
    # bisection needs its own key: that node is ORB_SLAM3_STEREO_ROS2, not
    # ORB_SLAM3_STEREO_INERTIAL_ROS2. A params file keyed to the wrong node name
    # loads without complaint and leaves every parameter at its compiled
    # default -- which for the topic names means subscribing to
    # "left/image_raw" and never receiving a single frame.
    node_name = ("ORB_SLAM3_STEREO_INERTIAL_ROS2" if inertial
                 else "ORB_SLAM3_STEREO_ROS2")
    imu_line = (f"    imu_topic_name: {ROS_TOPICS['imu']}\n" if inertial else "")
    return f"""# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
# SIMULATION ONLY.
{node_name}:
  ros__parameters:
    use_sim_time: true

    left_image_topic_name: {ROS_TOPICS['infra1_image']}
    right_image_topic_name: {ROS_TOPICS['infra2_image']}
{imu_line}
    robot_base_frame: base_link
    global_frame: map
    odom_frame: odom

    # base_link -> cam0 (left IR). Module at {P_MODULE_ON_BASE} on base_link,
    # cam0 at {P_CAM0} within the module.
    robot_x: {bx:.6f}
    robot_y: {by:.6f}
    robot_z: {bz:.6f}
    robot_qx: 0.0
    robot_qy: 0.0
    robot_qz: 0.0
    robot_qw: 1.0

    # Headless: no Pangolin viewer, and the wrapper must not fight the Gazebo
    # ground-truth TF, so it publishes no TF of its own.
    visualization: false
    odometry_mode: false
    publish_tf: false
    map_data_publish_frequency: 1000
    do_loop_closing: true
"""


def gen_openvins_estimator_config(d: dict) -> str:
    """OpenVINS estimator config for the simulated D455-mirror rig.

    SIMULATION ONLY. Derived from open_vins/config/rs_d455/estimator_config.yaml
    with the differences below. Written by the same generator that writes the
    SDF and the ORB-SLAM3 settings, so the sensor assumptions cannot drift apart
    -- `--verify` checks them against each other.

    Differences from the real-D455 config, each deliberate:

      calib_cam_timeoffset  true -> false. The real rig has a measured ~4.7 ms
            camera-IMU offset worth estimating. Here both streams are stamped
            from the same simulation clock and measured at EXACTLY 0.000000 ms
            offset (check_stereo_sync.py), so estimating it would add a free
            parameter with no signal behind it.

      gravity_mag  9.81 to match the patched world and ORB-SLAM3's constant.

      save_total_state  false -> true, with the filepaths pointed at the run
            directory. Required for NEES: the live path_gt consistency printout
            compares trajectories in UNALIGNED frames, so NEES has to be
            computed afterwards by ov_eval with posyaw alignment.

      track_frequency  31.0 -> 40.0, and the value is MEASURED, not guessed.
            ROS2Visualizer.cpp:565-568 drops a stereo pair when
                timestamp < camera_last_timestamp + 1/track_frequency
            and returns BEFORE updating camera_last_timestamp, with no log line.
            The bound is therefore set by the MINIMUM inter-frame gap, not by
            the average rate. check_stereo_sync.py measures that gap at exactly
            32.000 ms (Gazebo quantises stamps to its 4 ms physics step, so the
            instantaneous period is coarser than the 30.33 Hz average), which
            requires track_frequency > 31.250 Hz.
            The real-D455 config's 31.0 is BELOW that: it would have dropped
            frame 2, left camera_last_timestamp at frame 1, accepted frame 3 at
            +64 ms, and processed every OTHER frame -- ~15 Hz instead of 30.3,
            silently. 40.0 leaves margin; since the parameter is used only for
            this throttle (and one debug print), a higher value simply disables
            the throttle and costs nothing.

      init_dyn_use stays false (static initialisation only); dynamic
            initialisation segfaulted on the Jetson build.
    """
    return f"""%YAML:1.0
# GENERATED by docker/sim/tools/gen_d455_sim.py -- do not edit by hand.
# SIMULATION ONLY. Never copy this over a real sensor's configuration.

verbosity: "INFO"

use_fej: true
integration: "rk4"
use_stereo: true
max_cameras: 2

# Calibration states FROZEN. The simulated calibration is exact by
# construction -- the same spec generates the SDF the simulator loads -- so
# there is nothing to estimate and estimating it would only add noise.
calib_cam_extrinsics: false
calib_cam_intrinsics: false
calib_cam_timeoffset: false
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

gravity_mag: {GRAVITY_MAG}

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

try_zupt: false
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: false

# ==================================================================
# Initialisation. STATIC only.
# ==================================================================
# The scripted flight holds the airframe stationary on the ground for well
# over init_window_time before arming, then climbs -- which is the
# stationary-window-then-jerk sequence static initialisation needs.
#
# init_imu_thresh is MEASURED, not the stock 1.5 (PATCHES.md s46). OpenVINS's
# test is the std of the accel vector over each half of init_window_time: the
# newer half must exceed the threshold, the older half must not. On recorded
# x500_d455 bags that statistic is ~0.05 m/s^2 on the ground and peaks at
# ~1.07 m/s^2 during PX4's takeoff -- so at 1.5 OpenVINS never initialised on
# any flight ("no accel jerk detected", then "platform moving too much" once
# airborne). 0.5 is 10x the stationary floor with ~0.6 m/s^2 of headroom below
# the takeoff peak.
init_window_time: 2.0
init_imu_thresh: 0.5
init_max_disparity: 10.0
init_max_features: 50

init_dyn_use: false
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 6
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0
init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12
init_dyn_bias_g: [ 0.0, 0.0, 0.0 ]
init_dyn_bias_a: [ 0.0, 0.0, 0.0 ]

# ==================================================================
# Outputs. save_total_state is what makes NEES computable.
# ==================================================================
record_timing_information: false
record_timing_filepath: "/out/openvins/traj_timing.txt"

save_total_state: true
filepath_est: "/out/openvins/ov_estimate.txt"
filepath_std: "/out/openvins/ov_estimate_std.txt"
filepath_gt: "/out/openvins/ov_groundtruth.txt"

# ==================================================================
# Front end
# ==================================================================
use_klt: true
num_pts: 200
# fast_threshold by the pre-registered rule in PATCHES.md s48: highest value in
# {30,25,20,15,12,10,8,7,6,5} with >= 150 features tracked per frame (cam0 mean)
# on tuning bag 053636. None reached 150 (cam0 mean 75-81 at every threshold),
# so by the rule this is 5. The count is threshold-insensitive here.
fast_threshold: 5
grid_x: 5
grid_y: 5
min_px_dist: 15
knn_ratio: 0.70
# Must EXCEED 1/min_inter_frame_gap = 31.250 Hz (measured: min gap 32.000 ms),
# NOT merely the {IMG_RATE_HZ:g} Hz average -- see the docstring. Frames arriving
# sooner than 1/track_frequency are dropped silently.
track_frequency: 40.0
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"

use_aruco: false
num_aruco: 1024
downsize_aruco: true

# ==================================================================
up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

use_mask: false

# Per-camera intrinsics/extrinsics/topics and the IMU noise densities live in
# these two files, which this same generator writes from the same spec.
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""


FILES = {
    "docker/sim/models/d455/model.sdf": lambda d: gen_d455_model(d, True),
    "docker/sim/models/d455/model.config": lambda d: gen_model_config(
        "d455", "D455-mirror stereo-IR + depth + IMU module (simulation)"),
    "docker/sim/models/x500_d455/model.sdf": lambda d: gen_x500_d455(True),
    "docker/sim/models/x500_d455/model.config": lambda d: gen_model_config(
        "x500_d455", "x500 carrying the D455-mirror sensor module (simulation)"),
    # Depth-free variant for SLAM evaluation. Neither ORB-SLAM3 stereo-inertial
    # nor OpenVINS consumes depth, and rendering it is pure cost: it is a third
    # 848x480 render pass per frame. IR stays at 848x480 because resolution
    # changes feature counts and must match the hardware.
    "docker/sim/models/d455_nodepth/model.sdf": lambda d: gen_d455_model(d, False),
    "docker/sim/models/d455_nodepth/model.config": lambda d: gen_model_config(
        "d455_nodepth", "D455-mirror stereo-IR + IMU, no depth camera (simulation)"),
    "docker/sim/models/x500_d455_nodepth/model.sdf": lambda d: gen_x500_d455(False),
    "docker/sim/models/x500_d455_nodepth/model.config": lambda d: gen_model_config(
        "x500_d455_nodepth", "x500 carrying the D455-mirror module without depth (simulation)"),
    "docker/sim/config_sim_only/kalibr_imu_chain.yaml": lambda d: gen_imu_chain(d),
    "docker/sim/config_sim_only/kalibr_imucam_chain.yaml": lambda d: gen_imucam_chain(d),
    "docker/sim/config_sim_only/gz_bridge_d455.yaml": lambda d: gen_bridge_yaml(d, True),
    "docker/sim/config_sim_only/gz_bridge_d455_nodepth.yaml":
        lambda d: gen_bridge_yaml(d, False),
    "docker/sim/config_sim_only/orbslam3_d455_stereo_inertial.yaml":
        lambda d: gen_orbslam3_stereo_inertial(d),
    "docker/sim/config_sim_only/stereo_inertial_ros_params.yaml":
        lambda d: gen_wrapper_ros_params(d),
    # Stereo-only bisection: same rig, same intrinsics, no IMU and therefore no
    # VIBA. See gen_orbslam3_stereo_inertial's docstring.
    "docker/sim/config_sim_only/orbslam3_d455_stereo.yaml":
        lambda d: gen_orbslam3_stereo_inertial(d, inertial=False),
    "docker/sim/config_sim_only/stereo_ros_params.yaml":
        lambda d: gen_wrapper_ros_params(d, inertial=False),
    # OpenVINS reads relative_config_imu / relative_config_imucam as paths
    # RELATIVE to the estimator config, so the three files must sit in one
    # directory. config_sim_only/ already holds the two Kalibr files.
    "docker/sim/config_sim_only/openvins_estimator_config.yaml":
        lambda d: gen_openvins_estimator_config(d),
}



def verify_consistency(root: Path) -> int:
    """Cross-check the EMITTED files against each other.

    --check only proves the files match the spec. This proves the ORB-SLAM3
    baseline and the OpenVINS config state the SAME sensor assumptions, which is
    what makes a comparison between them fair. It parses the files rather than
    the spec, so it would also catch a bug in the generator itself.
    """
    import re as _re

    def num(txt, pat):
        m = _re.search(pat, txt)
        return float(m.group(1)) if m else None

    imu_chain = (root / "docker/sim/config_sim_only/kalibr_imu_chain.yaml").read_text()
    imucam = (root / "docker/sim/config_sim_only/kalibr_imucam_chain.yaml").read_text()
    orb = (root / "docker/sim/config_sim_only/orbslam3_d455_stereo_inertial.yaml").read_text()
    sdf = (root / "docker/sim/models/d455/model.sdf").read_text()

    ok = True
    print("IMU noise -- OpenVINS/Kalibr vs ORB-SLAM3 (both continuous densities)")
    pairs = [
        ("accel noise density", r"accelerometer_noise_density:\s*([0-9.eE+-]+)",
         r"IMU\.NoiseAcc:\s*([0-9.eE+-]+)"),
        ("gyro noise density", r"gyroscope_noise_density:\s*([0-9.eE+-]+)",
         r"IMU\.NoiseGyro:\s*([0-9.eE+-]+)"),
        ("accel random walk", r"accelerometer_random_walk:\s*([0-9.eE+-]+)",
         r"IMU\.AccWalk:\s*([0-9.eE+-]+)"),
        ("gyro random walk", r"gyroscope_random_walk:\s*([0-9.eE+-]+)",
         r"IMU\.GyroWalk:\s*([0-9.eE+-]+)"),
    ]
    for label, pk, po in pairs:
        a, b = num(imu_chain, pk), num(orb, po)
        same = (a is not None and b is not None and abs(a - b) <= 1e-12 * max(1.0, abs(a)))
        ok &= same
        print(f"  {label:<22} openvins={a!s:<12} orbslam3={b!s:<12} "
              f"{'MATCH' if same else 'MISMATCH'}")

    rate_k = num(imu_chain, r"update_rate:\s*([0-9.]+)")
    rate_o = num(orb, r"IMU\.Frequency:\s*([0-9.]+)")
    same = rate_k == rate_o
    ok &= same
    print(f"  {'imu rate':<22} openvins={rate_k!s:<12} orbslam3={rate_o!s:<12} "
          f"{'MATCH' if same else 'MISMATCH'}")

    print("\nGazebo per-sample sigmas derived from those densities")
    d = derived()
    for label, expect, pat in [
        ("accel stddev", d["acc_stddev"], r"<stddev>([0-9.eE+-]+)</stddev>"),
        ]:
        got = num(sdf, pat)
        # the first <stddev> in the SDF is the gyro block; just report both
    gyro_sd = _re.findall(r"<stddev>([0-9.eE+-]+)</stddev>", sdf)
    uniq = sorted({float(v) for v in gyro_sd})
    exp = sorted({round(d["gyr_stddev"], 12), round(d["acc_stddev"], 12)})
    same = len(uniq) == 2 and all(
        any(abs(u - e) <= 1e-9 for e in exp) for u in uniq)
    ok &= same
    print(f"  SDF <stddev> values  {uniq}")
    print(f"  expected             {exp}   {'MATCH' if same else 'MISMATCH'}")
    print(f"  (= density * sqrt({IMU_RATE_HZ:g}) = density * {d['sqrt_rate']:.6f})")

    print("\nExtrinsics -- T_cam_imu (OpenVINS) vs IMU.T_b_c1 (ORB-SLAM3) must be inverses")
    # Parse with a real YAML loader: a regex that strips "-" cannot tell the
    # list marker from a minus sign, and silently mangles negative entries.
    import yaml as _yaml

    def _load_cv_yaml(txt: str):
        """PyYAML-load a file written for OpenCV FileStorage.

        OpenCV's marker is `%YAML:1.0`, which is NOT a valid YAML directive --
        the spec wants `%YAML 1.0`, with a space. PyYAML rejects the colon form
        with "expected alphabetic or numeric character". OpenVINS needs the
        colon form, so strip the directive here rather than weaken the file.
        """
        lines = [l for l in txt.splitlines() if not l.startswith("%YAML")]
        return _yaml.safe_load("\n".join(lines))

    K = _load_cv_yaml(imucam)["cam0"]["T_cam_imu"]
    ot = [float(v) for v in _re.search(r"IMU\.T_b_c1:.*?data:\s*\[([^\]]*)\]",
                                       orb, _re.S).group(1).split(",")]
    O = [ot[i * 4:(i + 1) * 4] for i in range(4)]
    import itertools
    prod = [[sum(K[r][k] * O[k][c] for k in range(4)) for c in range(4)]
            for r in range(4)]
    ident = all(abs(prod[r][c] - (1.0 if r == c else 0.0)) < 1e-6
                for r, c in itertools.product(range(4), repeat=2))
    ok &= ident
    print("  T_cam_imu * T_b_c1 = " + ("identity (MATCH)" if ident
                                       else "NOT identity (MISMATCH)"))
    for row in prod:
        print("    [" + "  ".join(f"{v:7.4f}" for v in row) + "]")

    doc = root / "docker/sim/config_sim_only/D455_SIM_CALIBRATION.md"
    print(f"\nConversion document: {'present' if doc.exists() else 'MISSING'} "
          f"({doc.stat().st_size if doc.exists() else 0} bytes)")
    ok &= doc.exists()

    # ---- OpenVINS estimator config ------------------------------------------
    # Checks the knobs whose WRONG values fail silently rather than loudly.
    est_p = root / "docker/sim/config_sim_only/openvins_estimator_config.yaml"
    print("\nOpenVINS estimator config")
    if not est_p.exists():
        print("  MISSING")
        ok = False
    else:
        est = est_p.read_text()
        g = num(est, r"gravity_mag:\s*([0-9.]+)")
        tf = num(est, r"track_frequency:\s*([0-9.]+)")
        checks = [
            ("gravity_mag matches spec and the patched world",
             g is not None and abs(g - GRAVITY_MAG) < 1e-9, f"{g} vs {GRAVITY_MAG}"),
            # At or below the camera rate, ov_msckf's callback_stereo discards
            # frames with no message at all, which would break the
            # frames-processed == frames-published check.
            # The bound is 1/min_inter_frame_gap, NOT the average rate. Checking
            # against the average would pass 31.0, which drops every other frame.
            ("track_frequency above 1/min_frame_gap (not just the avg rate)",
             tf is not None and tf > MIN_TRACK_FREQ_HZ,
             f"{tf} > {MIN_TRACK_FREQ_HZ:.3f} (gap {MIN_FRAME_GAP_S*1e3:.1f} ms)"),
            ("calib_cam_extrinsics frozen",
             "calib_cam_extrinsics: false" in est, ""),
            ("calib_cam_intrinsics frozen",
             "calib_cam_intrinsics: false" in est, ""),
            # Measured at exactly 0.000000 ms by check_stereo_sync.py, so there
            # is no offset to estimate.
            ("calib_cam_timeoffset frozen",
             "calib_cam_timeoffset: false" in est, ""),
            ("static init only (init_dyn_use false)",
             "init_dyn_use: false" in est, ""),
            # Without this there is no covariance on disk and NEES cannot be
            # computed after the fact at all.
            ("save_total_state enabled (needed for NEES)",
             "save_total_state: true" in est, ""),
            ("references the generated Kalibr files",
             'relative_config_imu: "kalibr_imu_chain.yaml"' in est
             and 'relative_config_imucam: "kalibr_imucam_chain.yaml"' in est, ""),
            # OpenVINS parses these with cv::FileStorage.
            ("starts with the %YAML:1.0 directive",
             est.startswith("%YAML:1.0"), ""),
        ]
        for name, good, detail in checks:
            print(f"  {'MATCH ' if good else 'FAIL  '} {name}"
                  + (f"   [{detail}]" if detail else ""))
            ok &= bool(good)
        # The Kalibr files OpenVINS will read must sit beside it, because
        # relative_config_* are resolved relative to the estimator config.
        for sib in ("kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
            there = (est_p.parent / sib).exists()
            print(f"  {'MATCH ' if there else 'FAIL  '} {sib} present beside it")
            ok &= there

    print("\n" + ("ALL CONSISTENT -- baseline and OpenVINS share identical sensor assumptions"
                  if ok else "INCONSISTENCY FOUND"))
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="write the generated files")
    ap.add_argument("--check", action="store_true", help="verify files match the spec")
    ap.add_argument("--root", default=".")
    ap.add_argument("--verify", action="store_true",
                    help="cross-check the emitted ORB-SLAM3 and OpenVINS configs "
                         "against each other and against the SDF")
    args = ap.parse_args()
    d = derived()
    root = Path(args.root)

    if args.verify:
        return verify_consistency(root)

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
