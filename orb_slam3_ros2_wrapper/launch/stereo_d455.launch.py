#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Launch ORB-SLAM3 in STEREO-ONLY mode on the D455-mirror rig.

This is the bisection against stereo_inertial.launch.py. Stereo-only ORB-SLAM3
is constructed with System::STEREO, so it consumes no IMU and runs no
visual-inertial bundle adjustment -- no VIBA 1, no VIBA 2, and therefore no
retroactive scale/gravity correction. If the mid-flight error discontinuity seen
in the stereo-inertial runs is VIBA, it must be absent here.

Deliberately NOT the wrapper's own stereo.launch.py: that one namespaces the
node and rewrites its parameter file through nav2_common.RewrittenYaml, which
would change the node name and the topic namespace at the same time as the
sensor mode. A bisection has to vary one thing.

Paths come from the environment, as in stereo_inertial.launch.py:
  ORB_SLAM3_ROOT_DIR  tree holding Vocabulary/ORBvoc.txt
  ORB_SLAM3_SETTINGS_STEREO  camera settings yaml (no IMU block)
  STEREO_ROS_PARAMS   ROS parameter file for the node
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_CFG = "/opt/config_sim_only"

# Must match the name the node passes to SlamNodeBase, because the parameter
# file is keyed by node name; a mismatch loads nothing and leaves the topic
# names at their compiled defaults ("left/image_raw").
NODE_NAME = "ORB_SLAM3_STEREO_ROS2"


def generate_launch_description():
    def nodes(context):
        use_sim_time = (
            LaunchConfiguration("use_sim_time").perform(context).strip().lower()
            in ("true", "1", "yes", "on"))

        orb_root = os.environ.get(
            "ORB_SLAM3_ROOT_DIR",
            os.path.join(os.path.expanduser("~"), "ws_offboard_control", "src", "ORB_SLAM3"))
        vocab = os.environ.get(
            "ORB_SLAM3_VOCABULARY",
            os.path.join(orb_root, "Vocabulary", "ORBvoc.txt"))
        settings = os.environ.get(
            "ORB_SLAM3_SETTINGS_STEREO",
            os.path.join(SIM_CFG, "orbslam3_d455_stereo.yaml"))
        ros_params = os.environ.get(
            "STEREO_ROS_PARAMS",
            os.path.join(SIM_CFG, "stereo_ros_params.yaml"))

        return [Node(
            package="orb_slam3_ros2_wrapper",
            executable="stereo",
            name=NODE_NAME,
            output="screen",
            arguments=[vocab, settings],
            # use_sim_time as its own dict, not merely declared: see
            # rgbd.launch.py, where a declared-but-unapplied argument left the
            # SLAM node on the wall clock and silently invalidated evaluation.
            parameters=[ros_params, {"use_sim_time": use_sim_time}],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
