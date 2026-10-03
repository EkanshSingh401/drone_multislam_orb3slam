#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Launch ORB-SLAM3 in STEREO-INERTIAL mode.

Paths come from the environment so the same file works in the container and on a
host checkout:
  ORB_SLAM3_ROOT_DIR  tree holding Vocabulary/ORBvoc.txt
  ORB_SLAM3_SETTINGS  camera/IMU settings yaml
  SI_ROS_PARAMS       ROS parameter file for the node

use_sim_time is applied as its own parameter dict, not merely declared -- see
rgbd.launch.py, where the declared-but-unused argument left the SLAM node on the
wall clock and silently invalidated evaluation.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_CFG = "/opt/config_sim_only"


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
            "ORB_SLAM3_SETTINGS",
            os.path.join(SIM_CFG, "orbslam3_d455_stereo_inertial.yaml"))
        ros_params = os.environ.get(
            "SI_ROS_PARAMS",
            os.path.join(SIM_CFG, "stereo_inertial_ros_params.yaml"))

        return [Node(
            package="orb_slam3_ros2_wrapper",
            executable="stereo_inertial",
            name="ORB_SLAM3_STEREO_INERTIAL_ROS2",
            output="screen",
            arguments=[vocab, settings],
            parameters=[ros_params, {"use_sim_time": use_sim_time}],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
