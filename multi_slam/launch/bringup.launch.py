#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    gz_bridge_config = LaunchConfiguration("gz_bridge_config")
    use_ros1_bridge = LaunchConfiguration("use_ros1_bridge")

    declare_use_ros1_bridge = DeclareLaunchArgument(
        "use_ros1_bridge",
        default_value="false",
        description="Launch ros1_bridge (only needed to bring COVINS ROS 1 "
                    "pose/TF back into ROS 2; not required for ORB-SLAM3 -> COVINS)",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock for ros_gz_bridge",
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="uav_1",
        description="Namespace for static TF launch",
    )

    declare_gz_bridge_config = DeclareLaunchArgument(
        "gz_bridge_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("multi_slam"), "config", "gz_bridge.yaml"]
        ),
        description="Path to ros_gz_bridge YAML config",
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            ["config_file:=", gz_bridge_config],
            "-p",
            ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    )

    # ros1_bridge is OPTIONAL: it is only needed to pull COVINS's ROS 1
    # pose/TF output back into ROS 2. The ORB-SLAM3 -> COVINS path does NOT
    # use it (the frontend links covins_comm and opens its own TCP socket to
    # the backend), so the bridge is off by default. Unconditionally launching
    # it makes bringup fail on any system without ros1_bridge installed.
    ros1_bridge = ExecuteProcess(
        condition=IfCondition(use_ros1_bridge),
        cmd=["ros2", "run", "ros1_bridge", "parameter_bridge"],
        output="screen",
    )

    static_frames = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("multi_slam"), "launch", "static_frames.launch.py"]
            )
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Pass robot_namespace and use_sim_time through, otherwise the SLAM node
    # falls back to its own defaults and silently runs on wall clock while the
    # rest of the stack is on /clock.
    orb_slam3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orb_slam3_ros2_wrapper"), "launch", "rgbd.launch.py"]
            )
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_robot_namespace,
            declare_gz_bridge_config,
            declare_use_ros1_bridge,
            gz_bridge,
            ros1_bridge,
            static_frames,
            orb_slam3,
        ]
    )
