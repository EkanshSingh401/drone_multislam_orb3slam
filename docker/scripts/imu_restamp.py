#!/usr/bin/env python3
"""Republish Gazebo's IMU with the stamp corrected for the half-step lag (PATCHES s56).

    imu_restamp.py [--lag-s 0.002] [--in /camera/imu_gz] [--out /camera/imu]

Gazebo stamps each IMU sample with the end of its 4 ms physics step while the
gyro describes the middle of the step (measured 1.89-1.96 ms, constant over the
4,4,4,8 ms stamp pattern). The bridge publishes the raw sample on /camera/imu_gz;
this node moves the stamp back by LAG_S and republishes on /camera/imu, which is
what the estimators and the recorder consume. LAG_S = PHYSICS_STEP_S / 2 in
docker/sim/tools/gen_d455_sim.py (IMU_STAMP_LAG_S); test_gen_d455_sim.py checks
the two agree. Message contents are otherwise untouched.
"""
import argparse

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

LAG_S = 0.002

ap = argparse.ArgumentParser()
ap.add_argument("--lag-s", type=float, default=LAG_S)
ap.add_argument("--in", dest="topic_in", default="/camera/imu_gz")
ap.add_argument("--out", dest="topic_out", default="/camera/imu")
a, _ = ap.parse_known_args()
LAG_NS = int(round(a.lag_s * 1e9))


def main():
    rclpy.init()
    node = Node("imu_restamp")
    qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Imu, a.topic_out, qos)

    def cb(m):
        ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec - LAG_NS
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        pub.publish(m)

    node.create_subscription(Imu, a.topic_in, cb, qos)
    node.get_logger().info(f"{a.topic_in} -> {a.topic_out}, stamp -{a.lag_s * 1e3:.3f} ms")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass  # SIGINT/SIGTERM from bringup_sim.sh --stop


if __name__ == "__main__":
    main()
