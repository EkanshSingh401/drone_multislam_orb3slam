#!/usr/bin/env python3
"""Dump the IMU pose block of /openvins/joint_covariance (PATCHES s55). Run INSIDE the
sim container alongside a serial replay; stops after --idle seconds without messages.

    joint_cov_dump.py <out.txt> [--idle 20]

Each row: stamp, then the 6x6 [JPL-quat(3) pos(3)] block of the IMU state, row-major.
The publisher keeps depth 2, so a slow subscriber can miss messages; the row count
is reported and the NEES uses only stamps present here.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from active_slam_msgs.msg import JointCovariance

out, idle = sys.argv[1], float(sys.argv[sys.argv.index("--idle") + 1]) if "--idle" in sys.argv else 20.0
rows, last = [], [time.time()]


def cb(m):
    n = int(m.dim)
    b = next(x for x in m.blocks if x.type == "imu")
    C = np.asarray(m.covariance).reshape(n, n)[b.index:b.index + 6, b.index:b.index + 6]
    rows.append(np.r_[m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, C.ravel()])
    last[0] = time.time()


rclpy.init()
node = Node("joint_cov_dump")
node.create_subscription(JointCovariance, "/openvins/joint_covariance", cb,
                         QoSProfile(depth=1000, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
while rclpy.ok() and (not rows or time.time() - last[0] < idle) and time.time() - last[0] < 600:
    rclpy.spin_once(node, timeout_sec=0.5)
np.savetxt(out, np.array(rows), fmt="%.12g")
print(f"joint_cov_dump: {len(rows)} messages -> {out}")
