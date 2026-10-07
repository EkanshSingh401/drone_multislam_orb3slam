#!/usr/bin/env python3
"""VIO health gate (overnight Stage 1c): sits between the VIO stream and the PX4
vision bridge and STOPS forwarding when the stream is not self-consistent, so
PX4 sees a vision dropout (which it fails safe on: dead-reckon, then position
invalid -> failsafe descent) instead of fusing a jump.

    in   /ov_msckf/odomimu_px4   (OpenVINS, possibly corrupted by the sim fault injector)
    out  /ov_msckf/odomimu_gated (what openvins_to_px4 forwards to PX4)

Checks per message, against the previous FORWARDED one:
  * position step vs the step predicted from the reported velocity:
        | dp - R v dt | > max_step_err  -> latched OPEN (stop forwarding)
  * stamp regression / gap > max_gap  -> reset the reference (no comparison)
Once open, the gate stays open (no automatic re-closing): an estimate that has
jumped is not trusted again in this flight; PX4's failsafe takes over.
LIMITATION: a slow drift that is consistent with its own velocity passes; that
needs a second, independent sensor (see PATCHES / DECISIONS).
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


def rot(q, v):
    w, x, y, z = q
    # rotate v by unit quaternion (w, x, y, z)
    ux, uy, uz = x, y, z
    t = (2 * (uy * v[2] - uz * v[1]), 2 * (uz * v[0] - ux * v[2]), 2 * (ux * v[1] - uy * v[0]))
    return (v[0] + w * t[0] + uy * t[2] - uz * t[1], v[1] + w * t[1] + uz * t[0] - ux * t[2], v[2] + w * t[2] + ux * t[1] - uy * t[0])


class Gate(Node):
    def __init__(self):
        super().__init__("vio_health_gate", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.max_step_err = float(self.declare_parameter("max_step_err", 0.15).value)  # m per message
        self.max_gap = float(self.declare_parameter("max_gap", 0.5).value)
        self.pub = self.create_publisher(Odometry, "/ov_msckf/odomimu_gated", 50)
        self.create_subscription(Odometry, self.declare_parameter("in_topic", "/ov_msckf/odomimu_px4").value, self.cb, 50)
        self.prev = None
        self.open = False
        # Commanded hold (not latched): the executor sets it just before touchdown so
        # PX4 lands on IMU dead-reckoning -- OpenVINS diverges on ground contact and,
        # fused, keeps PX4's land detector from ever confirming (overnight Stage 1b).
        self.hold = False
        self.create_subscription(Bool, "/offboard_executor/vio_hold", self.on_hold, 10)

    def on_hold(self, m):
        if m.data != self.hold:
            print(f"GATE HOLD {'on' if m.data else 'off'} (commanded)", flush=True)
        self.hold = m.data
        if not m.data:
            self.prev = None  # re-reference after a commanded hold

    def cb(self, m):
        if self.open or self.hold:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        v = m.twist.twist.linear  # OpenVINS: body-frame velocity
        cur = (t, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (v.x, v.y, v.z))
        if self.prev is not None:
            dt = t - self.prev[0]
            if 0.0 < dt <= self.max_gap:
                vw0 = rot(self.prev[2], self.prev[3])
                vw1 = rot(cur[2], cur[3])
                pred = [self.prev[1][i] + 0.5 * (vw0[i] + vw1[i]) * dt for i in range(3)]
                err = math.dist(pred, cur[1])
                if err > self.max_step_err:
                    self.open = True
                    print(f"GATE OPEN t={t:.2f} step_err={err:.3f} m > {self.max_step_err} (dt={dt:.3f}): stopped forwarding VIO to PX4", flush=True)
                    return
        self.prev = cur
        self.pub.publish(m)


def main():
    rclpy.init()
    n = Gate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
