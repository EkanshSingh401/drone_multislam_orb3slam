#!/usr/bin/env python3
"""VIO fault injector (overnight Stage 1c): relays OpenVINS odometry to the PX4
vision bridge and, at a chosen time, corrupts it.

    in   /ov_msckf/odomimu               (the real estimate; planner, mapper and
                                           the sim-only GT watchdog keep using it)
    out  /ov_msckf/odomimu_px4           (what openvins_to_px4 forwards to PX4)

Parameters
    mode      none | dropout | drift | jump
    t_start   seconds after the drone is first above `armed_height` (airborne)
    duration  dropout length / drift duration [s]
    drift     drift rate [m/s] along OpenVINS +x (drift mode)
    jump      step offset [m] along OpenVINS +x (jump mode; held from t_start on)
Logs one line per state change: FAULT <mode> start|end t=<sim time>.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


class Injector(Node):
    def __init__(self):
        super().__init__("vio_fault_injector", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.mode = self.declare_parameter("mode", "none").value
        self.t_start = float(self.declare_parameter("t_start", 20.0).value)
        self.duration = float(self.declare_parameter("duration", 5.0).value)
        self.drift = float(self.declare_parameter("drift", 0.5).value)
        self.jump = float(self.declare_parameter("jump", 2.0).value)
        self.armed_height = float(self.declare_parameter("armed_height", 1.2).value)
        self.pub = self.create_publisher(Odometry, "/ov_msckf/odomimu_px4", 50)
        self.create_subscription(Odometry, "/ov_msckf/odomimu", self.cb, 50)
        self.t_air = None
        self.state = "idle"

    def log(self, what, t):
        print(f"FAULT {self.mode} {what} t={t:.2f}", flush=True)

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t_air is None and m.pose.pose.position.z > self.armed_height:
            self.t_air = t
        rel = (t - self.t_air - self.t_start) if self.t_air is not None else -1.0
        active = self.mode != "none" and rel >= 0.0 and (self.mode == "jump" or rel <= self.duration)
        if active and self.state == "idle":
            self.state = "active"; self.log("start", t)
        if not active and self.state == "active":
            self.state = "done"; self.log("end", t)
        if active and self.mode == "dropout":
            return
        if active and self.mode == "drift":
            m.pose.pose.position.x += self.drift * rel
        if active and self.mode == "jump":
            m.pose.pose.position.x += self.jump
        self.pub.publish(m)


def main():
    rclpy.init()
    n = Injector()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
