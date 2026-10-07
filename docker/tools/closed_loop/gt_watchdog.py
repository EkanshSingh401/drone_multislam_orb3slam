#!/usr/bin/env python3
"""SIM-ONLY safety pilot for the closed-loop harness (PATCHES s64) -- not part of the product.
Force-disarms PX4 if, while armed, OpenVINS (base_link) and Gazebo ground truth disagree by
> max_err_m after a rigid alignment fixed at takeoff, or ground truth leaves the scene box.
Logs a TERMINATED line; the flight then counts as a failure."""
import math, sys, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleStatus
P = (0.11448, 0.0051, 0.23026)
class W(Node):
    def __init__(s):
        super().__init__("gt_watchdog", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=5)
        s.max_err = 1.0; s.box = (-3.6, 5.1, -5.1, 5.1, 3.5)  # scene interior x, x, y, y, zmax (Gazebo)
        s.gt = None; s.ov = None; s.armed = False; s.align = None; s.done = False
        s.create_subscription(TFMessage, "/ground_truth/pose_info", lambda m: setattr(s, "gt", m.transforms[0].transform.translation), 10)
        s.create_subscription(Odometry, "/ov_msckf/odomimu", s.on_ov, 10)
        s.create_subscription(VehicleStatus, "/px4_1/fmu/out/vehicle_status_v1", lambda m: setattr(s, "armed", m.arming_state == VehicleStatus.ARMING_STATE_ARMED), q)
        s.pub = s.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", q)
        s.create_timer(0.1, s.tick)
    def on_ov(s, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        s.ov = (p.x - (math.cos(yaw) * P[0] - math.sin(yaw) * P[1]), p.y - (math.sin(yaw) * P[0] + math.cos(yaw) * P[1]), p.z - P[2])
    def tick(s):
        if s.done or s.gt is None or s.ov is None: return
        g = (s.gt.x, s.gt.y, s.gt.z)
        if s.align is None:  # fix the OpenVINS->Gazebo yaw+offset once, on the ground before arming
            if not s.armed:
                s.align_pts = getattr(s, "align_pts", []) + [(g, s.ov)]
                return
            g0, o0 = s.align_pts[-1]
            # yaw: OpenVINS x was mapped to Gazebo -x in s64 tests; estimate from heading is unreliable on the ground,
            # so align translation only and compare horizontal DISTANCES from takeoff plus height
            s.align = (g0, o0)
        g0, o0 = s.align
        dg = math.hypot(g[0] - g0[0], g[1] - g0[1]); do = math.hypot(s.ov[0] - o0[0], s.ov[1] - o0[1])
        err = max(abs(dg - do), abs((g[2] - g0[2]) - (s.ov[2] - o0[2])))
        out = not (s.box[0] < g[0] < s.box[1] and s.box[2] < g[1] < s.box[3] and g[2] < s.box[4])
        if s.armed and (err > s.max_err or out):
            m = VehicleCommand(); m.timestamp = int(s.get_clock().now().nanoseconds / 1000)
            m.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM; m.param1 = 0.0; m.param2 = 21196.0
            m.target_system, m.target_component, m.source_system, m.source_component = 2, 1, 1, 1; m.from_external = True
            s.pub.publish(m)
            print(f"TERMINATED t={s.get_clock().now().nanoseconds*1e-9:.2f} err={err:.2f} out_of_box={out} gt={g}", flush=True)
            s.done = True
rclpy.init(); n = W()
while rclpy.ok() and not n.done: rclpy.spin_once(n, timeout_sec=0.1)
rclpy.spin_once(n, timeout_sec=1.0)
