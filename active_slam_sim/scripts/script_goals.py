#!/usr/bin/env python3
"""script_goals.py (day 5 step 4b): scripted low-rotation trajectory for the offboard executor.

Stands in for the exploration planner (cl_flight.sh TYPE=script): publishes a MOVING goal on
/exploration_planner/goal at 20 Hz, which the executor follows with its own speed / yaw-rate limits
(0.5 m/s, 0.6 rad/s), so every segment below stays inside those limits and is tracked as scripted.
The script starts when the executor enters 'explore' (first log line with phase explore) and is
expressed relative to the takeoff pose: f = forward along the takeoff heading, l = left, yaw relative.
Segments (name, duration s, kind, args):
  hover   : hold the current target
  line    : move the target to (f, l) at constant speed, heading held
  rotate  : rotate in place at a constant rate (rad/s) for the duration
Publishes ~/segment (String) with the active segment name each second and at every change, so the
analysis can bin NEES by segment. When the script ends it publishes status 'done' on
/exploration_planner/status (the executor then flies home and lands)."""
import json, math
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SPEED = 0.3   # m/s on straight legs
SCRIPT = [("hover0", 20, "hover", None),
          ("line_f", None, "line", (2.5, 0.0)), ("hover1", 10, "hover", None),
          ("line_l", None, "line", (2.5, 2.5)), ("line_b", None, "line", (-2.5, 2.5)),
          ("line_r", None, "line", (-2.5, -2.5)), ("line_f2", None, "line", (2.5, -2.5)),
          ("line_home", None, "line", (0.0, 0.0)), ("hover2", 20, "hover", None),
          ("rot_0.05", 60, "rotate", 0.05), ("hover3", 10, "hover", None),
          ("rot_0.15", 40, "rotate", 0.15), ("hover4", 10, "hover", None),
          ("rot_0.4", 16, "rotate", 0.4), ("hover5", 10, "hover", None)]


class ScriptGoals(Node):
    def __init__(self):
        super().__init__("script_goals", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(PoseStamped, "/exploration_planner/goal", 10)
        self.pub_st = self.create_publisher(String, "/exploration_planner/status", 10)
        self.pub_seg = self.create_publisher(String, "~/segment", 10)
        self.create_subscription(String, "/offboard_executor/log", self.on_log, 10)
        self.create_subscription(Odometry, "/ov_msckf/odomimu", self.on_odom, 10)
        self.odom = None; self.t0 = None; self.base = None
        self.seg = -1; self.seg_t0 = None; self.target = None; self.start = None; self.last_seg_pub = 0.0
        self.create_timer(0.05, self.tick)

    def now(self): return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m): self.odom = m

    def on_log(self, m):
        if self.t0 is None and json.loads(m.data).get("phase") == "explore" and self.odom is not None:
            p, o = self.odom.pose.pose.position, self.odom.pose.pose.orientation
            yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            self.base = (p.x, p.y, yaw); self.t0 = self.now(); self.target = [p.x, p.y, yaw]
            self.get_logger().info(f"script starts at {self.base}")

    def world(self, f, l):
        x0, y0, h = self.base
        return x0 + f * math.cos(h) - l * math.sin(h), y0 + f * math.sin(h) + l * math.cos(h)

    def next_seg(self, t):
        self.seg += 1; self.seg_t0 = t; self.start = list(self.target)
        if self.seg < len(SCRIPT):
            self.get_logger().info(f"segment {SCRIPT[self.seg][0]}")
            self.pub_seg.publish(String(data=SCRIPT[self.seg][0]))

    def tick(self):
        if self.t0 is None:
            return
        t = self.now()
        if self.seg < 0:
            self.next_seg(t)
        if self.seg >= len(SCRIPT):
            self.pub_st.publish(String(data="done"))
            return
        name, dur, kind, arg = SCRIPT[self.seg]
        el = t - self.seg_t0
        if kind == "hover":
            done = el >= dur
        elif kind == "line":
            gx, gy = self.world(*arg); sx, sy = self.start[0], self.start[1]
            L = math.hypot(gx - sx, gy - sy); s = min(1.0, SPEED * el / L) if L > 1e-6 else 1.0
            self.target[0], self.target[1] = sx + s * (gx - sx), sy + s * (gy - sy)
            done = s >= 1.0 and el >= L / SPEED + 2.0     # 2 s settle at the corner
        else:
            self.target[2] = self.start[2] + arg * min(el, dur)
            done = el >= dur
        q = PoseStamped(); q.header.stamp = self.get_clock().now().to_msg(); q.header.frame_id = "global"
        q.pose.position.x, q.pose.position.y, q.pose.position.z = self.target[0], self.target[1], 1.5
        y = math.atan2(math.sin(self.target[2]), math.cos(self.target[2]))
        q.pose.orientation.z, q.pose.orientation.w = math.sin(y / 2), math.cos(y / 2)
        self.pub.publish(q)
        if t - self.last_seg_pub >= 1.0:
            self.last_seg_pub = t; self.pub_seg.publish(String(data=name))
        if done:
            self.next_seg(t)


def main():
    rclpy.init()
    n = ScriptGoals()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
