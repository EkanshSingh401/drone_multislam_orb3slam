#!/usr/bin/env python3
"""Offboard executor for closed-loop exploration in sim (PATCHES s64).

PX4 SITL flies on OpenVINS only (EKF2 vision fusion; see cl/ev_setup.sh), so the
PX4 local NED frame IS the OpenVINS global frame converted ENU->NED by the same
convention as active_slam_planner/openvins_to_px4.py:
    position (x, y, z)_OV -> (N, E, D) = (y, x, -z);  yaw_NED = pi/2 - yaw_OV.
Goals from the planner are IMU positions; setpoints are for the vehicle body
(base_link), so the lever arm is removed: p_base = p_imu - R(yaw) * P_BASE_IMU.

Phases: wait_for_fmu -> stream -> offboard -> arm -> climb -> explore -> home ->
land -> done. Explore follows ~/goal of the planner at cruise_speed (straight
line, yaw rate limited) until the planner reports "done" or max_mission_s.
Safety: hold position if the ESDF distance at the drone (nvblox interface slice)
drops below hold_distance; stop exploring beyond geofence_radius from takeoff.
Logs one JSON line per second to stdout (t, phase, OV pose, setpoint, esdf dist).
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLandDetected, VehicleStatus
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String

HZ = 20.0
P_BASE_IMU = (0.12 - 0.00552, 0.0051, 0.242 - 0.01174)  # IMU in base_link (FLU), as ov_prep.py


def px4_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OffboardExecutor(Node):
    def __init__(self):
        super().__init__("offboard_executor", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.ns = self.declare_parameter("px4_ns", "px4_1").value
        self.target_system = int(self.declare_parameter("target_system", 2).value)
        self.height = float(self.declare_parameter("flight_height", 1.5).value)   # IMU z in the OpenVINS frame
        self.speed = float(self.declare_parameter("cruise_speed", 0.5).value)
        self.yaw_rate = float(self.declare_parameter("yaw_rate", 0.6).value)
        self.max_mission = float(self.declare_parameter("max_mission_s", 180.0).value)
        self.hold_dist = float(self.declare_parameter("hold_distance", 0.45).value)
        self.geofence = float(self.declare_parameter("geofence_radius", 8.0).value)
        pre = f"/{self.ns}"
        q = px4_qos()
        self.pub_mode = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", q)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f"{pre}/fmu/in/trajectory_setpoint", q)
        self.pub_cmd = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", q)
        self.pub_log = self.create_publisher(String, "~/log", 10)
        self.pub_vio_hold = self.create_publisher(Bool, "~/vio_hold", 10)
        for s in ("_v1", ""):
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status{s}", self._on_status, q)
            self.create_subscription(VehicleLandDetected, f"{pre}/fmu/out/vehicle_land_detected{s}", self._on_land, q)
        self.land_det = None
        self.create_subscription(Odometry, self.declare_parameter("odom_topic", "/ov_msckf/odomimu").value, self._on_odom, 10)
        self.create_subscription(PoseStamped, self.declare_parameter("goal_topic", "/exploration_planner/goal").value, self._on_goal, 10)
        self.create_subscription(String, self.declare_parameter("status_topic", "/exploration_planner/status").value, self._on_pstatus, 10)
        self.create_subscription(PointCloud2, self.declare_parameter("esdf_topic", "/nvblox_node/static_esdf_pointcloud").value,
                                 self._on_esdf, 2)
        self.status = None
        self.odom = None
        self.goal = None           # (x, y, yaw) OpenVINS frame, IMU position
        self.planner_status = "exploring"
        self.esdf = {}
        self.phase, self.t_phase = "wait_for_fmu", None
        self.sp = None             # current setpoint (x, y, z, yaw) in the OpenVINS frame, IMU position
        self.home = None
        self.t_explore = None
        self.crumbs = []           # flown positions (known-free); home retraces them
        self.td_since = None       # touchdown detection
        self.last_log = 0.0
        self.create_timer(1.0 / HZ, self._tick)

    # ---------------- inputs ----------------
    def _on_status(self, m): self.status = m
    def _on_land(self, m): self.land_det = m
    def _on_odom(self, m): self.odom = m
    def _on_pstatus(self, m): self.planner_status = m.data

    def _on_goal(self, m):
        y = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
        self.goal = (m.pose.position.x, m.pose.position.y, y)

    def _on_esdf(self, m):
        self.esdf = {(round(float(p[0]) / 0.1), round(float(p[1]) / 0.1)): float(p[3])
                     for p in point_cloud2.read_points(m, field_names=("x", "y", "z", "intensity"), skip_nans=True)}

    def esdf_at(self, x, y):
        return self.esdf.get((round(x / 0.1), round(y / 0.1)), float("nan"))

    def ov_pose(self):
        p, o = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        return p.x, p.y, p.z, yaw

    # ---------------- outputs ----------------
    def now(self): return self.get_clock().now().nanoseconds * 1e-9
    def stamp(self): return int(self.get_clock().now().nanoseconds / 1000)

    def cmd(self, command, **p):
        m = VehicleCommand()
        m.timestamp = self.stamp()
        m.command = command
        for i in range(1, 8):
            setattr(m, f"param{i}", float(p.get(f"param{i}", 0.0)))
        m.target_system, m.target_component, m.source_system, m.source_component = self.target_system, 1, 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def heartbeat(self):
        m = OffboardControlMode()
        m.timestamp = self.stamp()
        m.position = True
        self.pub_mode.publish(m)

    def send_sp(self):
        x, y, z, yaw = self.sp
        # IMU position -> body origin, then OpenVINS (ENU-like) -> PX4 NED
        bx = x - (math.cos(yaw) * P_BASE_IMU[0] - math.sin(yaw) * P_BASE_IMU[1])
        by = y - (math.sin(yaw) * P_BASE_IMU[0] + math.cos(yaw) * P_BASE_IMU[1])
        bz = z - P_BASE_IMU[2]
        m = TrajectorySetpoint()
        m.timestamp = self.stamp()
        m.position = [float(by), float(bx), float(-bz)]
        m.yaw = float(wrap(math.pi / 2 - yaw))
        self.pub_sp.publish(m)

    def go(self, phase):
        self.get_logger().info(f"phase {self.phase} -> {phase}")
        self.phase, self.t_phase = phase, self.now()

    def step_toward(self, gx, gy, gz, gyaw):
        x, y, z, yaw = self.sp
        dt = 1.0 / HZ
        dx, dy, dz = gx - x, gy - y, gz - z
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        s = min(1.0, self.speed * dt / d) if d > 1e-6 else 0.0
        dyaw = wrap(gyaw - yaw)
        yaw += max(-self.yaw_rate * dt, min(self.yaw_rate * dt, dyaw))
        self.sp = (x + s * dx, y + s * dy, z + s * dz, wrap(yaw))
        return d < 0.05 and abs(dyaw) < 0.05

    def esdf_guard(self):
        """Hold position if the ESDF distance at the drone is below hold_distance (any airborne phase)."""
        px, py, _, _ = self.ov_pose()
        dist = self.esdf_at(px, py)
        if dist == dist and dist < self.hold_dist:
            self.sp = (px, py, self.height, self.sp[3])
            if not getattr(self, "_held", False):
                self.get_logger().warning(f"ESDF guard: {dist:.2f} m < {self.hold_dist} m, holding")
            self._held = True
            return True
        self._held = False
        return False

    # ---------------- state machine ----------------
    def _tick(self):
        if self.phase == "done":
            return
        t = self.now()
        if self.t_phase is None:
            self.t_phase = t
        tp = t - self.t_phase
        if self.phase == "wait_for_fmu":
            if self.status is not None and self.odom is not None:
                px, py, pz, pyaw = self.ov_pose()
                self.home = (px, py, pz, pyaw)
                self.sp = (px, py, pz, pyaw)
                self.go("stream")
            return
        self.heartbeat()
        self.send_sp()
        armed = self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.phase == "stream" and tp > 1.5:
            self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.go("arm")
        elif self.phase == "arm":
            if int(tp * HZ) % int(HZ) == 1:
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if armed:
                self.go("climb")
            elif tp > 20:
                self.get_logger().error("failed to arm")
                self.go("done")
        elif self.phase == "climb":
            hx, hy, _, hyaw = self.home
            self.step_toward(hx, hy, self.height, hyaw)
            pz = self.ov_pose()[2]
            if pz > self.height - 0.15:          # the DRONE reached height, not just the setpoint
                self.t_explore = t
                self.go("explore")
            elif tp > 40:
                self.get_logger().error(f"climb timed out at z={pz:.2f}; landing")
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.go("land")
        elif self.phase == "explore":
            px, py, pz, _ = self.ov_pose()
            dist = self.esdf_at(px, py)
            far = math.hypot(px - self.home[0], py - self.home[1]) > self.geofence
            if self.planner_status == "done" or t - self.t_explore > self.max_mission or far:
                self.get_logger().info(f"explore ends: planner={self.planner_status} "
                                       f"elapsed={t - self.t_explore:.0f}s far={far}")
                self.go("home")
            elif dist == dist and dist < self.hold_dist:
                self.sp = (px, py, self.height, self.sp[3])  # hold where we are
            elif self.goal is not None:
                self.step_toward(self.goal[0], self.goal[1], self.height, self.goal[2])
            if not self.crumbs or math.hypot(px - self.crumbs[-1][0], py - self.crumbs[-1][1]) > 0.1:
                self.crumbs.append((px, py))
        elif self.phase == "home" and self.esdf_guard():
            pass  # holding: too close to an obstacle
        elif self.phase == "home":
            # Retrace the flown path (known free space): breadcrumbs by position only,
            # yaw held; then align yaw at the takeoff point and land.
            hx, hy, _, hyaw = self.home
            if self.crumbs:
                tx, ty = self.crumbs[-1]
                # face the direction of travel: the forward camera must see where it goes
                # (s64 i2: yaw held, drone translated past a box with the camera turned away,
                # OpenVINS lost track and the watchdog terminated the flight)
                d = math.hypot(tx - self.sp[0], ty - self.sp[1])
                tyaw = math.atan2(ty - self.sp[1], tx - self.sp[0]) if d > 0.3 else self.sp[3]
                if abs(wrap(tyaw - self.sp[3])) > 0.3:      # turn first, then move
                    self.step_toward(self.sp[0], self.sp[1], self.height, tyaw)
                else:
                    self.step_toward(tx, ty, self.height, tyaw)
                if math.hypot(tx - self.sp[0], ty - self.sp[1]) < 0.1:
                    self.crumbs.pop()
            elif self.step_toward(hx, hy, self.height, hyaw):
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.go("land")
            if tp > 120:
                self.get_logger().warning("home timed out; landing here")
                self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.go("land")
        elif self.phase == "land":
            # Touchdown -> disarm. PRIMARY: PX4's land detector (landed / maybe_landed).
            # BACKUP: OpenVINS within 5 cm of the takeoff height and no longer
            # descending -- OpenVINS is accurate up to contact and diverges within ~1 s
            # after it (s64), so the backup must not wait. (The earlier 0.15 m threshold
            # disarmed in the air before contact, which is why PX4's detector never got a
            # chance to report; overnight Stage 1b.)
            _, _, pz, _ = self.ov_pose()
            vz = self.odom.twist.twist.linear.z
            ld = self.land_det
            # below 0.4 m: stop feeding vision so PX4 touches down on IMU dead-reckoning
            self.pub_vio_hold.publish(Bool(data=bool(pz < self.home[2] + 0.4)))
            px4_says = ld is not None and (ld.landed or ld.maybe_landed)
            ov_says = tp > 2.0 and pz < self.home[2] + 0.12 and abs(vz) < 0.15
            if armed and (px4_says or ov_says):
                if self.td_since is None:
                    self.td_since = t
                    self.get_logger().info(f"touchdown (px4 landed={ld.landed if ld else None} "
                                           f"maybe={ld.maybe_landed if ld else None} contact={ld.ground_contact if ld else None}, "
                                           f"ov z={pz:.2f}, by={'px4' if px4_says else 'openvins'}): force disarm")
                self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)
            if not armed and tp > 3.0:
                self.get_logger().info("landed and disarmed")
                self.go("done")
            elif tp > 60:
                self.get_logger().warning("landing timed out")
                self.go("done")
        if t - self.last_log >= 1.0 and self.odom is not None:
            self.last_log = t
            px, py, pz, pyaw = self.ov_pose()
            rec = {"t": round(t, 3), "phase": self.phase, "ov": [round(px, 3), round(py, 3), round(pz, 3), round(pyaw, 3)],
                   "sp": [round(v, 3) for v in self.sp], "esdf": round(self.esdf_at(px, py), 3), "planner": self.planner_status}
            self.pub_log.publish(String(data=json.dumps(rec)))
            print(json.dumps(rec), flush=True)


def main():
    rclpy.init()
    n = OffboardExecutor()
    try:
        while rclpy.ok() and n.phase != "done":
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
