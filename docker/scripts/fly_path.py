#!/usr/bin/env python3
"""Fly a short scripted path with PX4 offboard control (run INSIDE the sim container).

Drives the x500_depth through a path chosen to be informative for RGB-D SLAM:
climb, then a horizontal rectangle with the yaw slewed along each leg, so the
trajectory contains both translation and rotation (a pure straight line makes
ATE look flattering and barely exercises the backend).

Setpoints go over uXRCE-DDS via px4_msgs, so the Micro XRCE-DDS agent must be
running (bringup_sim.sh starts it).

    ./fly_path.py                       # auto-detect the PX4 namespace
    ./fly_path.py --side 6 --alt 2.5
    ./fly_path.py --namespace px4_1 --target-system 2
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

SETPOINT_HZ = 20.0


def px4_qos() -> QoSProfile:
    """PX4's uXRCE-DDS bridge publishes BEST_EFFORT/TRANSIENT_LOCAL with depth 5."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class ScriptedFlight(Node):
    def __init__(self, ns: str, target_system: int, side: float, alt: float,
                 leg_time: float, settle_time: float):
        super().__init__("scripted_flight")
        self.declare_parameter("use_sim_time", True)

        self.ns = ns
        self.target_system = target_system
        self.side = side
        self.alt = alt
        self.leg_time = leg_time
        self.settle_time = settle_time

        prefix = f"/{ns}" if ns else ""
        qos = px4_qos()
        self.pub_mode = self.create_publisher(
            OffboardControlMode, f"{prefix}/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f"{prefix}/fmu/in/vehicle_command", qos)
        self.create_subscription(
            VehicleStatus, f"{prefix}/fmu/out/vehicle_status", self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, f"{prefix}/fmu/out/vehicle_local_position",
            self._on_pos, qos)

        self.status: VehicleStatus | None = None
        self.pos: VehicleLocalPosition | None = None
        self.counter = 0
        self.phase = "wait_for_fmu"
        self.phase_ticks = 0
        self.leg_index = 0
        self.done = False

        # NED waypoints. PX4 local frame is North-East-DOWN, so "up" is negative z.
        s, z = self.side, -abs(self.alt)
        self.legs = [
            # (north, east, down, yaw)
            (0.0, 0.0, z, 0.0),
            (s,   0.0, z, 0.0),
            (s,   s,   z, math.pi / 2),
            (0.0, s,   z, math.pi),
            (0.0, 0.0, z, -math.pi / 2),
            (0.0, 0.0, z, 0.0),
        ]

        self.timer = self.create_timer(1.0 / SETPOINT_HZ, self._tick)
        self.get_logger().info(
            f"namespace={prefix or '<root>'} target_system={target_system} "
            f"side={side} m alt={alt} m leg_time={leg_time}s")

    # --- callbacks -----------------------------------------------------------
    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        self.pos = msg

    # --- helpers -------------------------------------------------------------
    def _stamp(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_cmd(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._stamp()
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f"param{i}", float(params.get(f"param{i}", 0.0)))
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _heartbeat(self) -> None:
        m = OffboardControlMode()
        m.timestamp = self._stamp()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_mode.publish(m)

    def _setpoint(self, n: float, e: float, d: float, yaw: float) -> None:
        sp = TrajectorySetpoint()
        sp.timestamp = self._stamp()
        sp.position = [float(n), float(e), float(d)]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def _goto_phase(self, phase: str) -> None:
        self.get_logger().info(f"phase: {self.phase} -> {phase}")
        self.phase = phase
        self.phase_ticks = 0

    # --- state machine -------------------------------------------------------
    def _tick(self) -> None:
        if self.done:
            return
        self.phase_ticks += 1
        self.counter += 1
        t = self.phase_ticks / SETPOINT_HZ

        if self.phase == "wait_for_fmu":
            # Never command anything before the FMU is actually talking, or the
            # arm/offboard commands are silently dropped.
            if self.status is not None and self.pos is not None:
                self._goto_phase("stream_setpoints")
            elif t > 60:
                self.get_logger().error(
                    "no VehicleStatus/VehicleLocalPosition after 60 s. Is "
                    "MicroXRCEAgent running and is the namespace right?")
                self.done = True
            return

        # From here on the offboard heartbeat must never stop.
        self._heartbeat()
        target = self.legs[min(self.leg_index, len(self.legs) - 1)]

        if self.phase == "stream_setpoints":
            # PX4 requires a setpoint stream BEFORE it will accept OFFBOARD.
            self._setpoint(0.0, 0.0, 0.0, 0.0)
            if t > 1.5:
                self._goto_phase("set_offboard")

        elif self.phase == "set_offboard":
            self._setpoint(0.0, 0.0, 0.0, 0.0)
            self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           param1=1.0, param2=6.0)   # custom mode = OFFBOARD
            self._goto_phase("arm")

        elif self.phase == "arm":
            self._setpoint(0.0, 0.0, 0.0, 0.0)
            if self.phase_ticks % int(SETPOINT_HZ) == 1:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                               param1=1.0)
            armed = (self.status is not None and
                     self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            if armed:
                self.get_logger().info("armed")
                self._goto_phase("climb")
            elif t > 20:
                self.get_logger().error("failed to arm within 20 s")
                self.done = True

        elif self.phase == "climb":
            self._setpoint(0.0, 0.0, -abs(self.alt), 0.0)
            reached = (self.pos is not None and
                       self.pos.z < -(abs(self.alt) - 0.3))
            if reached and t > self.settle_time:
                self.get_logger().info(f"reached {self.alt} m; starting path")
                self.leg_index = 0
                self._goto_phase("fly")
            elif t > 40:
                self.get_logger().warning("climb timed out; flying anyway")
                self._goto_phase("fly")

        elif self.phase == "fly":
            self._setpoint(*target)
            if t > self.leg_time:
                self.leg_index += 1
                if self.leg_index >= len(self.legs):
                    self._goto_phase("land")
                else:
                    self.get_logger().info(
                        f"leg {self.leg_index}/{len(self.legs)} -> "
                        f"N={self.legs[self.leg_index][0]:.1f} "
                        f"E={self.legs[self.leg_index][1]:.1f}")
                    self.phase_ticks = 0

        elif self.phase == "land":
            self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if t > 12:
                self.get_logger().info("scripted path complete")
                self.done = True


def autodetect_namespace() -> tuple[str, int]:
    """Find the PX4 topic namespace. SITL with -i N uses 'px4_N'."""
    import subprocess
    try:
        out = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                             text=True, timeout=30).stdout
    except Exception:
        return "", 1
    for line in out.splitlines():
        line = line.strip()
        if line.endswith("/fmu/in/trajectory_setpoint"):
            parts = [p for p in line.split("/") if p]
            if parts[0] == "fmu":
                return "", 1                      # root namespace -> sys id 1
            ns = parts[0]
            if ns.startswith("px4_"):
                try:
                    inst = int(ns.split("_", 1)[1])
                    return ns, inst + 1           # PX4 -i N  =>  MAV_SYS_ID N+1
                except ValueError:
                    pass
            return ns, 1
    return "", 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--namespace", default=None,
                    help="PX4 topic namespace, e.g. px4_1 (default: autodetect)")
    ap.add_argument("--target-system", type=int, default=None,
                    help="MAVLink target_system (default: derived from namespace)")
    ap.add_argument("--side", type=float, default=5.0, help="rectangle side [m]")
    ap.add_argument("--alt", type=float, default=2.0, help="altitude AGL [m]")
    ap.add_argument("--leg-time", type=float, default=12.0, help="seconds per leg")
    ap.add_argument("--settle-time", type=float, default=3.0,
                    help="seconds to hold after the climb")
    args = ap.parse_args()

    ns, derived_sys = (args.namespace, 1) if args.namespace else autodetect_namespace()
    if args.namespace and args.target_system is None:
        if ns.startswith("px4_"):
            try:
                derived_sys = int(ns.split("_", 1)[1]) + 1
            except ValueError:
                derived_sys = 1
    target_system = args.target_system if args.target_system is not None else derived_sys
    print(f"fly_path: namespace={ns or '<root>'} target_system={target_system}")

    rclpy.init()
    node = ScriptedFlight(ns, target_system, args.side, args.alt,
                          args.leg_time, args.settle_time)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
