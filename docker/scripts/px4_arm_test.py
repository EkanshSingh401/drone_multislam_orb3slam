#!/usr/bin/env python3
"""Fast arming diagnostic for PX4 over ROS 2 (run INSIDE the sim container).

Answers one question quickly: can PX4 enter OFFBOARD and arm, and if not, which
health flag is blocking it?

Deliberately uses WALL-CLOCK timeouts (use_sim_time is left False here) so it
reports in ~1 minute instead of ~30. fly_path.py runs on sim time, which is
correct for flying but makes a failed arm take half an hour to surface.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

ARMING = {0: "INIT", 1: "DISARMED", 2: "ARMED", 3: "STANDBY_ERROR",
          4: "SHUTDOWN", 5: "IN_AIR_RESTORE"}


def qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


class ArmTest(Node):
    def __init__(self, ns: str = "px4_1", target_system: int = 2):
        super().__init__("px4_arm_test")
        p = f"/{ns}"
        q = qos()
        self.pub_mode = self.create_publisher(OffboardControlMode, f"{p}/fmu/in/offboard_control_mode", q)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f"{p}/fmu/in/trajectory_setpoint", q)
        self.pub_cmd = self.create_publisher(VehicleCommand, f"{p}/fmu/in/vehicle_command", q)
        for suffix in ("_v1", ""):
            self.create_subscription(VehicleStatus, f"{p}/fmu/out/vehicle_status{suffix}", self._st, q)
        self.create_subscription(FailsafeFlags, f"{p}/fmu/out/failsafe_flags", self._ff, q)
        self.status: VehicleStatus | None = None
        self.flags: FailsafeFlags | None = None
        self.target_system = target_system

    def _st(self, m): self.status = m
    def _ff(self, m): self.flags = m

    def _stamp(self) -> int:
        return int(time.time() * 1e6)

    def heartbeat(self):
        m = OffboardControlMode()
        m.timestamp = self._stamp()
        m.position = True
        self.pub_mode.publish(m)
        sp = TrajectorySetpoint()
        sp.timestamp = self._stamp()
        sp.position = [0.0, 0.0, -2.0]
        sp.yaw = 0.0
        self.pub_sp.publish(sp)

    def cmd(self, command: int, **kw):
        m = VehicleCommand()
        m.timestamp = self._stamp()
        m.command = command
        for i in range(1, 8):
            setattr(m, f"param{i}", float(kw.get(f"param{i}", 0.0)))
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 255
        m.source_component = 190
        m.from_external = True
        self.pub_cmd.publish(m)


def blocking_flags(ff: FailsafeFlags) -> list[str]:
    """Report every *_unhealthy / invalid flag that is set."""
    out = []
    for name in dir(ff):
        if name.startswith("_") or name in ("timestamp",):
            continue
        try:
            v = getattr(ff, name)
        except Exception:
            continue
        if isinstance(v, bool) and v:
            out.append(name)
    return out


def main() -> int:
    ns = sys.argv[1] if len(sys.argv) > 1 else "px4_1"
    tgt = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    budget = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
    rclpy.init()
    n = ArmTest(ns, tgt)
    print(f"arm_test: namespace /{ns}, target_system {tgt}")

    # 1. wait for status
    t0 = time.time()
    while time.time() - t0 < 30 and n.status is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.status is None:
        print("arm_test: NO VehicleStatus within 30 s wall -- uXRCE link down?")
        return 2
    print(f"arm_test: arming_state={ARMING.get(n.status.arming_state, n.status.arming_state)} "
          f"nav_state={n.status.nav_state}")

    if n.flags is not None:
        fl = blocking_flags(n.flags)
        print(f"arm_test: failsafe flags set ({len(fl)}):")
        for f in fl:
            print(f"    {f}")

    # 2. stream setpoints, request OFFBOARD, then arm
    print(f"arm_test: streaming setpoints, requesting OFFBOARD + ARM "
          f"(budget {budget:.0f}s wall)", flush=True)
    t0 = time.time()
    armed = False
    last_cmd = 0.0
    while time.time() - t0 < budget:
        n.heartbeat()
        now = time.time()
        if now - t0 > 2 and now - last_cmd > 1.0:
            n.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            n.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            last_cmd = now
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.status and n.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            armed = True
            break

    print(f"arm_test: RESULT armed={armed} "
          f"arming_state={ARMING.get(n.status.arming_state, '?')} "
          f"nav_state={n.status.nav_state}")
    if not armed and n.flags is not None:
        print("arm_test: flags still set at failure:")
        for f in blocking_flags(n.flags):
            print(f"    {f}")

    # leave it disarmed so the stack stays in a known state
    n.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    for _ in range(10):
        n.heartbeat(); rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if armed else 1


if __name__ == "__main__":
    sys.exit(main())
