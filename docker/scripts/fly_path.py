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
from rclpy.parameter import Parameter
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
                 leg_time: float, settle_time: float, path_version: str = "A"):
        # use_sim_time must be supplied as a parameter OVERRIDE, not declared.
        # rclpy declares it automatically for every node, so calling
        # declare_parameter("use_sim_time", ...) raises
        # ParameterAlreadyDeclaredException and kills the node in its
        # constructor. It has to be true here: the phase timeouts below are
        # measured on the node clock, and on a GPU-less host the simulator runs
        # far slower than wall time, so wall-clock budgets would expire long
        # before PX4 had a chance to react.
        super().__init__(
            "scripted_flight",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )

        self.ns = ns
        self.target_system = target_system
        self.side = side
        self.alt = alt
        self.leg_time = leg_time
        self.path_version = path_version.strip().upper()
        if self.path_version not in ("A", "B", "C"):
            raise ValueError(f"unknown path version {path_version!r}; expected A, B or C")
        self.settle_time = settle_time

        prefix = f"/{ns}" if ns else ""
        qos = px4_qos()
        self.pub_mode = self.create_publisher(
            OffboardControlMode, f"{prefix}/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f"{prefix}/fmu/in/vehicle_command", qos)
        # PX4 out-topic names depend on the firmware's message-versioning era.
        # The pinned PX4 (main @ 6bc24c8c) publishes VERSIONED names:
        #   /px4_1/fmu/out/vehicle_status_v1
        #   /px4_1/fmu/out/vehicle_local_position_v1
        # while older firmware uses the unversioned names. The message TYPES are
        # unchanged (px4_msgs/msg/VehicleStatus, .../VehicleLocalPosition), only
        # the topic names differ. Subscribing to both covers either firmware;
        # only one will ever exist, and the callbacks just latch the newest
        # sample, so a duplicate would be harmless anyway.
        #
        # Getting this wrong is silent: the node simply never receives status
        # and sits in wait_for_fmu until it times out.
        for suffix in ("_v1", ""):
            self.create_subscription(
                VehicleStatus, f"{prefix}/fmu/out/vehicle_status{suffix}",
                self._on_status, qos)
            self.create_subscription(
                VehicleLocalPosition,
                f"{prefix}/fmu/out/vehicle_local_position{suffix}",
                self._on_pos, qos)

        self.status: VehicleStatus | None = None
        self.pos: VehicleLocalPosition | None = None
        self.counter = 0
        self.phase = "wait_for_fmu"
        self.phase_ticks = 0
        self.leg_index = 0
        self.done = False

        # NED waypoints. PX4 local frame is North-East-DOWN, so "up" is negative z.
        # Each entry is (north, east, down, yaw, dwell_seconds); dwell defaults
        # to leg_time so path A keeps its original timing exactly.
        self.legs = {"A": self._legs_path_a, "B": self._legs_path_b,
                     "C": self._legs_path_c}[self.path_version]()

        self.timer = self.create_timer(1.0 / SETPOINT_HZ, self._tick)
        self.get_logger().info(
            f"namespace={prefix or '<root>'} target_system={target_system} "
            f"side={side} m alt={alt} m leg_time={leg_time}s")

    # --- paths ---------------------------------------------------------------
    def _legs_path_a(self):
        """Path A: the original. A square at fixed altitude, yaw slewed per leg.

        Kept byte-for-byte so phase-1/2 results stay comparable. Its weakness is
        the reason path B exists: straight constant-velocity legs at one
        altitude leave accelerometer bias and inertial scale weakly observable,
        so a visual-inertial estimator gets little to work with. See
        docker/OPEN_ISSUES.md s2.
        """
        s, z, L = self.side, -abs(self.alt), self.leg_time
        return [
            (0.0, 0.0, z, 0.0, L),
            (s,   0.0, z, 0.0, L),
            (s,   s,   z, math.pi / 2, L),
            (0.0, s,   z, math.pi, L),
            (0.0, 0.0, z, -math.pi / 2, L),
            (0.0, 0.0, z, 0.0, L),
        ]

    def _legs_path_b(self):
        """Path B: excited. Three figure-eight laps, each at a different
        altitude, with yaw following the velocity and alternating dwell times.

        Chosen to excite the states path A leaves unobservable:
          * a lemniscate has continuously changing heading AND curvature, so
            yaw rate and lateral acceleration are never constant -- unlike a
            square, which is straight-line motion plus four corners;
          * the three laps sit at different altitudes with climbs between them,
            which exercises the vertical accelerometer axis that a fixed-
            altitude path never moves;
          * dwell alternates short/long, so the vehicle is accelerating or
            decelerating for most of the flight rather than cruising.

        Yaw is the numerical heading of the segment, so the airframe turns
        through the whole lap rather than snapping at corners.
        """
        s = self.side
        base = abs(self.alt)
        laps = [base, base + 0.8, max(0.8, base - 0.5)]
        pts_per_lap = 8
        legs = []
        for lap_i, lap_alt in enumerate(laps):
            z = -lap_alt
            for k in range(pts_per_lap):
                th = 2.0 * math.pi * k / pts_per_lap
                # Lemniscate of Gerono: crosses itself at the origin, which
                # revisits the same scene from a different heading -- useful for
                # loop closure as well as for excitation.
                n = s * math.cos(th)
                e = s * math.sin(th) * math.cos(th)
                # Heading from the analytic derivative of the curve.
                dn = -s * math.sin(th)
                de = s * (math.cos(2.0 * th))
                yaw = math.atan2(de, dn)
                # Alternate dwell: short legs are flown aggressively, long legs
                # settle. Varied acceleration is the point.
                dwell = self.leg_time * (0.7 if k % 2 == 0 else 1.3)
                legs.append((n, e, z, yaw, dwell))
            # Explicit altitude change between laps, held long enough that the
            # vertical motion is a real excitation and not a transient.
            if lap_i + 1 < len(laps):
                nz = -laps[lap_i + 1]
                legs.append((s, 0.0, nz, 0.0, self.leg_time))
        # Return to the origin at the base altitude so landing is predictable.
        legs.append((0.0, 0.0, -base, 0.0, self.leg_time))
        return legs

    def _legs_path_c(self):
        """Path C: viewpoint swings (PATCHES s63, distribution-shift test for the
        landmark usage model). Large viewing-angle changes on tracked landmarks
        need LATERAL translation relative to their depth while they stay
        anchored -- pure rotation barely changes a landmark's viewing ray. So:
          * each side of the square is flown strafing, yaw held perpendicular to
            the direction of travel (camera looking sideways = maximum parallax),
            at side/leg_time speed;
          * at every corner the airframe swings yaw +60 deg then -60 deg about
            the strafing heading and back, a large yaw change that sweeps
            landmarks across the image and out of view.
        """
        s, z, L = self.side, -abs(self.alt), self.leg_time
        corners = [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        swing, hold = math.radians(60.0), 0.5 * L
        legs = [(0.0, 0.0, z, 0.0, L)]
        for (n0, e0), (n1, e1) in zip(corners[:-1], corners[1:]):
            yaw = math.atan2(e1 - e0, n1 - n0) + math.pi / 2.0  # look sideways
            for dy in (0.0, swing, -swing, 0.0):                  # swings at the corner
                legs.append((n0, e0, z, yaw + dy, hold))
            legs.append((n1, e1, z, yaw, L))                       # strafe the side
        legs.append((0.0, 0.0, z, 0.0, L))
        return legs

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
            # target is (n, e, d, yaw, dwell): the setpoint takes the first four
            # and the dwell is per leg, which is what lets path B alternate
            # aggressive and settling legs.
            self._setpoint(*target[:4])
            if t > target[4]:
                self.leg_index += 1
                if self.leg_index >= len(self.legs):
                    self._goto_phase("land")
                else:
                    nxt = self.legs[self.leg_index]
                    self.get_logger().info(
                        f"leg {self.leg_index}/{len(self.legs)} -> "
                        f"N={nxt[0]:+.2f} E={nxt[1]:+.2f} "
                        f"alt={-nxt[2]:.2f} yaw={math.degrees(nxt[3]):+.0f}deg "
                        f"dwell={nxt[4]:.1f}s")
                    self.phase_ticks = 0

        elif self.phase == "land":
            self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if t > 12:
                self.get_logger().info("scripted path complete")
                self.done = True


def autodetect_namespace(timeout: float = 150.0) -> tuple[str, int]:
    """Find the PX4 topic namespace. SITL with -i N uses 'px4_N'.

    Retries, because this RACES ROS 2 discovery. A single `ros2 topic list`
    issued shortly after bringup can return an incomplete graph; the old
    single-shot version then fell back to the root namespace and the flight
    failed 60 s later with "no VehicleStatus/VehicleLocalPosition", having never
    talked to the FMU at all. Three of five runs died that way.

    Returns ("", 0) if nothing is found, which the caller treats as fatal rather
    than guessing -- flying against the wrong namespace is worse than not flying.
    """
    import subprocess
    import time as _t
    deadline = _t.time() + timeout
    attempt = 0
    while _t.time() < deadline:
        attempt += 1
        try:
            out = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                                 text=True, timeout=30).stdout
        except Exception:
            out = ""
        ns = _scan_for_fmu(out)
        if ns is not None:
            if attempt > 1:
                print(f"fly_path: namespace found on attempt {attempt}")
            return ns
        _t.sleep(3.0)
    print("fly_path: no */fmu/in/trajectory_setpoint topic found after "
          f"{timeout:.0f}s -- is MicroXRCEAgent running?", file=sys.stderr)
    return "", 0


def _scan_for_fmu(out: str):
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
    return None


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
    ap.add_argument("--path-version", default="A", choices=["A", "B", "C", "a", "b", "c"],
                    help="A = original square at fixed altitude (default); "
                         "B = three figure-eight laps at three altitudes with "
                         "yaw following the velocity and alternating dwell, to "
                         "excite the states A leaves unobservable")
    args = ap.parse_args()

    if args.namespace:
        ns, derived_sys = args.namespace, 1
    else:
        ns, derived_sys = autodetect_namespace()
        if derived_sys == 0:
            print("fly_path: refusing to fly without a confirmed PX4 namespace",
                  file=sys.stderr)
            return 3
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
                          args.leg_time, args.settle_time, args.path_version)
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
