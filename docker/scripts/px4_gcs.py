#!/usr/bin/env python3
"""Minimal MAVLink GCS for PX4 SITL (run INSIDE the sim container).

Two reasons this exists:

1. PX4's health_and_arming_checks reports "Preflight Fail: No connection to the
   GCS" when no ground station is attached, and that blocks arming. Rather than
   disabling the check, this attaches a real (if minimal) GCS: it sends GCS
   heartbeats, which is what the check is actually looking for.

2. `px4-commander status` is unusable here. PX4 runs with -d (daemon mode, see
   docker/PATCHES.md) and leaves /tmp/px4_lock-<i> empty, so the px4-* shims
   report "PX4 server not running" even though the server is up. MAVLink gives
   us an out-of-band view of arming state and health instead.

    ./px4_gcs.py --status          # report arming/health once, then exit
    ./px4_gcs.py --heartbeat       # run forever sending GCS heartbeats
    ./px4_gcs.py --set NAV_DLL_ACT 0
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

# PX4 SITL instance i opens its "Normal" (GCS-facing) MAVLink on 180?1 and
# streams to remote 14550. Binding 14550 makes us that GCS.
DEFAULT_LISTEN = "udpout:127.0.0.1:18571"  # unused; kept for CLI compat
DEFAULT_TARGET = "udpout:127.0.0.1:18571"


def connect(listen: str, target: str, timeout: float = 30.0):
    """Open one 'udpout' socket to PX4's GCS-facing MAVLink port.

    Determined empirically. PX4 SITL instance i listens on 1857i, and although
    its startup log says "remote port 14550", it actually replies to the source
    address:port the datagram arrived from. So a plain client socket
    ('udpout:127.0.0.1:1857i') both sends and receives correctly, while binding
    14550 and sending from it does not get answered.

    The frames must also be properly encoded: a hand-rolled heartbeat with a bad
    CRC is silently dropped by PX4, which then never registers a partner and
    streams nothing. pymavlink handles the CRC.
    """
    conn = mavutil.mavlink_connection(target, source_system=255,
                                      source_component=190)

    def send_gcs_heartbeat() -> None:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

    conn.send_gcs_heartbeat = send_gcs_heartbeat  # type: ignore[attr-defined]

    # Accept ANY message, not specifically HEARTBEAT. PX4's MAVLink stream rates
    # are driven by PX4 time, so at a real-time factor of ~0.03 its nominal 1 Hz
    # heartbeat only arrives about every 33 s of wall clock -- wait_heartbeat()
    # times out long before one shows up, even though PX4 is streaming happily.
    deadline = time.time() + timeout
    while time.time() < deadline:
        send_gcs_heartbeat()
        m = conn.recv_match(blocking=True, timeout=1.0)
        if m is not None:
            print(f"px4_gcs: {m.get_type()} from system {m.get_srcSystem()} "
                  f"via {target}", flush=True)
            return conn, conn
    print(f"px4_gcs: no heartbeat from PX4 within {timeout:.0f}s via {target}",
          file=sys.stderr)
    return None, None


ARMING = {0: "INIT", 1: "DISARMED", 2: "ARMED", 3: "STANDBY_ERROR",
          4: "SHUTDOWN", 5: "IN_AIR_RESTORE"}


def report(rx, tx, seconds: float = 12.0) -> int:
    """Collect health/arming signals for a few seconds and summarise."""
    end = time.time() + seconds
    statustexts: list[str] = []
    armed = None
    prearm_ok = None
    while time.time() < end:
        tx.send_gcs_heartbeat()
        m = rx.recv_match(
            type=["HEARTBEAT", "SYS_STATUS", "STATUSTEXT"],
            blocking=True, timeout=1.0)
        if m is None:
            continue
        t = m.get_type()
        if t == "HEARTBEAT":
            armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            prearm_ok = bool(m.system_status != mavutil.mavlink.MAV_STATE_UNINIT)
        elif t == "STATUSTEXT":
            txt = m.text.strip() if isinstance(m.text, str) else m.text.decode(errors="replace").strip()
            if txt and txt not in statustexts:
                statustexts.append(txt)

    print(f"  armed:        {armed}")
    print(f"  responsive:   {prearm_ok}")
    if statustexts:
        print("  STATUSTEXT from PX4:")
        for s in statustexts[-15:]:
            print(f"    {s}")
    else:
        print("  STATUSTEXT from PX4: (none in window)")
    return 0


def set_param(rx, tx, name: str, value: float, timeout: float = 180.0) -> int:
    """Set a PX4 parameter and wait for the PARAM_VALUE echo.

    The timeout is deliberately large. PX4's MAVLink responses are produced on
    PX4's own clock, so when the simulator runs at a real-time factor of ~0.03 a
    10 s wall budget is only ~0.3 s of PX4 time -- nowhere near enough for an
    ack, which looks indistinguishable from the param set being rejected.
    The request is also re-sent periodically in case the first frame was dropped.
    """
    deadline = time.time() + timeout
    last_send = 0.0
    while time.time() < deadline:
        if time.time() - last_send > 5.0:
            tx.mav.param_set_send(rx.target_system or 1, rx.target_component or 1,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            tx.send_gcs_heartbeat()
            last_send = time.time()
        m = rx.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if m is None:
            continue
        pid = m.param_id if isinstance(m.param_id, str) else m.param_id.decode(errors="replace")
        if pid.strip("\x00") == name:
            print(f"  {name} = {m.param_value}", flush=True)
            return 0
    print(f"px4_gcs: no PARAM_VALUE ack for {name}", file=sys.stderr)
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", default=DEFAULT_LISTEN)
    ap.add_argument("--target", default=DEFAULT_TARGET)
    ap.add_argument("--status", action="store_true", help="report once and exit")
    ap.add_argument("--window", type=float, default=12.0)
    ap.add_argument("--serve", action="store_true",
                    help="set the params that block offboard arming, then "
                         "heartbeat forever (use this during flights)")
    ap.add_argument("--heartbeat", action="store_true",
                    help="send GCS heartbeats forever (keeps the GCS check satisfied)")
    ap.add_argument("--set", nargs=2, metavar=("PARAM", "VALUE"))
    args = ap.parse_args()

    rx, tx = connect(args.listen, args.target)
    if rx is None:
        return 2

    if args.set:
        return set_param(rx, tx, args.set[0], float(args.set[1]))
    if args.status:
        return report(rx, tx, args.window)
    if args.serve:
        # One long-lived process that claims PX4's single MAVLink partner slot,
        # fixes the params that block offboard arming, then keeps heartbeating.
        #
        # Why it must be one process: PX4's MAVLink instance locks onto the
        # first partner's source address:port and does not re-learn, so a second
        # client gets no replies. Doing the PARAM_SET and the heartbeats from
        # the same socket is the only arrangement that works.
        #
        # Why NAV_DLL_ACT must be set here rather than via PX4_PARAM_NAV_DLL_ACT:
        # rcS applies PX4_PARAM_* before the airframe config, and setting the
        # param to 0 is a no-op because 0 is already the compiled default -- PX4
        # records no change, so the airframe's later `param set-default
        # NAV_DLL_ACT 2` wins. Confirmed by reading the ulog's initial
        # parameters: NAV_DLL_ACT was 2 despite the env var.
        for name, value in (("NAV_DLL_ACT", 0.0),):
            print(f"px4_gcs: setting {name}={value}")
            set_param(rx, tx, name, value)
        print("px4_gcs: serving GCS heartbeats at 1 Hz (wall clock)", flush=True)
        last = 0.0
        while True:
            now = time.time()
            if now - last >= 1.0:
                tx.send_gcs_heartbeat()
                last = now
            m = rx.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
            if m is not None:
                txt = m.text if isinstance(m.text, str) else m.text.decode(errors="replace")
                print(f"  PX4: {txt.strip()}", flush=True)

    if args.heartbeat:
        print("px4_gcs: sending GCS heartbeats at 1 Hz (Ctrl-C to stop)")
        while True:
            tx.send_gcs_heartbeat()
            rx.recv_match(blocking=False)
            time.sleep(1.0)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
