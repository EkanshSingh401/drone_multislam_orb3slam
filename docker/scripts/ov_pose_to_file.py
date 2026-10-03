#!/usr/bin/env python3
"""Record OpenVINS odometry + covariance in ov_eval's NEES format (run in sim).

Why this exists: OpenVINS ships `ov_eval/src/pose_to_file.cpp` for exactly this
job, but it is **ROS 1 only** (`#include <ros/ros.h>`, `nav_msgs/Odometry.h`),
so it does not build on Jazzy. This is the ROS 2 equivalent, writing the same
columns in the same order so `ov_eval error_singlerun` consumes it unchanged.

The format `ov_eval::Loader::load_data` parses (Loader.cpp:51) is

    timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 \
                                      Pt11 Pt12 Pt13 Pt22 Pt23 Pt33

20 fields for a row with covariance, 8 for one without. Two details matter:

  * fields are split on a SINGLE SPACE (`std::getline(s, field, ' ')`), so tabs
    or runs of spaces would silently produce a short row -- which Loader drops
    without a word, giving an empty trajectory and a confusing NEES failure;
  * `Pr` is the ORIENTATION block and `Pt` the POSITION block, in that order,
    which is the reverse of how nav_msgs/Odometry lays its 6x6 out
    (position first, then orientation).

The covariance indices below mirror `ov_eval::Recorder::callback_odometry`
(Recorder.h:92-104) exactly rather than being re-derived.
"""
from __future__ import annotations

import argparse
import signal
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Upper-triangle index pairs into nav_msgs/Odometry's row-major 6x6 pose
# covariance. Position occupies rows/cols 0-2, orientation 3-5.
POS_IDX = ((0, 1, 2), (6, 7, 8), (12, 13, 14))     # -> Pt11 Pt12 Pt13 Pt22 Pt23 Pt33
ROT_IDX = ((21, 22, 23), (27, 28, 29), (33, 34, 35))  # -> Pr11 Pr12 Pr13 Pr22 Pr23 Pr33


def _upper(cov, idx) -> tuple[float, ...]:
    """(11, 12, 13, 22, 23, 33) of the 3x3 block selected by idx."""
    m = [[cov[j] for j in row] for row in idx]
    return (m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2])


class OvRecorder(Node):
    def __init__(self, topic: str, out_path: str, require_cov: bool):
        super().__init__(
            "ov_pose_to_file",
            parameter_overrides=[rclpy.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.out = open(out_path, "w")
        self.out.write("# timestamp(s) tx ty tz qx qy qz qw "
                       "Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 "
                       "Pt11 Pt12 Pt13 Pt22 Pt23 Pt33\n")
        self.n = 0
        self.n_zero_cov = 0
        self.require_cov = require_cov
        # OpenVINS publishes odomimu with depth 2; a sensor-data (best effort)
        # subscription would be free to drop samples, and a dropped sample is a
        # missing NEES point. Default reliable QoS with a deep queue instead.
        self.create_subscription(Odometry, topic, self._cb, 200)
        self.get_logger().info(f"recording {topic} -> {out_path}")

    def _cb(self, msg: Odometry) -> None:
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cov = list(msg.pose.covariance)
        pr = _upper(cov, ROT_IDX)
        pt = _upper(cov, POS_IDX)
        # A covariance of exactly zero means OpenVINS had nothing to report
        # (typically before initialisation); NEES on it would divide by zero.
        if pr[0] == 0.0 and pr[3] == 0.0 and pr[5] == 0.0:
            self.n_zero_cov += 1
            if self.require_cov:
                return
        vals = [f"{t:.5f}",
                f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}"]
        vals += [f"{v:.10f}" for v in pr]
        vals += [f"{v:.10f}" for v in pt]
        self.out.write(" ".join(vals) + "\n")
        self.n += 1

    def finish(self) -> None:
        try:
            self.out.flush()
            self.out.close()
        except Exception:
            pass
        print(f"ov_pose_to_file: wrote {self.n} poses "
              f"({self.n_zero_cov} had zero covariance"
              + (", skipped" if self.require_cov else ", kept") + ")",
              flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/ov_msckf/odomimu")
    ap.add_argument("--out", required=True)
    ap.add_argument("--keep-zero-cov", action="store_true",
                    help="keep rows whose covariance is all zero (pre-init). "
                         "They are dropped by default because NEES divides by "
                         "the covariance.")
    args = ap.parse_args()

    rclpy.init()
    node = OvRecorder(args.topic, args.out, require_cov=not args.keep_zero_cov)

    # Flush on SIGTERM as well as SIGINT: record_and_eval stops helpers with
    # whichever is handy, and a recorder that dies without closing its file
    # loses the run. rclpy raises ExternalShutdownException rather than
    # KeyboardInterrupt when shut down from a signal handler.
    def _stop(_sig, _frm):
        rclpy.try_shutdown()
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
    return 0


if __name__ == "__main__":
    sys.exit(main())
