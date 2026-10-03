#!/usr/bin/env python3
"""Measure stereo pair timestamp offsets on the D455-mirror rig (run INSIDE sim).

Why this matters: a real D455 hardware-syncs its two IR imagers, and BOTH
estimators pair stereo frames with a timestamp synchroniser (ORB-SLAM3 via
message_filters ApproximateTime, OpenVINS via its own stereo buffer). If the two
simulated cameras are stepped at even slightly different sim times, every pair
carries a baseline-direction error that looks exactly like a calibration fault,
and no amount of estimator tuning fixes it.

Default measurement uses camera_info, NOT the images: a 848x480 mono8 Image is
~407 kB and subscribing to both from Python collapses the real-time factor
(0.289 -> 0.026 was measured on this host), which would itself perturb the
stamps being measured. camera_info is ~400 B and is published by the same
ros_gz_bridge conversion off the same gz message, so it carries the sensor's
stamp. --images cross-checks that assumption directly.

    ./check_stereo_sync.py --duration 60
    ./check_stereo_sync.py --duration 20 --images     # verify info==image stamps
"""
from __future__ import annotations

import argparse
import statistics as st
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class StereoSync(Node):
    def __init__(self, use_images: bool, duration: float):
        super().__init__("stereo_sync_check",
                         parameter_overrides=[rclpy.Parameter(
                             "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.left: list[float] = []
        self.right: list[float] = []
        # Image stamps kept separately so --images can compare info vs image
        # for the SAME camera rather than assuming they agree.
        self.left_img: list[float] = []
        self.right_img: list[float] = []
        self.duration = duration

        base = "/camera/infra{}/camera_info"
        self.create_subscription(CameraInfo, base.format(1), lambda m: self.left.append(sec(m.header.stamp)), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, base.format(2), lambda m: self.right.append(sec(m.header.stamp)), qos_profile_sensor_data)
        if use_images:
            ibase = "/camera/infra{}/image_rect_raw"
            self.create_subscription(Image, ibase.format(1), lambda m: self.left_img.append(sec(m.header.stamp)), qos_profile_sensor_data)
            self.create_subscription(Image, ibase.format(2), lambda m: self.right_img.append(sec(m.header.stamp)), qos_profile_sensor_data)

        # The measurement window must start from a VALID simulation clock.
        # Capturing t0 in the constructor captures 0: with use_sim_time the node's
        # clock reads zero until the first /clock message arrives, so the very
        # first tick then saw elapsed = (current sim time - 0) = hundreds of
        # seconds and exited immediately. That silently truncated the collection
        # to ~1 s wall and produced first a short sample set and then none at all.
        self.t0 = None
        self.create_timer(1.0, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        t = self._now()
        if self.t0 is None:
            if t <= 0.0:
                return  # /clock not received yet
            self.t0 = t
            # Discard anything collected before the window opened.
            for buf in (self.left, self.right, self.left_img, self.right_img):
                buf.clear()
            print(f"stereo_sync: window open at sim t={t:.3f}s, "
                  f"collecting {self.duration:g} s", flush=True)
            return
        if t - self.t0 >= self.duration:
            raise SystemExit(0)


def stream_stats(ts: list[float], label: str, nominal_hz: float) -> None:
    """Per-stream arrival statistics -- separates DROPS from sync offsets."""
    if len(ts) < 3:
        print(f"  {label}: too few samples ({len(ts)})")
        return
    t = sorted(ts)
    span = t[-1] - t[0]
    gaps = [b - a for a, b in zip(t, t[1:])]
    nom = 1.0 / nominal_hz
    # A gap of ~2x the nominal period means exactly one frame never arrived.
    dropped = sum(round(g / nom) - 1 for g in gaps if round(g / nom) >= 2)
    print(f"  {label}: n={len(t)} over {span:.2f} s sim = {len(t)/span:.2f} Hz "
          f"(nominal {nominal_hz:g})")
    print(f"      gap min={min(gaps)*1e3:.3f} ms  median={st.median(gaps)*1e3:.3f} ms"
          f"  max={max(gaps)*1e3:.3f} ms  missing frames implied by gaps: {dropped}")
    # The MINIMUM gap is what OpenVINS's frame throttle must clear.
    # ROS2Visualizer.cpp:565-568 drops a stereo pair when
    #   timestamp < camera_last_timestamp + 1/track_frequency
    # and does so SILENTLY, without updating camera_last_timestamp. So
    # track_frequency must exceed 1/min_gap, not the nominal or average rate --
    # Gazebo quantises stamps to the physics step, which makes the instantaneous
    # period coarser than the average.
    print(f"      -> OpenVINS track_frequency must EXCEED "
          f"{1.0/min(gaps):.3f} Hz (1/min_gap) or pairs are dropped silently")


def report(left: list[float], right: list[float], label: str,
           nominal_hz: float = 30.0) -> float | None:
    """Pair each left stamp with its nearest right stamp and summarise offsets.

    Reports the offset DISTRIBUTION rather than just the maximum. That matters:
    a rig whose imagers are stepped together but whose transport occasionally
    loses one sample produces median 0 with a few full-frame-period outliers,
    and a bare max would misdiagnose that as a sync fault.
    """
    if len(left) < 10 or len(right) < 10:
        print(f"  {label}: too few samples (L={len(left)} R={len(right)})")
        return None
    import bisect
    nom = 1.0 / nominal_hz
    rs = sorted(right)
    offs, unmatched = [], 0
    for t in sorted(left):
        i = bisect.bisect_left(rs, t)
        cands = [rs[j] for j in (i - 1, i) if 0 <= j < len(rs)]
        if not cands:
            continue
        o = min((t - c for c in cands), key=abs)
        # Beyond half a frame period there is no counterpart to pair with: the
        # nearest right stamp belongs to a DIFFERENT exposure.
        if abs(o) > nom / 2:
            unmatched += 1
        else:
            offs.append(o)
    if not offs:
        print(f"  {label}: no pairings within half a frame period")
        return None
    a = [abs(o) for o in offs]
    exact = sum(1 for v in a if v == 0.0)
    print(f"  {label}: paired={len(offs)}  exact-zero={exact} "
          f"({100.0*exact/len(offs):.1f}%)  unpaired={unmatched}")
    print(f"      mean={st.mean(offs)*1e3:+.6f} ms  median={st.median(offs)*1e3:+.6f} ms"
          f"  max|off|={max(a)*1e3:.6f} ms"
          f"  stdev={(st.stdev(offs)*1e3 if len(offs) > 1 else 0.0):.6f} ms")
    return max(a)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60.0, help="sim seconds to collect")
    ap.add_argument("--images", action="store_true",
                    help="also subscribe to the full Image topics to verify that "
                         "camera_info stamps equal image stamps. Perturbs the "
                         "simulator -- use only for a dedicated check.")
    ap.add_argument("--tolerance-ms", type=float, default=0.001,
                    help="max |offset| still counted as hardware-synced")
    args = ap.parse_args()

    rclpy.init()
    node = StereoSync(args.images, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass

    print()
    print("=" * 70)
    print("STEREO PAIR TIMESTAMP OFFSET  (left - nearest right)")
    print("=" * 70)
    print("PER-STREAM ARRIVALS")
    stream_stats(node.left, "infra1 camera_info", 30.0)
    stream_stats(node.right, "infra2 camera_info", 30.0)
    if args.images:
        stream_stats(node.left_img, "infra1 image     ", 30.0)
        stream_stats(node.right_img, "infra2 image     ", 30.0)
    print()
    print("PAIRING")
    worst = report(node.left, node.right, "camera_info")
    if args.images:
        report(node.left_img, node.right_img, "image      ")
        # Same-camera info vs image: confirms camera_info is a valid proxy.
        report(node.left, node.left_img, "L info/image")
        report(node.right, node.right_img, "R info/image")

    print()
    if worst is None:
        print("VERDICT: INCONCLUSIVE -- not enough samples.")
        rc = 2
    elif worst <= args.tolerance_ms * 1e-3:
        print(f"VERDICT: SYNCHRONISED. Worst |offset| {worst*1e3:.6f} ms is within "
              f"{args.tolerance_ms} ms;\n"
              "         the two imagers are stepped in the same simulation update,\n"
              "         which is what a hardware-synced D455 does.")
        rc = 0
    else:
        print(f"VERDICT: NOT SYNCHRONISED. Worst in-pair |offset| {worst*1e3:.4f} ms.\n"
              "         Both estimators pair stereo frames by timestamp, so this\n"
              "         injects a baseline-direction error into every pair. Fix the\n"
              "         SDF/bridge before trusting any ATE from this rig.\n"
              "         NOTE: check the 'unpaired' count and the per-stream gaps\n"
              "         above first. Offsets at ~one frame period with a median of\n"
              "         zero are DROPPED samples, not a stepping offset -- a\n"
              "         different defect with a different fix.")
        rc = 1
    print("=" * 70)
    try:
        rclpy.try_shutdown()
    except Exception:
        pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
