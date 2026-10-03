#!/usr/bin/env python3
"""Sample real-time factor and topic rates while a run is in progress.

Why this exists: `gz topic -e -t /world/<w>/stats` reports a real_time_factor
field that is instantaneous and strongly bimodal under software rendering --
sampling it 12 times gave 0.032 in one run and 0.538 in another, while the true
average over the same flight was 0.289. The only honest RTF is simulated time
divided by wall time over the window of interest, which is what this measures.

OBSERVER EFFECT -- read before enabling --images. Subscribing to the full Image
topics from Python costs real CPU (640x480x3 is ~0.9 MB per frame, 848x480 mono
is ~0.4 MB) and it PERTURBS the simulator it is measuring. Measured on this host:
with image subscriptions active the real-time factor collapsed from 0.289 to
~0.026 and the RGB stream reported 17.6 Hz in sim time against a 30 Hz nominal,
while camera_info -- which is tiny -- still showed 30.3 Hz. The images were not
genuinely dropping; the monitor was starving the renderer.

So by default this subscribes ONLY to /clock, which is cheap, and reports the
real-time factor. That is safe to run during a flight. Pass --images for a
DEDICATED measurement pass (no flight, no SLAM) when you actually want stream
rates, and treat the RTF reported in that mode as a lower bound.

    ./rate_monitor.py --duration 300                  # RTF only, flight-safe
    ./rate_monitor.py --images --duration 120         # dedicated rate pass
    ./rate_monitor.py --images --stereo --duration 120
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from geometry_msgs.msg import PoseStamped


def sensor_qos(depth: int = 20) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class RateMonitor(Node):
    def __init__(self, topics: dict[str, type], window: float):
        # Wall clock on purpose: this node measures the relationship BETWEEN
        # sim time and wall time, so it must not itself run on sim time.
        super().__init__(
            "rate_monitor",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, False)])
        self.counts: dict[str, int] = {n: 0 for n in topics}
        self.total: dict[str, int] = {n: 0 for n in topics}
        self.window = window
        self.sim_first: float | None = None
        self.sim_last: float = 0.0
        self.wall_first = time.time()
        self.samples: list[tuple[float, float, dict[str, float]]] = []

        for name, cls in topics.items():
            self.create_subscription(
                cls, name, self._make_cb(name),
                sensor_qos() if cls is not Clock else 50)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)
        self._t0 = time.time()

    def _make_cb(self, name: str):
        def cb(_msg):
            self.counts[name] += 1
            self.total[name] += 1
        return cb

    def _on_clock(self, msg: Clock) -> None:
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_first is None:
            self.sim_first = sim
        self.sim_last = sim

    def tick(self) -> None:
        now = time.time()
        dt = now - self._t0
        if dt < self.window:
            return
        rates = {n: c / dt for n, c in self.counts.items()}
        self.samples.append((now - self.wall_first, self.sim_last, dict(rates)))
        line = "  ".join(f"{n.split('/')[-1]}={r:5.2f}" for n, r in rates.items())
        sim_span = (self.sim_last - self.sim_first) if self.sim_first is not None else 0.0
        wall_span = now - self.wall_first
        rtf = sim_span / wall_span if wall_span > 0 else 0.0
        print(f"[{wall_span:7.1f}s wall | {sim_span:7.1f}s sim | RTF {rtf:5.3f}] {line}",
              flush=True)
        for n in self.counts:
            self.counts[n] = 0
        self._t0 = now


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="uav_1")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--window", type=float, default=15.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--stereo", action="store_true",
                    help="monitor the D455-mirror stereo/IMU topic set instead")
    ap.add_argument("--images", action="store_true",
                    help="also subscribe to the image/IMU streams. PERTURBS the "
                         "simulator -- use only for a dedicated measurement pass, "
                         "never during a flight being evaluated.")
    args = ap.parse_args()

    ns = args.namespace
    if not args.images:
        # Clock-only: measures the real-time factor without touching the heavy
        # topics, so it can run alongside a flight without distorting it.
        topics = {}
    elif args.stereo:
        topics = {
            "/camera/infra1/image_rect_raw": Image,
            "/camera/infra2/image_rect_raw": Image,
            "/camera/depth/image_rect_raw": Image,
            "/camera/imu": Imu,
        }
    else:
        topics = {
            f"/{ns}/rgb/image_raw": Image,
            f"/{ns}/depth/image": Image,
            f"/{ns}/rgb/camera_info": CameraInfo,
            f"/{ns}/robot_pose_slam": PoseStamped,
        }

    # Print the summary even when killed. record_and_eval stops this process
    # when the flight ends; without a handler the summary never appears and the
    # run's real-time factor is lost (it showed up as "nan" in the aggregate).
    stop = {"now": False}

    def _stop(signum, _frame):
        stop["now"] = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    rclpy.init()
    n = RateMonitor(topics, args.window)
    if topics:
        print("rate_monitor: MEASUREMENT PASS (perturbs the sim) -- "
              + ", ".join(topics), flush=True)
    else:
        print("rate_monitor: clock-only (flight-safe); RTF = sim/wall", flush=True)
    end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end and not stop["now"]:
            rclpy.spin_once(n, timeout_sec=0.05)
            n.tick()
    except KeyboardInterrupt:
        pass

    wall = time.time() - n.wall_first
    sim = (n.sim_last - n.sim_first) if n.sim_first is not None else 0.0
    rtf = sim / wall if wall > 0 else 0.0
    print("\n==================== RUN SUMMARY ====================", flush=True)
    print(f"  wall duration      : {wall:.1f} s")
    print(f"  simulated duration : {sim:.1f} s")
    print(f"  real-time factor   : {rtf:.3f}   (sim/wall, measured)")
    for name, tot in n.total.items():
        print(f"  {name:<42} {tot/wall if wall>0 else 0:6.2f} Hz wall"
              f"   {tot/sim if sim>0 else 0:7.2f} Hz sim-time   ({tot} msgs)")
    print("=====================================================", flush=True)

    if args.out:
        with open(args.out, "w") as fh:
            fh.write("wall_s,sim_s," + ",".join(topics) + "\n")
            for w, s, r in n.samples:
                fh.write(f"{w:.3f},{s:.3f}," + ",".join(f"{r[t]:.4f}" for t in topics) + "\n")
            fh.write(f"# summary wall={wall:.2f} sim={sim:.2f} rtf={rtf:.4f}\n")
            for name, tot in n.total.items():
                fh.write(f"# {name} total={tot} hz_wall={tot/wall if wall>0 else 0:.3f} "
                         f"hz_sim={tot/sim if sim>0 else 0:.3f}\n")
        print(f"rate_monitor: wrote {args.out}")

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
