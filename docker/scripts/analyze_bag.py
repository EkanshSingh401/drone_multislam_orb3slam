#!/usr/bin/env python3
"""Compute RTF, path length and duration POST-HOC from a recorded bag.

This is the trustworthy way to get a run's real-time factor. The two live
alternatives both failed:

  * Gazebo's own /world/<w>/stats real_time_factor field is instantaneous and
    strongly bimodal -- it read 0.032, 0.289, 0.538 and 1.0 on the same setup.
  * A live /clock subscriber works when it works, but its subscription can be
    starved or never discovered: across five runs one monitor received ZERO
    clock messages over 900 s wall, and another showed 66 s and 900 s gaps
    between windows. Averaging those produced a meaningless 0.128 +/- 0.149.

The bag already contains /clock with per-message bag receive times, so sim/wall
can be recovered exactly after the fact, with no load on the simulator and no
discovery to race.

    ./analyze_bag.py /out/eval/<run>/flight.bag [--gt-index 0]
"""
from __future__ import annotations

import argparse
import json
import math
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--gt-topic", default="/ground_truth/pose_info")
    ap.add_argument("--gt-index", type=int, default=0)
    ap.add_argument("--clock-topic", default="/clock")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    # Same reader selection as bag_to_tum.py (file-compressed sensor bags, s45).
    info = rosbag2_py.Info().read_metadata(args.bag, "")
    r = (rosbag2_py.SequentialCompressionReader()
         if (info.compression_mode or "").upper() == "FILE"
         else rosbag2_py.SequentialReader())
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
           rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                       output_serialization_format="cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if args.clock_topic not in types:
        print(f"analyze_bag: no {args.clock_topic} in bag", file=sys.stderr)
        return 2
    ccls = get_message(types[args.clock_topic])
    gcls = get_message(types[args.gt_topic]) if args.gt_topic in types else None

    clock: list[tuple[float, float]] = []
    gt: list[tuple[float, float, float, float]] = []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic == args.clock_topic:
            m = deserialize_message(data, ccls)
            clock.append((t_ns * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))
        elif gcls is not None and topic == args.gt_topic:
            m = deserialize_message(data, gcls)
            if len(m.transforms) > args.gt_index:
                p = m.transforms[args.gt_index].transform.translation
                gt.append((t_ns * 1e-9, p.x, p.y, p.z))
    if len(clock) < 2:
        print("analyze_bag: too few clock samples", file=sys.stderr)
        return 2
    clock.sort()

    sim_span = clock[-1][1] - clock[0][1]
    wall_span = clock[-1][0] - clock[0][0]
    rtf = sim_span / wall_span if wall_span > 0 else 0.0

    out = {"rtf": rtf, "sim_s": sim_span, "wall_s": wall_span,
           "clock_samples": len(clock)}

    if gt:
        plen = sum(math.dist(gt[i][1:], gt[i - 1][1:]) for i in range(1, len(gt)))
        zs = [g[3] for g in gt]
        out.update({"path_length_m": plen,
                    "gt_samples": len(gt),
                    "alt_min_m": min(zs), "alt_max_m": max(zs),
                    "mean_speed_mps": plen / sim_span if sim_span > 0 else 0.0})

    if args.json:
        print(json.dumps(out))
    else:
        print(f"  real-time factor : {rtf:.3f}   ({sim_span:.1f} s sim in {wall_span:.1f} s wall)")
        print(f"  clock samples    : {len(clock)}")
        if gt:
            print(f"  path length      : {out['path_length_m']:.2f} m")
            print(f"  mean speed       : {out['mean_speed_mps']:.3f} m/s")
            print(f"  altitude         : {out['alt_min_m']:.2f} .. {out['alt_max_m']:.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
