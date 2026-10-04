#!/usr/bin/env python3
"""Extract TUM-format trajectories from a ROS 2 bag (run INSIDE the sim container).

Writes `timestamp tx ty tz qx qy qz qw` (TUM), which is what `evo` consumes.

Two sources are handled:
  * geometry_msgs/PoseStamped  -- the ORB-SLAM3 estimate (/uav_1/robot_pose_slam)
  * tf2_msgs/TFMessage         -- Gazebo ground truth bridged from
                                  /world/<world>/dynamic_pose/info, which carries
                                  every model, so a child_frame_id filter is required

    ./bag_to_tum.py BAG --pose-topic /uav_1/robot_pose_slam --out est.tum
    ./bag_to_tum.py BAG --tf-topic /ground_truth/pose_info \
                        --tf-child x500_depth_1 --out gt.tum
"""
from __future__ import annotations

import argparse
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def make_reader(path: str):
    """A reader that can open this bag (PATCHES.md s45).

    RECORD_SENSORS=1 bags are written with FILE-level zstd compression, and the
    plain SequentialReader cannot open those: it reads the zstd frame header
    where the MCAP magic should be ("invalid magic bytes in Header:
    0x28B52FFD..."). The compression mode is read from metadata.yaml and the
    matching reader chosen. Note that SequentialCompressionReader decompresses
    to an uncompressed .mcap beside the .zstd and leaves it there.
    """
    info = rosbag2_py.Info().read_metadata(path, "")
    if (info.compression_mode or "").upper() == "FILE":
        return rosbag2_py.SequentialCompressionReader()
    return rosbag2_py.SequentialReader()


def open_reader(path: str) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    reader = make_reader(path)
    # storage_id="" lets rosbag2 sniff mcap vs sqlite3 from the bag itself.
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--pose-topic", default=None)
    ap.add_argument("--tf-topic", default=None)
    ap.add_argument("--tf-child", default=None,
                    help="child_frame_id to keep from the TF stream")
    ap.add_argument("--tf-index", type=int, default=None,
                    help="select a transform by POSITION instead of name. Needed "
                         "because ros_gz_bridge's gz.msgs.Pose_V -> TFMessage "
                         "conversion drops the entity names, leaving every "
                         "frame_id/child_frame_id empty. Gazebo orders "
                         "<world>/dynamic_pose/info as [model, link, link, ...], "
                         "so index 0 is the model's world pose. Verify before "
                         "trusting it: only the model index should vary, the link "
                         "indices stay constant (see --variance).")
    ap.add_argument("--variance", action="store_true",
                    help="print the per-index coordinate ranges of a TF stream and "
                         "exit, to confirm which index is the moving model")
    ap.add_argument("--out", required=True)
    ap.add_argument("--time-from-clock", metavar="CLOCK_TOPIC", default=None,
                    help="Re-time samples onto SIMULATION time by interpolating the "
                         "bag's /clock topic. Needed for the ground-truth stream: "
                         "ros_gz_bridge stamps its Pose_V -> TFMessage output from "
                         "the SYSTEM clock regardless of use_sim_time, so those "
                         "stamps are Unix epoch while the SLAM estimate is on sim "
                         "time -- evo then finds zero matching timestamps. Maps each "
                         "sample's bag receive time through (receive_time -> "
                         "/clock value).")
    ap.add_argument("--list", action="store_true",
                    help="just list topics and message counts, then exit")
    args = ap.parse_args()

    reader, types = open_reader(args.bag)

    if args.list:
        counts: dict[str, int] = {}
        while reader.has_next():
            topic, _, _ = reader.read_next()
            counts[topic] = counts.get(topic, 0) + 1
        print(f"{'topic':<48} {'type':<42} count")
        for t in sorted(types):
            print(f"{t:<48} {types[t]:<42} {counts.get(t, 0)}")
        return 0

    # Build a wall -> sim interpolation from the recorded /clock, if asked.
    clock_pairs: list[tuple[float, float]] = []
    if args.time_from_clock:
        if args.time_from_clock not in types:
            print(f"bag_to_tum: clock topic {args.time_from_clock!r} not in bag",
                  file=sys.stderr)
            return 2
        ccls = get_message(types[args.time_from_clock])
        creader, _ = open_reader(args.bag)
        while creader.has_next():
            topic, data, t_ns = creader.read_next()
            if topic != args.time_from_clock:
                continue
            cm = deserialize_message(data, ccls)
            sim = cm.clock.sec + cm.clock.nanosec * 1e-9
            clock_pairs.append((t_ns * 1e-9, sim))
        clock_pairs.sort()
        if len(clock_pairs) < 2:
            print("bag_to_tum: not enough /clock samples to re-time", file=sys.stderr)
            return 2
        print(f"bag_to_tum: {len(clock_pairs)} clock samples, "
              f"sim {clock_pairs[0][1]:.2f}..{clock_pairs[-1][1]:.2f} s")

    def to_sim(recv_wall: float) -> float:
        """Linear interpolation of bag receive time onto simulation time."""
        import bisect
        xs = [c[0] for c in clock_pairs]
        i = bisect.bisect_left(xs, recv_wall)
        if i <= 0:
            return clock_pairs[0][1]
        if i >= len(clock_pairs):
            return clock_pairs[-1][1]
        x0, y0 = clock_pairs[i - 1]
        x1, y1 = clock_pairs[i]
        if x1 == x0:
            return y1
        return y0 + (y1 - y0) * (recv_wall - x0) / (x1 - x0)

    target = args.pose_topic or args.tf_topic
    if not target:
        ap.error("need --pose-topic or --tf-topic (or --list)")
    if target not in types:
        print(f"bag_to_tum: topic {target!r} not in bag. Available:", file=sys.stderr)
        for t in sorted(types):
            print(f"  {t}  ({types[t]})", file=sys.stderr)
        return 2
    if args.tf_topic and not (args.tf_child or args.tf_index is not None or args.variance):
        ap.error("--tf-topic requires --tf-child, --tf-index or --variance "
                 "(the TF stream carries every entity)")

    if args.variance:
        msg_cls = get_message(types[target])
        series: dict[int, list[tuple[float, float, float]]] = {}
        count = 0
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != target:
                continue
            msg = deserialize_message(data, msg_cls)
            for i, tr in enumerate(msg.transforms):
                t_ = tr.transform.translation
                series.setdefault(i, []).append((t_.x, t_.y, t_.z))
            count += 1
        print(f"{count} messages on {target}")
        print("idx       x-range            y-range            z-range")
        for i, v in sorted(series.items()):
            xs = [a[0] for a in v]; ys = [a[1] for a in v]; zs = [a[2] for a in v]
            span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            tag = "  <-- moves (model)" if span > 0.5 else ""
            print(f"  {i}  {min(xs):8.3f}..{max(xs):7.3f}  {min(ys):8.3f}..{max(ys):7.3f}"
                  f"  {min(zs):7.3f}..{max(zs):7.3f}{tag}")
        return 0

    msg_cls = get_message(types[target])
    rows: list[tuple[float, float, float, float, float, float, float, float]] = []
    seen_children: set[str] = set()

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != target:
            continue
        msg = deserialize_message(data, msg_cls)

        if args.pose_topic:
            # Prefer the message header stamp; fall back to bag receive time.
            st = msg.header.stamp
            ts = st.sec + st.nanosec * 1e-9
            if ts <= 0.0:
                ts = t_ns * 1e-9
            if clock_pairs:
                ts = to_sim(t_ns * 1e-9)
            p, q = msg.pose.position, msg.pose.orientation
            rows.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif args.tf_index is not None:
            if args.tf_index >= len(msg.transforms):
                continue
            tr = msg.transforms[args.tf_index]
            st = tr.header.stamp
            ts = st.sec + st.nanosec * 1e-9
            if ts <= 0.0:
                ts = t_ns * 1e-9
            if clock_pairs:
                ts = to_sim(t_ns * 1e-9)
            p, q = tr.transform.translation, tr.transform.rotation
            rows.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        else:
            for tr in msg.transforms:
                seen_children.add(tr.child_frame_id)
                if tr.child_frame_id != args.tf_child:
                    continue
                st = tr.header.stamp
                ts = st.sec + st.nanosec * 1e-9
                if ts <= 0.0:
                    ts = t_ns * 1e-9
                if clock_pairs:
                    ts = to_sim(t_ns * 1e-9)
                p, q = tr.transform.translation, tr.transform.rotation
                rows.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    if not rows:
        print(f"bag_to_tum: no samples extracted from {target!r}", file=sys.stderr)
        if seen_children:
            print(f"  child_frame_ids present: {sorted(seen_children)}", file=sys.stderr)
            print(f"  (you asked for {args.tf_child!r})", file=sys.stderr)
        return 3

    rows.sort(key=lambda r: r[0])
    # evo rejects non-monotonic stamps; drop duplicates/regressions.
    cleaned, last = [], None
    for r in rows:
        if last is None or r[0] > last:
            cleaned.append(r)
            last = r[0]

    with open(args.out, "w") as fh:
        for r in cleaned:
            fh.write(" ".join(f"{v:.9f}" for v in r) + "\n")

    dropped = len(rows) - len(cleaned)
    span = cleaned[-1][0] - cleaned[0][0]
    print(f"bag_to_tum: wrote {len(cleaned)} poses to {args.out} "
          f"({span:.1f} s span"
          + (f", dropped {dropped} non-monotonic" if dropped else "") + ")")
    return 0


if __name__ == "__main__":
    sys.exit(main())
