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


def open_reader(path: str) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
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
    ap.add_argument("--out", required=True)
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

    target = args.pose_topic or args.tf_topic
    if not target:
        ap.error("need --pose-topic or --tf-topic (or --list)")
    if target not in types:
        print(f"bag_to_tum: topic {target!r} not in bag. Available:", file=sys.stderr)
        for t in sorted(types):
            print(f"  {t}  ({types[t]})", file=sys.stderr)
        return 2
    if args.tf_topic and not args.tf_child:
        ap.error("--tf-topic requires --tf-child (the TF stream carries every model)")

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
            p, q = msg.pose.position, msg.pose.orientation
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
