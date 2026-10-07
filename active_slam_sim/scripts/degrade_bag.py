#!/usr/bin/env python3
"""Apply a camera degradation preset to a recorded flight bag (overnight Stage 5).

    degrade_bag.py <in_bag> <out_bag> <clean|mild|moderate|severe> [--seed N]

Both IR streams are degraded with active_slam_sim_py.degrade (independent noise
per camera); motion blur uses the IMU gyro averaged over each frame's exposure
window, rotated into the optical frame (cameras are axis-aligned with the IMU:
optical = (-y, -z, x) of FLU). All other topics, stamps and receive times are
copied unchanged, so OpenVINS replays it exactly like the clean bag.
"""
import argparse
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
try:
    from active_slam_sim_py.degrade import PRESETS, degrade
except ImportError:  # installed layout
    sys.path.insert(0, "/opt/active_slam_sim_py")
    from degrade import PRESETS, degrade  # noqa

FX = FY = 446.802773
R_OPT_FROM_FLU = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)


def reader(path):
    info = rosbag2_py.Info().read_metadata(path, "")
    r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    return r


ap = argparse.ArgumentParser()
ap.add_argument("inbag"); ap.add_argument("outbag"); ap.add_argument("preset", choices=sorted(PRESETS))
ap.add_argument("--seed", type=int, default=5)
a = ap.parse_args()
d = PRESETS[a.preset]

# pass 1: gyro series
r = reader(a.inbag); r.set_filter(rosbag2_py.StorageFilter(topics=["/camera/imu"]))
g = []
while r.has_next():
    _, b, _ = r.read_next(); m = deserialize_message(b, Imu)
    g.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
g = np.array(sorted(g))

def omega_opt(t):
    t0 = t - max(d.exposure_s, 1e-3)
    s = (g[:, 0] >= t0) & (g[:, 0] <= t)
    w = g[s, 1:].mean(0) if s.any() else np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    return R_OPT_FROM_FLU @ w

rngs = {"/camera/infra1/image_rect_raw": np.random.default_rng(a.seed), "/camera/infra2/image_rect_raw": np.random.default_rng(a.seed + 1)}
r = reader(a.inbag)
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=a.outbag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
for i, tp in enumerate(r.get_all_topics_and_types()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=tp.name, type=tp.type, serialization_format="cdr"))
n = 0
while r.has_next():
    topic, data, trecv = r.read_next()
    if topic in rngs:
        m = deserialize_message(data, Image)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        m.data = degrade(img, d, omega_opt(t), FX, FY, rngs[topic]).tobytes()
        data = serialize_message(m); n += 1
    w.write(topic, data, trecv)
del w
print(f"degrade_bag: {n} images degraded with preset {a.preset} -> {a.outbag}")
