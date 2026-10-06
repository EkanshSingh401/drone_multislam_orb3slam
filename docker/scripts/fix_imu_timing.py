#!/usr/bin/env python3
"""Correct the Gazebo IMU timing in a recorded bag (PATCHES s56). Run in the sim container.

    fix_imu_timing.py <in_bag> <out_bag> [--gyro-lag-ms 2.0] [--acc-lag-ms A]

Measured (s56, imu_lag.py / imu_vs_gt_robust.py): Gazebo's IMU sample stamped t
describes the motion at t - 2.0 ms for the gyro (constant over the 4,4,4,8 ms
stamp pattern: half a 4 ms physics step) and at about t - 4.4 ms for the
accelerometer. The fix:
  * every /camera/imu header stamp moves to t - gyro_lag (gyro is then exact);
  * with --acc-lag-ms, the accelerometer is re-timed as well: sample k is
    replaced by the linear interpolation of the accel series (placed at its
    true times t_j - acc_lag) at the new stamp t_k - gyro_lag.
Everything else (images, GT, /clock) is copied byte-for-byte with its receive
time. Writes uncompressed MCAP.
"""
import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser()
ap.add_argument("inbag"); ap.add_argument("outbag")
ap.add_argument("--gyro-lag-ms", type=float, default=2.0)
ap.add_argument("--acc-lag-ms", type=float, default=None)
a = ap.parse_args()
LG = a.gyro_lag_ms * 1e-3


def reader(path):
    info = rosbag2_py.Info().read_metadata(path, "")
    r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    return r


# pass 1: the accel series (only needed for --acc-lag-ms)
acc_t = acc = None
if a.acc_lag_ms is not None:
    r = reader(a.inbag); r.set_filter(rosbag2_py.StorageFilter(topics=["/camera/imu"]))
    rows = []
    while r.has_next():
        _, d, _ = r.read_next(); m = deserialize_message(d, Imu)
        rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.linear_acceleration.x,
                     m.linear_acceleration.y, m.linear_acceleration.z])
    rows = np.array(rows)
    acc_t, acc = rows[:, 0] - a.acc_lag_ms * 1e-3, rows[:, 1:]

r = reader(a.inbag)
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=a.outbag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
for i, tp in enumerate(r.get_all_topics_and_types()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=tp.name, type=tp.type, serialization_format="cdr"))
n = 0
while r.has_next():
    topic, data, trecv = r.read_next()
    if topic == "/camera/imu":
        m = deserialize_message(data, Imu)
        ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec - int(round(LG * 1e9))
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        if acc is not None:
            tn = ns * 1e-9
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
                (float(np.interp(tn, acc_t, acc[:, j])) for j in range(3))
        data = serialize_message(m); n += 1
    w.write(topic, data, trecv)
del w
print(f"fix_imu_timing: {n} IMU messages, stamps -{a.gyro_lag_ms} ms"
      + (f", accel re-timed for {a.acc_lag_ms} ms lag" if acc is not None else ""))
