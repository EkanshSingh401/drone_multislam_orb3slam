#!/usr/bin/env python3
"""Swap test (PATCHES s54): rewrite a flight bag with a synthetic /camera/imu.

    python3 synth_imu.py <rundir> <outbag> <variant> [--seed N]

variant  cfg   white noise + bias random walk exactly per the OpenVINS config
               (density/sqrt(dt) per sample, RW*sqrt(dt) per step), bias starts at 0
         gz    same white noise, bias random walk at Gazebo's EFFECTIVE density
               (dynamic_bias_stddev used as a density, gz-sensors8 GaussianNoiseModel.cc:135),
               started from N(0, sigma_b^2 * t) at the first IMU stamp (bias = 0 at sim t=0)
Signal: GT (base_link, /clock-mapped stamps) snapped to the 4 ms grid with a
leave-one-out check for mis-snaps, moved to the IMU point, interpolating cubic
spline (position) and slerp-free rotation spline; specific force = R^T (a + g z),
body rate by symmetric finite difference of the rotation spline. Every other
topic is copied byte-for-byte with its original receive time; IMU messages keep
their original stamps and receive times.
"""
import sys, argparse
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R, RotationSpline
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu

ap = argparse.ArgumentParser(); ap.add_argument("rundir"); ap.add_argument("outbag"); ap.add_argument("variant", choices=["cfg", "gz"])
ap.add_argument("--seed", type=int, default=1); a = ap.parse_args()
STEP = 0.004; P_IMU = np.array([0.12 - 0.00552, 0.0051, 0.242 - 0.01174]); g = 9.81
DENS = {"g": 1.6e-4, "a": 2e-3}
RW = {"cfg": {"g": 2e-6, "a": 3e-4}, "gz": {"g": 8.48528137e-05, "a": 0.0127279221}}[a.variant]
rng = np.random.default_rng(a.seed)

gt = np.loadtxt(f"{a.rundir}/gt.tum")
gt[:, 0] = np.round(gt[:, 0] / STEP) * STEP
gt = gt[np.r_[True, np.diff(gt[:, 0]) > 1e-6]]
# mis-snap repair: each interior knot vs a cubic through its 4 neighbours, try t, t+-4 ms
t = gt[:, 0].copy(); fixed = 0
for i in range(2, len(t) - 2):
    nb = [i - 2, i - 1, i + 1, i + 2]
    c = [np.polyfit(t[nb] - t[i], gt[nb, k], 3) for k in (1, 2, 3)]
    err = [np.linalg.norm([np.polyval(cc, d) - gt[i, k + 1] for k, cc in enumerate(c)]) for d in (0, -STEP, STEP)]
    j = int(np.argmin(err))
    if j and err[j] < 0.3 * err[0] and t[i - 1] < t[i] + (-STEP, STEP)[j - 1] < t[i + 1]:
        t[i] += (-STEP, STEP)[j - 1]; fixed += 1
gt[:, 0] = t
gt = gt[np.r_[True, np.diff(gt[:, 0]) > 1e-6]]
Rk = R.from_quat(gt[:, 4:8]); pk = gt[:, 1:4] + Rk.apply(P_IMU)
ps = CubicSpline(gt[:, 0], pk); rs = RotationSpline(gt[:, 0], Rk)

def truth(ts):
    acc = ps(ts, 2) + np.array([0, 0, g]); Rt = rs(ts)
    f = Rt.inv().apply(acc)
    eps = 5e-4
    w = (rs(ts - eps).inv() * rs(ts + eps)).as_rotvec() / (2 * eps)
    return w, f

info = rosbag2_py.Info().read_metadata(f"{a.rundir}/flight.bag", "")
rd = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=f"{a.rundir}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
topics = rd.get_all_topics_and_types()
wr = rosbag2_py.SequentialWriter()
wr.open(rosbag2_py.StorageOptions(uri=a.outbag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
for i, tp in enumerate(topics):
    wr.create_topic(rosbag2_py.TopicMetadata(id=i, name=tp.name, type=tp.type, serialization_format="cdr"))

bg = ba = None; last = None; n = 0; resid = []
while rd.has_next():
    topic, data, trecv = rd.read_next()
    if topic == "/camera/imu":
        m = deserialize_message(data, Imu); ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        dt = 0.005 if last is None else ts - last
        if bg is None:  # bias at first sample
            bg = np.zeros(3) if a.variant == "cfg" else rng.normal(0, RW["g"] * np.sqrt(ts), 3)
            ba = np.zeros(3) if a.variant == "cfg" else rng.normal(0, RW["a"] * np.sqrt(ts), 3)
        else:
            bg = bg + rng.normal(0, RW["g"] * np.sqrt(dt), 3); ba = ba + rng.normal(0, RW["a"] * np.sqrt(dt), 3)
        last = ts
        if gt[0, 0] + 0.05 < ts < gt[-1, 0] - 0.05:
            w, f = truth(np.array([ts]))
            w, f = w[0], f[0]
        else:  # outside GT coverage: at rest
            w, f = np.zeros(3), np.array([0, 0, g])
        resid.append(np.r_[ts, np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]) - f])
        w = w + bg + rng.normal(0, DENS["g"] / np.sqrt(dt), 3)
        f = f + ba + rng.normal(0, DENS["a"] / np.sqrt(dt), 3)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, w)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, f)
        data = serialize_message(m); n += 1
    wr.write(topic, data, trecv)
del wr
r = np.array(resid); mad = 1.4826 * np.median(np.abs(r[:, 1:] - np.median(r[:, 1:], 0)), 0)
print(f"synth_imu {a.variant}: {n} IMU msgs, GT knots {len(gt)}, mis-snaps repaired {fixed}; "
      f"real-minus-spline accel MAD {mad.round(4)} (Gazebo white 0.0283/sample)")
