#!/usr/bin/env python3
"""imu_long.py <flight_dir> [t_from t_to] (day 5 step 4a, in flight): IMU error over LONG spans, where the
GT stamp jitter that defeated s54's 12-20 ms knots no longer matters.

Accel: second difference of the GT IMU position over span T, p(t+2T) - 2 p(t+T) + p(t), equals the
hat-weighted integral of world acceleration, int M(s) (R_ItoG(s) f(s) - g e_z) ds, M(s) = T - |s - t - T|
(IMU samples, GT attitude slerped). Gyro: Log(R(t)^T R(t+T)) vs int omega ds (small rotation).
Residuals on non-overlapping windows; consecutive differences remove a constant bias. Compared with the
same operators applied to the CONFIGURED noise (white density + bias random walk, OpenVINS config) on
the same stamps: ratio = measured / expected std (1 = the sensor generates what OpenVINS assumes).
GT = gt_imu.txt (IMU point, IMU frame orientation). Prints JSON."""
import json, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R, Slerp

DENS_G, DENS_A, RW_G, RW_A, G = 1.6e-4, 2e-3, 2e-6, 3e-4, 9.81
d = sys.argv[1]
g = np.loadtxt(f'{d}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]
up = np.nonzero(g[:, 3] > 0.5)[0]
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else g[up[0], 0] + 5
t_to = float(sys.argv[3]) if len(sys.argv) > 3 else g[up[-1], 0] - 5
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/imu'])); im = []
while r.has_next():
    _, b, _ = r.read_next(); m = deserialize_message(b, Imu)
    im.append([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
               m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
im = np.array(im); im = im[(im[:, 0] > t_from - 3) & (im[:, 0] < t_to + 3)]
ti = im[:, 0]; dt = np.r_[np.diff(ti), np.diff(ti)[-1]]
Rg = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))(ti)
aw = Rg.apply(im[:, 4:7]) - np.array([0, 0, G])           # world acceleration from the IMU
P = lambda t: np.column_stack([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
rng = np.random.default_rng(0)

def synth_noise(dens, rw):
    """configured noise on the same stamps: white (density / sqrt(dt) per sample) + random-walk bias."""
    w = rng.normal(0, 1, (len(ti), 3)) * dens / np.sqrt(dt)[:, None]
    b = np.cumsum(rng.normal(0, 1, (len(ti), 3)) * rw * np.sqrt(dt)[:, None], 0)
    return w + b

out = {'flight': d, 'window': [round(t_from, 1), round(t_to, 1)], 'n_imu': len(ti), 'spans': []}
na, ng = synth_noise(DENS_A, RW_A), synth_noise(DENS_G, RW_G)
for T in (0.1, 0.25, 0.5, 1.0, 2.0):
    starts = np.arange(t_from, t_to - 2 * T, 2 * T)
    ra, rg, sa, sg = [], [], [], []
    for t0 in starts:
        k = (ti >= t0) & (ti < t0 + 2 * T); s = ti[k]; M = T - np.abs(s - t0 - T)
        lhs = P(np.array([t0 + 2 * T]))[0] - 2 * P(np.array([t0 + T]))[0] + P(np.array([t0]))[0]
        ra.append(lhs - (M[:, None] * aw[k] * dt[k][:, None]).sum(0)); sa.append((M[:, None] * Rg[k].apply(na[k]) * dt[k][:, None]).sum(0))
        k2 = (ti >= t0) & (ti < t0 + T)
        Ra, Rb = R.from_quat(g[np.searchsorted(g[:, 0], t0), 4:8]), R.from_quat(g[np.searchsorted(g[:, 0], t0 + T), 4:8])
        rg.append((Ra.inv() * Rb).as_rotvec() - (im[k2, 1:4] * dt[k2][:, None]).sum(0)); sg.append((ng[k2] * dt[k2][:, None]).sum(0))
    ra, rg, sa, sg = map(np.array, (ra, rg, sa, sg))
    dd = lambda x: np.diff(x, axis=0) / np.sqrt(2)          # bias-free: consecutive-window differences
    mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x, 0)), 0)
    out['spans'].append({'T_s': T, 'n': len(starts),
                         'acc_meas_mad_xyz': np.round(mad(dd(ra)), 6).tolist(), 'acc_cfg_mad_xyz': np.round(mad(dd(sa)), 6).tolist(),
                         'acc_ratio_xyz': np.round(mad(dd(ra)) / mad(dd(sa)), 2).tolist(),
                         'gyr_meas_mad_xyz': np.round(mad(dd(rg)), 7).tolist(), 'gyr_cfg_mad_xyz': np.round(mad(dd(sg)), 7).tolist(),
                         'gyr_ratio_xyz': np.round(mad(dd(rg)) / mad(dd(sg)), 2).tolist()})
print(json.dumps(out))
