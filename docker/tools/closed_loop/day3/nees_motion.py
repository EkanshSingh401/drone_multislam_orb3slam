#!/usr/bin/env python3
"""nees_motion.py <out.json> <flight_dir>... (day 3 step 4): orientation NEES vs motion and features.

Per flight, on the GT airborne window (nees_window.py, full-covariance orientation NEES from
est_cov_full.txt = /openvins/joint_covariance IMU block), per NEES sample:
  yaw rate  |omega_z| from GT orientation (IMU frame -> world z), central differences
  speed     GT horizontal speed
  n_slam    SLAM landmarks in the joint covariance at that update (features in the state)
  n_msckf   MSCKF features used in the update (/ov_msckf/points_msckf), if recorded
  n_slam_upd SLAM features in /ov_msckf/points_slam, if recorded
Per-flight summaries (means; NEES also median) and pooled per-sample bins. Run in the container.
"""
import json, os, subprocess, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

def bag_series(d):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/openvins/joint_covariance', '/ov_msckf/points_msckf', '/ov_msckf/points_slam') if t in names]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    types = {t: get_message(names[t]) for t in want}
    out = {t: [] for t in want}
    while r.has_next():
        tp, raw, _ = r.read_next()
        m = deserialize_message(raw, types[tp])
        ts = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if tp == '/openvins/joint_covariance':
            if m.stage in ('', 'post'): out[tp].append((ts, len(m.landmarks)))
        else:
            out[tp].append((ts, m.width * m.height))
    return {k: np.array(v) for k, v in out.items()}

def nearest(series, t):
    if series is None or len(series) == 0: return np.full(len(t), np.nan)
    i = np.clip(np.searchsorted(series[:, 0], t), 1, len(series) - 1)
    j = np.where(np.abs(series[i - 1, 0] - t) < np.abs(series[i, 0] - t), i - 1, i)
    v = series[j, 1].astype(float)
    v[np.abs(series[j, 0] - t) > 0.2] = np.nan
    return v

def flight(d):
    dump = f'{d}/nees_full_dump.txt'
    p = subprocess.run(['python3', '/out/diag_s54/nees_window.py', d, f'{d}/gt.tum', '--est', 'est_cov_full.txt', '--dump', dump],
                       capture_output=True, text=True)
    if not os.path.exists(dump): return None
    nd = np.loadtxt(dump)
    t, ori, rp = nd[:, 0], nd[:, 2], nd[:, 3]
    g = np.loadtxt(f'{d}/gt_imu.txt')
    gt_t, keep = np.unique(g[:, 0], return_index=True); g = g[keep]
    rot = R.from_quat(g[:, 4:8])
    yaw = np.unwrap(rot.as_euler('ZYX')[:, 0])
    yr = np.gradient(yaw, gt_t); vx = np.gradient(g[:, 1], gt_t); vy = np.gradient(g[:, 2], gt_t)
    # smooth over ~0.2 s (GT at ~50 Hz) to suppress differencing noise
    k = np.ones(9) / 9
    yr = np.convolve(yr, k, 'same'); sp = np.convolve(np.hypot(vx, vy), k, 'same')
    yr_s, sp_s = np.abs(np.interp(t, gt_t, yr)), np.interp(t, gt_t, sp)
    S = bag_series(d)
    nsl = nearest(S.get('/openvins/joint_covariance'), t)
    nms = nearest(S.get('/ov_msckf/points_msckf'), t)
    nsu = nearest(S.get('/ov_msckf/points_slam'), t)
    return dict(t=t, ori=ori, rp=rp, yaw_rate=yr_s, speed=sp_s, n_slam=nsl, n_msckf=nms, n_slam_upd=nsu)

def main():
    out_path, dirs = sys.argv[1], sys.argv[2:]
    per, pooled = [], []
    for d in dirs:
        f = flight(d)
        if f is None: print('skip', d, file=sys.stderr); continue
        name = os.path.basename(d.rstrip('/'))
        s = {'flight': name, 'n': int(len(f['t'])), 'nees_ori_mean': float(np.mean(f['ori'])), 'nees_ori_median': float(np.median(f['ori'])),
             'nees_rp_mean': float(np.mean(f['rp']))}
        for k in ('yaw_rate', 'speed', 'n_slam', 'n_msckf', 'n_slam_upd'):
            v = f[k]
            s[k + '_mean'] = float(np.nanmean(v)) if np.isfinite(v).any() else None
        s['frac_yaw_rate_gt_0.3'] = float(np.mean(f['yaw_rate'] > 0.3))
        s['frac_n_slam_lt_10'] = float(np.nanmean(f['n_slam'] < 10)) if np.isfinite(f['n_slam']).any() else None
        np.savez(f'{d}/nees_motion.npz', **f)
        per.append(s)
        pooled.append(np.column_stack([f['ori'], f['yaw_rate'], f['speed'], f['n_slam'], f['n_msckf'], f['n_slam_upd']]))
        print(json.dumps(s))
    P = np.vstack(pooled)
    bins = {}
    for j, (name, edges) in enumerate([('yaw_rate', [0, 0.05, 0.15, 0.3, 0.6, 3]), ('speed', [0, 0.1, 0.3, 0.5, 2]),
                                       ('n_slam', [0, 5, 10, 20, 35, 51]), ('n_msckf', [0, 10, 20, 30, 41]), ('n_slam_upd', [0, 5, 10, 20, 35, 51])], 1):
        x = P[:, j]; rows = []
        for a, b in zip(edges[:-1], edges[1:]):
            m = (x >= a) & (x < b) & np.isfinite(x)
            if m.sum() >= 20: rows.append({'bin': [a, b], 'n': int(m.sum()), 'nees_mean': float(P[m, 0].mean()), 'nees_median': float(np.median(P[m, 0]))})
        bins[name] = rows
    json.dump({'flights': per, 'pooled_bins': bins}, open(out_path, 'w'), indent=1)

main()
