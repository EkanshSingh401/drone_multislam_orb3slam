#!/usr/bin/env python3
"""nees_vs_rate.py <replay_dir> <flight_dir> (day 5 step 4b): roll/pitch, yaw, full orientation NEES
(nees_dump.txt from nees_replay.sh) binned by GT angular rate (|omega| from gt_imu.txt orientations,
1 s central differences) and, for scripted flights, by /script_goals/segment. Prints JSON."""
import json, sys, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
od, rd = sys.argv[1], sys.argv[2]
n = np.loadtxt(f'{od}/nees_dump.txt'); gt = np.loadtxt(f'{rd}/gt_imu.txt'); gt = gt[np.unique(gt[:, 0], return_index=True)[1]]
def rate(t):
    a = np.clip(np.searchsorted(gt[:, 0], t - 0.5), 0, len(gt) - 1); b = np.clip(np.searchsorted(gt[:, 0], t + 0.5), 0, len(gt) - 1)
    dt = gt[b, 0] - gt[a, 0]
    return np.linalg.norm((R.from_quat(gt[a, 4:8]).inv() * R.from_quat(gt[b, 4:8])).as_rotvec()) / dt if dt > 0 else np.nan
def speed(t):
    a = np.clip(np.searchsorted(gt[:, 0], t - 0.5), 0, len(gt) - 1); b = np.clip(np.searchsorted(gt[:, 0], t + 0.5), 0, len(gt) - 1)
    return np.linalg.norm(gt[b, 1:4] - gt[a, 1:4]) / max(gt[b, 0] - gt[a, 0], 1e-6)
w = np.array([rate(t) for t in n[:, 0]]); v = np.array([speed(t) for t in n[:, 0]])
edges = [0, 0.02, 0.05, 0.1, 0.2, 0.4, 10]
out = {'replay': od, 'n': len(n), 'by_rate': []}
for lo, hi in zip(edges[:-1], edges[1:]):
    k = (w >= lo) & (w < hi)
    if k.sum() >= 5:
        out['by_rate'].append({'rate': [lo, hi], 'n': int(k.sum()), 'rp_med': round(float(np.median(n[k, 2])), 2), 'yaw_med': round(float(np.median(n[k, 3])), 3),
                               'ori_med': round(float(np.median(n[k, 1])), 2), 'rp_mean': round(float(np.mean(n[k, 2])), 2)})
# segments (scripted flights)
try:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{rd}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/script_goals/segment'])); seg = []
    while r.has_next():
        _, b, ts = r.read_next(); seg.append((ts, deserialize_message(b, String).data))
    if seg:
        # bag receive time -> sim time: use /clock-free approximation via the dump's first/last? use header-less: map with odom
        r2 = rosbag2_py.SequentialReader(); r2.open(rosbag2_py.StorageOptions(uri=f'{rd}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        from rosgraph_msgs.msg import Clock
        r2.set_filter(rosbag2_py.StorageFilter(topics=['/clock'])); ck = []
        while r2.has_next():
            _, b, ts = r2.read_next(); c = deserialize_message(b, Clock).clock; ck.append((ts, c.sec + 1e-9 * c.nanosec))
        ck = np.array(ck); st = np.interp([s[0] for s in seg], ck[:, 0], ck[:, 1])
        lab = np.array([seg[max(0, np.searchsorted(st, t) - 1)][1] if t >= st[0] else 'pre' for t in n[:, 0]])
        lab[n[:, 0] > st[-1] + 2.0] = 'post'
        out['by_segment'] = []
        for s in dict.fromkeys(x[1] for x in seg):
            k = lab == s
            if k.sum() >= 3:
                out['by_segment'].append({'seg': s, 'n': int(k.sum()), 'rate_med': round(float(np.nanmedian(w[k])), 3), 'speed_med': round(float(np.median(v[k])), 2),
                                          'rp_med': round(float(np.median(n[k, 2])), 2), 'yaw_med': round(float(np.median(n[k, 3])), 3), 'ori_med': round(float(np.median(n[k, 1])), 2)})
except Exception as e:
    out['seg_error'] = str(e)
print(json.dumps(out))
