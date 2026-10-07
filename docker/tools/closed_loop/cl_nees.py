#!/usr/bin/env python3
"""cl_nees.py <flight_dir> : diag and full-covariance NEES for a closed-loop flight (overnight Stage 1d).
Needs ov_prep.py to have produced est_cov.txt / gt_imu.txt; extracts the IMU pose block of every
/openvins/joint_covariance ('post' stage) from flight.bag into jc_imu.txt, then full_cov_est.py and
nees_window.py (diag over all states; diag and full on the joint-covariance subset)."""
import sys, json, subprocess, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from active_slam_msgs.msg import JointCovariance
d = sys.argv[1]
subprocess.run(['python3', '/opt/scripts/ov_prep.py', d, '--gt-bag', f'{d}/flight.bag'], capture_output=True)
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/openvins/joint_covariance'])); rows = []
while r.has_next():
    _, b, _ = r.read_next(); m = deserialize_message(b, JointCovariance)
    if m.stage not in ('', 'post'): continue
    n = m.dim; C = np.array(m.covariance).reshape(n, n); bi = next(x for x in m.blocks if x.type == 'imu')
    rows.append(np.r_[m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, C[bi.index:bi.index + 6, bi.index:bi.index + 6].ravel()])
np.savetxt(f'{d}/jc_imu.txt', np.array(rows), fmt='%.12g')
subprocess.run(['python3', '/out/diag_s54/full_cov_est.py', d, f'{d}/jc_imu.txt'], capture_output=True)
e = np.loadtxt(f'{d}/est_cov.txt'); f = np.loadtxt(f'{d}/est_cov_full.txt')
np.savetxt(f'{d}/est_cov_diagsub.txt', e[np.isin(np.round(e[:, 0], 5), np.round(f[:, 0], 5))], fmt='%.12g')
out = {'jc_msgs': len(rows)}
for k, est in (('diag_all', 'est_cov.txt'), ('diag_sub', 'est_cov_diagsub.txt'), ('full_sub', 'est_cov_full.txt')):
    p = subprocess.run(['python3', '/out/diag_s54/nees_window.py', d, f'{d}/gt.tum', '--est', est], capture_output=True, text=True)
    j = json.loads([l for l in p.stdout.splitlines() if l.startswith('{')][-1])
    out[k] = {'n': j['n'], 'pos': round(j['pos']['mean'], 3), 'ori': round(j['ori']['mean'], 3), 'rp': round(j['rp']['mean'], 3),
              'ori_in95': round(j['ori']['in95'], 3)}
print(json.dumps(out))
