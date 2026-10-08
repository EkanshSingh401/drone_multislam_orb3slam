#!/usr/bin/env python3
"""make_traj.py <gt_imu.txt> <out.txt>: OpenVINS simulator trajectory from a flight's GT IMU poses:
airborne part (z > 0.5 m, from 3 s after first exceeding it to the last time above it), resampled to
20 Hz (position: 0.25 s moving average to remove GT stamp jitter; rotation: slerp), time from 0.
Format: t x y z qx qy qz qw (q = R_ItoG, Hamilton)."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
g = np.loadtxt(sys.argv[1]); g = g[np.unique(g[:, 0], return_index=True)[1]]
up = np.nonzero(g[:, 3] > 0.5)[0]; t0 = g[up[0], 0] + 3.0; t1 = g[up[-1], 0] - 1.0
t = np.arange(t0, t1, 0.05)
p = np.column_stack([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
k = 5; p = np.vstack([np.convolve(np.pad(p[:, j], (k // 2, k // 2), mode='edge'), np.ones(k) / k, mode='valid') for j in range(3)]).T
q = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))(t).as_quat()
np.savetxt(sys.argv[2], np.column_stack([t - t[0], p, q]), fmt='%.6f', header='t x y z qx qy qz qw (from %s)' % sys.argv[1])
w = np.linalg.norm((R.from_quat(q[:-1]).inv() * R.from_quat(q[1:])).as_rotvec(), axis=1) / 0.05
print(sys.argv[2], len(t), 'samples, %.1f s, median |omega| %.3f rad/s' % (t[-1] - t[0], np.median(w)))
