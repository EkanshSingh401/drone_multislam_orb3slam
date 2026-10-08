#!/usr/bin/env python3
"""sim_nees.py <ovsim_dir>: full-covariance orientation NEES for OpenVINS-simulator runs (same world frame,
no alignment): ori (3), roll/pitch (2, perpendicular to gravity), yaw (1); binned by true |omega|.
est/gt files: t, JPL q_GtoI (xyzw) = Hamilton q_ItoG, p. Covariance: jc_imu.txt 6x6 [dth dp] (local)."""
import json, sys, numpy as np
from scipy.spatial.transform import Rotation as R
d = sys.argv[1]
e = np.loadtxt(f'{d}/ov_state_est.txt'); g = np.loadtxt(f'{d}/ov_state_gt.txt'); c = np.loadtxt(f'{d}/jc_imu.txt')
gi = {round(t, 4): k for k, t in enumerate(g[:, 0])}; ei = {round(t, 4): k for k, t in enumerate(e[:, 0])}
rows = []
for r in c:
    t = round(r[0], 4)
    if t not in gi or t not in ei or t < 10: continue
    G, E = g[gi[t]], e[ei[t]]
    Rt, Re = R.from_quat(G[1:5]), R.from_quat(E[1:5])            # ItoG
    dth = (Rt.inv() * Re).as_rotvec(); P = r[1:].reshape(6, 6)[:3, :3]
    gI = Rt.inv().apply([0, 0, 1.0]); B = np.linalg.svd(np.eye(3) - np.outer(gI, gI))[0][:, :2]; e2 = B.T @ dth
    k = gi[t]; k2 = min(k + 15, len(g) - 1); k1 = max(k - 15, 0)
    w = np.linalg.norm((R.from_quat(g[k1, 1:5]).inv() * R.from_quat(g[k2, 1:5])).as_rotvec()) / (g[k2, 0] - g[k1, 0])
    rows.append([t, dth @ np.linalg.solve(P, dth), e2 @ np.linalg.solve(B.T @ P @ B, e2), (gI @ dth) ** 2 / (gI @ P @ gI), w])
A = np.array(rows); out = {'dir': d, 'n': len(A), 'ori_med': round(float(np.median(A[:, 1])), 2), 'ori_mean': round(float(A[:, 1].mean()), 2),
                         'rp_med': round(float(np.median(A[:, 2])), 2), 'rp_mean': round(float(A[:, 2].mean()), 2), 'yaw_med': round(float(np.median(A[:, 3])), 3), 'by_rate': []}
for lo, hi in zip([0, 0.02, 0.05, 0.1, 0.2, 0.4], [0.02, 0.05, 0.1, 0.2, 0.4, 10]):
    k = (A[:, 4] >= lo) & (A[:, 4] < hi)
    if k.sum() >= 5: out['by_rate'].append({'rate': [lo, hi], 'n': int(k.sum()), 'rp_med': round(float(np.median(A[k, 2])), 2), 'ori_med': round(float(np.median(A[k, 1])), 2)})
print(json.dumps(out))
