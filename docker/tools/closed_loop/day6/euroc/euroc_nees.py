#!/usr/bin/env python3
"""euroc_nees.py <run_dir> <seq_dir> (day 6 step 2): OpenVINS consistency on EuRoC by motion.

GT: asl/mav0/state_groundtruth_estimate0/data.csv (IMU/body frame, ns, p, q_wxyz). Estimate: ov_state_est.txt
(t, JPL q_GtoI = Hamilton q_ItoG, p); covariance: jc_imu.txt (full 6x6 [dth dp], local), samples > 10 s after
the first estimate. NEES: roll/pitch (2 DoF, gravity-relative, no alignment, as day4/nees_yaw.py), yaw (1 DoF)
and full orientation (3) and position (3) after a position+yaw alignment over the sequence.
Motion from GT (1 s central differences): |omega| bins and classes hover (speed < 0.1 m/s and |omega| <
0.05 rad/s), slow (speed < 0.3 and |omega| < 0.15, not hover), fast (rest). Prints JSON."""
import json, sys
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
od, sd = sys.argv[1], sys.argv[2]
g = np.loadtxt(f'{sd}/asl/mav0/state_groundtruth_estimate0/data.csv', delimiter=',', comments='#')
tg = g[:, 0] * 1e-9; pg = g[:, 1:4]; qg = R.from_quat(g[:, [5, 6, 7, 4]])
e = np.loadtxt(f'{od}/ov_state_est.txt'); c = np.loadtxt(f'{od}/jc_imu.txt')
def near(tq):
    k = np.searchsorted(e[:, 0], tq); k = np.clip(k, 1, len(e) - 1); k = np.where(np.abs(e[k - 1, 0] - tq) < np.abs(e[k, 0] - tq), k - 1, k)
    return k
kk = near(c[:, 0]); ok = np.abs(e[kk, 0] - c[:, 0]) < 6e-3   # jc_imu.txt stamps have 12 significant digits (10 ms at epoch time)
rows = [(r, e[k]) for r, k, o in zip(c, kk, ok) if o]
t = np.array([x[1][0] for x in rows]); keep = (t > e[0, 0] + 10) & (t > tg[0]) & (t < tg[-1]); rows = [x for x, k in zip(rows, keep) if k]; t = t[keep]
Rg = Slerp(tg, qg)(t)
P = np.column_stack([np.interp(t, tg, pg[:, k]) for k in range(3)])
Re = R.from_quat(np.array([x[1][1:5] for x in rows])); pe = np.array([x[1][5:8] for x in rows])
Cv = np.array([x[0][1:].reshape(6, 6) for x in rows])
A, B = pe - pe.mean(0), P - P.mean(0)
yaw = np.arctan2((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum(), (A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum())
Ry = R.from_euler('z', yaw); pa = Ry.apply(pe - pe.mean(0)) + P.mean(0); Rea = Ry * Re
OFFSET = '--gt-offset' in sys.argv   # remove one constant GT-body-to-IMU rotation (fitted after the world-yaw alignment)
if OFFSET:
    Roff = (Rg.inv() * Rea).mean(); Rg = Rg * Roff
def motion(tt):
    a = np.clip(np.searchsorted(tg, tt - 0.5), 0, len(tg) - 1); b = np.clip(np.searchsorted(tg, tt + 0.5), 0, len(tg) - 1); dt = tg[b] - tg[a]
    return np.linalg.norm((qg[a].inv() * qg[b]).as_rotvec()) / dt, np.linalg.norm(pg[b] - pg[a]) / dt
M = np.array([motion(x) for x in t]); w, v = M[:, 0], M[:, 1]
ori, rp, yw, pos = [], [], [], []; rperr, rpsig = [], []
for i in range(len(t)):
    Pr = Cv[i, :3, :3]; dth = (Rea[i].inv() * Rg[i]).as_rotvec(); gI = Rg[i].inv().apply([0, 0, 1.0])
    ori.append(dth @ np.linalg.solve(Pr, dth)); yw.append((gI @ dth) ** 2 / (gI @ Pr @ gI))
    d0 = (Re[i].inv() * Rg[i]).as_rotvec(); Bm = np.linalg.svd(np.eye(3) - np.outer(gI, gI))[0][:, :2]; e2 = Bm.T @ d0
    rp.append(e2 @ np.linalg.solve(Bm.T @ Pr @ Bm, e2)); rperr.append(np.linalg.norm(e2)); rpsig.append(np.sqrt(np.trace(Bm.T @ Pr @ Bm)))
    dp = pa[i] - P[i]; Pp = Ry.as_matrix() @ Cv[i, 3:, 3:] @ Ry.as_matrix().T; pos.append(dp @ np.linalg.solve(Pp, dp))
ori, rp, yw, pos = map(np.array, (ori, rp, yw, pos))
med = lambda x: round(float(np.median(x)), 2) if len(x) else None
out = {'run': od.rstrip('/').split('/')[-1], 'gt_offset_removed': OFFSET, 'gt_offset_deg': round(float(np.degrees(Roff.magnitude())), 3) if OFFSET else 0.0, 'n': len(t), 'rp_med': med(rp), 'rp_mean': round(float(rp.mean()), 2), 'yaw_med': med(yw), 'ori_med': med(ori),
       'pos_med': med(pos), 'rp_err_rms_deg': round(float(np.degrees(np.sqrt(np.mean(np.square(rperr))))), 4), 'rp_sigma_med_deg': round(float(np.degrees(np.median(rpsig))), 4), 'ate_m': round(float(np.sqrt(np.mean(np.sum((pa - P) ** 2, 1)))), 4), 'by_rate': [], 'by_class': {}}
for lo, hi in zip([0, 0.05, 0.1, 0.2, 0.4, 0.8], [0.05, 0.1, 0.2, 0.4, 0.8, 10]):
    k = (w >= lo) & (w < hi)
    if k.sum() >= 5: out['by_rate'].append({'rate': [lo, hi], 'n': int(k.sum()), 'rp_med': med(rp[k]), 'yaw_med': med(yw[k]), 'ori_med': med(ori[k])})
hover = (v < 0.1) & (w < 0.05); slow = (v < 0.3) & (w < 0.15) & ~hover; fast = ~hover & ~slow
for n_, k in (('hover', hover), ('slow', slow), ('fast', fast)):
    out['by_class'][n_] = {'n': int(k.sum()), 'rp_med': med(rp[k]), 'yaw_med': med(yw[k]), 'ori_med': med(ori[k])}
print(json.dumps(out))
