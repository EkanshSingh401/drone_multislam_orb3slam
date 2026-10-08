#!/usr/bin/env python3
"""nees_yaw.py <dir> <gt.tum> [--est FILE] [--dump OUT] (day 4 step 2c): orientation NEES split into
yaw (1 DoF, about gravity) and roll/pitch (2 DoF), same window, interpolation and posyaw alignment
as /out/diag_s54/nees_window.py (whose window() and conventions are reused).

dth = Log(R_est_aligned^T R_true) (local IMU frame), Pr = orientation covariance block (local).
g_I = gravity direction in the IMU frame. yaw:  e = g_I . dth,   var = g_I^T Pr g_I  (posyaw-aligned)
rp: as nees_window.py (unaligned, projected perpendicular to g_I). ori: full 3-DoF (aligned).
Dump columns: t ori rp yaw."""
import json, os, sys
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
sys.path.insert(0, '/out/diag_s54')
from nees_window import window, upper

def main():
    od, gtf = sys.argv[1], sys.argv[2]
    est_name = sys.argv[sys.argv.index('--est') + 1] if '--est' in sys.argv else 'est_cov.txt'
    w0, w1 = window(gtf)
    est = np.loadtxt(os.path.join(od, est_name)); gt = np.loadtxt(os.path.join(od, 'gt_imu.txt'))
    gt = gt[np.unique(gt[:, 0], return_index=True)[1]]
    est = est[(est[:, 0] >= w0) & (est[:, 0] <= w1) & (est[:, 0] > gt[0, 0]) & (est[:, 0] < gt[-1, 0])]
    tq = est[:, 0]
    pg = np.column_stack([np.interp(tq, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
    Rg = Slerp(gt[:, 0], R.from_quat(gt[:, 4:8]))(tq)
    pe, Re = est[:, 1:4], R.from_quat(est[:, 4:8])
    me, mg = pe.mean(0), pg.mean(0); A, B = pe - me, pg - mg
    yaw = np.arctan2((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum(), (A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum())
    Rea = R.from_euler('z', yaw) * Re
    o, rp, yw = [], [], []
    for i in range(len(est)):
        Pr = upper(est[i, 8:14])
        dth = (Rea[i].inv() * Rg[i]).as_rotvec()
        o.append(dth @ np.linalg.solve(Pr, dth))
        gI = Rg[i].inv().apply([0, 0, 1.0])
        yw.append((gI @ dth) ** 2 / (gI @ Pr @ gI))
        d0 = (Re[i].inv() * Rg[i]).as_rotvec()
        Bm = np.linalg.svd(np.eye(3) - np.outer(gI, gI))[0][:, :2]; e2 = Bm.T @ d0
        rp.append(e2 @ np.linalg.solve(Bm.T @ Pr @ Bm, e2))
    o, rp, yw = map(np.array, (o, rp, yw))
    if '--dump' in sys.argv: np.savetxt(sys.argv[sys.argv.index('--dump') + 1], np.column_stack([tq, o, rp, yw]), fmt='%.9g')
    st = lambda x, k: {'mean': round(float(x.mean()), 2), 'median': round(float(np.median(x)), 2), 'dof': k}
    print(json.dumps({'dir': os.path.basename(od.rstrip('/')), 'est': est_name, 'n': len(o), 'ori': st(o, 3), 'rp': st(rp, 2), 'yaw': st(yw, 1),
                      'rms_yaw_err_deg': round(float(np.degrees(np.sqrt(np.mean([(Rg[i].inv().apply([0, 0, 1.0]) @ (Rea[i].inv() * Rg[i]).as_rotvec()) ** 2 for i in range(len(o))])))), 3),
                      'median_sigma_yaw_deg': round(float(np.degrees(np.median([np.sqrt(Rg[i].inv().apply([0, 0, 1.0]) @ upper(est[i, 8:14]) @ Rg[i].inv().apply([0, 0, 1.0])) for i in range(len(o))]))), 3)}))

main()
