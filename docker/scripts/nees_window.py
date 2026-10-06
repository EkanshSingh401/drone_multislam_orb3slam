#!/usr/bin/env python3
"""OpenVINS NEES on the ground-truth airborne window (step 3, PATCHES s53).

    /opt/scripts/nees_window.py <replay_dir> <gt_base.tum> [--json]

Run INSIDE the sim container. Inputs from ov_prep.py: <replay_dir>/est_cov.txt
(IMU pose + DIAGONAL covariance from ov_state_std.txt, ov_eval 20-column format)
and <replay_dir>/gt_imu.txt (ground truth moved to the IMU). <gt_base.tum> only
supplies takeoff / touchdown (base_link GT, same rule as gt_window_eval.py).

Conventions. OpenVINS stores JPL q_GtoI, numerically the Hamilton q_ItoG used
here for both files. Its orientation error is LOCAL (IMU frame):
R_ItoG_true = R_ItoG_est * Exp(dth), so dth = Log(R_est^T R_true) and the
covariance block Pr is in that same frame.

    posyaw   4-DOF alignment (yaw + translation) of the estimate onto GT, fitted
             on the window positions; position error/covariance rotated by it.
    pos      NEES 3-DOF   e_p^T (Rz Pp Rz^T)^-1 e_p
    ori      NEES 3-DOF   dth^T Pr^-1 dth           (after posyaw)
    rp       NEES 2-DOF   roll/pitch only, NO alignment of any kind: dth is
             projected onto the plane perpendicular to gravity expressed in
             the IMU frame. A world-yaw offset enters dth exactly along that
             direction, so this component is independent of yaw alignment.
GT is interpolated to each estimate stamp (no nearest-neighbour association).
Reported: mean NEES (consistent ~ DOF) and the fraction of samples inside the
two-sided 95% chi-square interval (consistent ~ 0.95).
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import chi2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
TAKEOFF_H, DELAY = 0.05, 3.0


def window(gtf):
    g = np.loadtxt(gtf); t, z = g[:, 0], g[:, 3]
    h = z - np.median(z[: max(5, len(z) // 50)])
    above = np.where(h > 0.5)[0]
    t_to = t[int(np.argmax(h > TAKEOFF_H))]
    t_td = t[int(above[-1] + np.argmax(h[above[-1]:] < TAKEOFF_H))]
    return float(t_to + DELAY), float(t_td)


def upper(v):  # 6 upper-triangular -> 3x3
    a, b, c, d, e, f = v
    return np.array([[a, b, c], [b, d, e], [c, e, f]])


def stats(x, dof):
    lo, hi = chi2.ppf(0.025, dof), chi2.ppf(0.975, dof)
    return {"mean": float(np.mean(x)), "median": float(np.median(x)), "dof": dof,
            "in95": float(np.mean((x >= lo) & (x <= hi))), "above95": float(np.mean(x > hi))}


def main() -> int:
    od, gtf = sys.argv[1], sys.argv[2]
    # --est FILE: score another est_cov-format file, e.g. est_cov_full.txt with the
    # full 3x3 blocks from /openvins/joint_covariance (PATCHES s55)
    est_name = sys.argv[sys.argv.index("--est") + 1] if "--est" in sys.argv else "est_cov.txt"
    w0, w1 = window(gtf)
    est = np.loadtxt(os.path.join(od, est_name))
    gt = np.loadtxt(os.path.join(od, "gt_imu.txt"))
    est = est[(est[:, 0] >= w0) & (est[:, 0] <= w1)]
    # GT interpolated to the estimate stamps (linear position, slerp orientation).
    # Nearest-neighbour on the 50 Hz GT mis-times by up to 10 ms, which at the
    # flight's rotation rates alone is ~0.2 deg -- several sigma for OpenVINS.
    from scipy.spatial.transform import Slerp
    est = est[(est[:, 0] > gt[0, 0]) & (est[:, 0] < gt[-1, 0])]
    tq = est[:, 0]
    pgi = np.column_stack([np.interp(tq, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
    qgi = Slerp(gt[:, 0], R.from_quat(gt[:, 4:8]))(tq).as_quat()
    gt = np.column_stack([tq, pgi, qgi])
    pe, pg = est[:, 1:4], gt[:, 1:4]
    Re, Rg = R.from_quat(est[:, 4:8]), R.from_quat(gt[:, 4:8])

    # posyaw: yaw + translation (closed form on centred xy)
    me, mg = pe.mean(0), pg.mean(0)
    A, B = pe - me, pg - mg
    yaw = np.arctan2((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum(), (A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum())
    Rz = R.from_euler("z", yaw)
    pa = Rz.apply(pe - me) + mg
    Rea = Rz * Re

    nees_p, nees_o, nees_rp, tilt_deg = [], [], [], []
    Rzm = Rz.as_matrix()
    for i in range(len(est)):
        Pr, Pp = upper(est[i, 8:14]), upper(est[i, 14:20])
        ep = pg[i] - pa[i]
        nees_p.append(ep @ np.linalg.solve(Rzm @ Pp @ Rzm.T, ep))
        dth = (Rea[i].inv() * Rg[i]).as_rotvec()
        nees_o.append(dth @ np.linalg.solve(Pr, dth))
        # roll/pitch: unaligned estimate, error projected perpendicular to gravity in IMU frame
        d0 = (Re[i].inv() * Rg[i]).as_rotvec()
        gI = Rg[i].inv().apply([0, 0, 1.0])
        Bm = np.linalg.svd(np.eye(3) - np.outer(gI, gI))[0][:, :2]   # basis of the plane
        e2 = Bm.T @ d0
        nees_rp.append(e2 @ np.linalg.solve(Bm.T @ Pr @ Bm, e2))
        tilt_deg.append(np.degrees(np.linalg.norm(e2)))
    out = {"dir": od, "window": [w0, w1], "n": int(len(est)), "posyaw_yaw_deg": float(np.degrees(yaw)),
           "pos": stats(np.array(nees_p), 3), "ori": stats(np.array(nees_o), 3),
           "rp": stats(np.array(nees_rp), 2),
           "rmse_pos_m": float(np.sqrt(np.mean(np.sum((pg - pa) ** 2, 1)))),
           "rmse_tilt_deg": float(np.sqrt(np.mean(np.square(tilt_deg)))),
           "median_std_pos_m": float(np.median(np.sqrt(est[:, [14, 17, 19]].sum(1)))),
           "median_std_tilt_deg": float(np.degrees(np.median(np.sqrt(est[:, 8] + est[:, 11]))))}
    if est_name != "est_cov.txt":
        out["est"] = est_name
        print(json.dumps(out))
        return 0
    # files for the ov_eval posyaw cross-check
    np.savetxt(os.path.join(od, "est_cov_gtwin.txt"), est, fmt="%.9f")
    np.savetxt(os.path.join(od, "gt_imu_gtwin.txt"), gt[:, :8], fmt="%.9f")
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
