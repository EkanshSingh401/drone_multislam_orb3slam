#!/usr/bin/env python3
"""Prepare one OpenVINS replay directory for evaluation (run INSIDE the sim container).

    /opt/scripts/ov_prep.py /out/replay/<run>_openvins [--t-start T]

Inputs (written by replay_estimator.sh --estimator openvins):
    ov_state_est.txt ov_state_std.txt
                     OpenVINS's own per-update state and its std devs (save_total_state).
                     These are the ESTIMATE used here: they are bit-identical across
                     replays of one bag, whereas the published /odomimu stream is not
                     (PATCHES.md s46).
    ov_est_cov.txt   /odomimu with full 6x6 covariance (ov_pose_to_file.py), used only
                     for the NEES cross-check
    out.bag          the replay's recording, carrying /ground_truth/pose_info + /clock;
                     --gt-bag points elsewhere (the serial runner has no out.bag, so
                     ground truth is read from the input flight.bag itself)

Outputs:
    gt.tum                  ground truth, base_link frame (same extraction as record_and_eval.sh)
    est_openvins.tum        OpenVINS state estimate moved from the IMU to base_link
    est_cov.txt             state estimate (IMU) in ov_eval's 20-column format, covariance
                            DIAGONAL from ov_state_std.txt (off-diagonals are not saved)
    gt_imu.txt              ground truth moved from base_link to the IMU, ov_eval format
    est_cov_window.txt      est_cov.txt cropped to t >= T             (with --t-start)
    est_cov_window_odom.txt /odomimu rows at state-update stamps, full covariance,
                            cropped to t >= T: the NEES cross-check   (with --t-start)
    gt_imu_window.txt       gt_imu.txt cropped to t >= T              (with --t-start)

Why the frame changes are needed (PATCHES.md s44):
ORB-SLAM3's wrapper publishes the BASE pose (it conjugates the camera pose by
robotBase_to_cameraLink), and Gazebo ground truth index 0 is the model, i.e.
base_link. OpenVINS publishes the IMU pose. The IMU sits 0.257 m from base_link
(x500_d455: d455_link at (0.12, 0, 0.242), IMU at (-0.00552, 0.0051, -0.01174)
inside it, no rotation). Under yaw that lever arm sweeps a circle, so scoring the
IMU position against the base would charge OpenVINS up to ~0.5 m of error that
is pure bookkeeping. ATE therefore compares base to base, and NEES -- whose
covariance belongs to the IMU state -- compares IMU to IMU.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

# IMU origin in base_link, from docker/sim/models/x500_d455_nodepth + d455_nodepth.
# Rotation is identity: neither <pose> carries an orientation.
P_BASE_IMU = np.array([0.12 - 0.00552, 0.0 + 0.00510, 0.242 - 0.01174])


def load(path: str) -> np.ndarray:
    rows = []
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            rows.append([float(x) for x in line.split()])
    return np.array(rows)


def shift(traj: np.ndarray, offset_in_body: np.ndarray) -> np.ndarray:
    """p' = p + R(q) * offset; orientation unchanged (the mounting has no rotation)."""
    out = traj.copy()
    rot = R.from_quat(traj[:, 4:8])          # TUM / ov_eval order: qx qy qz qw
    out[:, 1:4] = traj[:, 1:4] + rot.apply(offset_in_body)
    return out


def save(path: str, traj: np.ndarray) -> None:
    # ov_eval's Loader splits on a SINGLE space; keep that format everywhere.
    with open(path, "w") as f:
        for r in traj:
            f.write(" ".join(f"{v:.9f}" for v in r) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rundir")
    ap.add_argument("--t-start", type=float, default=None)
    ap.add_argument("--gt-bag", default=None,
                    help="bag to take ground truth from (default: <rundir>/out.bag)")
    a = ap.parse_args()
    d = a.rundir

    for need in ("ov_state_est.txt", "ov_state_std.txt"):
        f = os.path.join(d, need)
        if not os.path.exists(f) or len(load(f)) == 0:
            print(f"ov_prep: {f} missing or has no states (did OpenVINS initialise?)",
                  file=sys.stderr)
            return 3

    gt_tum = os.path.join(d, "gt.tum")
    subprocess.run(
        ["python3", "/opt/scripts/bag_to_tum.py", a.gt_bag or os.path.join(d, "out.bag"),
         "--tf-topic", "/ground_truth/pose_info", "--tf-index", "0",
         "--time-from-clock", "/clock", "--out", gt_tum],
        check=True)

    # save_total_state rows: t, q(4: x y z w), p(3), v(3), bg, ba, ...  The stored
    # JPL q_GtoI is numerically the Hamilton q_ItoG, i.e. the same four numbers
    # /odomimu publishes (checked row by row against ov_est_cov.txt).
    st = load(os.path.join(d, "ov_state_est.txt"))
    sd = load(os.path.join(d, "ov_state_std.txt"))
    assert len(st) == len(sd) and np.array_equal(st[:, 0], sd[:, 0]), "est/std rows misaligned"
    imu = np.column_stack([st[:, 0], st[:, 5:8], st[:, 1:5]])      # t p q
    # ov_eval order after the pose: Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33
    vr, vp = sd[:, 1:4] ** 2, sd[:, 5:8] ** 2
    z = np.zeros(len(st))
    est = np.column_stack([imu, vr[:, 0], z, z, vr[:, 1], z, vr[:, 2],
                                vp[:, 0], z, z, vp[:, 1], z, vp[:, 2]])
    save(os.path.join(d, "est_cov.txt"), est)
    gt = load(gt_tum)
    # est holds T_w_imu; base = imu - R * p_base_imu
    save(os.path.join(d, "est_openvins.tum"), shift(imu, -P_BASE_IMU))
    # gt holds T_w_base; imu = base + R * p_base_imu
    gt_imu = shift(gt[:, :8], P_BASE_IMU)
    save(os.path.join(d, "gt_imu.txt"), gt_imu)

    if a.t_start is not None:
        save(os.path.join(d, "est_cov_window.txt"), est[est[:, 0] >= a.t_start])
        odom_f = os.path.join(d, "ov_est_cov.txt")
        if os.path.exists(odom_f) and os.path.getsize(odom_f) > 0:
            od = load(odom_f)
            keep = np.isin(np.round(od[:, 0], 5), np.round(st[:, 0], 5)) & (od[:, 0] >= a.t_start)
            save(os.path.join(d, "est_cov_window_odom.txt"), od[keep])
        save(os.path.join(d, "gt_imu_window.txt"), gt_imu[gt_imu[:, 0] >= a.t_start])

    print(f"ov_prep: est {len(est)} poses, gt {len(gt)} poses, "
          f"lever arm |p| = {np.linalg.norm(P_BASE_IMU):.4f} m"
          + (f", window t >= {a.t_start}" if a.t_start is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
