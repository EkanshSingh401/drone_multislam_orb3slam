#!/usr/bin/env python3
"""Structural checks on /openvins/joint_covariance (run INSIDE the sim container).

Stock OpenVINS publishes only the 6x6 marginal pose covariance on /odomimu.
active_slam_msgs/JointCovariance carries the joint pose+map block including the
IMU<->feature cross-covariance. This verifies the three properties that have to
hold for it to be usable, and reports dimensions and conditioning.

  1. SYMMETRY          max |C - C^T|, absolutely and relative to max|C|.
  2. PSD on the IMU+feature marginal. Checked on the IMU and SLAM-feature
     blocks only -- NOT the whole matrix -- because the whole matrix is
     deliberately rank deficient (see 3), so "not PD" is expected there and
     says nothing.
  3. RANK DEFICIT exactly 6, localized to (IMU pose - newest clone). OpenVINS's
     newest clone IS the current IMU pose, cloned, so their difference carries
     no uncertainty and the matrix must have a 6-dimensional null space along
     exactly those directions. Checked two ways that can disagree: counting
     small eigenvalues, AND confirming the 6 constructed difference directions
     are annihilated and span the same subspace as the numerical null basis.
     A deficit of 6 that is NOT along those directions is a different bug and
     is reported as such.

    ./check_joint_cov.py --messages 5
    ./check_joint_cov.py --messages 5 --json
"""
from __future__ import annotations

import argparse
import json
import signal
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from active_slam_msgs.msg import JointCovariance
except ImportError:  # pragma: no cover
    print("check_joint_cov: active_slam_msgs is not on the AMENT path.\n"
          "  If OpenVINS also warns 'joint covariance publisher NOT compiled in'\n"
          "  then active_slam_msgs was absent when ov_msckf was built -- rebuild\n"
          "  active_slam_msgs FIRST, then ov_msckf (docker/sim/Dockerfile 9b).",
          file=sys.stderr)
    raise


def analyse(msg, tol_rel: float = 1e-9) -> dict:
    n = int(msg.dim)
    C = np.asarray(msg.covariance, dtype=float).reshape(n, n)
    blocks = [{"type": b.type, "index": int(b.index), "size": int(b.size),
               "state_id": int(b.state_id),
               "clone_timestamp": float(b.clone_timestamp),
               "feature_id": int(b.feature_id),
               "parameterization": b.parameterization,
               "is_anchored": bool(b.is_anchored)} for b in msg.blocks]

    out: dict = {
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "dim": n,
        "full_state_dim": int(msg.full_state_dim),
        "num_clones": int(msg.num_clones),
        "num_slam_features": int(msg.num_slam_features),
        "includes": {"imu": bool(msg.includes_imu),
                     "clones": bool(msg.includes_clones),
                     "slam_features": bool(msg.includes_slam_features),
                     "calibration": bool(msg.includes_calibration)},
        "blocks": [{k: b[k] for k in ("type", "index", "size")} for b in blocks],
        "block_size_sum": sum(b["size"] for b in blocks),
    }

    # --- 1. symmetry ---------------------------------------------------------
    asym = float(np.abs(C - C.T).max()) if n else 0.0
    scale = float(np.abs(C).max()) if n else 0.0
    out["symmetry"] = {
        "max_abs_asymmetry": asym,
        "max_abs_value": scale,
        "relative": (asym / scale) if scale > 0 else 0.0,
        "ok": scale == 0.0 or asym / scale < 1e-10,
    }

    # Symmetrise before any eigenvalue work: tiny asymmetry from the wire
    # format would otherwise give complex eigenvalues.
    S = 0.5 * (C + C.T)
    ev_all = np.linalg.eigvalsh(S)
    lam_max = float(ev_all.max()) if n else 0.0
    thresh = tol_rel * lam_max if lam_max > 0 else 0.0

    # --- 2. PSD on the IMU + SLAM-feature marginal ---------------------------
    sel: list[int] = []
    for b in blocks:
        if b["type"] in ("imu", "slam_feature"):
            sel.extend(range(b["index"], b["index"] + b["size"]))
    psd: dict = {"n_selected": len(sel)}
    if sel:
        M = S[np.ix_(sel, sel)]
        ev = np.linalg.eigvalsh(M)
        psd.update({
            "min_eig": float(ev.min()),
            "max_eig": float(ev.max()),
            # Condition number of the marginal, which is the number that says
            # whether a planner can invert it.
            "cond": float(ev.max() / ev.min()) if ev.min() > 0 else float("inf"),
            # A marginal of a valid covariance is strictly PD; allow only
            # round-off below zero.
            "ok": bool(ev.min() > -1e-9 * max(1.0, float(ev.max()))),
        })
    out["psd_imu_feature_marginal"] = psd

    # --- 3. rank deficit, and whether it sits where it should ----------------
    small = [float(v) for v in ev_all if v <= thresh]
    rank_def = len(small)
    rk: dict = {
        "eig_max": lam_max,
        "eig_min": float(ev_all.min()) if n else 0.0,
        "threshold": thresh,
        "rank_deficit": rank_def,
        "expected": 6,
        "smallest_eigs": sorted(small)[:10],
        # Reported for the record; infinite for a rank-deficient matrix, which
        # is why check 2 uses the marginal instead.
        "cond_full": (float(lam_max / ev_all[ev_all > thresh].min())
                      if (ev_all > thresh).any() else float("inf")),
    }

    imu_b = next((b for b in blocks if b["type"] == "imu"), None)
    clones = [b for b in blocks if b["type"] == "clone"]
    newest = max(clones, key=lambda b: b["clone_timestamp"]) if clones else None
    if imu_b is not None and newest is not None:
        # The IMU block is 15x15 ordered quat(3) pos(3) vel(3) bg(3) ba(3), so
        # its POSE is the first 6. The clone block is 6x6 quat(3) pos(3).
        N = np.zeros((n, 6))
        for k in range(6):
            N[imu_b["index"] + k, k] = 1.0
            N[newest["index"] + k, k] = -1.0
        N /= np.linalg.norm(N, axis=0, keepdims=True)
        residual = float(np.abs(S @ N).max())
        rk["newest_clone_timestamp"] = newest["clone_timestamp"]
        rk["constructed_null_residual"] = residual
        rk["constructed_null_annihilated"] = bool(
            lam_max == 0.0 or residual < 1e-8 * lam_max)
        if rank_def > 0:
            # Do the numerical null vectors span the SAME subspace as the
            # constructed ones? Compare by projecting each numerical null
            # vector onto span(N): a residual near zero means same subspace.
            _, V = np.linalg.eigh(S)
            Vn = V[:, :rank_def]
            Q, _ = np.linalg.qr(N)
            proj_res = float(np.abs(Vn - Q @ (Q.T @ Vn)).max())
            rk["null_subspace_matches_imu_minus_clone"] = bool(proj_res < 1e-6)
            rk["null_subspace_projection_residual"] = proj_res
    else:
        rk["newest_clone_timestamp"] = None
        rk["constructed_null_annihilated"] = None
    rk["ok"] = (rank_def == 6 and rk.get("constructed_null_annihilated") is True
                and rk.get("null_subspace_matches_imu_minus_clone", False))
    out["rank"] = rk
    out["all_ok"] = bool(out["symmetry"]["ok"] and psd.get("ok", False) and rk["ok"])
    return out


class CovChecker(Node):
    def __init__(self, topic: str, want: int):
        super().__init__(
            "check_joint_cov",
            parameter_overrides=[rclpy.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.results: list[dict] = []
        self.want = want
        self.create_subscription(JointCovariance, topic, self._cb, 10)
        self.get_logger().info(f"waiting for {want} message(s) on {topic}")

    def _cb(self, msg: JointCovariance) -> None:
        if len(self.results) >= self.want:
            return
        self.results.append(analyse(msg))
        if len(self.results) >= self.want:
            rclpy.try_shutdown()


def report(rs: list[dict]) -> int:
    if not rs:
        print("check_joint_cov: NO MESSAGES RECEIVED.\n"
              "  Either OpenVINS has not initialised yet, or the joint covariance\n"
              "  publisher was compiled out (look for 'NOT compiled in' in the\n"
              "  OpenVINS log -- see docker/sim/Dockerfile 9b).", file=sys.stderr)
        return 2
    print("=" * 74)
    print(f"JOINT COVARIANCE STRUCTURE  ({len(rs)} message(s))")
    print("=" * 74)
    allok = True
    for i, r in enumerate(rs):
        inc = ",".join(k for k, v in r["includes"].items() if v)
        print(f"\n[{i}] t={r['stamp']:.3f}  dim={r['dim']}x{r['dim']}  "
              f"full_state_dim={r['full_state_dim']}")
        print(f"    clones={r['num_clones']} slam_features={r['num_slam_features']}"
              f"  includes: {inc}")
        print(f"    blocks sum to {r['block_size_sum']} "
              f"({'consistent with dim' if r['block_size_sum'] == r['dim'] else 'MISMATCH vs dim'})")
        sy = r["symmetry"]
        print(f"    symmetry : max|C-C^T| = {sy['max_abs_asymmetry']:.3e} "
              f"(rel {sy['relative']:.3e})  {'OK' if sy['ok'] else 'FAIL'}")
        ps = r["psd_imu_feature_marginal"]
        if "min_eig" in ps:
            print(f"    PSD      : IMU+feature marginal {ps['n_selected']}x{ps['n_selected']}"
                  f"  eig [{ps['min_eig']:.3e}, {ps['max_eig']:.3e}]"
                  f"  cond {ps['cond']:.3e}  {'OK' if ps['ok'] else 'FAIL'}")
        else:
            print(f"    PSD      : no imu/slam_feature blocks present  SKIPPED")
        rk = r["rank"]
        print(f"    rank     : deficit {rk['rank_deficit']} (expect 6)"
              f"  eig_min {rk['eig_min']:.3e}  eig_max {rk['eig_max']:.3e}")
        print(f"               cond(full, nonzero eigs) {rk['cond_full']:.3e}")
        if rk.get("constructed_null_annihilated") is not None:
            print(f"               (IMU pose - newest clone) annihilated: "
                  f"{rk['constructed_null_annihilated']}"
                  f"  residual {rk.get('constructed_null_residual', float('nan')):.3e}")
            if "null_subspace_projection_residual" in rk:
                print(f"               null space matches that subspace: "
                      f"{rk['null_subspace_matches_imu_minus_clone']}"
                      f"  (residual {rk['null_subspace_projection_residual']:.3e})")
        print(f"    VERDICT  : {'ALL CHECKS PASS' if r['all_ok'] else 'CHECK FAILED'}")
        allok &= r["all_ok"]
    print("\n" + ("=" * 74))
    print("ALL MESSAGES PASS" if allok else "AT LEAST ONE CHECK FAILED")
    print("=" * 74)
    return 0 if allok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/openvins/joint_covariance")
    ap.add_argument("--messages", type=int, default=3)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = CovChecker(args.topic, args.messages)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    if args.json:
        print(json.dumps(node.results, indent=2))
        return 0 if (node.results and all(r["all_ok"] for r in node.results)) else 1
    return report(node.results)


if __name__ == "__main__":
    sys.exit(main())
