#!/usr/bin/env python3
"""Largest discontinuity in an ONLINE pose stream, and when it happened.

Why this is a first-class metric and not a curiosity: a planner consumes the
estimate *during* flight, including start-up. ORB-SLAM3's visual-inertial BA
rewrites the map retroactively, but the wrapper publishes each pose once, as its
frame is tracked, and never revises it -- so the stream carries a step where a
correction lands. A 3 m retroactive jump corrupts both the decisions the planner
made before it and the map it built before it, and NEITHER is visible in an ATE,
which averages over the whole window and is computed after the fact.

OpenVINS is a sliding-window filter with no retroactive global correction, so
this is the metric on which the two should differ most. Reported alongside
converged ATE for every estimator.

Definition. A raw step in the estimate also contains genuine motion (~1 cm per
frame at 30 Hz and 0.3 m/s), so the step alone is not the discontinuity. After
SE(3) alignment, the discontinuity at sample i is

    d_i = || (p_est[i+1] - p_est[i]) - (p_gt[i+1] - p_gt[i]) ||

i.e. how far the estimate moved that the vehicle did NOT. Genuine motion
cancels; a retroactive jump does not. Reported in metres, with its timestamp,
alongside the median for scale.

    ./pose_discontinuity.py /out/eval/<run>
    ./pose_discontinuity.py /out/eval/<run> --est ov_est.tum --split-at 51.9
    ./pose_discontinuity.py /out/eval/<run> --json
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from diagnose_run import associate, load_tum, umeyama_se3  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rundir")
    ap.add_argument("--est", default="est_orbslam3.tum")
    ap.add_argument("--gt", default="gt.tum")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--split-at", type=float, default=None,
                    help="simulation time separating start-up from the converged "
                         "window (normally VIBA 2 completion). Discontinuities "
                         "are then reported for each side separately, which is "
                         "the comparison that matters: a filter with no "
                         "retroactive correction should look the same on both.")
    ap.add_argument("--top", type=int, default=5)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rd = pathlib.Path(args.rundir)
    t_gt, p_gt = load_tum(rd / args.gt)
    t_es, p_es = load_tum(rd / args.est)
    ke, kg = associate(t_gt, p_gt, t_es, p_es, args.tol)
    if len(ke) < 10:
        raise SystemExit(f"discontinuity: only {len(ke)} associations")

    E, G, T = p_es[ke], p_gt[kg], t_es[ke]
    R, tr = umeyama_se3(E, G)
    A = (R @ E.T).T + tr                      # estimate in the ground-truth frame

    dA = np.diff(A, axis=0)
    dG = np.diff(G, axis=0)
    disc = np.linalg.norm(dA - dG, axis=1)    # motion the vehicle did not make
    tmid = T[1:]

    def block(mask, label) -> dict:
        if mask.sum() < 2:
            return {"label": label, "n": int(mask.sum())}
        d = disc[mask]
        t = tmid[mask]
        i = int(np.argmax(d))
        return {
            "label": label,
            "n": int(mask.sum()),
            "max_m": round(float(d[i]), 4),
            "max_at_t": round(float(t[i]), 3),
            "median_m": round(float(np.median(d)), 5),
            "p99_m": round(float(np.percentile(d, 99)), 4),
            # How far above routine sample-to-sample noise the worst step sits.
            "max_over_median": round(float(d[i] / np.median(d)), 1)
            if np.median(d) > 0 else None,
        }

    all_mask = np.ones(len(disc), bool)
    out: dict = {"rundir": str(rd), "est": args.est,
                 "overall": block(all_mask, "whole stream")}
    order = np.argsort(disc)[::-1][:args.top]
    out["largest"] = [{"t": round(float(tmid[i]), 3), "d_m": round(float(disc[i]), 4)}
                      for i in order]
    if args.split_at is not None:
        out["startup"] = block(tmid < args.split_at, f"t < {args.split_at}")
        out["converged"] = block(tmid >= args.split_at, f"t >= {args.split_at}")

    if args.json:
        print(json.dumps(out, indent=2))
        return 0

    print("=" * 70)
    print(f"ONLINE POSE DISCONTINUITY  {rd.name}  ({args.est})")
    print("=" * 70)
    for key in ("overall", "startup", "converged"):
        b = out.get(key)
        if not b or "max_m" not in b:
            continue
        print(f"  {b['label']:<22} n={b['n']:5d}  max {b['max_m']:8.4f} m "
              f"at t={b['max_at_t']:8.2f}s   median {b['median_m']:.5f} m"
              f"   max/median {b['max_over_median']}")
    print("\n  largest steps:")
    for e in out["largest"]:
        print(f"    t={e['t']:8.2f}s   {e['d_m']:8.4f} m")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
