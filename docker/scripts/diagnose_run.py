#!/usr/bin/env python3
"""Post-hoc diagnosis of one evaluated run (run INSIDE the sim container).

Answers the questions an ATE number alone cannot:

  coverage    -- does the estimate actually span the flight, or only part of it?
  excursions  -- is the error spread evenly (drift) or concentrated in a few
                 jumps, and do those jumps coincide with Tracking LOST events?
  per-segment -- which part of the scripted path accumulates the error?

The alignment is SE(3) Umeyama WITHOUT scale, matching how record_and_eval.sh
invokes evo (`-a`, no `-s`), so the numbers here are comparable to the reported
ATE rather than a second, differently-aligned opinion.

    ./diagnose_run.py /out/eval/<run> [--est est_orbslam3.tum] [--json]
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import sys

import numpy as np


def load_tum(p: pathlib.Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if len(f) < 8:
            continue
        rows.append([float(x) for x in f[:8]])
    if not rows:
        raise SystemExit(f"diagnose: no poses in {p}")
    a = np.asarray(rows)
    return a[:, 0], a[:, 1:4]


def associate(t_gt, p_gt, t_es, p_es, tol=0.05):
    """Nearest-neighbour association, the same rule evo's --t_max_diff applies."""
    idx = np.searchsorted(t_gt, t_es)
    keep_e, keep_g = [], []
    for k, te in enumerate(t_es):
        i = idx[k]
        best, bestd = None, None
        for j in (i - 1, i):
            if 0 <= j < len(t_gt):
                d = abs(t_gt[j] - te)
                if bestd is None or d < bestd:
                    best, bestd = j, d
        if best is not None and bestd <= tol:
            keep_e.append(k)
            keep_g.append(best)
    return np.asarray(keep_e, int), np.asarray(keep_g, int)


def umeyama_se3(src: np.ndarray, dst: np.ndarray):
    """Rigid SE(3) fit mapping src onto dst. No scale -- stereo/RGB-D are metric."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(S)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1.0
    R = U @ D @ Vt
    return R, mu_d - R @ mu_s


def gaps(t: np.ndarray, nominal: float) -> dict:
    d = np.diff(t)
    if len(d) == 0:
        return {"max_gap_s": 0.0, "n_gaps_over_1s": 0}
    return {
        "median_dt_s": float(np.median(d)),
        "rate_hz": float(1.0 / np.median(d)) if np.median(d) > 0 else float("nan"),
        "max_gap_s": float(d.max()),
        "n_gaps_over_1s": int((d > 1.0).sum()),
        "n_gaps_over_5x": int((d > 5 * nominal).sum()),
    }


def lost_events(log: pathlib.Path) -> list[str]:
    if not log.is_file():
        return []
    txt = re.sub(r"\x1b\[[0-9;]*m", "", log.read_text(errors="replace"))
    return [l for l in txt.splitlines() if "Tracking LOST" in l]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rundir")
    ap.add_argument("--est", default="est_orbslam3.tum")
    ap.add_argument("--gt", default="gt.tum")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rd = pathlib.Path(args.rundir)
    t_gt, p_gt = load_tum(rd / args.gt)
    t_es, p_es = load_tum(rd / args.est)

    ke, kg = associate(t_gt, p_gt, t_es, p_es, args.tol)
    if len(ke) < 10:
        raise SystemExit(f"diagnose: only {len(ke)} associations; cannot diagnose")

    E, G = p_es[ke], p_gt[kg]
    T = t_es[ke]
    R, tr = umeyama_se3(E, G)
    err = np.linalg.norm((R @ E.T).T + tr - G, axis=1)

    # ---- coverage -----------------------------------------------------------
    gt_span = (float(t_gt[0]), float(t_gt[-1]))
    es_span = (float(t_es[0]), float(t_es[-1]))
    overlap = max(0.0, min(gt_span[1], es_span[1]) - max(gt_span[0], es_span[0]))
    cov = {
        "gt_span_s": [round(x, 3) for x in gt_span],
        "est_span_s": [round(x, 3) for x in es_span],
        "gt_duration_s": round(gt_span[1] - gt_span[0], 3),
        "est_duration_s": round(es_span[1] - es_span[0], 3),
        "est_covers_frac_of_gt": round((es_span[1] - es_span[0]) / (gt_span[1] - gt_span[0]), 4),
        "late_start_s": round(es_span[0] - gt_span[0], 3),
        "associated": int(len(ke)),
        "est_samples": int(len(t_es)),
        "assoc_frac_of_est": round(len(ke) / len(t_es), 4),
        "est_gaps": gaps(t_es, 1.0 / 8.0),
    }

    # ---- excursions ---------------------------------------------------------
    # Drift and jumps look identical in an RMSE. Separating them: what fraction
    # of the squared error (which is what RMSE reports) comes from the worst 5%
    # of samples? A pure-drift run spreads it; a jump run concentrates it.
    order = np.argsort(err)[::-1]
    n5 = max(1, int(0.05 * len(err)))
    sse = float((err ** 2).sum())
    top5_sse = float((err[order[:n5]] ** 2).sum())
    step = np.abs(np.diff(err))
    jump_i = np.argsort(step)[::-1][:8]
    exc = {
        "ate_rmse_m": round(float(math.sqrt((err ** 2).mean())), 4),
        "ate_mean_m": round(float(err.mean()), 4),
        "ate_median_m": round(float(np.median(err)), 4),
        "ate_max_m": round(float(err.max()), 4),
        "worst5pct_share_of_sse": round(top5_sse / sse, 4) if sse > 0 else 0.0,
        "largest_error_jumps": [
            {"t": round(float(T[i]), 3),
             "d_err_m": round(float(step[i]), 4),
             "err_after_m": round(float(err[i + 1]), 4)}
            for i in jump_i
        ],
        "time_of_max_error_s": round(float(T[int(order[0])]), 3),
    }

    # ---- error vs time, in deciles of the flight ---------------------------
    dec = []
    edges = np.linspace(T[0], T[-1], 11)
    for a, b in zip(edges, edges[1:]):
        m = (T >= a) & (T < b)
        dec.append({"t0": round(float(a), 2), "t1": round(float(b), 2),
                    "n": int(m.sum()),
                    "rmse_m": round(float(math.sqrt((err[m] ** 2).mean())), 4) if m.sum() else None})

    out = {
        "rundir": str(rd),
        "coverage": cov,
        "excursions": exc,
        "deciles": dec,
        "tracking_lost_events": len(lost_events(rd / "orb_slam3.log")),
    }

    if args.json:
        print(json.dumps(out, indent=2))
        return 0

    print("=" * 72)
    print(f"DIAGNOSIS  {rd.name}   ({args.est})")
    print("=" * 72)
    print("COVERAGE")
    print(f"  ground truth  : {cov['gt_span_s'][0]:.2f} .. {cov['gt_span_s'][1]:.2f} s "
          f"({cov['gt_duration_s']:.2f} s)")
    print(f"  estimate      : {cov['est_span_s'][0]:.2f} .. {cov['est_span_s'][1]:.2f} s "
          f"({cov['est_duration_s']:.2f} s)")
    print(f"  covers        : {100*cov['est_covers_frac_of_gt']:.1f}% of the gt window, "
          f"starting {cov['late_start_s']:.2f} s late")
    print(f"  associated    : {cov['associated']} of {cov['est_samples']} est samples "
          f"({100*cov['assoc_frac_of_est']:.1f}%) within {args.tol*1000:.0f} ms")
    g = cov["est_gaps"]
    print(f"  estimate rate : {g['rate_hz']:.2f} Hz (median dt {1000*g['median_dt_s']:.1f} ms), "
          f"max gap {g['max_gap_s']:.3f} s, gaps >1 s: {g['n_gaps_over_1s']}")
    print()
    print("ERROR STRUCTURE  (SE(3)-aligned, no scale)")
    print(f"  rmse {exc['ate_rmse_m']:.3f} m | mean {exc['ate_mean_m']:.3f} | "
          f"median {exc['ate_median_m']:.3f} | max {exc['ate_max_m']:.3f} "
          f"at t={exc['time_of_max_error_s']:.2f}s")
    print(f"  worst 5% of samples carry {100*exc['worst5pct_share_of_sse']:.1f}% of the "
          f"squared error  (5% would be uniform; >>5% means JUMPS, ~5-15% means DRIFT)")
    print(f"  Tracking LOST events in log: {out['tracking_lost_events']}")
    print("  largest single-sample error steps:")
    for j in exc["largest_error_jumps"][:5]:
        print(f"    t={j['t']:8.2f}s  +{j['d_err_m']:.3f} m -> {j['err_after_m']:.3f} m")
    print()
    print("ERROR BY DECILE OF THE FLIGHT")
    for d in dec:
        bar = "#" * int(40 * (d["rmse_m"] or 0) / max(1e-9, exc["ate_max_m"]))
        print(f"  {d['t0']:7.2f}..{d['t1']:7.2f}s  n={d['n']:4d}  "
              f"rmse={d['rmse_m'] if d['rmse_m'] is not None else float('nan'):6.3f}  {bar}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
