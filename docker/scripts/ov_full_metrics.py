#!/usr/bin/env python3
"""OpenVINS full post-init metrics + window coverage (run INSIDE the sim container).

    /opt/scripts/ov_full_metrics.py /out/replay/<run>_ovser_g<g> --t-start T

Needs ov_prep.py to have run (gt.tum, est_openvins.tum, gt_imu.txt, est_cov.txt).
Prints one JSON line:

    ate_full, ate_full_max   OpenVINS ATE over its WHOLE post-init trajectory (SE(3), no
                             scale). OpenVINS has no retroactive start-up transient, so
                             this is a valid accuracy figure for it, unlike for ORB-SLAM3.
    sim3_scale_full          Sim(3) scale over the same span (1.0 = no scale error)
    path_full_m              ground-truth path length over OpenVINS's span
    path_window_m            ground-truth path length inside the matched window (t >= T)
    window_fraction          path_window_m / path_full_m

Why (PATCHES.md s49): the matched window starts at ORB-SLAM3's VIBA-2 completion,
which can land after most of the flight. On the first validation bag it covered
1.9 m of a 17.6 m path, mostly hover and landing, so a window-only ATE validated
little and a yaw alignment fitted to near-stationary positions was ill-conditioned
(it made ov_eval's orientation RMSE 5.3 deg while the true orientation error was
under 1 deg). Reporting coverage and the full-trajectory figure next to the
window figure keeps that visible.
"""
from __future__ import annotations

import argparse
import copy
import json
import os

import numpy as np
from evo.core import sync
from evo.tools import file_interface


def plen(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rundir")
    ap.add_argument("--t-start", type=float, required=True)
    a = ap.parse_args()
    gt = file_interface.read_tum_trajectory_file(os.path.join(a.rundir, "gt.tum"))
    est = file_interface.read_tum_trajectory_file(os.path.join(a.rundir, "est_openvins.tum"))
    g, e = sync.associate_trajectories(gt, est, 0.05)
    e1 = copy.deepcopy(e)
    e1.align(g)
    err = np.linalg.norm(e1.positions_xyz - g.positions_xyz, axis=1)
    e2 = copy.deepcopy(e)
    _, _, s = e2.align(g, correct_scale=True)
    pw = g.positions_xyz[g.timestamps >= a.t_start]
    out = {
        "ate_full": float(np.sqrt((err ** 2).mean())),
        "ate_full_max": float(err.max()),
        "sim3_scale_full": float(s),
        "path_full_m": plen(g.positions_xyz),
        "path_window_m": plen(pw),
    }
    out["window_fraction"] = out["path_window_m"] / out["path_full_m"] if out["path_full_m"] > 0 else float("nan")
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
