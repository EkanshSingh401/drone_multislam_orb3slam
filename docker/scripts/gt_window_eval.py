#!/usr/bin/env python3
"""Ground-truth airborne window ATE, identical for every estimator (PATCHES s51).

    /opt/scripts/gt_window_eval.py <gt.tum> <name>=<est.tum> [<name>=<est.tum> ...]

Run INSIDE the sim container (needs evo). Window, from ground truth only:
    takeoff   = first GT sample with height above initial ground > TAKEOFF_H
    touchdown = first GT sample with height < TAKEOFF_H after the last time it
                was above 0.5 m
    window    = [takeoff + DELAY, touchdown]
Per estimator (all estimates must be base_link, GT clock):
    ate          RMSE position error, SE(3) Umeyama on the window, no scale,
                 nearest-neighbour association within 0.05 s
    ate_max, sim3_scale (Sim(3) on the same window)
    coverage     fraction of GT window samples with an associated estimate
    t_first_rel  first estimate time minus takeoff (s; negative = before takeoff)
                 -- the time-to-usable-estimate, reported separately
Prints one JSON line.
"""
from __future__ import annotations

import copy
import json
import sys

import numpy as np
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

TAKEOFF_H = 0.05
DELAY = 3.0


def sub(traj, t0, t1):
    m = (traj.timestamps >= t0) & (traj.timestamps <= t1)
    return PoseTrajectory3D(positions_xyz=traj.positions_xyz[m],
                            orientations_quat_wxyz=traj.orientations_quat_wxyz[m],
                            timestamps=traj.timestamps[m])


def main() -> int:
    gt = file_interface.read_tum_trajectory_file(sys.argv[1])
    t, z = gt.timestamps, gt.positions_xyz[:, 2]
    h = z - np.median(z[: max(5, len(z) // 50)])
    above = np.where(h > 0.5)[0]
    i_to = int(np.argmax(h > TAKEOFF_H))
    i_td = int(above[-1] + np.argmax(h[above[-1]:] < TAKEOFF_H))
    t_to, t_td = float(t[i_to]), float(t[i_td])
    w0, w1 = t_to + DELAY, t_td
    gw = sub(gt, w0, w1)
    out = {"gt": sys.argv[1], "t_takeoff": t_to, "t_touchdown": t_td,
           "window_s": w1 - w0, "gt_samples": int(len(gw.timestamps))}
    for arg in sys.argv[2:]:
        name, path = arg.split("=", 1)
        r = {}
        try:
            est = file_interface.read_tum_trajectory_file(path)
            r["t_first_rel"] = float(est.timestamps[0] - t_to)
            ew = sub(est, w0 - 0.05, w1 + 0.05)
            g, e = sync.associate_trajectories(gw, ew, 0.05)
            r["coverage"] = len(g.timestamps) / max(1, len(gw.timestamps))
            e1 = copy.deepcopy(e); e1.align(g)
            err = np.linalg.norm(e1.positions_xyz - g.positions_xyz, axis=1)
            r["ate"] = float(np.sqrt((err ** 2).mean())); r["ate_max"] = float(err.max())
            e2 = copy.deepcopy(e); _, _, s = e2.align(g, correct_scale=True)
            r["sim3_scale"] = float(s)
        except Exception as ex:  # no estimate in window, etc.
            r["error"] = str(ex)
        out[name] = r
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
