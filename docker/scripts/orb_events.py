#!/usr/bin/env python3
"""ORB-SLAM3 initialisation / map-segment report for one replay (PATCHES s52).

    /opt/scripts/orb_events.py <replay_dir> <gt.tum> [--json]

Run INSIDE the sim container (needs evo). Reads the [ORBEV] lines that the
instrumented Tracking.cc prints (frame time = simulation time):
    [ORBEV] t=.. map=.. imu=.. ba1=.. ba2=.. state=.. kfs=..   on any change
    [ORBEV] reset t=.. map=.. kfs=..                            on an active-map reset
and <replay_dir>/est.tum (replayed estimate, base frame as published).

Definitions
    takeoff / touchdown / GT window   as gt_window_eval.py
    segment boundary   an active-map reset or a change of active map id
    final segment      from the last boundary to the end of the bag
    t_usable           ORB-SLAM3 time-to-usable-estimate: the time IMU init is
                       complete (imu=1) inside the final segment -- i.e. the start
                       of its final continuous map segment after IMU init, with
                       no reset after it -- reported relative to takeoff.
                       Stereo-only (no IMU): the start of the final segment.
                       None if IMU init never completes in the final segment.
    ate_gt_window      ATE on the GT window (same window as every estimator)
    ate_usable         ATE on [t_usable, touchdown] (its own usable segment, airborne end)
"""
from __future__ import annotations

import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
from evo.tools import file_interface  # noqa: E402

import gt_window_eval as gw  # noqa: E402

EV = re.compile(r"\[ORBEV\] t=([0-9.]+) map=(\d+) imu=(\d) ba1=(\d) ba2=(\d) state=(-?\d+) kfs=(\d+)")
FR = re.compile(r"\[ORBEV\] frames=(\d+) t=([0-9.]+)")
RS = re.compile(r"\[ORBEV\] reset t=([0-9.]+) map=(\d+) kfs=(\d+)")


def ate(gt, est, t0, t1):
    import copy
    from evo.core import sync
    g = gw.sub(gt, t0, t1); e = gw.sub(est, t0 - 0.05, t1 + 0.05)
    g, e = sync.associate_trajectories(g, e, 0.05)
    e1 = copy.deepcopy(e); e1.align(g)
    err = np.linalg.norm(e1.positions_xyz - g.positions_xyz, axis=1)
    e2 = copy.deepcopy(e); _, _, s = e2.align(g, correct_scale=True)
    return {"ate": float(np.sqrt((err ** 2).mean())), "ate_max": float(err.max()),
            "sim3_scale": float(s), "n": int(len(err)), "span_s": float(t1 - t0)}


def main() -> int:
    od, gtf = sys.argv[1], sys.argv[2]
    log = open(os.path.join(od, "orb_slam3.log"), errors="replace").read()
    log = re.sub(r"\x1b\[[0-9;]*m", "", log)
    ev, resets = [], []
    frames = None
    for line in log.splitlines():
        m = FR.search(line)
        if m:
            frames = (int(m.group(1)), float(m.group(2))); continue
        m = RS.search(line)
        if m:
            resets.append(float(m.group(1))); ev.append(("reset", float(m.group(1)), int(m.group(2))))
            continue
        m = EV.search(line)
        if m:
            t, mp, imu, b1, b2, st, k = m.groups()
            ev.append(("state", float(t), int(mp), int(imu), int(b1), int(b2), int(st), int(k)))
    gt = file_interface.read_tum_trajectory_file(gtf)
    tt, z = gt.timestamps, gt.positions_xyz[:, 2]
    h = z - np.median(z[: max(5, len(z) // 50)])
    above = np.where(h > 0.5)[0]
    t_to = float(tt[int(np.argmax(h > gw.TAKEOFF_H))])
    t_td = float(tt[int(above[-1] + np.argmax(h[above[-1]:] < gw.TAKEOFF_H))])
    inertial = "stereo_inertial" in od or "_si" in os.path.basename(od)

    # boundaries: resets and map-id changes
    bounds, last_map, firsts = [], None, {}
    for e in ev:
        if e[0] == "reset":
            bounds.append(e[1]); continue
        _, t, mp, imu, b1, b2, st, k = e
        if last_map is not None and mp != last_map:
            bounds.append(t)
        last_map = mp
        for key, val in (("imu_init", imu), ("viba1", b1), ("viba2", b2)):
            if val and key not in firsts:
                firsts[key] = t
    t_seg = max(bounds) if bounds else (ev[0][1] if ev else None)
    # IMU-init / VIBA completion inside the final segment
    fin = {}
    for e in ev:
        if e[0] == "state" and t_seg is not None and e[1] >= t_seg:
            for key, idx in (("imu_init", 3), ("viba1", 4), ("viba2", 5)):
                if e[idx] and key not in fin:
                    fin[key] = e[1]
    t_usable = fin.get("imu_init") if inertial else t_seg

    rel = lambda x: None if x is None else round(x - t_to, 3)  # noqa: E731
    out = {"dir": od, "inertial": inertial, "t_takeoff": t_to, "t_touchdown": t_td,
           "flight_s": round(t_td - t_to, 2),
           "path_m": round(float(np.linalg.norm(np.diff(gw.sub(gt, t_to, t_td).positions_xyz, axis=0), axis=1).sum()), 2),
           "n_resets": len(resets), "n_maps": len({e[2] for e in ev if e[0] == "state"}),
           "n_boundaries": len(bounds),
           "first_rel": {k: rel(v) for k, v in firsts.items()},
           "final_segment_start_rel": rel(t_seg),
           "final_segment_rel": {k: rel(v) for k, v in fin.items()},
           "t_usable_rel": rel(t_usable),
           "resets_rel": [rel(r) for r in resets][:20]}
    # Dropped-frame check: frames reaching Track() vs bag images up to the same
    # time. Each reset swallows one frame before the counter, hence + n_resets.
    stf = os.path.join(od, "cam_stamps.txt")
    if frames and os.path.exists(stf):
        st = np.loadtxt(stf)
        exp = int((st <= frames[1] + 1e-6).sum())
        out["frames_counted"], out["frames_expected"] = frames[0], exp
        out["frames_missing"] = exp - frames[0] - len(resets)
    estf = os.path.join(od, "est.tum")
    try:
        est = file_interface.read_tum_trajectory_file(estf)
        out["gt_window"] = ate(gt, est, t_to + gw.DELAY, t_td)
        if t_usable is not None and t_usable < t_td:
            out["usable"] = ate(gt, est, t_usable, t_td)
    except Exception as ex:
        out["error"] = str(ex)
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
