#!/usr/bin/env python3
"""Per-run phase-3 table: OpenVINS vs ORB-SLAM3 with scene-degradation columns (run on the HOST).

    python3 docker/scripts/phase3_table.py <experiment_dir> [gravity=9.81] [--exclude RUN ...]

For each run listed in <experiment_dir>/rundirs.txt, reads what
phase3_openvins.sh evaluate wrote under docker/out/replay/:
    <run>_ovser_g<g>_matched.txt   window-matched ATE / RPE / worst step (both estimators)
    <run>_ovser_g<g>_full.json     ov_full_metrics.py (full post-init ATE, Sim(3) scale, window coverage)
    <run>_ovser_g<g>_nees_full.txt ov_eval error_singlerun posyaw over the full post-init trajectory
                                   (the window-only NEES is dropped: on a near-stationary window its
                                   position-fitted yaw is ill-conditioned, PATCHES.md s49)
    <run>_ovser_g<g>_scene.json    ov_scene_stats.py (feature depth, rejection rates)
    /out/replay/<run>_ovser_g<g>/frames.txt   frames processed vs published
and prints one row per run plus mean / std / min..max of each column, so that
degradation (depth, rejection rate) sits next to accuracy (PATCHES.md s49).
"""
from __future__ import annotations

import json
import os
import re
import sys

import numpy as np

ANSI = re.compile(r"\x1b\[[0-9;]*m")
REPLAY = "docker/out/replay"


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def row_for(run: str, g: str) -> dict:
    base = os.path.join(REPLAY, f"{run}_ovser_g{g}")
    r = {"run": run}
    try:
        m = ANSI.sub("", open(base + "_matched.txt").read())
        w = re.search(r"window start : sim t = (\S+)", m)
        r["t_start"] = num(w.group(1)) if w else float("nan")
        for key, lab in (("orb", "ORB-SLAM3 SI"), ("ov", "OpenVINS")):
            mm = re.search(lab + r"\s+ATE (\S+)\s+RPE (\S+)\s+max-step\(all\) (\S+)\s+max-step\(window\) (\S+)", m)
            if mm:
                r[f"{key}_ate"], r[f"{key}_rpe"], r[f"{key}_step_all"], r[f"{key}_step_win"] = map(num, mm.groups())
    except FileNotFoundError:
        pass
    try:
        n = ANSI.sub("", open(base + "_nees.txt").read())
        mm = re.search(r"Normalized Estimation Error Squared.*?mean_ori = (\S+) \| mean_pos = (\S+)", n, re.S)
        if mm:
            r["nees_ori"], r["nees_pos"] = num(mm.group(1)), num(mm.group(2))
    except FileNotFoundError:
        pass
    try:
        s = json.load(open(base + "_scene.json"))
        r["depth_ok"] = s["median_feature_depth_m"]
        r["depth_all"] = s.get("median_attempt_depth_m", float("nan"))
        r["tri_rej"] = s["tri_reject_rate"]
        r["msckf_rej"] = s["msckf_reject_rate"]
        r["tracked"] = s["tracked_cam0_mean"]
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        pass
    try:
        f = json.load(open(base + "_full.json"))
        r["ov_ate_full"], r["sim3_full"] = f["ate_full"], f["sim3_scale_full"]
        r["path_win"], r["win_frac"] = f["path_window_m"], f["window_fraction"]
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        pass
    try:
        n = ANSI.sub("", open(base + "_nees_full.txt").read())
        mm = re.search(r"Normalized Estimation Error Squared.*?mean_ori = (\S+) \| mean_pos = (\S+)", n, re.S)
        if mm:
            r["nees_ori_full"], r["nees_pos_full"] = num(mm.group(1)), num(mm.group(2))
    except FileNotFoundError:
        pass
    fp = os.path.join("docker/out/replay", f"{run}_ovser_g{g}", "frames.txt")
    try:
        f = dict(l.strip().split("=", 1) for l in open(fp) if "=" in l)
        r["frames"] = f"{f.get('processed_frames', '?')}/{f.get('published_infra1', '?')}"
    except FileNotFoundError:
        r["frames"] = "?"
    return r


COLS = [("path_win", "win path m", "{:.1f}"), ("win_frac", "win frac", "{:.0%}"),
        ("ov_ate", "OV ATE", "{:.3f}"), ("ov_ate_full", "OV ATE full", "{:.3f}"), ("sim3_full", "OV Sim3 s", "{:.3f}"),
        ("nees_pos_full", "NEESp full", "{:.1f}"), ("nees_ori_full", "NEESo full", "{:.1f}"), ("ov_step_win", "OV step(w)", "{:.3f}"), ("ov_step_all", "OV step(all)", "{:.3f}"),
        ("orb_ate", "ORB ATE", "{:.3f}"), ("orb_step_win", "ORB step(w)", "{:.2f}"),
        ("depth_all", "depth all", "{:.2f}"), ("depth_ok", "depth ok", "{:.2f}"),
        ("tri_rej", "tri rej", "{:.0%}"), ("msckf_rej", "msckf rej", "{:.0%}"), ("tracked", "tracked", "{:.0f}")]


def main() -> int:
    args = [a for a in sys.argv[1:]]
    excl = []
    if "--exclude" in args:
        i = args.index("--exclude")
        excl, args = args[i + 1:], args[:i]
    exp = args[0]
    g = args[1] if len(args) > 1 else "9.81"
    runs = [os.path.basename(x) for x in re.findall(r"/out/eval/([0-9-]+)", open(os.path.join(exp, "rundirs.txt")).read())]
    rows = [row_for(r, g) for r in runs if r not in excl]
    print(f"{os.path.basename(exp)}  gravity_mag {g}" + (f"  (excluded: {' '.join(excl)})" if excl else ""))
    print(f"{'run':16s} {'frames':10s} " + " ".join(f"{h:>12s}" for _, h, _ in COLS))
    for r in rows:
        print(f"{r['run']:16s} {r.get('frames', '?'):10s} " +
              " ".join(f"{(fmt.format(r[k]) if k in r and np.isfinite(r[k]) else 'n/a'):>12s}" for k, _, fmt in COLS))
    print(f"{'mean':16s} {'':10s} " + " ".join(
        f"{(fmt.format(np.nanmean([r.get(k, np.nan) for r in rows])) if any(k in r for r in rows) else 'n/a'):>12s}" for k, _, fmt in COLS))
    print(f"{'std':16s} {'':10s} " + " ".join(
        f"{(fmt.format(np.nanstd([r.get(k, np.nan) for r in rows], ddof=1)) if sum(k in r for r in rows) > 1 else 'n/a'):>12s}" for k, _, fmt in COLS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
