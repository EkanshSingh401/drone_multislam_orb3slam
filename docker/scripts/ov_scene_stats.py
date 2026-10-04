#!/usr/bin/env python3
"""Scene-degradation statistics from one OpenVINS run log (run INSIDE the sim container).

    /opt/scripts/ov_scene_stats.py /out/replay/<run>_ovser_g9.81/openvins.log [--t-start T] [--json]

Parses the DEBUG-level lines the fork emits (open_vins 5b14b93, PATCHES.md s49):
    [FI] ok z=.. ratio=..                        feature triangulated (anchor-frame depth z)
    [FI] reject=tri cond=. near=. far=. nan=. .. triangulation rejected, with the check(s) tripped
    [FI] reject=refine ..                        Gauss-Newton refine rejected
    [MSCKF] in=N used=M                          MSCKF features before / after the updater
    [TRACK] cam0=N cam1=M                        features tracked this frame

and reports, so that degradation is MEASURED alongside ATE:
    median_feature_depth_m   median z of successfully triangulated features
                             (biased near: distant features are what gets rejected)
    median_attempt_depth_m   median z over ALL triangulation attempts, accepted and
                             rejected (NaN excluded): the scene-depth figure to compare
    tri_reject_rate          triangulation rejects / triangulation attempts
    msckf_reject_rate        1 - used / in, over all MSCKF updates
    tracked_cam0_mean        mean features tracked per frame in cam0

Note that the initializer serves both MSCKF updates and SLAM delayed init, so
the [FI] counts cover both. A refine attempt follows every triangulation, even
a failed one, so refine rejects are reported separately and NOT added to the
triangulation rate. The log has no per-line sim time, so --t-start is accepted
for interface symmetry but the statistics cover the whole run.
"""
from __future__ import annotations

import argparse
import json
import re
import sys

import numpy as np

ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--t-start", type=float, default=None)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    txt = ANSI.sub("", open(a.log, errors="replace").read())

    ok = np.array([float(z) for z in re.findall(r"\[FI\] ok z=(\S+)", txt)])
    tri = re.findall(r"\[FI\] reject=tri cond=(\d) near=(\d) far=(\d) nan=(\d)", txt)
    tri_z = np.array([float(z) for z in re.findall(r"\[FI\] reject=tri [^\n]*? z=(\S+)", txt)])
    all_z = np.concatenate([ok, tri_z[np.isfinite(tri_z)]])
    ref = re.findall(r"\[FI\] reject=refine near=(\d) far=(\d) baseline=(\d) nan=(\d)", txt)
    mi = np.array([[int(x), int(y)] for x, y in re.findall(r"\[MSCKF\] in=(\d+) used=(\d+)", txt)])
    tr = np.array([int(x) for x in re.findall(r"\[TRACK\] cam0=(\d+)", txt)])
    if len(ok) + len(tri) == 0:
        print(f"ov_scene_stats: no [FI] lines in {a.log} -- needs open_vins >= 5b14b93 at DEBUG", file=sys.stderr)
        return 3
    T = np.array(tri, int) if tri else np.zeros((0, 4), int)
    attempts = len(ok) + len(T)
    out = {
        "median_feature_depth_m": float(np.median(ok)) if len(ok) else float("nan"),
        "median_attempt_depth_m": float(np.median(all_z)) if len(all_z) else float("nan"),
        "feature_depth_p10_m": float(np.percentile(ok, 10)) if len(ok) else float("nan"),
        "feature_depth_p90_m": float(np.percentile(ok, 90)) if len(ok) else float("nan"),
        "tri_attempts": attempts,
        "tri_reject_rate": len(T) / attempts if attempts else float("nan"),
        "tri_reject_by_check": {"cond": int(T[:, 0].sum()), "near": int(T[:, 1].sum()),
                                "far": int(T[:, 2].sum()), "nan": int(T[:, 3].sum())},
        "refine_rejects": len(ref),
        "refine_baseline_rejects": int(sum(int(r[2]) for r in ref)),
        "msckf_in": int(mi[:, 0].sum()) if len(mi) else 0,
        "msckf_used": int(mi[:, 1].sum()) if len(mi) else 0,
        "msckf_reject_rate": float(1 - mi[:, 1].sum() / mi[:, 0].sum()) if len(mi) and mi[:, 0].sum() else float("nan"),
        "tracked_cam0_mean": float(tr.mean()) if len(tr) else float("nan"),
    }
    if a.json:
        print(json.dumps(out))
    else:
        print(f"  scene: median feature depth {out['median_feature_depth_m']:.2f} m "
              f"(p10 {out['feature_depth_p10_m']:.2f}, p90 {out['feature_depth_p90_m']:.2f}), "
              f"all attempts {out['median_attempt_depth_m']:.2f} m | "
              f"triangulation reject {100 * out['tri_reject_rate']:.0f}% of {attempts} "
              f"(cond {out['tri_reject_by_check']['cond']}, near {out['tri_reject_by_check']['near']}, "
              f"far {out['tri_reject_by_check']['far']}) | MSCKF reject {100 * out['msckf_reject_rate']:.0f}% | "
              f"tracked cam0 {out['tracked_cam0_mean']:.0f}/frame")
    return 0


if __name__ == "__main__":
    sys.exit(main())
