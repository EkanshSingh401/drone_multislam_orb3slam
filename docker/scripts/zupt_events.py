#!/usr/bin/env python3
"""Match every OpenVINS ZUPT decision to ground truth (phase 3, PATCHES s50).

    zupt_events.py <replay_dir> [--json]

Reads the fork's [ZUPTEV] lines (image time, accepted flag, estimated |v|) from
<replay_dir>/openvins.log and the Gazebo ground truth <replay_dir>/gt.tum
(base_link, same clock). For each decision it reports the ground-truth speed
(central difference over +-0.1 s) and height above the starting ground level.

Airborne = ground-truth height above the initial ground level > 0.05 m.
The check that matters: accepted ZUPTs while airborne must be zero, hover
included. Exit status 1 if any accepted ZUPT is airborne.
"""
import json
import re
import sys

import numpy as np

AIR_H = 0.05      # m above initial ground level
SPEED_DT = 0.10   # s half-window for the GT speed

def main():
    od = sys.argv[1]
    as_json = "--json" in sys.argv
    import os
    gtf = f"{od}/gt.tum"
    if not os.path.exists(gtf):  # replay only: use the live run's GT (same bag, same clock)
        run = os.path.basename(od.rstrip("/")).split("_ovser")[0]
        gtf = os.path.join(os.path.dirname(od.rstrip("/")), "..", "eval", run, "gt.tum")
    gt = np.loadtxt(gtf)
    t, p = gt[:, 0], gt[:, 1:4]
    z0 = np.median(p[: max(5, len(p) // 50), 2])
    pat = re.compile(r"\[ZUPTEV\] t=([0-9.]+) accepted=(\d) v=([0-9.naife+-]+)")
    ev = []
    with open(f"{od}/openvins.log", errors="replace") as f:
        for line in f:
            m = pat.search(line)
            if m:
                ev.append((float(m.group(1)), int(m.group(2)), float(m.group(3))))
    if not ev:
        print(json.dumps({"dir": od, "events": 0}) if as_json else f"{od}: no [ZUPTEV] lines")
        return 2

    def gt_at(tq):
        i0 = np.searchsorted(t, tq - SPEED_DT); i1 = min(np.searchsorted(t, tq + SPEED_DT), len(t) - 1)
        i0 = min(i0, len(t) - 2); i1 = max(i1, i0 + 1)
        v = np.linalg.norm(p[i1] - p[i0]) / (t[i1] - t[i0])
        h = np.interp(tq, t, p[:, 2]) - z0
        return v, h

    # touchdown: GT height < AIR_H after the last time it was above 0.5 m
    h_all = p[:, 2] - z0
    above = np.where(h_all > 0.5)[0]
    t_td = t[above[-1] + np.argmax(h_all[above[-1]:] < AIR_H)] if len(above) else None
    t_to = t[np.argmax(h_all > AIR_H)] if len(above) else None

    rows = []
    for te, acc, vest in ev:
        v, h = gt_at(te)
        rows.append({"t": te, "acc": acc, "v_est": vest, "v_gt": v, "h": h, "air": bool(h > AIR_H)})
    acc = [r for r in rows if r["acc"]]
    acc_air = [r for r in acc if r["air"]]
    acc_ground = [r for r in acc if not r["air"]]
    out = {
        "dir": od, "decisions": len(rows), "accepted": len(acc),
        "accepted_airborne": len(acc_air), "accepted_ground": len(acc_ground),
        "t_takeoff": t_to, "t_touchdown": t_td,
        "accepted_ground_before_takeoff": sum(1 for r in acc_ground if t_to is not None and r["t"] < t_to),
        "accepted_ground_after_touchdown": sum(1 for r in acc_ground if t_td is not None and r["t"] >= t_td),
        "first_accept_after_touchdown_s": (min(r["t"] for r in acc_ground if r["t"] >= t_td) - t_td)
            if t_td is not None and any(r["t"] >= t_td for r in acc_ground) else None,
        "max_vgt_accepted": max((r["v_gt"] for r in acc), default=None),
        "airborne_min_vgt_decision": min((r["v_gt"] for r in rows if r["air"]), default=None),
        "airborne_accepts": acc_air[:20],
    }
    if as_json:
        print(json.dumps(out))
    else:
        for k, v in out.items():
            if k != "airborne_accepts":
                print(f"  {k}: {v}")
        for r in acc_air[:20]:
            print(f"  AIRBORNE ZUPT t={r['t']:.3f} h={r['h']:.3f} v_gt={r['v_gt']:.3f} v_est={r['v_est']:.3f}")
    return 1 if acc_air else 0

if __name__ == "__main__":
    sys.exit(main())
