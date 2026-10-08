#!/usr/bin/env python3
"""klt_error.py <replay_dir> <flight_dir> (day 7 step 2): KLT tracking error against ground truth in sim.

[TRK] t id u v lines (cam0, scratch build, OV_TRK_LOG) from <replay_dir>/openvins.log. For each track the 3D
point is fixed by casting the FIRST observation's pixel ray from the GT cam0 pose into the scene geometry
(scenes.py boxes + floor; the world from the flight's bringup.log). Its true pixel in every later frame is
the pinhole projection with the GT cam0 pose at that frame's stamp (GT IMU pose slerped, T_CI of the rig;
the sim camera has no distortion). Error e_k = tracked - true (px); e_0 = 0 by construction, so this
measures what tracking adds after detection.
Reported: |e| vs track age, mean error (bias) per axis, error increment d_k = e_k - e_(k-1) (the per-frame
tracking error) with its lag-1 autocorrelation within tracks (0 = independent per frame), and the lag-1
autocorrelation of e itself. Split by the surface type the feature lies on (textured / plain / dim / floor;
office_plain sets from gen_office_world). Prints JSON."""
import json, os, re, sys
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
sys.path.insert(0, '/out/cl')
from scenes import SCENES
PLAIN = {"corr_n0", "corr_n1", "outer_w1", "outer_n0", "outer_w2", "part0_n0", "part0_n1", "nw_desk", "nw_shelf", "nw_cab"}
DIM = {"outer_s0", "outer_w0", "part0_s0", "part0_s1", "corr_s0", "sw_cab", "sw_shelf"}
FX, FY, CX, CY = 446.802773, 446.802773, 424.0, 240.0
R_CI = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]); P_CI = np.array([0.0424, 0.01174, -0.00552])   # cam0 <- IMU
od, rd = sys.argv[1], sys.argv[2]
world = next(m.group(1) for l in open(f'{rd}/bringup.log') for m in [re.search(r'world=(\w+)', l)] if m)
solids = SCENES[world][0]; names = [s[0] for s in solids]
LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
g = np.loadtxt(f'{rd}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]
sl = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
def cam(t):
    Ri = sl([t])[0].as_matrix(); p = np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    R_GC = Ri @ R_CI.T; return R_GC, p + Ri @ (-R_CI.T @ P_CI)
def cast(o, d):
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1 = (LO - o) * inv; t2 = (HI - o) * inv; tn = np.minimum(t1, t2).max(1); tf = np.maximum(t1, t2).min(1)
        tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf); k = int(np.argmin(tb))
        tz = -o[2] / d[2] if d[2] < -1e-9 else np.inf
    if tz < tb[k]: return o + tz * d, 'floor_dim' if (world == 'office_plain' and (o + tz * d)[0] < 6 and (o + tz * d)[1] < 0) else 'floor'
    if not np.isfinite(tb[k]): return None, None
    n = names[k]; kind = 'plain' if (world == 'office_plain' and n in PLAIN) else ('dim' if (world == 'office_plain' and n in DIM) else 'textured')
    return o + tb[k] * d, kind
tr = defaultdict(list)
for l in open(f'{od}/openvins.log', errors='ignore'):
    if l.startswith('[TRK]'):
        p = l.split(); t = float(p[1])
        if g[0, 0] < t < g[-1, 0]: tr[int(p[2])].append((t, float(p[3]), float(p[4])))
cams = {}
def camc(t):
    if t not in cams: cams[t] = cam(t)
    return cams[t]
by = defaultdict(lambda: {'age_err': defaultdict(list), 'e': [], 'd_pairs': [], 'e_pairs': [], 'tracks': 0})
for fid, obs in tr.items():
    if len(obs) < 3: continue
    obs.sort(); t0, u0, v0 = obs[0]; R0, o0 = camc(t0)
    dC = np.array([(u0 - CX) / FX, (v0 - CY) / FY, 1.0]); dG = R0 @ dC; dG /= np.linalg.norm(dG)
    X, kind = cast(o0, dG)
    if X is None or np.linalg.norm(X - o0) > 12: continue
    B = by[kind]; B['tracks'] += 1; prev_e = None; prev_d = None
    for k, (t, u, v) in enumerate(obs):
        Rk, ok_ = camc(t); pc = Rk.T @ (X - ok_)
        if pc[2] < 0.2: break
        e = np.array([u - (FX * pc[0] / pc[2] + CX), v - (FY * pc[1] / pc[2] + CY)])
        B['age_err'][min(k, 30)].append(np.linalg.norm(e))
        if k > 0: B['e'].append(e)
        if prev_e is not None:
            d = e - prev_e
            if k > 1: B['e_pairs'].append((prev_e, e))
            if prev_d is not None: B['d_pairs'].append((prev_d, d))
            prev_d = d
        prev_e = e
out = {'replay': od.rstrip('/').split('/')[-1], 'world': world, 'tracks_total': len(tr), 'median_frame_gap_s': None, 'by_surface': {}}
gaps = [b[0] - a[0] for v in tr.values() for a, b in zip(sorted(v)[:-1], sorted(v)[1:])]
out['median_frame_gap_s'] = round(float(np.median(gaps)), 4) if gaps else None
for kind, B in by.items():
    if not B['e']: continue
    E = np.array(B['e']); D = np.array([p[1] for p in B['d_pairs']]); Dp = np.array([p[0] for p in B['d_pairs']])
    Ep = np.array([p[0] for p in B['e_pairs']]); Ec = np.array([p[1] for p in B['e_pairs']])
    c = lambda a, b, i: round(float(np.corrcoef(a[:, i], b[:, i])[0, 1]), 3) if len(a) > 10 else None
    out['by_surface'][kind] = {'tracks': B['tracks'], 'samples': len(E),
        'err_px_median_by_age': {a: round(float(np.median(v)), 3) for a, v in sorted(B['age_err'].items()) if a in (1, 2, 3, 5, 10, 20, 30)},
        'bias_px_uv': [round(float(E[:, 0].mean()), 3), round(float(E[:, 1].mean()), 3)],
        'incr_sd_px_uv': [round(float(D[:, 0].std()), 3), round(float(D[:, 1].std()), 3)] if len(D) else None,
        'incr_lag1_corr_uv': [c(Dp, D, 0), c(Dp, D, 1)], 'err_lag1_corr_uv': [c(Ep, Ec, 0), c(Ep, Ec, 1)]}
print(json.dumps(out))
