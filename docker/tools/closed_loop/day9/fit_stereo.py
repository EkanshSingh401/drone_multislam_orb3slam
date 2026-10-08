#!/usr/bin/env python3
"""fit_stereo.py <out.jsonl> <ov_dir> <flight_dir> (training only): per-track first-sample disparity error (cam1 at its first
common frame, cam0 = 0 by construction) and robust per-track rates: single camera q2 = median_k |e0_k|^2/(1.386 k),
disparity q2d = median_k |delta_k - delta_0|^2/(1.386 k), delta = e0 - e1 (2 axes). Appends one JSON line.
stereo_corr.py <replay_ov_dir> <flight_dir>: cam0 vs cam1 KLT error of the same feature (training scenes):
3D point from cam0's first observation (GT ray cast), errors e0_k, e1_k in each camera by GT projection;
reports correlation of the per-frame increments (d0, d1) per axis and of the errors, and the growth of the
disparity error |e0_u - e1_u| vs |e0_u| with track age."""
import re, sys, json
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
sys.path.insert(0, '/out/cl')
from scenes import SCENES
FX, FY, CX, CY = 446.802773, 446.802773, 424.0, 240.0
Rci = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]); T = [np.array([0.0424, 0.01174, -0.00552]), np.array([-0.0526, 0.01174, -0.00552])]
outp, od, rd = sys.argv[1], sys.argv[2], sys.argv[3]
world = next(m.group(1) for l in open(f'{rd}/bringup.log') for m in [re.search(r'world=(\w+)', l)] if m)
S = SCENES[world][0]; LO = np.array([[c[1]-c[4]/2, c[2]-c[5]/2, c[3]-c[6]/2] for c in S]); HI = np.array([[c[1]+c[4]/2, c[2]+c[5]/2, c[3]+c[6]/2] for c in S])
g = np.loadtxt(f'{rd}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]; sl = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
def cam(t, c):
    Ri = sl([t])[0].as_matrix(); p = np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)]); return Ri @ Rci.T, p + Ri @ (-Rci.T @ T[c]), p[2]
tr = [defaultdict(list), defaultdict(list)]
for l in open(f'{od}/openvins.log', errors='ignore'):
    if l.startswith('[TRK]') or l.startswith('[TRK1]'):
        p = l.split(); t = float(p[1])
        if g[0, 0] < t < g[-1, 0]: tr[0 if p[0] == '[TRK]' else 1][int(p[2])].append((round(t, 4), float(p[3]), float(p[4])))
base_d, q2s, q2d = [], [], []
for fid, o0 in tr[0].items():
    o1 = dict((a[0], a[1:]) for a in tr[1].get(fid, []))
    o0.sort()
    if len(o0) < 4 or len(o1) < 4: continue
    t0, u0, v0 = o0[0]; RG, oc, z = cam(t0, 0)
    if z < 0.5: continue
    d = RG @ np.array([(u0 - CX) / FX, (v0 - CY) / FY, 1.0]); d /= np.linalg.norm(d)
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1 / np.where(np.abs(d) < 1e-12, 1e-12, d); a1 = (LO - oc) * inv; a2 = (HI - oc) * inv
        tn = np.minimum(a1, a2).max(1); tf = np.maximum(a1, a2).min(1); tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf).min()
        tz = -oc[2] / d[2] if d[2] < -1e-9 else np.inf
    rng_ = min(tb, tz)
    if not np.isfinite(rng_) or rng_ > 12: continue
    X = oc + rng_ * d; base1 = None; E0 = []; DL = []
    for k, (t, u, v) in enumerate(o0):
        if t not in o1: prev = None; continue
        e = []
        for c, (uu, vv) in ((0, (u, v)), (1, o1[t])):
            RG, oc_, _ = cam(t, c); pc = RG.T @ (X - oc_)
            e.append(np.array([uu - (FX * pc[0] / pc[2] + CX), vv - (FY * pc[1] / pc[2] + CY)]))
        if base1 is None: base1 = e[1] - e[0]; base_d.append(base1)
        E0.append(np.linalg.norm(e[0])); DL.append(np.linalg.norm((e[0] - e[1]) + base1))
    if len(E0) >= 5:
        k = np.arange(1, len(E0)); q2s.append(float(np.median(np.square(E0[1:]) / (1.386 * k)))); q2d.append(float(np.median(np.square(DL[1:]) / (1.386 * k))))
B = np.array(base_d)
mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x)))
open(outp, 'a').write(json.dumps({'replay': od, 'tracks': len(q2s), 'sd_first_u': float(mad(B[:, 0])), 'sd_first_v': float(mad(B[:, 1])),
                                  'q2_single_median': float(np.median(q2s)), 'q2_disp_median': float(np.median(q2d)),
                                  'rho_from_medians': float(1 - np.median(q2d) / (2 * np.median(q2s)))}) + '\n')
