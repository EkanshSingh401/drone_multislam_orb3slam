#!/usr/bin/env python3
"""yaw_drivers.py <windows.jsonl> (day 9 step 4b): what goes with raw yaw drift per 20 s window?
Candidates: rotation total and rate, speed, path, SLAM landmarks, GT range, return fraction, OpenVINS sigma_yaw
growth, and the estimator's gyro bias: |bg| (mean over the window), |bg_z| (axis ~ gravity at level flight), and
its change |delta bg| over the window (ov_state_est.txt columns 11-13). Spearman with 95% CI over flights,
per world and pooled."""
import json, sys
from collections import defaultdict
import numpy as np
from scipy import stats
W = [json.loads(l) for l in open(sys.argv[1])]; W = [w for w in W if w.get('path_m', 0) >= 1.0]
cache = {}
def bg(fl, t0, t1):
    if fl not in cache:
        try: e = np.loadtxt(f'/out/cl/runs/{fl}/ov_state_est.txt'); cache[fl] = (e[:, 0], e[:, 11:14])
        except Exception: cache[fl] = None
    if cache[fl] is None: return (np.nan,) * 3
    t, B = cache[fl]; k = (t >= t0) & (t <= t1)
    if k.sum() < 5: return (np.nan,) * 3
    b = B[k]; return float(np.linalg.norm(b, axis=1).mean()), float(np.abs(b[:, 2]).mean()), float(np.linalg.norm(b[-1] - b[0]))
for w in W: w['bg'], w['bg_z'], w['dbg'] = bg(w['flight'], w['t0'], w['t0'] + 20)
for w in W:
    s0, s1 = w.get('sig_y0'), w.get('sig_y1'); w['sig_rel_yaw'] = float(np.sqrt(max(s1 ** 2 - s0 ** 2, 0))) if s0 is not None else np.nan
keys = ['rot_rad', 'rate', 'speed', 'path_m', 'n_slam', 'range_med', 'return_frac', 'sig_rel_yaw', 'bg', 'bg_z', 'dbg']
y = np.array([w['drift_yaw_deg'] for w in W]); fl = np.array([w['flight'] for w in W]); wo = np.array([w['world'] for w in W])
rng = np.random.default_rng(0)
def boot(x, yy, f):
    u = sorted(set(f)); v = []
    for _ in range(400):
        k = np.concatenate([np.nonzero(f == q)[0] for q in rng.choice(u, len(u))]); ok = np.isfinite(x[k]) & np.isfinite(yy[k])
        if ok.sum() > 10: v.append(stats.spearmanr(x[k][ok], yy[k][ok])[0])
    return [round(float(np.percentile(v, 2.5)), 2), round(float(np.percentile(v, 97.5)), 2)]
out = {}
for scope in ['all'] + sorted(set(wo)):
    m = np.ones(len(W), bool) if scope == 'all' else (wo == scope)
    if m.sum() < 30: continue
    r = {'windows': int(m.sum())}
    for k in keys:
        x = np.array([w.get(k) if w.get(k) is not None else np.nan for w in W], float)[m]; ok = np.isfinite(x) & np.isfinite(y[m])
        if ok.sum() > 20 and np.std(x[ok]) > 0: r[k] = (round(float(stats.spearmanr(x[ok], y[m][ok])[0]), 2), boot(x, y[m], fl[m]))
    out[scope] = r
print(json.dumps(out, indent=0))
