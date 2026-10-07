#!/usr/bin/env python3
"""nees_motion_split.py <flight_dir>... (day 3 step 4): from nees_motion.npz, orientation NEES
split by motion state, per flight and pooled by group (prefix before the flight index), and
per-flight Spearman correlations of mean / median NEES with the motion and feature means.
'still' = |yaw rate| < 0.05 rad/s and speed < 0.1 m/s; 'moving' = everything else.
Also NEES vs time since the vehicle last moved (hover duration)."""
import sys, os, json, numpy as np
from scipy.stats import spearmanr
rows = []
for d in sys.argv[1:]:
    z = np.load(f'{d}/nees_motion.npz')
    still = (z['yaw_rate'] < 0.05) & (z['speed'] < 0.1)
    t = z['t']; ts = np.full(len(t), np.nan); last = None
    for i in range(len(t)):
        if not still[i]: last = t[i]
        ts[i] = t[i] - last if last is not None else np.nan
    name = os.path.basename(d.rstrip('/'))
    r = dict(flight=name, window_s=float(t[-1] - t[0]), frac_still=float(still.mean()),
             nees_still_med=float(np.median(z['ori'][still])) if still.any() else None,
             nees_moving_med=float(np.median(z['ori'][~still])) if (~still).any() else None,
             nees_mean=float(z['ori'].mean()), nees_med=float(np.median(z['ori'])),
             yaw_rate=float(z['yaw_rate'].mean()), speed=float(z['speed'].mean()), n_slam=float(np.nanmean(z['n_slam'])),
             n_msckf=float(np.nanmean(z['n_msckf'])) if np.isfinite(z['n_msckf']).any() else None)
    for a, b in ((0, 2), (2, 10), (10, 30), (30, 1e9)):
        m = still & (ts >= a) & (ts < b)
        r[f'nees_still_{a}-{int(min(b, 999))}s_med'] = float(np.median(z['ori'][m])) if m.sum() >= 10 else None
    rows.append(r); print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}))
y, ym = np.array([r['nees_mean'] for r in rows]), np.array([r['nees_med'] for r in rows])
for k in ('frac_still', 'yaw_rate', 'speed', 'n_slam', 'n_msckf', 'window_s'):
    x = np.array([np.nan if r[k] is None else r[k] for r in rows], float)
    if np.isfinite(x).sum() < 4: continue
    m = np.isfinite(x)
    a, b = spearmanr(x[m], y[m]), spearmanr(x[m], ym[m])
    print(f'spearman {k:10s} vs mean NEES rho={a[0]:+.2f} p={a[1]:.3f} | vs median NEES rho={b[0]:+.2f} p={b[1]:.3f} (n={m.sum()})')
