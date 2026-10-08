#!/usr/bin/env python3
"""ceiling.py <out_prefix> <name>=<windows.jsonl> ... (day 8 step 1): how high can Spearman(covariance growth,
drift) be when the filter is PERFECTLY calibrated, given that each window's realized drift is one draw?

Per window (>= 1 m GT path): the filter's predicted covariance of the window's relative error, from its own
marginal sigmas at both ends, assuming the error accrued in the window is independent of the error at its
start: sigma_rel^2 = max(sigma(t1)^2 - sigma(t0)^2, 0) per position axis, and the same for yaw. (Cross-
covariances between t0 and t1 are not logged; this is the random-walk approximation and is noted.)
Predictors scored, both per metre of path: (A) 'dsig' = sigma(t1) - sigma(t0) (the day-6/7 metric) and
(B) 'sig_rel' = sqrt(sum sigma_rel^2). Ceiling: simulate drift from a calibrated filter, |N(0, diag sigma_rel^2)|
(position) and |N(0, sigma_rel_yaw^2)| (yaw), divide by path, Spearman with the predictor; 500 draws ->
mean and 95% interval. Also the realized Spearman and the geometry regression (log drift per m on speed,
rotation rate, SLAM landmarks, GT range, return fraction; leave-one-flight-out predictions) on the same windows.
Calibration: windows binned by sigma_rel (quintiles); per bin the RMS of the normalized error
z = drift / sigma_rel (position: |e| / sqrt(sum sigma_rel^2); 1 = calibrated). Writes <out_prefix>.json and
<out_prefix>_calibration.png."""
import json, sys
import numpy as np
from scipy import stats
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(p):
    W = [json.loads(l) for l in open(p)]
    return [w for w in W if w.get('path_m', 0) >= 1.0 and 'sig_p0' in w]

def geom_lofo(W, y):
    keys = ['speed', 'rate', 'n_slam', 'range_med', 'return_frac']
    X = np.array([[w.get(k) if w.get(k) is not None else np.nan for k in keys] for w in W], float)
    fl = np.array([w['flight'] for w in W]); ok = np.all(np.isfinite(X), 1) & (y > 0)
    pred = np.full(len(W), np.nan)
    if ok.sum() < 30 or len(set(fl[ok])) < 3: return pred
    Z = X[ok]; yl = np.log(y[ok]); fo = fl[ok]; p = np.zeros(ok.sum())
    for f in set(fo):
        tr = fo != f; mu, sd = Z[tr].mean(0), Z[tr].std(0) + 1e-12
        A = np.c_[np.ones(tr.sum()), (Z[tr] - mu) / sd]; b = np.linalg.lstsq(A, yl[tr], rcond=None)[0]
        p[~tr] = np.c_[np.ones((~tr).sum()), (Z[~tr] - mu) / sd] @ b
    pred[ok] = p
    return pred

import os
RAW = os.environ.get('CEIL_NORM', 'per_m') == 'raw'   # raw: per 20 s window, no path normalization
out = {}; rng = np.random.default_rng(0)
fig, axes = plt.subplots(len(sys.argv) - 2, 2, figsize=(9, 3.2 * (len(sys.argv) - 2)), squeeze=False)
for row, arg in enumerate(sys.argv[2:]):
    name, path = arg.split('=', 1); W = load(path); n = len(W)
    L = np.ones(len(W)) if RAW else np.array([w['path_m'] for w in W])
    s0 = np.array([w['sig_p0'] for w in W]); s1 = np.array([w['sig_p1'] for w in W])
    y0 = np.array([w['sig_y0'] for w in W]); y1 = np.array([w['sig_y1'] for w in W])
    rel_p = np.sqrt(np.clip(s1 ** 2 - s0 ** 2, 0, None)); rel_y = np.sqrt(np.clip(y1 ** 2 - y0 ** 2, 0, None))
    dp = np.array([w['drift_pos_m'] for w in W]); dy = np.radians(np.array([w['drift_yaw_deg'] for w in W]))
    res = {'windows': n, 'flights': len({w['flight'] for w in W}),
           'frac_sigma_shrank_pos': round(float(np.mean(np.linalg.norm(s1, axis=1) < np.linalg.norm(s0, axis=1))), 3),
           'frac_sigma_shrank_yaw': round(float(np.mean(y1 < y0)), 3)}
    for tgt, actual, rel, dsig in (('pos', dp, np.linalg.norm(rel_p, axis=1), np.linalg.norm(s1, axis=1) - np.linalg.norm(s0, axis=1)),
                                   ('yaw', dy, rel_y, y1 - y0)):
        r = {}
        for pname, pred in (('dsig_per_m', dsig / L), ('sig_rel_per_m', rel / L)):
            sims = []
            for _ in range(500):
                if tgt == 'pos': e = np.linalg.norm(rng.normal(0, 1, (n, 3)) * rel_p, axis=1)
                else: e = np.abs(rng.normal(0, 1, n) * rel_y)
                sims.append(stats.spearmanr(pred, e / L)[0])
            sims = np.array(sims)
            r[pname] = {'realized': round(float(stats.spearmanr(pred, actual / L)[0]), 3),
                        'ceiling_mean': round(float(np.nanmean(sims)), 3),
                        'ceiling_95': [round(float(np.nanpercentile(sims, 2.5)), 3), round(float(np.nanpercentile(sims, 97.5)), 3)]}
        g = geom_lofo(W, actual / L); ok = np.isfinite(g)
        r['geometry_lofo'] = round(float(stats.spearmanr(g[ok], (actual / L)[ok])[0]), 3) if ok.sum() > 20 else None
        # calibration
        z = actual / np.where(rel > 0, rel, np.nan); okz = np.isfinite(z) & (rel > 0)
        q = np.nanpercentile(rel[okz], [0, 20, 40, 60, 80, 100]); bins = []
        for lo, hi in zip(q[:-1], q[1:]):
            k = okz & (rel >= lo) & (rel <= hi)
            bins.append({'sigma_rel_median': float(np.median(rel[k])), 'n': int(k.sum()), 'rms_z': round(float(np.sqrt(np.mean(z[k] ** 2))), 2)})
        r['calibration'] = bins
        r['rms_z_all'] = round(float(np.sqrt(np.nanmean(z[okz] ** 2))), 2)
        res[tgt] = r
        ax = axes[row][0 if tgt == 'pos' else 1]
        xs = [b['sigma_rel_median'] * (1 if tgt == 'pos' else 180 / np.pi) for b in bins]
        ax.plot(xs, [b['rms_z'] for b in bins], color='#2a6fdb', lw=2, marker='o', ms=8)
        ax.axhline(1.0, color='#888888', lw=1, ls='--'); ax.set_xscale('log'); ax.set_yscale('log')
        from matplotlib.ticker import NullFormatter, LogFormatterSciNotation
        for a_ in (ax.xaxis, ax.yaxis): a_.set_minor_formatter(NullFormatter()); a_.set_major_formatter(LogFormatterSciNotation())
        ax.tick_params(colors='#666', labelsize=8)
        ax.set_xlabel('predicted window σ (%s)' % ('m' if tgt == 'pos' else 'deg'), color='#444')
        ax.set_ylabel('RMS realized / predicted', color='#444'); ax.set_title(f'{name}: {"position" if tgt == "pos" else "yaw"}', fontsize=10)
        for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
        ax.grid(alpha=0.2)
    out[name] = res
fig.tight_layout(); fig.savefig(sys.argv[1] + '_calibration.png', dpi=120)
out['norm'] = 'raw' if RAW else 'per_m'
json.dump(out, open(sys.argv[1] + '.json', 'w'), indent=1)
for name, res in out.items():
    if name == 'norm': continue
    print(name, res['windows'], 'windows', res['flights'], 'flights; sigma shrank pos/yaw', res['frac_sigma_shrank_pos'], res['frac_sigma_shrank_yaw'])
    for t in ('pos', 'yaw'):
        r = res[t]
        print('  ', t, '| dsig: realized', r['dsig_per_m']['realized'], 'ceiling', r['dsig_per_m']['ceiling_mean'], r['dsig_per_m']['ceiling_95'],
              '| sig_rel: realized', r['sig_rel_per_m']['realized'], 'ceiling', r['sig_rel_per_m']['ceiling_mean'], r['sig_rel_per_m']['ceiling_95'],
              '| geometry', r['geometry_lofo'], '| RMS z', r['rms_z_all'], [b['rms_z'] for b in r['calibration']])
