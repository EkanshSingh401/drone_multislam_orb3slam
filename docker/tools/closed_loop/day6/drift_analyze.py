#!/usr/bin/env python3
"""drift_analyze.py <drift_windows.jsonl> (day 6 step 4): does drift per metre vary across paths, and what
predicts it? Windows with >= 1 m of GT path. Targets: GT drift per metre (position, yaw) and OpenVINS's own
sigma growth per metre (position, yaw). Spread: median, IQR, p10-p90, ratio p90/p10, share of variance
between flights. Predictors: speed, rotation rate, SLAM landmarks in state, GT range to surfaces, fraction of
returning rays, plain/dim fraction (office_plain). Spearman (95% CI, bootstrap over flights) and
leave-one-flight-out R^2 of log target on standardized predictors. Also calibration of the filter: Spearman
between GT drift and sigma growth."""
import json, sys
from collections import defaultdict
import numpy as np
from scipy import stats
W = [json.loads(l) for l in open(sys.argv[1])]
W = [w for w in W if 'drift_pos_per_m' in w]
fl = np.array([w['flight'] for w in W]); worlds = np.array([w['world'] for w in W])
def arr(k): return np.array([w.get(k) if w.get(k) is not None else np.nan for w in W], float)
T = {'drift_pos_per_m': arr('drift_pos_per_m'), 'drift_yaw_per_m': arr('drift_yaw_per_m'),
     'dsig_pos_per_m': arr('dsig_pos_per_m'), 'dsig_yaw_per_m': arr('dsig_yaw_per_m')}
X = {k: arr(k) for k in ('speed', 'rate', 'n_slam', 'range_med', 'return_frac', 'plain_frac')}
def boot(x, y, B=500, seed=0):
    rng = np.random.default_rng(seed); u = sorted(set(fl)); idx = {f: np.nonzero(fl == f)[0] for f in u}; v = []
    for _ in range(B):
        k = np.concatenate([idx[f] for f in rng.choice(u, len(u))]); ok = np.isfinite(x[k]) & np.isfinite(y[k])
        if ok.sum() > 10: v.append(stats.spearmanr(x[k][ok], y[k][ok])[0])
    return [round(float(np.percentile(v, 2.5)), 3), round(float(np.percentile(v, 97.5)), 3)]
out = {'windows': len(W), 'flights': len(set(fl)), 'by_world': {w: int((worlds == w).sum()) for w in set(worlds)}, 'targets': {}}
for name, y in T.items():
    ok = np.isfinite(y); yy = y[ok]
    p10, p50, p90 = np.percentile(yy, [10, 50, 90])
    fm = defaultdict(list)
    for f, v in zip(fl[ok], yy): fm[f].append(v)
    between = np.var([np.mean(v) for v in fm.values()]) / np.var(yy)
    r = {'median': round(float(p50), 5), 'iqr': [round(float(np.percentile(yy, q)), 5) for q in (25, 75)], 'p10_p90': [round(float(p10), 5), round(float(p90), 5)],
         'ratio_p90_p10': round(float(p90 / p10), 1) if p10 > 0 else None, 'share_var_between_flights': round(float(between), 3),
         'by_world_median': {w: round(float(np.nanmedian(y[(worlds == w) & ok])), 5) for w in sorted(set(worlds))}, 'spearman': {}}
    for k, x in X.items():
        o = ok & np.isfinite(x)
        if o.sum() > 20 and np.nanstd(x[o]) > 0:
            r['spearman'][k] = {'rho': round(float(stats.spearmanr(x[o], y[o])[0]), 3), 'ci': boot(x, y), 'n': int(o.sum())}
    # leave-one-flight-out R^2 on log target (positive targets only), predictors available everywhere
    keys = [k for k in ('speed', 'rate', 'n_slam', 'range_med', 'return_frac') if np.isfinite(X[k]).mean() > 0.5]
    o = ok & np.all([np.isfinite(X[k]) for k in keys], 0) & (y > 0)
    if o.sum() > 50:
        Z = np.column_stack([X[k][o] for k in keys]); Z = (Z - Z.mean(0)) / Z.std(0); A = np.c_[np.ones(o.sum()), Z]; yl = np.log(y[o]); fo = fl[o]
        pred = np.zeros_like(yl)
        for f in set(fo):
            tr = fo != f; b = np.linalg.lstsq(A[tr], yl[tr], rcond=None)[0]; pred[~tr] = A[~tr] @ b
        b = np.linalg.lstsq(A, yl, rcond=None)[0]
        # day 7: leave-one-SCENE-out (train on the other scenes, standardize with the training set)
        wo = worlds[o]; loso = {}
        Zr = np.column_stack([X[k][o] for k in keys])
        for sc in sorted(set(wo)):
            tr = wo != sc
            if tr.sum() < 30 or (~tr).sum() < 10: continue
            mu, sd = Zr[tr].mean(0), Zr[tr].std(0); At = np.c_[np.ones(tr.sum()), (Zr[tr] - mu) / sd]; Ae = np.c_[np.ones((~tr).sum()), (Zr[~tr] - mu) / sd]
            bb = np.linalg.lstsq(At, yl[tr], rcond=None)[0]; pe = Ae @ bb; ye = yl[~tr]
            loso[sc] = {'n': int((~tr).sum()), 'r2': round(float(1 - np.sum((ye - pe) ** 2) / np.sum((ye - ye.mean()) ** 2)), 3),
                        'spearman_pred': round(float(stats.spearmanr(pe, ye)[0]), 3)}
        pooled = []
        r['log_ols_loso'] = loso
        r['log_ols'] = {'n': int(o.sum()), 'predictors': keys, 'std_coef': [round(float(v), 3) for v in b[1:]],
                        'r2_lofo': round(float(1 - np.sum((yl - pred) ** 2) / np.sum((yl - yl.mean()) ** 2)), 3)}
    out['targets'][name] = r
for a, b in (('drift_pos_per_m', 'dsig_pos_per_m'), ('drift_yaw_per_m', 'dsig_yaw_per_m')):
    o = np.isfinite(T[a]) & np.isfinite(T[b]); out[f'calib_{a}_vs_{b}'] = {'rho': round(float(stats.spearmanr(T[a][o], T[b][o])[0]), 3), 'ci': boot(T[a], T[b])}
print(json.dumps(out, indent=1))
