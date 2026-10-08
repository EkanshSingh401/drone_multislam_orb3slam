#!/usr/bin/env python3
"""fit_rw.py <train.jsonl> <test.jsonl> <out.json> (day 8 step 2): random-walk KLT error model.

Tracks from klt_tracks.py. Model, per image axis: e_k = e_(k-1) + d_k, d_k ~ N(0, q2) per frame, with
  log q2 = b0 + b1 log(min_eig) + b2 log(depth) + b3 cos_view + b4 log(flow + 0.1) + b5 log(dt)
fitted by least squares on the per-track log of the mean squared increment (training tracks only: the
validation scene; office and plain office are fully held out), Huber-robust (3 IRLS passes).
Held-out evaluation: Spearman and R^2 (log) of predicted vs measured q2; random-walk shape check:
median over tracks of |e_k|^2 / (1.386 k q2_hat) by age k (1 at every age if the error grows as a random walk
at the predicted rate; a white-noise error would fall like 1/k); targets are the robust per-track rates
q2_rw = median_k |e_k|^2 / (1.386 k); same with the pooled
constant q2 (no covariates) for comparison. Per surface kind for the plain office."""
import json, sys
import numpy as np
from scipy import stats
F = ['min_eig', 'depth', 'cos_view', 'flow', 'dt']
def X(rows):
    return np.column_stack([np.log(np.maximum([r['min_eig'] for r in rows], 1e-3)), np.log([r['depth'] for r in rows]),
                            [r['cos_view'] for r in rows], np.log(np.array([r['flow'] for r in rows]) + 0.1), np.log([r['dt'] for r in rows])])
def q2_rw(r):
    """robust random-walk rate of one track: median over ages k of |e_k|^2 / (1.386 k) (2-D Gaussian:
    median |e|^2 = 2 ln2 x per-axis variance)."""
    e = np.array(r['e_abs'][1:]); k = np.arange(1, len(e) + 1)
    return float(np.median(e ** 2 / (1.386 * k)))
def load(p):
    rows = [r for r in map(json.loads, open(p)) if r['n'] >= 5]
    for r in rows: r['q2'] = q2_rw(r)
    return [r for r in rows if r['q2'] > 0]
tr, te = load(sys.argv[1]), load(sys.argv[2])
Xtr, ytr = X(tr), np.log([r['q2'] for r in tr])
mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-12
A = np.c_[np.ones(len(tr)), (Xtr - mu) / sd]; w = np.ones(len(tr))
for _ in range(3):
    b = np.linalg.lstsq(A * w[:, None], ytr * w, rcond=None)[0]; r = ytr - A @ b
    s = 1.4826 * np.median(np.abs(r)); w = np.sqrt(np.minimum(1.0, 1.345 * s / np.maximum(np.abs(r), 1e-12)))
def predict(rows): return A_of(rows) @ b
def A_of(rows): return np.c_[np.ones(len(rows)), (X(rows) - mu) / sd]
q_const = float(np.exp(np.median(ytr)))
def evaluate(rows):
    if len(rows) < 20: return None
    y = np.log([r['q2'] for r in rows]); p = A_of(rows) @ b
    out = {'tracks': len(rows), 'spearman_q2': round(float(stats.spearmanr(p, y)[0]), 3),
           'r2_log_q2': round(float(1 - np.sum((y - p) ** 2) / np.sum((y - y.mean()) ** 2)), 3),
           'median_log_ratio_pred_over_meas': round(float(np.median(p - y)), 3)}
    for name, qh in (('model', np.exp(p)), ('constant', np.full(len(rows), q_const))):
        zs = {a: [] for a in (1, 2, 5, 10, 20, 30)}
        for r, q in zip(rows, qh):
            for a in zs:
                if a < len(r['e_abs']): zs[a].append(r['e_abs'][a] ** 2 / (1.386 * a * q))
        # 1.0 at every age = the error grows as a random walk at the predicted rate (median over tracks)
        out[f'median_ratio_by_age_{name}'] = {a: round(float(np.median(v)), 2) for a, v in zs.items() if len(v) > 10}
    out['frac_tracks_gross'] = round(float(np.mean([max(r['e_abs']) > 5 for r in rows])), 3)
    return out
res = {'features': F, 'train_tracks': len(tr), 'train_flights': sorted({r['flight'] for r in tr}),
       'coef_standardized': dict(zip(['intercept'] + F, [round(float(v), 3) for v in b])),
       'feature_mean_sd': {f: [round(float(m), 3), round(float(s), 3)] for f, m, s in zip(F, mu, sd)},
       'q2_constant_px2_per_frame': round(q_const, 4),
       'train_fit': evaluate(tr), 'test_all': evaluate(te),
       'test_by_world': {w_: evaluate([r for r in te if r['world'] == w_]) for w_ in sorted({r['world'] for r in te})},
       'test_by_kind': {k: evaluate([r for r in te if r['kind'] == k]) for k in sorted({r['kind'] for r in te})},
       'test_by_rate': {str(d): evaluate([r for r in te if abs(r['dt'] - d) < 0.03]) for d in (0.068, 0.228)}}
# expose the fitted model for the predictor (coefficients in raw feature units)
res['model_raw'] = {'b': [float(v) for v in b], 'mu': [float(v) for v in mu], 'sd': [float(v) for v in sd]}
json.dump(res, open(sys.argv[3], 'w'), indent=1)
print(json.dumps({k: res[k] for k in ('train_tracks', 'coef_standardized', 'q2_constant_px2_per_frame', 'train_fit', 'test_all')}, indent=1))
for k in ('test_by_world', 'test_by_kind', 'test_by_rate'):
    for kk, v in res[k].items():
        if v: print(k, kk, v['tracks'], 'rho', v['spearman_q2'], 'R2', v['r2_log_q2'], 'bias', v['median_log_ratio_pred_over_meas'], 'ratio', v['median_ratio_by_age_model'], 'gross', v['frac_tracks_gross'])
