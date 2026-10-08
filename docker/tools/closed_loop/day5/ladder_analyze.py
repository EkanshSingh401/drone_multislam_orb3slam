#!/usr/bin/env python3
"""ladder_analyze.py <ladder_dir> [--world validation] (day 5 step 5a-c): read gain_ladder RES lines +
ladder_extract meta.jsonl; print a JSON summary and write <ladder_dir>/table.md.

5a  per rung: n, median predicted, median realized, Spearman / Pearson with realized (95% CI by
    bootstrap over FLIGHTS), median (pred - realized). Rungs, top = planner: logged dI_pose (incl.
    virtual frontier landmarks), plan_real (planner path, real landmarks; gate vs logged
    dI_prop + dI_meas_real), act_real, act_real_win, dense_win, oracle.
5b  model split of the oracle: SLAM part, MSCKF part; 'lost to landmarks leaving' = act_real -
    act_real_win (same path and spacing, persistent vs windowed landmarks).
5c  realized ~ simple features: path length, rotation, visible GT surface in the FOV (emulated depth rays
    returning within 8 m at the arrival pose, scenes.py geometry), visible real landmarks (lm_vis, logged
    at the planned endpoint), predicted gain (logged). Spearman each; OLS on standardized features,
    leave-one-flight-out R^2."""
import json, os, sys
from collections import defaultdict
import numpy as np
from scipy import stats
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from scenes import solids

D = sys.argv[1]
WORLD = sys.argv[sys.argv.index('--world') + 1] if '--world' in sys.argv else 'validation'
meta = {j['id']: j for j in map(json.loads, open(f'{D}/meta.jsonl'))}
res = defaultdict(dict)
for l in open(f'{D}/res.txt'):
    p = l.split()
    if p[0] != 'RES': continue
    res[p[1]][p[2]] = {'total': float(p[6]), 'n_meas': int(p[7]), 'n_wp': int(p[8]), 'prior': float(p[4]), 'post': float(p[5]), 'now': float(p[3])}
ids = [i for i in meta if i in res]

# ---- visible GT surface at the arrival pose (emulated depth, every 12th pixel) ----
S = [x[1:] for x in solids(WORLD)]
LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for cx, cy, cz, sx, sy, sz in S]); HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for cx, cy, cz, sx, sy, sz in S])
FX, CX, CY = 446.802773, 424.0, 240.0
u, v = np.meshgrid(np.arange(6, 848, 12), np.arange(6, 480, 12))
DC = np.stack([(u.ravel() - CX) / FX, (v.ravel() - CY) / FX, np.ones(u.size)], 1); DC /= np.linalg.norm(DC, axis=1)[:, None]
R_CI = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]])
def surface_rays(x, y, z, yaw):
    R_ItoG = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    D_ = DC @ (R_ItoG @ R_CI.T).T; o = np.array([x, y, z])
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / np.where(np.abs(D_) < 1e-12, 1e-12, D_)
        t1 = (LO[None] - o) * inv[:, None]; t2 = (HI[None] - o) * inv[:, None]
        tn = np.minimum(t1, t2).max(2); tf = np.maximum(t1, t2).min(2)
        tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf).min(1)
        tz = np.where(D_[:, 2] < -1e-9, -o[2] / D_[:, 2], np.inf)
    r = np.minimum(tb, tz)
    return int(np.sum(r <= 8.0)), float(np.mean(1.0 / r[r <= 8.0])) if np.any(r <= 8.0) else 0.0

def boot_ci(x, y, fl, f, B=1000, seed=0):
    rng = np.random.default_rng(seed); u_ = sorted(set(fl)); fl = np.array(fl); vals = []
    for _ in range(B):
        pick = rng.choice(u_, len(u_)); k = np.concatenate([np.nonzero(fl == p)[0] for p in pick])
        if len(k) > 3: vals.append(f(x[k], y[k]))
    return [round(float(np.percentile(vals, 2.5)), 3), round(float(np.percentile(vals, 97.5)), 3)]

real = np.array([meta[i]['realized'] for i in ids]); fl = [meta[i]['flight'] for i in ids]
rows = {'planner (logged dI_pose)': np.array([meta[i]['pred_total'] for i in ids])}
gate_log = np.array([(meta[i]['pred_prop'] or 0) + (meta[i]['pred_meas_real'] or 0) for i in ids])
for r in ['plan_real', 'act_real', 'act_real_win', 'dense_win', 'oracle', 'oracle_slam', 'oracle_msckf']:
    rows[r] = np.array([res[i].get(r, {}).get('total', np.nan) for i in ids])
sp = lambda a, b: stats.spearmanr(a, b)[0]
out = {'n_goals': len(ids), 'flights': len(set(fl)), 'horizon_s_median': round(float(np.median([meta[i]['horizon_s'] for i in ids])), 2),
       'horizon_s_iqr': [round(float(np.percentile([meta[i]['horizon_s'] for i in ids], q)), 2) for q in (25, 75)],
       'realized_median': round(float(np.median(real)), 3), 'realized_iqr': [round(float(np.percentile(real, q)), 3) for q in (25, 75)]}
g = np.abs(rows['plan_real'] - gate_log); ok = np.isfinite(g)
out['gate_plan_real_vs_logged'] = {'n': int(ok.sum()), 'median_abs_diff': round(float(np.median(g[ok])), 5), 'frac_within_0.01': round(float(np.mean(g[ok] < 0.01)), 3),
                                   'p95_abs_diff': round(float(np.percentile(g[ok], 95)), 4)}
lad = {}
for name, x in rows.items():
    k = np.isfinite(x)
    lad[name] = {'n': int(k.sum()), 'pred_median': round(float(np.median(x[k])), 3), 'pred_iqr': [round(float(np.percentile(x[k], q)), 3) for q in (25, 75)],
                 'spearman': round(float(sp(x[k], real[k])), 3), 'spearman_ci': boot_ci(x[k], real[k], [f for f, kk in zip(fl, k) if kk], sp),
                 'pearson': round(float(np.corrcoef(x[k], real[k])[0, 1]), 3), 'bias_median': round(float(np.median(x[k] - real[k])), 3)}
out['ladder'] = lad
# 5b model split
o, os_, om = rows['oracle'], rows['oracle_slam'], rows['oracle_msckf']; dw = rows['dense_win']
out['split_model'] = {'oracle_minus_slam_only (marginal MSCKF value)': round(float(np.nanmedian(o - os_)), 3),
                      'oracle_minus_msckf_only (marginal SLAM value)': round(float(np.nanmedian(o - om)), 3),
                      'oracle_slam_minus_dense_win (new landmarks)': round(float(np.nanmedian(os_ - dw)), 3),
                      'lost_to_landmarks_leaving (act_real - act_real_win)': round(float(np.nanmedian(rows['act_real'] - rows['act_real_win'])), 3),
                      'lost_iqr': [round(float(np.nanpercentile(rows['act_real'] - rows['act_real_win'], q)), 3) for q in (25, 75)],
                      'path_effect (plan_real - act_real)': round(float(np.nanmedian(rows['plan_real'] - rows['act_real'])), 3),
                      'density_effect (dense_win - act_real_win)': round(float(np.nanmedian(dw - rows['act_real_win'])), 3)}
# 5c features
feat = defaultdict(list)
for i in ids:
    m = meta[i]
    feat['path_len'].append(m['path_len']); feat['rotation'].append(m['rotation_rad'])
    feat['lm_vis'].append(m['lm_vis'] if m['lm_vis'] is not None else np.nan)
    feat['pred_logged'].append(m['pred_total'])
    feat['horizon_s'].append(m['horizon_s'])
    a = m.get('arrival')
feat_surf = []
# arrival pose from the tasks file's last WP line per task
last = {}
cur = None
for l in open(f'{D}/tasks.txt'):
    if l.startswith('TASK'): cur = l.split()[1]
    elif l.startswith('WP') and cur:
        p = l.split(); last[cur] = p
for i in ids:
    p = last[i]; x, y, z = map(float, p[2:5]); qx, qy, qz, qw = map(float, p[5:9])
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    n, idp = surface_rays(x, y, z, yaw); feat['surface_rays'].append(n); feat['mean_inv_depth'].append(idp)
F = {k: np.array(v, float) for k, v in feat.items()}
out['features_spearman'] = {k: {'rho': round(float(sp(v[np.isfinite(v)], real[np.isfinite(v)])), 3),
                                'ci': boot_ci(v[np.isfinite(v)], real[np.isfinite(v)], [f for f, kk in zip(fl, np.isfinite(v)) if kk], sp)} for k, v in F.items()}
names = ['path_len', 'rotation', 'surface_rays', 'lm_vis', 'pred_logged', 'horizon_s']
X = np.stack([F[k] for k in names], 1); k = np.all(np.isfinite(X), 1)
X, y, flk = X[k], real[k], np.array(fl)[k]
Z = (X - X.mean(0)) / X.std(0)
A = np.c_[np.ones(len(Z)), Z]; beta = np.linalg.lstsq(A, y, rcond=None)[0]
pred = np.zeros_like(y)
for f in sorted(set(flk)):
    tr = flk != f; b = np.linalg.lstsq(A[tr], y[tr], rcond=None)[0]; pred[~tr] = A[~tr] @ b
ss = np.sum((y - y.mean()) ** 2)
out['ols'] = {'n': int(len(y)), 'std_coef': {n_: round(float(b_), 4) for n_, b_ in zip(names, beta[1:])},
              'r2_in_sample': round(float(1 - np.sum((y - A @ beta) ** 2) / ss), 3), 'r2_leave_one_flight_out': round(float(1 - np.sum((y - pred) ** 2) / ss), 3)}
for one in names:
    Aj = np.c_[np.ones(len(Z)), Z[:, names.index(one)]]; pj = np.zeros_like(y)
    for f in sorted(set(flk)):
        tr = flk != f; b = np.linalg.lstsq(Aj[tr], y[tr], rcond=None)[0]; pj[~tr] = Aj[~tr] @ b
    out['ols'].setdefault('r2_lofo_single', {})[one] = round(float(1 - np.sum((y - pj) ** 2) / ss), 3)
# by pick (argmax vs random) where present
picks = defaultdict(list)
for i in ids: picks[meta[i]['pick']].append(i)
out['n_by_pick'] = {k: len(v) for k, v in picks.items()}
print(json.dumps(out, indent=1))
with open(f'{D}/table.md', 'w') as t:
    t.write('| rung | n | predicted median [IQR] | Spearman vs realized [95% CI, flights] | Pearson | median pred - realized |\n|---|---|---|---|---|---|\n')
    for name, r in lad.items():
        t.write(f"| {name} | {r['n']} | {r['pred_median']} [{r['pred_iqr'][0]}, {r['pred_iqr'][1]}] | {r['spearman']} {r['spearman_ci']} | {r['pearson']} | {r['bias_median']} |\n")
    t.write(f"\nrealized median {out['realized_median']} {out['realized_iqr']} (n {out['n_goals']}, {out['flights']} flights, horizon {out['horizon_s_median']} s {out['horizon_s_iqr']})\n")
