#!/usr/bin/env python3
"""step4_compare.py <s4_dir> <train_windows.jsonl> <s0> <out.json> (day 8 step 4): rank realized drift on the held-out
office / plain-office windows (day-7 replays, 15 and 5 Hz) with
  rw       drift_model, random-walk measurement model (q2 from the fitted texture/geometry model)
  white    drift_model, independent 1 px measurements (same tool, same tracks)
  ov       OpenVINS's own window sigma_rel (sqrt(sigma1^2 - sigma0^2), as in ceiling.py)
  geometry log drift per metre on speed, rotation rate, SLAM landmarks, GT range, return fraction, trained on
           the VALIDATION-scene logged windows only (train_windows.jsonl), applied to the held-out windows
  ceiling  Spearman a calibrated filter would reach with the ov sigma_rel (500 draws; also with rw's sigma)
All predictors and targets per metre of GT path. Spearman with 95% CI by bootstrap over flights."""
import glob, json, os, sys
import numpy as np
from scipy import stats
s4, trainp, s0, outp = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
NORM = sys.argv[5] if len(sys.argv) > 5 else 'per_m'   # per_m | raw (per 20 s window, no path normalization)
pred = {}
for f in glob.glob(f'{s4}/*/drift_s0_{s0}.txt'):
    for l in open(f):
        p = l.split()
        if p[0] == 'DRIFT': pred.setdefault(p[1], {})[p[2]] = (float(p[3]), float(p[4]))
W = {}
for f in (os.environ.get('WIN_FILE', '/out/cl/day8/dwrep_tf20.jsonl'),) if os.environ.get('WIN_FILE') else ('/out/cl/day8/dwrep_tf20.jsonl', '/out/cl/day8/dwrep_tf5.jsonl'):
    for l in open(f):
        w = json.loads(l)
        if w.get('path_m', 0) >= 1.0: W[f"{w['flight']}_{w['t0']:.2f}"] = w
keys = ['speed', 'rate', 'n_slam', 'range_med', 'return_frac']
T = [json.loads(l) for l in open(trainp)]
T = [w for w in T if w.get('world') == 'validation' and w.get('path_m', 0) >= 1.0 and all(w.get(k) is not None for k in keys) and w['drift_pos_m'] > 0 and w['drift_yaw_deg'] > 0]
def gfit(target):
    X = np.array([[w[k] for k in keys] for w in T]); y = np.log([w[target] if w[target] > 0 else np.nan for w in T]); ok = np.isfinite(y)
    mu, sd = X[ok].mean(0), X[ok].std(0); A = np.c_[np.ones(ok.sum()), (X[ok] - mu) / sd]; b = np.linalg.lstsq(A, y[ok], rcond=None)[0]
    return lambda w: float(np.r_[1, (np.array([w[k] for k in keys]) - mu) / sd] @ b) if all(w.get(k) is not None for k in keys) else np.nan
gp, gy = (gfit('drift_pos_per_m'), gfit('drift_yaw_per_m')) if NORM == 'per_m' else (gfit('drift_pos_m'), gfit('drift_yaw_deg'))
rng = np.random.default_rng(0); out = {'s0_px': float(s0), 'norm': NORM}
for rate in (('live',) if os.environ.get('WIN_FILE') else ('tf20', 'tf5')):
    ids = [i for i in pred if i in W and (rate == 'live' and i.startswith(os.environ.get('HELD_PREFIX', 'd7_')) or f'_{rate}_' in i) and 'white' in pred[i] and 'random_walk' in pred[i]]
    HAS_ST = all('random_walk_stereo' in pred[i] for i in ids)
    if not ids: continue
    w = [W[i] for i in ids]; Lp = np.array([x['path_m'] for x in w]); L = Lp if NORM == 'per_m' else np.ones(len(w)); fl = np.array([x['flight'] for x in w])
    rel = lambda a1, a0: np.sqrt(np.clip(np.array(a1) ** 2 - np.array(a0) ** 2, 0, None))
    ov_p = np.linalg.norm(rel([x['sig_p1'] for x in w], [x['sig_p0'] for x in w]), axis=1); ov_y = rel([x['sig_y1'] for x in w], [x['sig_y0'] for x in w])
    real = {'pos': np.array([x['drift_pos_m'] for x in w]) / L, 'yaw': np.radians([x['drift_yaw_deg'] for x in w]) / L}
    preds = {'pos': {**({'rw_stereo': np.array([pred[i]['random_walk_stereo'][0] for i in ids]) / L} if HAS_ST else {}), 'rw': np.array([pred[i]['random_walk'][0] for i in ids]) / L, 'white': np.array([pred[i]['white'][0] for i in ids]) / L,
                     'ov': ov_p / L, 'geometry': np.array([gp(x) for x in w]), 'path_only': 1.0 / Lp if NORM == 'per_m' else Lp},
             'yaw': {**({'rw_stereo': np.array([pred[i]['random_walk_stereo'][1] for i in ids]) / L} if HAS_ST else {}), 'rw': np.array([pred[i]['random_walk'][1] for i in ids]) / L, 'white': np.array([pred[i]['white'][1] for i in ids]) / L,
                     'ov': ov_y / L, 'geometry': np.array([gy(x) for x in w]), 'path_only': 1.0 / Lp if NORM == 'per_m' else Lp}}
    def boot(a, y):
        u = sorted(set(fl)); v = []
        for _ in range(500):
            k = np.concatenate([np.nonzero(fl == f)[0] for f in rng.choice(u, len(u))]); ok = np.isfinite(a[k]) & np.isfinite(y[k])
            if ok.sum() > 10: v.append(stats.spearmanr(a[k][ok], y[k][ok])[0])
        return [round(float(np.percentile(v, 2.5)), 3), round(float(np.percentile(v, 97.5)), 3)]
    r = {'windows': len(ids), 'flights': len(set(fl))}
    for t in ('pos', 'yaw'):
        r[t] = {}
        for k, a in preds[t].items():
            ok = np.isfinite(a) & np.isfinite(real[t])
            r[t][k] = {'spearman': round(float(stats.spearmanr(a[ok], real[t][ok])[0]), 3), 'ci': boot(a, real[t])}
        for k, sig in (('ceiling_ov', ov_p if t == 'pos' else ov_y), ('ceiling_rw', np.array([pred[i]['random_walk_stereo' if HAS_ST else 'random_walk'][0 if t == 'pos' else 1] for i in ids]))):
            sims = [stats.spearmanr(sig / L, np.abs(rng.normal(0, 1, len(sig)) * sig) / L if t == 'yaw' else np.linalg.norm(rng.normal(0, 1, (len(sig), 3)), axis=1) * sig / np.sqrt(3) / L)[0] for _ in range(500)]
            r[t][k] = {'mean': round(float(np.mean(sims)), 3), 'ci': [round(float(np.percentile(sims, 2.5)), 3), round(float(np.percentile(sims, 97.5)), 3)]}
        r[t]['level_median'] = {'realized': float(np.median(real[t] * L)), 'rw': float(np.median(preds[t]['rw'] * L)), 'white': float(np.median(preds[t]['white'] * L)), 'ov': float(np.median((ov_p if t == 'pos' else ov_y)))}
    out[rate] = r
json.dump(out, open(outp, 'w'), indent=1)
for rate in ('tf20', 'tf5', 'live'):
    if rate not in out: continue
    r = out[rate]; print(rate, r['windows'], 'windows', r['flights'], 'flights')
    for t in ('pos', 'yaw'):
        print('  ', t, ' | '.join(f"{k} {v['spearman']} {v['ci']}" for k, v in r[t].items() if 'spearman' in v), '| ceiling ov', r[t]['ceiling_ov']['mean'], 'rw', r[t]['ceiling_rw']['mean'])
        print('     level median', {k: round(v, 5) for k, v in r[t]['level_median'].items()})
