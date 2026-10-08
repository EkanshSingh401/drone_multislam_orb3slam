#!/usr/bin/env python3
"""combo.py (day 9 step 4a): random-walk prediction + geometry features combined (raw position drift per window).
Deployable variant: log drift ~ log(stereo random-walk prediction, step-3 deployable) + speed, rate, SLAM landmarks,
GT range, return fraction; trained on the validation-scene sensor flights' live windows, tested on the held-out
office windows. Oracle variant (step 1 predictions, held-out replays): leave-one-flight-out within the held-out
windows only (no validation predictions with oracle tracks exist) -- flagged."""
import glob, json, numpy as np
from scipy import stats
keys = ['speed', 'rate', 'n_slam', 'range_med', 'return_frac']
def preds(d):
    P = {}
    for f in glob.glob(f'{d}/*/drift_s0_0.5.txt'):
        for l in open(f):
            p = l.split()
            if p[0] == 'DRIFT': P.setdefault(p[1], {})[p[2]] = float(p[3])
    return P
def table(W, P):
    rows = []
    for w in W:
        k = f"{w['flight']}_{w['t0']:.2f}"
        if k in P and 'random_walk_stereo' in P[k] and all(w.get(x) is not None for x in keys) and w['drift_pos_m'] > 0 and w.get('path_m', 0) >= 1:
            rows.append(([np.log(P[k]['random_walk_stereo'])] + [w[x] for x in keys], np.log(w['drift_pos_m']), w['flight']))
    return rows
def fit(tr, cols):
    X = np.array([r[0] for r in tr])[:, cols]; y = np.array([r[1] for r in tr]); mu, sd = X.mean(0), X.std(0) + 1e-12
    b = np.linalg.lstsq(np.c_[np.ones(len(X)), (X - mu) / sd], y, rcond=None)[0]
    return lambda rows: np.c_[np.ones(len(rows)), (np.array([r[0] for r in rows])[:, cols] - mu) / sd] @ b
out = {}
L = [json.loads(l) for l in open('/out/cl/day9/live_windows.jsonl')]
P = preds('/out/cl/day9/s3')
tr = table([w for w in L if w['world'] == 'validation'], P); te = table([w for w in L if w['world'] in ('office', 'office_plain')], P)
y = np.array([r[1] for r in te])
for name, cols in (('rw_only', [0]), ('geometry_only', [1, 2, 3, 4, 5]), ('rw+geometry', [0, 1, 2, 3, 4, 5])):
    out[f'deployable_{name}'] = round(float(stats.spearmanr(fit(tr, cols)(te), y)[0]), 3)
out['deployable_n_train_test'] = [len(tr), len(te)]
# oracle (step 1, replay windows): leave-one-flight-out within held-out
W = [json.loads(l) for l in open('/out/cl/day8/dwrep_tf20.jsonl')]
Po = preds('/out/cl/day9/s1'); ro = table(W, Po); y = np.array([r[1] for r in ro]); fl = np.array([r[2] for r in ro])
for name, cols in (('rw_only', [0]), ('geometry_only', [1, 2, 3, 4, 5]), ('rw+geometry', [0, 1, 2, 3, 4, 5])):
    p = np.zeros(len(ro))
    for f in set(fl):
        trr = [r for r, q in zip(ro, fl) if q != f]; tee = [r for r, q in zip(ro, fl) if q == f]; p[fl == f] = fit(trr, cols)(tee)
    out[f'oracle_lofo_{name}'] = round(float(stats.spearmanr(p, y)[0]), 3)
out['oracle_n'] = len(ro)
print(json.dumps(out, indent=1)); json.dump(out, open('/out/cl/day9/combo.json', 'w'), indent=1)
