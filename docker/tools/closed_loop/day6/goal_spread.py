#!/usr/bin/env python3
"""goal_spread.py <ladder_dir_textured> <ladder_dir_plain> (day 6 step 3): does the choice of goal matter?

Realized = log det Sigma_pose(decision) - log det Sigma_pose(arrival) per flown goal (ladder_extract.py
meta.jsonl; argmax and random_pick goals). Per scene: n, median, IQR, SD, 5-95% range, also per second of
travel (realized / horizon), and by pick. SD ratio plain / textured with a 95% CI from a bootstrap over
flights. A goal choice can only matter if realized differs across goals by more than its noise; the
spread is the upper bound on what any objective could exploit."""
import json, sys
import numpy as np

def load(d):
    return [json.loads(l) for l in open(f'{d}/meta.jsonl')]

def stats(x):
    x = np.asarray(x, float)
    return {'n': int(len(x)), 'median': round(float(np.median(x)), 3), 'iqr': [round(float(np.percentile(x, q)), 3) for q in (25, 75)],
            'sd': round(float(np.std(x)), 3), 'p5_p95': [round(float(np.percentile(x, q)), 3) for q in (5, 95)]}

def boot_sd_ratio(a, b, B=2000, seed=0):
    rng = np.random.default_rng(seed)
    fa, fb = sorted({r['flight'] for r in a}), sorted({r['flight'] for r in b})
    out = []
    for _ in range(B):
        pa = [r['realized'] for f in rng.choice(fa, len(fa)) for r in a if r['flight'] == f]
        pb = [r['realized'] for f in rng.choice(fb, len(fb)) for r in b if r['flight'] == f]
        if len(pa) > 3 and len(pb) > 3: out.append(np.std(pb) / np.std(pa))
    return [round(float(np.percentile(out, q)), 2) for q in (2.5, 50, 97.5)]

T, P = load(sys.argv[1]), load(sys.argv[2])
res = {}
for name, rows in (('textured', T), ('plain', P)):
    res[name] = {'all': stats([r['realized'] for r in rows]), 'per_s': stats([r['realized'] / r['horizon_s'] for r in rows]),
                 'horizon_s': stats([r['horizon_s'] for r in rows]), 'flights': len({r['flight'] for r in rows})}
    for pk in ('argmax', 'random'):
        rr = [r['realized'] for r in rows if r['pick'] == pk]
        if rr: res[name][pk] = stats(rr)
res['sd_ratio_plain_over_textured'] = boot_sd_ratio(T, P)
print(json.dumps(res, indent=1))
