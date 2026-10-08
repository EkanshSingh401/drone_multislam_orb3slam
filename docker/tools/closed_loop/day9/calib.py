#!/usr/bin/env python3
"""calib.py <pred_dir> <s0> <out_prefix>: absolute level of drift_model predictions vs realized drift
(held-out windows): per model and rate, median realized/predicted, log-log OLS slope (1 = level scales right),
binned (quintiles of predicted) RMS of realized / predicted (position: |e| / sigma, sigma = sqrt(trace) so
E[|e|^2/sigma^2] = 1; yaw: |e| / sigma). Writes JSON and a PNG (one panel per target)."""
import glob, json, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.ticker import NullFormatter, LogFormatterSciNotation
pd, s0, op = sys.argv[1], sys.argv[2], sys.argv[3]
W = {}
for f in ('/out/cl/day8/dwrep_tf20.jsonl', '/out/cl/day8/dwrep_tf5.jsonl'):
    for l in open(f):
        w = json.loads(l)
        if w.get('path_m', 0) >= 1: W[w['flight'] + '_%.2f' % w['t0']] = w
P = {}
for f in glob.glob(f'{pd}/*/drift_s0_{s0}.txt'):
    for l in open(f):
        p = l.split()
        if p[0] == 'DRIFT' and p[1] in W: P.setdefault(p[1], {})[p[2]] = (float(p[3]), float(p[4]))
out = {}; fig, axes = plt.subplots(1, 2, figsize=(9, 3.4))
colors = {'random_walk_stereo': '#2a6fdb', 'white': '#d9822b', 'random_walk': '#9aa5b1'}
for ti, (t, idx, real_key, conv) in enumerate((('pos', 0, 'drift_pos_m', 1.0), ('yaw', 1, 'drift_yaw_deg', np.pi / 180))):
    for model in ('random_walk_stereo', 'random_walk', 'white'):
        for rate in ('tf20', 'tf5'):
            ks = [k for k in P if model in P[k] and f'_{rate}_' in k]
            pr = np.array([P[k][model][idx] for k in ks]); re = np.array([W[k][real_key] * conv for k in ks]); ok = (pr > 0) & (re > 0)
            sl = np.polyfit(np.log(pr[ok]), np.log(re[ok]), 1)[0]
            q = np.percentile(pr[ok], [0, 20, 40, 60, 80, 100]); bins = []
            for lo, hi in zip(q[:-1], q[1:]):
                m = ok & (pr >= lo) & (pr <= hi); bins.append((float(np.median(pr[m])), float(np.sqrt(np.mean((re[m] / pr[m]) ** 2)))))
            out[f'{t}_{model}_{rate}'] = {'n': int(ok.sum()), 'median_ratio_real_over_pred': round(float(np.median(re[ok] / pr[ok])), 3),
                                         'loglog_slope': round(float(sl), 3), 'binned_rms_ratio': [(round(a, 5), round(b_, 3)) for a, b_ in bins]}
            if rate == 'tf20':
                ax = axes[ti]; ax.plot([b_[0] / conv for b_ in bins], [b_[1] for b_ in bins], marker='o', ms=8, lw=2, color=colors[model], label=model.replace('_', ' '))
    ax = axes[ti]; ax.axhline(1, color='#888', lw=1, ls='--'); ax.set_xscale('log'); ax.set_yscale('log')
    for a_ in (ax.xaxis, ax.yaxis): a_.set_minor_formatter(NullFormatter()); a_.set_major_formatter(LogFormatterSciNotation())
    ax.set_xlabel('predicted window σ (%s)' % ('m' if t == 'pos' else 'deg'), color='#444'); ax.set_ylabel('RMS realized / predicted', color='#444')
    ax.set_title(f'held-out offices, 15 Hz: {"position" if t == "pos" else "yaw"}', fontsize=10); ax.legend(frameon=False, fontsize=8)
    for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
    ax.grid(alpha=0.2); ax.tick_params(colors='#666', labelsize=8)
fig.tight_layout(); fig.savefig(op + '.png', dpi=120); json.dump(out, open(op + '.json', 'w'), indent=1)
for k, v in out.items(): print(k, 'n', v['n'], 'median real/pred', v['median_ratio_real_over_pred'], 'slope', v['loglog_slope'], 'bins', [b_[1] for b_ in v['binned_rms_ratio']])
