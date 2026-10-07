#!/usr/bin/env python3
"""step4_plot.py : lambda sweep (pose_cov) vs frontier and old IG references (day 2 step 4)."""
import json, re, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
B = '/out/cl'
L = [json.loads(l) for l in open(f'{B}/d2_step4_results.jsonl')]
S2 = [json.loads(l) for l in open(f'{B}/stage2_results_v2.jsonl')]
groups = {}
for r in L: groups.setdefault(float(re.search(r'lam([0-9.]+)_', r['flight']).group(1)), []).append(r)
refs = {'frontier': [r for r in S2 if 'frontier' in r['flight']], 'old IG (σ_l=1)': [r for r in S2 if '_ig_' in r['flight']]}
def med(v): v = [x for x in v if x is not None]; return (np.median(v), min(v), max(v)) if v else (np.nan,) * 3
print('| group | n | known @180 s (m³) | path (m) | ATE (m) | NEES full ori | chosen dI_pose | chosen coverage (m³) | in-flight failures | contacts |')
print('|---|---|---|---|---|---|---|---|---|---|')
def row(name, rs):
    f = lambda k: '%.3g [%.3g–%.3g]' % med([r.get(k) for r in rs]) if any(r.get(k) is not None for r in rs) else 'n/a'
    air = sum(1 for r in rs if r.get('watchdog_terminated') and 'TERMINATED' in open(f"{B}/runs/{r['flight']}/watchdog.log").read() and float(re.search(r'gt=\([^,]+, [^,]+, ([-0-9.e]+)\)', open(f"{B}/runs/{r['flight']}/watchdog.log").read()).group(1)) > 0.2)
    print(f"| {name} | {len(rs)} | {f('known_m3@180s')} | {f('path_m')} | {f('ate_m')} | {f('nees_full_ori')} | {f('chosen_dI_pose_median')} | {f('chosen_coverage_gain_median')} | {air} | {sum(bool(r.get('contact')) for r in rs)} |")
for k, rs in refs.items(): row(k, rs)
for l in sorted(groups): row(f'pose_cov λ={l}', groups[l])
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
for a, yk, yl in ((ax[0], 'ate_m', 'OpenVINS ATE [m]'), (ax[1], 'nees_full_ori', 'orientation NEES (full cov)')):
    for l in sorted(groups):
        xs = [r['known_m3@180s'] for r in groups[l]]; ys = [r[yk] for r in groups[l]]
        a.scatter(xs, ys, s=18, alpha=0.5); a.errorbar(np.median(xs), np.median(ys), fmt='o', ms=8, label=f'pose_cov λ={l}')
    for (k, rs), mk in zip(refs.items(), ('kx', 'k^')):
        xs = [r['known_m3@180s'] for r in rs]; ys = [r[yk] for r in rs]
        a.scatter(xs, ys, s=16, c='k', alpha=0.3, marker=mk[1]); a.errorbar(np.median(xs), np.median(ys), fmt=mk, ms=10, label=k)
    a.set_xlabel('known volume after 180 s [m³]'); a.set_ylabel(yl); a.set_yscale('log')
ax[0].legend(fontsize=7); fig.suptitle('Day 2 step 4: λ sweep of score = dI_pose + λ·coverage (dots: flights, large: medians)')
fig.tight_layout(); fig.savefig(f'{B}/d2_step4_tradeoff.png', dpi=110); print('plot: d2_step4_tradeoff.png')
