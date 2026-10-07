#!/usr/bin/env python3
"""aggregate.py : overnight Stages 2-4 tables (median [min..max]) and Stage 3 trade-off plot."""
import json, re, numpy as np, os
B = '/out/cl'
def load(f): return [json.loads(l) for l in open(f'{B}/{f}')] if os.path.exists(f'{B}/{f}') else []
def wd_air(name):
    w = open(f'{B}/runs/{name}/watchdog.log').read() if os.path.exists(f'{B}/runs/{name}/watchdog.log') else ''
    m = re.search(r'gt=\(([-0-9.e]+), ([-0-9.e]+), ([-0-9.e]+)\)', w)
    return (m is not None and float(m.group(3)) > 0.2)
def fmt(v):
    v = [x for x in v if x is not None and x == x]
    if not v: return 'n/a'
    return f"{np.median(v):.3g} [{min(v):.3g}–{max(v):.3g}]"
def table(rows, title):
    print(f"\n### {title}  (n={len(rows)})")
    keys = [('known_m3@60s','known m³ @60 s'),('known_m3@120s','known m³ @120 s'),('known_m3@180s','known m³ @180 s'),('path_m','path m'),
            ('known_m3_per_m','known m³ per m'),('ate_m','ATE m'),('ate_per_m','ATE per m'),('nees_full_ori','NEES full ori'),
            ('nees_full_rp','NEES full rp'),('nees_full_pos','NEES full pos'),('min_clearance_m','min clearance m'),('goals','goals'),('real_share_median','IG real share')]
    for k, lab in keys: print(f"| {lab} | {fmt([r.get(k) for r in rows])} |")
    air = sum(wd_air(r['flight']) for r in rows); post = sum(r['watchdog_terminated'] for r in rows) - air
    contact = sum(bool(r.get('contact')) for r in rows)
    print(f"| in-flight failures (watchdog in air / contact) | {air} / {contact} |")
    print(f"| post-landing watchdog disarms | {post} |")
    print(f"| arm failures | {sum(r['arm_failed'] for r in rows)} |  guard holds total | {sum(r['guard_holds'] for r in rows)} |")
    print(f"| disarm by px4 / openvins / neither | {sum(r['touchdown_by']=='px4' for r in rows)} / {sum(r['touchdown_by']=='openvins' for r in rows)} / {sum(r['touchdown_by'] is None for r in rows)} |")
s2, s3, s4 = load('stage2_results.jsonl'), load('stage3_results.jsonl'), load('stage4_results.jsonl')
table([r for r in s2 if 'frontier' in r['flight']], 'Stage 2 validation — frontier')
table([r for r in s2 if '_ig_' in r['flight']], 'Stage 2 validation — IG (sigma_l 1.0)')
groups = {}
for r in s3:
    s = float(re.search(r'sig([0-9.]+)_', r['flight']).group(1)); groups.setdefault(s, []).append(r)
groups.setdefault(1.0, []).extend([r for r in s2 if r['flight'] in ('s2_ig_1', 's2_ig_2', 's2_ig_3')])
for s in sorted(groups): table(groups[s], f'Stage 3 sigma_l = {s}')
table([r for r in s4 if 'frontier' in r['flight']], 'Stage 4 forest — frontier')
table([r for r in s4 if '_ig_' in r['flight']], 'Stage 4 forest — IG (sigma_l 1.0)')
# plot
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
fr = [r for r in s2 if 'frontier' in r['flight']]
for a, ykey, ylab in ((ax[0], 'ate_m', 'OpenVINS ATE [m]'), (ax[1], 'nees_full_ori', 'orientation NEES (full cov)')):
    for s in sorted(groups):
        xs = [r['known_m3@180s'] for r in groups[s]]; ys = [r[ykey] for r in groups[s]]
        a.scatter(xs, ys, s=18, alpha=0.5); a.errorbar(np.median(xs), np.median(ys), fmt='o', ms=8, label=f'IG σ_l={s}')
    xs = [r['known_m3@180s'] for r in fr]; ys = [r[ykey] for r in fr]
    a.scatter(xs, ys, s=18, c='k', alpha=0.4, marker='x'); a.errorbar(np.median(xs), np.median(ys), fmt='kx', ms=10, label='frontier')
    a.set_xlabel('known volume after 180 s [m³]'); a.set_ylabel(ylab); a.set_yscale('log')
ax[0].legend(fontsize=7); fig.suptitle('Stage 3: coverage vs estimation quality (dots: flights, large markers: medians)')
fig.tight_layout(); fig.savefig(f'{B}/stage3_tradeoff.png', dpi=110); print('\nplot: stage3_tradeoff.png')
