#!/usr/bin/env python3
"""step5_report.py : day 3 step 5 table + plots (run in the container after the batch and after
nees_motion.py has written nees_motion.npz for every flight).
Failures (protocol): in-flight = sim-only watchdog terminated with GT altitude > 0.2 m (VIO
divergence / loss of control in the air) or GT contact (< 0.33 m). Home stalls (executor ESDF-guard
hover in the home phase > 60 s) are counted separately. NEES: protocol value (whole GT airborne
window) and explore-phase only (defined before running, day 3)."""
import json, re, os, sys, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
B = '/out/cl'
L = [json.loads(l) for l in open(f'{B}/d3_step5_results.jsonl')]

def phase_info(name):
    d = f'{B}/runs/{name}'
    rec = []
    for l in open(f'{d}/executor.log'):
        l = re.sub(r'\x1b\[[0-9;]*m', '', l).strip()
        if l.startswith('{'):
            try: j = json.loads(l); rec.append((j['t'], j['phase'], j['ov']))
            except Exception: pass
    still_home = sum(1 for a, b in zip(rec, rec[1:]) if b[1] == 'home' and b[2][2] > 0.3 and
                     np.hypot(b[2][0] - a[2][0], b[2][1] - a[2][1]) < 0.1 and abs(np.remainder(b[2][3] - a[2][3] + np.pi, 2 * np.pi) - np.pi) < 0.05)
    nees_explore = None
    if os.path.exists(f'{d}/nees_motion.npz'):
        z = np.load(f'{d}/nees_motion.npz'); ts = np.array([r[0] for r in rec]); ph = [r[1] for r in rec]
        idx = np.clip(np.searchsorted(ts, z['t'], side='right') - 1, 0, len(ts) - 1)
        m = np.array([ph[i] == 'explore' for i in idx])
        if m.sum() >= 10: nees_explore = float(np.median(z['ori'][m]))
    return still_home, nees_explore

def in_air(name):
    w = open(f'{B}/runs/{name}/watchdog.log').read()
    if 'TERMINATED' not in w: return False
    m = re.search(r'gt=\([^,]+, [^,]+, ([-0-9.e]+)\)', w)
    return m is not None and float(m.group(1)) > 0.2

groups = {}
for r in L:
    r['home_still_s'], r['nees_ori_explore_med'] = phase_info(r['flight'])
    r['fail_air'] = in_air(r['flight']); r['failure'] = r['fail_air'] or bool(r.get('contact'))
    g = 'frontier' if 'frontier' in r['flight'] else 'λ=' + re.search(r'lam([0-9.]+)_', r['flight']).group(1)
    groups.setdefault(g, []).append(r)
order = ['frontier'] + sorted([g for g in groups if g != 'frontier'], key=lambda g: float(g[2:]))
def med(v): v = [x for x in v if x is not None]; return (np.median(v), min(v), max(v)) if v else (np.nan,) * 3
f = lambda rs, k: ('%.3g [%.3g–%.3g]' % med([r.get(k) for r in rs])) if any(r.get(k) is not None for r in rs) else 'n/a'
lines = ['| | n | known @180 s (m³) | path (m) | ATE (m) | NEES ori (window) | NEES ori explore (median) | chosen dI_pose | failures (air / contact) | home stalls |',
         '|---|---|---|---|---|---|---|---|---|---|']
for g in order:
    rs = groups[g]
    lines.append(f"| {g} | {len(rs)} | {f(rs,'known_m3@180s')} | {f(rs,'path_m')} | {f(rs,'ate_m')} | {f(rs,'nees_full_ori')} | {f(rs,'nees_ori_explore_med')} | "
                 f"{f(rs,'chosen_dI_pose_median')} | {sum(r['fail_air'] for r in rs)} / {sum(bool(r.get('contact')) for r in rs)} | {sum(r['home_still_s'] > 60 for r in rs)} |")
open(f'{B}/d3_step5_table.md', 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines))
json.dump(L, open(f'{B}/d3_step5_results_annot.json', 'w'), indent=1)

fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
cm = plt.get_cmap('viridis')
for a, yk, yl in ((ax[0], 'ate_m', 'OpenVINS ATE [m]'), (ax[1], 'nees_full_ori', 'orientation NEES, airborne window (mean)'),
                  (ax[2], 'nees_ori_explore_med', 'orientation NEES, explore phase (median)')):
    for i, g in enumerate(order):
        rs = [r for r in groups[g] if r.get(yk) is not None and r.get('known_m3@180s') is not None]
        if not rs: continue
        c = 'k' if g == 'frontier' else cm((i - 1) / max(1, len(order) - 2))
        xs = [r['known_m3@180s'] for r in rs]; ys = [r[yk] for r in rs]
        a.scatter(xs, ys, s=18, alpha=0.45, color=c, marker='^' if g == 'frontier' else 'o')
        fx = [x for x, r in zip(xs, rs) if r['failure']]; fy = [y for y, r in zip(ys, rs) if r['failure']]
        a.scatter(fx, fy, s=70, facecolors='none', edgecolors='r')
        a.errorbar(np.median(xs), np.median(ys), fmt='^' if g == 'frontier' else 'o', ms=9, color=c, mec='k', label=g)
    a.set_xlabel('known volume after 180 s [m³]'); a.set_ylabel(yl); a.set_yscale('log')
ax[0].legend(fontsize=7, title='large: median; red ring: failure', title_fontsize=7)
fig.suptitle('Day 3 step 5: pose_cov λ sweep, 8 yaws + path-integrated gain, vs frontier (validation, 180 s, 5 flights each)')
fig.tight_layout(); fig.savefig(f'{B}/d3_step5_tradeoff.png', dpi=110); print('plot: d3_step5_tradeoff.png')
