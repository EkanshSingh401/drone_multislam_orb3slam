#!/usr/bin/env python3
"""stage_split.py <stages.txt> [t_from t_to] (day 5 step 5b): realized pose information per OpenVINS
update stage. For each update cycle: prop = ld(propagated) - ld(prev post) (> 0 = information lost to
propagation + clone augmentation), msckf = ld(propagated) - ld(msckf), slam = ld(msckf) - ld(slam),
init = ld(slam) - ld(init) (new SLAM landmarks), marg = ld(init) - ld(post) (marginalizing clones /
SLAM landmarks; 0 for the pose marginal in exact arithmetic). Columns ld6 (pose), ld3 (orientation),
ld2 (local x/y orientation ~ roll/pitch). Prints per-second rates and per-5 s window medians (nats)."""
import json, sys
import numpy as np
L = [l.split() for l in open(sys.argv[1])]
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else -1e9; t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
cyc, cur, prev_post = [], None, None
for p in L:
    t, st = float(p[1]), p[2]; ld = np.array([float(p[3]), float(p[4]), float(p[5])]); n = int(p[6])
    if st == 'propagated':
        if cur and 'post' in cur: cyc.append(cur)
        cur = {'t': t, 'prev': prev_post, 'propagated': ld}
    elif cur is not None:
        cur[st] = ld; cur['n_' + st] = n
        if st == 'post': prev_post = ld
if cur and 'post' in cur: cyc.append(cur)
rows = []
for c in cyc:
    if c['prev'] is None or not (t0 <= c['t'] <= t1) or not all(k in c for k in ('msckf', 'slam', 'init')): continue
    rows.append([c['t'], *(c['propagated'] - c['prev']), *(c['propagated'] - c['msckf']), *(c['msckf'] - c['slam']),
                 *(c['slam'] - c['init']), *(c['init'] - c['post']), c.get('n_msckf', 0), c.get('n_slam', 0), c.get('n_init', 0)])
R = np.array(rows); T = R[-1, 0] - R[0, 0]
names = ['prop', 'msckf', 'slam', 'init', 'marg']
out = {'cycles': len(R), 'span_s': round(T, 1)}
for j, nm in enumerate(names):
    for k, d in enumerate(['pose6', 'ori3', 'rp2']):
        out[f'{nm}_{d}_per_s'] = round(float(R[:, 1 + 3 * j + k].sum() / T), 4)
gain6 = R[:, [4, 7, 10]].sum(1)
out['share_of_update_gain_pose6'] = {nm: round(float(R[:, 1 + 3 * j].sum() / gain6.sum()), 3) for j, nm in zip((1, 2, 3), names[1:4])}
out['net_pose6_per_s'] = round(float((R[:, 4] + R[:, 7] + R[:, 10] - R[:, 1] + R[:, 13]).sum() / T), 4)
# 5 s windows
w = []
for a in np.arange(R[0, 0], R[-1, 0] - 5, 5.0):
    k = (R[:, 0] >= a) & (R[:, 0] < a + 5)
    w.append([R[k, 1].sum(), R[k, 4].sum(), R[k, 7].sum(), R[k, 10].sum()])
w = np.array(w)
out['window5s_median'] = {'prop_loss': round(float(np.median(w[:, 0])), 3), 'msckf': round(float(np.median(w[:, 1])), 3),
                          'slam': round(float(np.median(w[:, 2])), 3), 'init': round(float(np.median(w[:, 3])), 3),
                          'net_gain': round(float(np.median(w[:, 1] + w[:, 2] + w[:, 3] - w[:, 0])), 3)}
out['median_n_used'] = {'msckf': float(np.median(R[:, -3])), 'slam': float(np.median(R[:, -2])), 'init': float(np.median(R[:, -1]))}
print(json.dumps(out))
