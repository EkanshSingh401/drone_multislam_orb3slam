#!/usr/bin/env python3
"""horizon_table.py <rec_dir>: model (gain_ladder dense_win) vs realized (stage log) by horizon."""
import json, sys, glob, numpy as np
from scipy import stats
D = sys.argv[1]; out = []
for d in sorted(glob.glob(f'{D}/cyc_h*'), key=lambda x: float(x.split('_h')[1])):
    m = {j['id']: j for j in map(json.loads, open(f'{d}/meta.jsonl'))}; R = {}
    for l in open(f'{d}/res.txt'):
        p = l.split()
        if p[0] == 'RES' and p[2] == 'dense_win': R[p[1]] = [float(x) for x in p[3:6]]
    ids = [i for i in R if i in m]
    mp = np.array([R[i][1] - R[i][0] for i in ids]); mm = np.array([R[i][1] - R[i][2] for i in ids]); mt = mm - mp
    rp = np.array([m[i]['prop'] for i in ids]); rm = np.array([m[i]['meas'] for i in ids]); rt = np.array([m[i]['realized'] for i in ids])
    out.append({'horizon_s': float(d.split('_h')[1]), 'n': len(ids), 'prop_model': round(float(np.median(mp)), 3), 'prop_real': round(float(np.median(rp)), 3),
                'meas_model': round(float(np.median(mm)), 3), 'meas_real': round(float(np.median(rm)), 3), 'net_model': round(float(np.median(mt)), 3),
                'net_real': round(float(np.median(rt)), 3), 'spearman_net': round(float(stats.spearmanr(mt, rt)[0]), 3),
                'spearman_meas': round(float(stats.spearmanr(mm, rm)[0]), 3)})
print(json.dumps(out, indent=0))
