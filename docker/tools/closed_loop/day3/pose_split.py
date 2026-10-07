#!/usr/bin/env python3
"""pose_split.py <out.json> <flight_dir>... (day 3 step 2): is the pose term just a distance penalty?

Per pose_cov decision: dI_pose = dI_meas (a, measurement gain) + dI_prop (b, propagation over
the travel time; <= 0). Score = dI_pose + lambda * coverage_gain (what the planner maximized).
Reported per flight / pooled:
  changed_by_a   fraction of decisions where removing (a) changes the argmax
                 (argmax(b + lambda cov) != argmax(a + b + lambda cov))
  chosen_rank_without_a  median rank of the chosen candidate under b + lambda cov (0 = still first)
  spread         within-decision std of a and of b over candidates (which term separates them)
  rho_b_len      Spearman of b with travel time proxy (path length + turning), within decision
  chosen_a, chosen_b medians; frac_a_zero = chosen candidates with a < 0.01 nats
  nearest        fraction of decisions where the chosen candidate is the cheapest (frontier cost)."""
import json, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from scipy.stats import spearmanr

def decisions(d):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/exploration_planner/decision']))
    out = []
    while r.has_next():
        _, raw, _ = r.read_next()
        out.append(json.loads(deserialize_message(raw, String).data))
    return out

def f(v): return np.nan if v is None else float(v)

def analyse(decs):
    ch_a, rank, sa, sb, rho, ca, cb, cr, near, n = [], [], [], [], [], [], [], [], [], 0
    for x in decs:
        C = x.get('candidates') or []
        if x.get('type') != 'pose_cov' or x.get('chosen', -1) < 0 or not C or 'dI_meas' not in C[0]: continue
        lam = x['lambda']
        a = np.array([f(c['dI_meas']) for c in C]); b = np.array([f(c['dI_prop']) for c in C])
        cov = np.array([f(c['coverage_gain']) for c in C]); cost = np.array([f(c.get('cost')) for c in C])
        ok = np.isfinite(a) & np.isfinite(b) & np.isfinite(cov)
        if ok.sum() < 2 or not ok[x['chosen']]: continue
        n += 1
        full = np.where(ok, a + b + lam * cov, -np.inf); noa = np.where(ok, b + lam * cov, -np.inf)
        k = int(np.argmax(full))
        ch_a.append(int(np.argmax(noa)) != k)
        rank.append(int((noa > noa[k]).sum()))
        sa.append(float(np.std(a[ok]))); sb.append(float(np.std(b[ok])))
        if np.std(b[ok]) > 0 and np.std(cost[ok]) > 0: rho.append(spearmanr(b[ok], cost[ok])[0])
        ca.append(a[k]); cb.append(b[k]); cr.append(f(C[k].get('dI_meas_real')))
        near.append(k == int(np.nanargmin(np.where(ok, cost, np.inf))))
    if not n: return None
    return {'decisions': n, 'changed_by_a': float(np.mean(ch_a)), 'chosen_rank_without_a_median': float(np.median(rank)),
            'chosen_rank_without_a_mean': float(np.mean(rank)),
            'spread_a_median': float(np.median(sa)), 'spread_b_median': float(np.median(sb)),
            'rho_b_vs_cost_median': float(np.median(rho)) if rho else None,
            'chosen_a_median': float(np.median(ca)), 'chosen_b_median': float(np.median(cb)),
            'chosen_a_real_median': float(np.nanmedian(cr)) if np.isfinite(cr).any() else None,
            'frac_chosen_a_lt_0.01': float(np.mean(np.array(ca) < 0.01)), 'chosen_is_cheapest': float(np.mean(near))}

def main():
    out, per, groups = sys.argv[1], {}, {}
    for d in sys.argv[2:]:
        name = os.path.basename(d.rstrip('/'))
        decs = decisions(d)
        s = analyse(decs)
        if s is None: continue
        per[name] = s
        groups.setdefault(name.rsplit('_', 1)[0], []).extend(decs)
        print(name, json.dumps(s))
    G = {g: analyse(v) for g, v in groups.items()}
    for g, s in G.items(): print('GROUP', g, json.dumps(s))
    json.dump({'flights': per, 'groups': G}, open(out, 'w'), indent=1)

main()
