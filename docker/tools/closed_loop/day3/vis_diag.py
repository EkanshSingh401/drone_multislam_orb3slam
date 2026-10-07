#!/usr/bin/env python3
"""vis_diag.py <out.json> <flight_dir>... (day 3 step 1): real-landmark visibility from candidates.

Per decision (planner ~/decision JSON, day-3 fields): n_lm = SLAM landmarks in the joint
covariance; per candidate vis = (landmark, camera) pairs [visible, fov, range, occluded by an
OCCUPIED coarse cell], unk_vis = visible pairs whose ray crosses unknown space, lm_vis = landmarks
with >= 1 pair visible as scored (no occlusion test), lm_vis_occ = after the occupied-cell test.
Reports per flight and pooled: fraction of candidates with >= 1 real landmark visible, pair
breakdown, n_lm distribution, and the same for the chosen candidate."""
import json, sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

def decisions(d):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/exploration_planner/decision']))
    out = []
    while r.has_next():
        _, raw, _ = r.read_next()
        out.append(json.loads(deserialize_message(raw, String).data))
    return out

def summarize(decs):
    D = [x for x in decs if x.get('candidates') and 'vis' in x['candidates'][0]]
    if not D: return None
    nl = np.array([x['n_lm'] for x in D])
    fr_any = [np.mean([c['lm_vis'] > 0 for c in x['candidates']]) for x in D]
    fr_occ = [np.mean([c['lm_vis_occ'] > 0 for c in x['candidates']]) for x in D]
    pairs = np.sum([[c['vis'][k] for k in range(4)] for x in D for c in x['candidates']], 0)
    unk = sum(c['unk_vis'] for x in D for c in x['candidates'])
    ch = [x['candidates'][x['chosen']] for x in D if x.get('chosen', -1) >= 0]
    # decisions where the map has landmarks at all
    Dl = [x for x in D if x['n_lm'] > 0]
    fr_any_l = [np.mean([c['lm_vis'] > 0 for c in x['candidates']]) for x in Dl]
    # fraction of landmarks visible from at least one candidate
    reach = [np.max([c['lm_vis'] for c in x['candidates']]) / x['n_lm'] for x in Dl]
    tot = pairs.sum()
    return {
        'decisions': len(D), 'candidates': int(sum(len(x['candidates']) for x in D)),
        'n_lm_median': float(np.median(nl)), 'n_lm_frac_zero': float(np.mean(nl == 0)), 'n_lm_frac_lt5': float(np.mean(nl < 5)),
        'n_lm_quartiles': [float(q) for q in np.percentile(nl, [25, 50, 75])],
        'frac_cand_ge1_visible_median_per_decision': float(np.median(fr_any)),
        'frac_cand_ge1_visible_pooled': float(np.mean([c['lm_vis'] > 0 for x in D for c in x['candidates']])),
        'frac_cand_ge1_visible_after_occ_pooled': float(np.mean([c['lm_vis_occ'] > 0 for x in D for c in x['candidates']])),
        'frac_cand_ge1_visible_when_n_lm_gt0_pooled': float(np.mean([c['lm_vis'] > 0 for x in Dl for c in x['candidates']])) if Dl else None,
        'frac_landmarks_seen_by_best_candidate_median': float(np.median(reach)) if reach else None,
        'pairs': {'total': int(tot), 'visible': float(pairs[0] / tot), 'fov': float(pairs[1] / tot), 'range': float(pairs[2] / tot),
                  'occluded_occupied': float(pairs[3] / tot)},
        'visible_pairs_crossing_unknown': float(unk / max(1, pairs[0] + pairs[3])),
        'chosen_frac_ge1_visible': float(np.mean([c['lm_vis'] > 0 for c in ch])) if ch else None,
        'chosen_dI_meas_real_median': float(np.nanmedian([c['dI_meas_real'] if c['dI_meas_real'] is not None else np.nan for c in ch])) if ch else None,
    }

def main():
    out, per, pooled = sys.argv[1], {}, []
    for d in sys.argv[2:]:
        decs = decisions(d)
        s = summarize(decs)
        if s is None: continue
        per[os.path.basename(d.rstrip('/'))] = s
        pooled += decs
        print(os.path.basename(d.rstrip('/')), json.dumps(s))
    res = {'flights': per, 'pooled': summarize(pooled)}
    print('POOLED', json.dumps(res['pooled']))
    json.dump(res, open(out, 'w'), indent=1)

main()
