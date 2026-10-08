#!/usr/bin/env python3
"""curse.py <out.json> <flight_dir>... (day 4 step 4): optimizer's curse check.

Per pose_cov decision with a flown goal: predicted = dI_pose of the chosen candidate (logged);
realized = log det Sigma_pose(at the decision) - log det Sigma_pose(on arrival), Sigma_pose = the
IMU orientation+position block of /openvins/joint_covariance ('post'). Arrival = first odometry
sample within reach_tolerance (0.35 m) and 15 deg of the goal, before the next decision; goals
not reached are skipped (counted). Error = predicted - realized (> 0: over-predicted).
Compared between goals picked by argmax and goals picked uniformly at random (planner random_pick),
same flights. Also the measurement part alone is reported (predicted dI_meas vs realized measured
gain is not separable online, so only the total is compared)."""
import json, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(d):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    tops = ['/openvins/joint_covariance', '/exploration_planner/decision', '/ov_msckf/odomimu']
    r.set_filter(rosbag2_py.StorageFilter(topics=tops)); ty = {t: get_message(names[t]) for t in tops}
    jc, dec, od = [], [], []
    while r.has_next():
        tp, raw, _ = r.read_next(); m = deserialize_message(raw, ty[tp])
        if tp == tops[0]:
            if m.stage not in ('', 'post'): continue
            n = m.dim; C = np.array(m.covariance).reshape(n, n); b = next(x for x in m.blocks if x.type == 'imu')
            S = C[b.index:b.index + 6, b.index:b.index + 6]
            jc.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, np.linalg.slogdet(0.5 * (S + S.T))[1]))
        elif tp == tops[1]:
            dec.append(json.loads(m.data))
        else:
            o, p = m.pose.pose.orientation, m.pose.pose.position
            yaw = np.arctan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            od.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, yaw))
    return np.array(jc), dec, np.array(od)

def main():
    out, rows, skipped = sys.argv[1], [], 0
    for d in sys.argv[2:]:
        jc, dec, od = read(d)
        dec = [x for x in dec if x.get('chosen', -1) >= 0 and x.get('type') == 'pose_cov']
        for i, x in enumerate(dec):
            c = x['candidates'][x['chosen']]
            if c.get('dI_pose') is None: continue
            t0 = x['t']; t1 = dec[i + 1]['t'] if i + 1 < len(dec) else t0 + 30
            m = (od[:, 0] > t0) & (od[:, 0] <= t1)
            near = (np.hypot(od[m, 1] - c['xy'][0], od[m, 2] - c['xy'][1]) < 0.35) & \
                   (np.abs(np.remainder(od[m, 3] - c['yaw'] + np.pi, 2 * np.pi) - np.pi) < np.radians(15))
            if not near.any(): skipped += 1; continue
            ta = od[m, 0][np.argmax(near)]
            k0 = np.searchsorted(jc[:, 0], t0, side='right') - 1; k1 = np.searchsorted(jc[:, 0], ta, side='right') - 1
            if k0 < 0 or k1 <= k0: skipped += 1; continue
            real = jc[k0, 1] - jc[k1, 1]
            # rank of the chosen candidate by score
            sc = np.array([np.nan if q.get('score_pc') is None else q['score_pc'] for q in x['candidates']])
            rank = int(np.sum(sc > sc[x['chosen']]))
            rows.append({'flight': os.path.basename(d.rstrip('/')), 't': t0, 'pick': x.get('pick', 'argmax'), 'pred': c['dI_pose'],
                         'pred_meas': c.get('dI_meas'), 'pred_prop': c.get('dI_prop'), 'real': float(real), 'err': c['dI_pose'] - float(real),
                         'travel_s': float(ta - t0), 'rank': rank, 'n_cand': len(x['candidates'])})
    res = {'rows': len(rows), 'skipped_not_reached': skipped}
    for p in ('argmax', 'random'):
        e = np.array([r['err'] for r in rows if r['pick'] == p]); pr = np.array([r['pred'] for r in rows if r['pick'] == p])
        re_ = np.array([r['real'] for r in rows if r['pick'] == p])
        if len(e):
            bs = [np.median(np.random.default_rng(k).choice(e, len(e))) for k in range(2000)]
            res[p] = {'n': len(e), 'err_median': float(np.median(e)), 'err_mean': float(e.mean()), 'err_median_ci95': [float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))],
                      'pred_median': float(np.median(pr)), 'real_median': float(np.median(re_)),
                      'ratio_pred_over_real_median': float(np.median(pr) / np.median(re_)) if np.median(re_) != 0 else None}
    if 'argmax' in res and 'random' in res:
        from scipy.stats import mannwhitneyu
        a = [r['err'] for r in rows if r['pick'] == 'argmax']; b = [r['err'] for r in rows if r['pick'] == 'random']
        res['mannwhitney_argmax_gt_random_p'] = float(mannwhitneyu(a, b, alternative='greater').pvalue)
    print(json.dumps(res, indent=1))
    json.dump({'summary': res, 'rows': rows}, open(out, 'w'), indent=1)

main()
