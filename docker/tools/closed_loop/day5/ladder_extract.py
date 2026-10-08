#!/usr/bin/env python3
"""ladder_extract.py <out_dir> <flight_dir>... (day 5 step 5a): tasks for gain_ladder.

Per pose_cov decision with a flown goal (arrival as day4/curse.py: within 0.35 m and 15 deg of the goal
before the next decision; goals not reached are skipped and counted):
  JC at the decision  = latest 'post' /openvins/joint_covariance stamped <= decision t (what the planner
                        held), written as raw CDR for gain_ladder
  realized            = log det Sigma_pose(JC at t0) - log det Sigma_pose(latest JC <= arrival)
  flown waypoints     = /ov_msckf/odomimu poses: every 0.1 s (sparse source) and at every OpenVINS clone
                        time in the window (dense = camera rate), each with the ids of the SLAM landmarks
                        in the state at that time (latest JC)
  new landmarks       = ids in the window's JCs absent at t0: first position, first / last time seen
  MSCKF               = points of /ov_msckf/points_msckf in the window (position, publish stamp = use time)
Writes <out_dir>/tasks.txt, <out_dir>/jc/<id>.cdr and <out_dir>/meta.jsonl (logged prediction terms,
realized, horizon, decision features for 5c)."""
import json, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

def slog(S): return float(np.linalg.slogdet(0.5 * (S + S.T))[1])

def read(d):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    tops = [t for t in ['/openvins/joint_covariance', '/exploration_planner/decision', '/ov_msckf/odomimu', '/ov_msckf/points_msckf'] if t in names]
    r.set_filter(rosbag2_py.StorageFilter(topics=tops)); ty = {t: get_message(names[t]) for t in tops}
    jc, dec, od, ms = [], [], [], []
    while r.has_next():
        tp, raw, _ = r.read_next(); m = deserialize_message(raw, ty[tp])
        if tp == '/openvins/joint_covariance':
            if m.stage not in ('', 'post'): continue
            n = m.dim; C = np.array(m.covariance).reshape(n, n); b = next(x for x in m.blocks if x.type == 'imu')
            ids = {int(L.feature_id): tuple(L.p_fing) for L in m.landmarks}
            clones = [x.clone_timestamp for x in m.blocks if x.type == 'clone']
            jc.append({'t': m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, 'ld': slog(C[b.index:b.index + 6, b.index:b.index + 6]),
                       'raw': bytes(raw), 'ids': ids, 'clones': clones})
        elif tp == '/exploration_planner/decision':
            dec.append(json.loads(m.data))
        elif tp == '/ov_msckf/odomimu':
            o, p = m.pose.pose.orientation, m.pose.pose.position
            od.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        else:
            t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            pts = [(float(a), float(b), float(c)) for a, b, c in point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)]
            ms.append((t, pts))
    return jc, dec, np.array(od), ms

def yaw_of(q): x, y, z, w = q; return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def main():
    out = sys.argv[1]; os.makedirs(f'{out}/jc', exist_ok=True)
    ft = open(f'{out}/tasks.txt', 'w'); fm = open(f'{out}/meta.jsonl', 'w'); skipped = 0; nt = 0
    for d in sys.argv[2:]:
        fl = os.path.basename(d.rstrip('/'))
        jc, dec, od, ms = read(d)
        if not jc or len(od) == 0: continue
        jt = np.array([j['t'] for j in jc])
        dec = [x for x in dec if x.get('chosen', -1) >= 0 and x.get('type') == 'pose_cov']
        oy = np.array([yaw_of(r[4:8]) for r in od])
        for i, x in enumerate(dec):
            c = x['candidates'][x['chosen']]
            if c.get('dI_pose') is None: continue
            t0 = x['t']; tn = dec[i + 1]['t'] if i + 1 < len(dec) else t0 + 30
            m = (od[:, 0] > t0) & (od[:, 0] <= tn)
            near = (np.hypot(od[m, 1] - c['xy'][0], od[m, 2] - c['xy'][1]) < 0.35) & \
                   (np.abs(np.remainder(oy[m] - c['yaw'] + np.pi, 2 * np.pi) - np.pi) < np.radians(15))
            if not near.any(): skipped += 1; continue
            t1 = od[m, 0][np.argmax(near)]
            k0 = np.searchsorted(jt, t0, side='right') - 1; k1 = np.searchsorted(jt, t1, side='right') - 1
            if k0 < 0 or k1 <= k0: skipped += 1; continue
            tid = f'{fl}_{i}'; nt += 1
            open(f'{out}/jc/{tid}.cdr', 'wb').write(jc[k0]['raw'])
            i0 = np.searchsorted(od[:, 0], t0)
            yaw0 = float(oy[min(i0, len(od) - 1)])
            ft.write(f"TASK {tid} {out}/jc/{tid}.cdr {c['xy'][0]} {c['xy'][1]} {c['yaw']} {yaw0} {x.get('path_spacing', 0.5)} 1.5 0.5 0.6\n")
            def wp(t, dense):
                k = min(np.searchsorted(od[:, 0], t), len(od) - 1); r = od[k]
                kj = max(0, np.searchsorted(jt, t, side='right') - 1); ids = list(jc[kj]['ids'])
                ft.write(f"WP {t - t0:.4f} {r[1]:.5f} {r[2]:.5f} {r[3]:.5f} {r[4]:.7f} {r[5]:.7f} {r[6]:.7f} {r[7]:.7f} {int(dense)} {len(ids)} {' '.join(map(str, ids))}\n")
            clones = sorted({ct for j in jc[k0:k1 + 1] for ct in j['clones'] if t0 < ct <= t1})
            ev = sorted([(t, 0) for t in np.arange(t0 + 0.1, t1, 0.1)] + [(t1, 0)] + [(t, 1) for t in clones])
            for t, dn in ev: wp(t, dn)
            ids0 = set(jc[k0]['ids']); first = {}
            for j in jc[k0 + 1:k1 + 1]:
                for fid, p in j['ids'].items():
                    if fid in ids0: continue
                    if fid not in first: first[fid] = [p, j['t'], j['t']]
                    first[fid][2] = j['t']
            for fid, (p, ta, tb) in first.items():
                ft.write(f"NEWLM {fid} {p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {ta - t0:.4f} {tb - t0:.4f}\n")
            nm = 0
            for t, pts in ms:
                if t0 < t <= t1:
                    for p in pts: ft.write(f"MSCKF {p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {t - t0:.4f}\n"); nm += 1
            ft.write("END\n")
            # realized and simple features (5c)
            seg = od[(od[:, 0] >= t0) & (od[:, 0] <= t1)]
            plen = float(np.sum(np.hypot(np.diff(seg[:, 1]), np.diff(seg[:, 2])))) if len(seg) > 1 else 0.0
            syaw = np.array([yaw_of(r[4:8]) for r in seg]); rot = float(np.sum(np.abs(np.diff(np.unwrap(syaw))))) if len(seg) > 1 else 0.0
            fm.write(json.dumps({'id': tid, 'flight': fl, 't0': t0, 't1': t1, 'horizon_s': t1 - t0, 'pick': x.get('pick', 'argmax'),
                                 'lambda': x.get('lambda'), 'pred_total': c['dI_pose'], 'pred_prop': c.get('dI_prop'), 'pred_meas': c.get('dI_meas'),
                                 'pred_meas_real': c.get('dI_meas_real'), 'realized': jc[k0]['ld'] - jc[k1]['ld'],
                                 'path_len': plen, 'rotation_rad': rot, 'coverage_gain': c.get('coverage_gain'),
                                 'lm_vis': c.get('lm_vis'), 'n_meas_real': c.get('n_meas_real'), 'n_lm_t0': len(jc[k0]['ids']),
                                 'n_new_lm': len(first), 'n_msckf_pts': nm, 'n_clones': len(clones)}) + '\n')
    print(json.dumps({'tasks': nt, 'skipped': skipped}))

if __name__ == '__main__':
    main()
