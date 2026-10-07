#!/usr/bin/env python3
"""vis_gt.py <flight_dir> <n_decisions> [out.json] (day 3 step 1): the visibility predictor against
ground-truth geometry, validation scene only.

For n decisions (evenly spaced among those whose joint covariance has >= 1 landmark):
  * re-implements the planner's visibility test (FOV with 10 px border, depth 0.3..20 m, no
    occlusion) from the joint covariance and the logged candidate (xy, yaw, z = 1.5) and checks it
    reproduces the planner's logged counts (vis[0] + vis[3] visible as scored, vis[1] fov, vis[2]);
  * maps landmarks and candidate poses into the world with a 4-DoF (yaw + translation) fit of the
    OpenVINS IMU track (/ov_msckf/odomimu) onto ground truth (gt_imu.txt) over +-15 s;
  * GT visibility of each (landmark, camera, candidate) pair: same FOV/depth test with the GT-mapped
    poses, and the segment camera -> landmark (last 5 cm excluded) must not cross a scene solid
    (clearance.py SOLIDS) or the floor;
  * landmark-on-surface check: distance from each mapped landmark to the nearest solid / floor.
Reports pair confusion (predicted-as-scored vs GT), and per candidate the planner's occupied-cell
occlusion count (vis[0]) against GT-visible pairs."""
import json, sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/out/cl')
WALL_H, WALL_T = 4.0, 0.2
X_MIN, X_MAX, Y_MIN, Y_MAX = -4.0, 5.5, -5.5, 5.5
SOLIDS = [("wall_w", X_MIN, 0.0, WALL_H/2, WALL_T, Y_MAX-Y_MIN, WALL_H), ("wall_e", X_MAX, 0.0, WALL_H/2, WALL_T, Y_MAX-Y_MIN, WALL_H),
  ("wall_s", (X_MIN+X_MAX)/2, Y_MIN, WALL_H/2, X_MAX-X_MIN, WALL_T, WALL_H), ("wall_n", (X_MIN+X_MAX)/2, Y_MAX, WALL_H/2, X_MAX-X_MIN, WALL_T, WALL_H),
  ("box_ne", 4.6, 4.2, 0.9, 1.4, 2.0, 1.8), ("box_e", 4.8, 0.5, 1.25, 1.0, 1.2, 2.5), ("box_se", 4.5, -4.4, 0.6, 1.6, 1.6, 1.2),
  ("box_nw", -3.2, 4.4, 1.1, 1.2, 1.6, 2.2), ("box_w", -3.4, -0.8, 0.75, 0.8, 1.6, 1.5), ("box_sw", -3.0, -4.6, 1.4, 1.6, 1.2, 2.8),
  ("box_n", 1.0, 4.8, 0.8, 1.8, 1.0, 1.6), ("box_s", 0.5, -4.8, 1.0, 1.4, 1.0, 2.0)]
BOX = [(np.array([cx-sx/2, cy-sy/2, cz-sz/2]), np.array([cx+sx/2, cy+sy/2, cz+sz/2])) for _, cx, cy, cz, sx, sy, sz in SOLIDS]
W, H, BORDER, ZMIN, ZMAX, FLY_Z = 848, 480, 10.0, 0.3, 20.0, 1.5

def jpl_rot(q):  # OpenVINS quat_2_Rot, JPL [x y z w]
    v, w = np.array(q[:3]), q[3]
    S = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (2 * w * w - 1) * np.eye(3) - 2 * w * S + 2 * np.outer(v, v)

def seg_hits_box(a, b, lo, hi):
    d = b - a; t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if a[k] < lo[k] or a[k] > hi[k]: return False
        else:
            u, v = (lo[k] - a[k]) / d[k], (hi[k] - a[k]) / d[k]
            t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
            if t0 > t1: return False
    return True

def gt_occluded(a, b):
    L = np.linalg.norm(b - a)
    if L < 0.06: return False
    b2 = a + (b - a) * (L - 0.05) / L
    if min(a[2], b2[2]) < 0.0: return True
    return any(seg_hits_box(a, b2, lo, hi) for lo, hi in BOX)

def surface_dist(p):
    d = [np.linalg.norm(np.maximum(0, np.maximum(lo - p, p - hi))) if not np.all((p >= lo) & (p <= hi)) else 0.0 for lo, hi in BOX]
    return min(min(d), abs(p[2]))

def classify(R_GtoI, p_I, R_ItoC, p_IinC, K, pf):
    pc = R_ItoC @ (R_GtoI @ (pf - p_I)) + p_IinC
    Z = pc[2]
    if not Z > 0: return 1
    u, v = K[0] * pc[0] / Z + K[2], K[1] * pc[1] / Z + K[3]
    if u < BORDER or u > W - BORDER or v < BORDER or v > H - BORDER: return 1
    if not Z > ZMIN or Z > ZMAX: return 2
    return 0

def main():
    d, n = sys.argv[1], int(sys.argv[2])
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    tops = ['/openvins/joint_covariance', '/exploration_planner/decision', '/ov_msckf/odomimu']
    r.set_filter(rosbag2_py.StorageFilter(topics=tops))
    ty = {t: get_message(names[t]) for t in tops}
    jcs, decs, od = [], [], []
    while r.has_next():
        tp, raw, _ = r.read_next()
        m = deserialize_message(raw, ty[tp])
        if tp == tops[0]:
            if m.stage in ('', 'post'): jcs.append(m)
        elif tp == tops[1]:
            decs.append(json.loads(m.data))
        else:
            p = m.pose.pose.position; od.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z))
    od = np.array(od)
    jt = np.array([j.header.stamp.sec + 1e-9 * j.header.stamp.nanosec for j in jcs])
    gt = np.loadtxt(f'{d}/gt_imu.txt')
    cand_decs = []
    for x in decs:
        if not x.get('candidates') or 'vis' not in x['candidates'][0]: continue
        k = np.searchsorted(jt, x['t'] + 0.5, side='right') - 1
        if k < 0: continue
        # the planner used its latest received message; stamps lag receipt -> pick the candidate
        # message whose landmark count equals the logged n_lm, nearest first
        ks = [j for j in range(k, max(-1, k - 8), -1) if len(jcs[j].landmarks) == x['n_lm']]
        if ks and x['n_lm'] > 0: cand_decs.append((x, jcs[ks[0]]))
    pick = [cand_decs[i] for i in np.linspace(0, len(cand_decs) - 1, min(n, len(cand_decs))).astype(int)] if cand_decs else []
    conf = np.zeros((2, 2), int)  # [pred visible as scored][gt visible]
    rows, reproduce_ok, reproduce_n, sdist = [], 0, 0, []
    for x, jc in pick:
        t = x['t']
        # 4-DoF alignment OV global -> world over +-15 s
        m = (od[:, 0] > t - 15) & (od[:, 0] < t + 15)
        A = od[m, 1:4]
        G = np.column_stack([np.interp(od[m, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
        ma, mg = A.mean(0), G.mean(0); a, g = A - ma, G - mg
        yaw = np.arctan2((a[:, 0] * g[:, 1] - a[:, 1] * g[:, 0]).sum(), (a[:, 0] * g[:, 0] + a[:, 1] * g[:, 1]).sum())
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        tw = mg - Rz @ ma
        resid = float(np.sqrt(np.mean(np.sum((A @ Rz.T + tw - G) ** 2, 1))))
        cams = [(jpl_rot(c.q_itoc), np.array(c.p_iinc), list(c.intrinsics)[:4]) for c in jc.cameras]
        lms = [np.array(l.p_fing) for l in jc.landmarks]
        lw = [Rz @ p + tw for p in lms]
        sdist += [surface_dist(p) for p in lw]
        for c in x['candidates']:
            Rg = np.array([[np.cos(c['yaw']), np.sin(c['yaw']), 0], [-np.sin(c['yaw']), np.cos(c['yaw']), 0], [0, 0, 1]])  # R_GtoI, level
            pI = np.array([c['xy'][0], c['xy'][1], FLY_Z])
            RgW = Rg @ Rz.T; pIW = Rz @ pI + tw  # same pose in the world
            cnt = [0, 0, 0]; gtv = 0
            for p, pw in zip(lms, lw):
                for R_ItoC, p_IinC, K in cams:
                    k = classify(Rg, pI, R_ItoC, p_IinC, K, p); cnt[k] += 1
                    kg = classify(RgW, pIW, R_ItoC, p_IinC, K, pw)
                    cw = pIW + RgW.T @ (-R_ItoC.T @ p_IinC)
                    vis_gt = kg == 0 and not gt_occluded(cw, pw)
                    gtv += vis_gt
                    conf[int(k == 0), int(vis_gt)] += 1
            reproduce_n += 1
            reproduce_ok += (cnt[0] == c['vis'][0] + c['vis'][3] and cnt[1] == c['vis'][1] and cnt[2] == c['vis'][2])
            rows.append({'t': round(t, 1), 'n_lm': len(lms), 'pred_scored': cnt[0], 'pred_occ_test': c['vis'][0], 'gt_visible': gtv, 'align_rmse': round(resid, 3)})
    pv = conf[1].sum()
    res = {'flight': os.path.basename(d.rstrip('/')), 'decisions': len(pick), 'candidates': reproduce_n,
           'reimplementation_reproduces_planner_counts': f'{reproduce_ok}/{reproduce_n}',
           'pairs': int(conf.sum()), 'confusion[pred_scored][gt]': conf.tolist(),
           'precision_scored': float(conf[1, 1] / pv) if pv else None,
           'recall_scored': float(conf[1, 1] / conf[:, 1].sum()) if conf[:, 1].sum() else None,
           'cand_pred_occ_test_vs_gt': {'sum_pred_occ_test': int(sum(r['pred_occ_test'] for r in rows)), 'sum_gt': int(sum(r['gt_visible'] for r in rows)),
                                        'sum_pred_scored': int(sum(r['pred_scored'] for r in rows))},
           'landmark_surface_dist_m': {'median': float(np.median(sdist)), 'p90': float(np.percentile(sdist, 90)), 'n': len(sdist)} if sdist else None,
           'align_rmse_m_max': max(r['align_rmse'] for r in rows) if rows else None}
    print(json.dumps(res))
    if len(sys.argv) > 3: json.dump({'summary': res, 'candidates': rows}, open(sys.argv[3], 'w'), indent=1)

main()
