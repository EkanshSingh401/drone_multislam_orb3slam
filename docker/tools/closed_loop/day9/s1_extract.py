#!/usr/bin/env python3
"""s1_extract.py (day 9 step 1: step4_extract.py + the second camera; replay root via REP_ROOT)
step4_extract.py <rw_model.json> <windows.jsonl> <out_dir> <replay_name>... (day 8 step 4): drift_model tasks.

Per replay (day 7 rep/<name>: ov/openvins.log with [TRK], ov/ov_state_est.txt, flight via ov/ links) and per
window of <windows.jsonl> belonging to it (t0, 20 s): prior = the live flight's /openvins/joint_covariance
at or before t0 (raw CDR); waypoints = the replay's estimated IMU poses at the tracked frames, subsampled to
5 Hz (every 3rd frame at 15 Hz, every frame at 5 Hz), times relative to the first; tracks = [TRK] samples of
each feature at those frames, with the frame count since its previous sample at the tracking rate; the 3D
point = the first observation's ray cast into scenes.py geometry from the GT camera pose (as klt_tracks.py),
moved into the estimate's frame by the rigid transform between the GT and estimated IMU poses at t0;
q2 = the fitted random-walk model (fit_rw.py) from texture at the track's first observation (image from
the bag), depth, viewing angle, flow and frame interval."""
import json, os, re, sys
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/out/cl')
from scenes import SCENES
FX, FY, CX, CY = 446.802773, 446.802773, 424.0, 240.0
R_CI = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]); P_CI = np.array([0.0424, 0.01174, -0.00552])
M = json.load(open(sys.argv[1]))['model_raw']; b, mu, sd = map(np.array, (M['b'], M['mu'], M['sd']))
W_all = [json.loads(l) for l in open(sys.argv[2])]
out = sys.argv[3]; os.makedirs(f'{out}/jc', exist_ok=True)
ft = open(f'{out}/tasks.txt', 'a'); fm = open(f'{out}/meta.jsonl', 'a')
for name in sys.argv[4:]:
    od = f"{os.environ.get('REP_ROOT', '/out/cl/day9/rep')}/{name}/ov"; rd = os.path.dirname(os.path.realpath(f'{od}/flight.bag'))
    rate = 15.0 if name.endswith('_tf20') else 5.0; sub = 3 if rate == 15.0 else 1
    wins = [w for w in W_all if w['flight'] == name and w.get('path_m', 0) >= 1.0]
    if not wins: continue
    world = next(m.group(1) for l in open(f'{rd}/bringup.log') for m in [re.search(r'world=(\w+)', l)] if m)
    solids = SCENES[world][0]
    LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    g = np.loadtxt(f'{rd}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]; sl = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
    e = np.loadtxt(f'{od}/ov_state_est.txt')
    def gt_pose(t): return sl([t])[0], np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    def cast(o, d):
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
            t1 = (LO - o) * inv; t2 = (HI - o) * inv; tmin = np.minimum(t1, t2); tn = tmin.max(1); tf = np.maximum(t1, t2).min(1)
            tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf); k = int(np.argmin(tb))
            tz = -o[2] / d[2] if d[2] < -1e-9 else np.inf
        if tz < tb[k]: return o + tz * d, abs(d[2])
        if not np.isfinite(tb[k]): return None, None
        ax = int(np.argmax(tmin[k])); return o + tb[k] * d, abs(d[ax])
    tr = defaultdict(list); tr1 = defaultdict(list); frames = set()
    for l in open(f'{od}/openvins.log', errors='ignore'):
        if l.startswith('[TRK]'):
            p = l.split(); t = round(float(p[1]), 4); tr[int(p[2])].append((t, float(p[3]), float(p[4]))); frames.add(t)
        elif l.startswith('[TRK1]'):
            p = l.split(); tr1[int(p[2])].append((round(float(p[1]), 4), float(p[3]), float(p[4])))
    frames = np.array(sorted(frames))
    first = defaultdict(list)
    for fid, obs in tr.items(): obs.sort(); first[obs[0][0]].append(fid)
    tex = {}
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{rd}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/infra1/image_rect_raw', '/openvins/joint_covariance']))
    Ti = get_message(names['/camera/infra1/image_rect_raw']); Tj = get_message(names['/openvins/joint_covariance'])
    jcs = []
    while r.has_next():
        tp, raw, _ = r.read_next()
        if tp == '/openvins/joint_covariance':
            m = deserialize_message(raw, Tj)
            if m.stage in ('', 'post'): jcs.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, bytes(raw)))
            continue
        m = deserialize_message(raw, Ti); t = round(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, 4)
        if t not in first: continue
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).astype(float)
        gx = ndimage.sobel(img, 1) / 8.0; gy = ndimage.sobel(img, 0) / 8.0
        for fid in first[t]:
            u, v = tr[fid][0][1], tr[fid][0][2]; iu, iv = int(round(u)), int(round(v))
            if not (7 <= iu < m.width - 7 and 7 <= iv < m.height - 7): continue
            sx = gx[iv - 7:iv + 8, iu - 7:iu + 8]; sy = gy[iv - 7:iv + 8, iu - 7:iu + 8]
            a, bb, c = (sx * sx).mean(), (sx * sy).mean(), (sy * sy).mean()
            tex[fid] = (a + c) / 2 - np.sqrt(((a - c) / 2) ** 2 + bb * bb)
    jt = np.array([x[0] for x in jcs])
    for w in wins:
        t0 = w['t0']; t1 = t0 + 20.0
        k = np.searchsorted(jt, t0, side='right') - 1
        if k < 0: continue
        fr = frames[(frames >= t0 - 1e-4) & (frames <= t1 + 1e-4)][::sub]
        if len(fr) < 10: continue
        ke = [int(np.argmin(np.abs(e[:, 0] - t))) for t in fr]
        if max(abs(e[i, 0] - t) for i, t in zip(ke, fr)) > 0.02: continue
        tid = f'{name}_{t0:.2f}'; open(f'{out}/jc/{tid}.cdr', 'wb').write(jcs[k][1])
        # GT -> estimate frame at t0 (IMU poses)
        Rg0, pg0 = gt_pose(fr[0]); Re0 = R.from_quat(e[ke[0], 1:5]); pe0 = e[ke[0], 5:8]
        Rge = Re0 * Rg0.inv()
        ft.write(f'TASK {tid} {out}/jc/{tid}.cdr\n')
        for t, i in zip(fr, ke):
            q = e[i, 1:5]; ft.write(f'WP {t - fr[0]:.4f} {e[i,5]:.5f} {e[i,6]:.5f} {e[i,7]:.5f} {q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n')
        idx = {round(t, 4): j for j, t in enumerate(fr)}; ntr = 0; n_st = [0]
        for fid, obs in tr.items():
            s = [(idx[o[0]], o) for o in obs if o[0] in idx]
            if len(s) < 2 or fid not in tex: continue
            tf0, u0, v0 = obs[0]
            if tf0 < g[0, 0] or tf0 > g[-1, 0]: continue
            Rg, pg = gt_pose(tf0); RGC = Rg.as_matrix() @ R_CI.T; o = pg + Rg.as_matrix() @ (-R_CI.T @ P_CI)
            d = RGC @ np.array([(u0 - CX) / FX, (v0 - CY) / FY, 1.0]); d /= np.linalg.norm(d)
            X, cosv = cast(o, d)
            if X is None or np.linalg.norm(X - o) > 12: continue
            uv = np.array([(o_[1], o_[2]) for _, o_ in s]); flow = float(np.median(np.linalg.norm(np.diff(uv, axis=0), axis=1)) / sub)
            x = np.array([np.log(max(tex[fid], 1e-3)), np.log(np.linalg.norm(X - o)), cosv, np.log(flow + 0.1), np.log(1.0 / rate)])
            q2 = float(np.exp(b[0] + ((x - mu) / sd) @ b[1:]))
            Xe = Rge.apply(X - pg0) + pe0
            parts = []; prev = None
            for j, o_ in s:
                parts.append(f'{j} {1.0 if prev is None else (j - prev) * sub:.1f}'); prev = j
            ft.write(f'TRKC 0 {Xe[0]:.5f} {Xe[1]:.5f} {Xe[2]:.5f} {q2:.6g} {len(s)} {" ".join(parts)}\n'); ntr += 1
            s1 = [(idx[o[0]], o) for o in sorted(tr1.get(fid, [])) if o[0] in idx]
            if len(s1) >= 2:
                parts = []; prev = None
                for j, o_ in s1:
                    parts.append(f'{j} {1.0 if prev is None else (j - prev) * sub:.1f}'); prev = j
                ft.write(f'TRKC 1 {Xe[0]:.5f} {Xe[1]:.5f} {Xe[2]:.5f} {q2:.6g} {len(s1)} {" ".join(parts)}\n'); n_st[0] += 1
        ft.write('END\n')
        fm.write(json.dumps({'id': tid, 'flight': name, 't0': t0, 'tracks': ntr, 'stereo_tracks': n_st[0], 'waypoints': len(fr)}) + '\n')
    print(name, 'windows', len(wins))
