#!/usr/bin/env python3
"""s3_extract.py <rw_model.json> <windows.jsonl> <out_dir> <flight>... (day 9 step 3): DEPLOYABLE drift_model tasks.

Only what the drone has at the window start t0 (live flight data, system rate 15 Hz):
  path        the live OpenVINS estimate over the window at 5 Hz (the candidate path; in planning it would be
              the planned path)
  map         the mapper's occupied voxels at t0, rebuilt like the mapper: depth emulated against the scene
              geometry from the GT camera pose (validated vs real depth, day 4/5), inserted at the OpenVINS
              camera pose, 0.1 m voxels, 2 Hz, flight start .. t0
  landmarks   (i) the estimator's SLAM landmarks in the joint covariance at t0 (estimates, OpenVINS frame);
              (ii) predicted features on mapped surfaces: a 16 x 9 pixel grid cast from the window's waypoints
              every 2 s into the voxel map (first occupied voxel within 8 m), deduplicated at 0.1 m, at most
              1000 per window (seeded subsample)
  tracks      each landmark's samples = waypoints where it projects into the image (2..846 x 2..478 px),
              is 0.3..8 m deep and not occluded by an occupied voxel nearer than it (ray march, 0.05 m), per
              camera (stereo); consecutive visible waypoints form a track, gaps restart it
  rate q2     fitted random-walk model with texture and viewing angle unknown (standardized value 0 =
              training mean), depth and image motion per frame from the predicted samples, frame interval
              1/15 s
Writes drift_model tasks (TRKC lines) and meta.jsonl."""
import json, os, re, sys
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/out/cl')
from scenes import SCENES
FX, FY, CX, CY, Wd, Hd = 446.802773, 446.802773, 424.0, 240.0, 848, 480
CAM = [(np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]), np.array([0.0424, 0.01174, -0.00552])),
       (np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]), np.array([0.0424 - 0.095, 0.01174, -0.00552]))]  # cam1: 0.095 m baseline along camera x
RES = 0.1
M = json.load(open(sys.argv[1]))['model_raw']; b, mu, sd = map(np.array, (M['b'], M['mu'], M['sd']))
W_all = [json.loads(l) for l in open(sys.argv[2])]
out = sys.argv[3]; os.makedirs(f'{out}/jc', exist_ok=True)
ft = open(f'{out}/tasks.txt', 'a'); fm = open(f'{out}/meta.jsonl', 'a')
gu, gv = np.meshgrid(np.linspace(30, 818, 16), np.linspace(30, 450, 9)); DCg = np.stack([(gu.ravel() - CX) / FX, (gv.ravel() - CY) / FY, np.ones(gu.size)], 1)
DCg /= np.linalg.norm(DCg, axis=1)[:, None]
eu, ev = np.meshgrid(np.arange(6, 848, 12), np.arange(6, 480, 12)); DCe = np.stack([(eu.ravel() - CX) / FX, (ev.ravel() - CY) / FY, np.ones(eu.size)], 1)
DCe /= np.linalg.norm(DCe, axis=1)[:, None]

def cam_pose(R_ItoG, p_I, c):
    Rci, tci = CAM[c]; return R_ItoG @ Rci.T, p_I + R_ItoG @ (-Rci.T @ tci)

for name in sys.argv[4:]:
    rd = f'/out/cl/runs/{name}'
    wins = [w for w in W_all if w['flight'] == name and w.get('path_m', 0) >= 1.0]
    if not wins: continue
    world = next(m.group(1) for l in open(f'{rd}/bringup.log') for m in [re.search(r'world=(\w+)', l)] if m)
    solids = SCENES[world][0]
    LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    g = np.loadtxt(f'{rd}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]; slg = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
    e = np.loadtxt(f'{rd}/ov_state_est.txt'); sle = Slerp(e[:, 0], R.from_quat(e[:, 1:5]))
    def est(t): return sle([t])[0].as_matrix(), np.array([np.interp(t, e[:, 0], e[:, k]) for k in (5, 6, 7)])
    def gtp(t): return slg([t])[0].as_matrix(), np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    def first_hit(o, D):
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / np.where(np.abs(D) < 1e-12, 1e-12, D)
            t1 = (LO[None] - o) * inv[:, None]; t2 = (HI[None] - o) * inv[:, None]
            tn = np.minimum(t1, t2).max(2); tf = np.maximum(t1, t2).min(2)
            tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf).min(1)
            tz = np.where(D[:, 2] < -1e-9, -o[2] / D[:, 2], np.inf)
        return np.minimum(tb, tz)
    # live joint covariances (for the prior and the estimator's landmarks)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{rd}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/openvins/joint_covariance'])); Tj = get_message(names['/openvins/joint_covariance']); jcs = []
    while r.has_next():
        _, raw, _ = r.read_next(); m = deserialize_message(raw, Tj)
        if m.stage in ('', 'post'): jcs.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, bytes(raw), [tuple(L.p_fing) for L in m.landmarks]))
    jt = np.array([x[0] for x in jcs])
    # mapper map at 2 Hz insertion times (emulated depth at GT pose, inserted at the OpenVINS pose)
    tins = np.arange(max(g[0, 0], e[0, 0]) + 0.5, min(g[-1, 0], e[-1, 0]), 0.5); vox_t = []
    for t in tins:
        Rg, pg = gtp(t)
        if pg[2] < 0.3: continue
        RGc, oc = cam_pose(Rg, pg, 0); rng_ = first_hit(oc, DCe @ RGc.T)
        Re, pe = est(t); REc, oe = cam_pose(Re, pe, 0); k = rng_ <= 8.0
        P = oe + (DCe[k] @ REc.T) * rng_[k, None]
        vox_t.append((t, np.unique(np.floor(P / RES).astype(np.int64), axis=0)))
    for w in wins:
        t0 = w['t0']; t1 = t0 + 20.0
        k = np.searchsorted(jt, t0, side='right') - 1
        if k < 0: continue
        T_MAP = t0 + (20.0 if os.environ.get('MAP_AT_END') == '1' else 0.0)   # reference variant: map at the window end (NOT deployable)
        Vs = [V for t, V in vox_t if t <= T_MAP]
        if not Vs: continue
        Vall = np.unique(np.concatenate(Vs), axis=0)
        if len(Vall) < 100: continue
        vlo = Vall.min(0) - 2; vhi = Vall.max(0) + 3
        grid = np.zeros(vhi - vlo, bool); grid[tuple((Vall - vlo).T)] = True
        def in_occ(P):
            I = np.floor(P / RES).astype(np.int64) - vlo
            ok = np.all((I >= 0) & (I < grid.shape), axis=1); out_ = np.zeros(len(P), bool)
            out_[ok] = grid[tuple(I[ok].T)]
            return out_
        fr = np.arange(t0, t1 + 1e-6, 0.2)
        if fr[-1] > e[-1, 0]: continue
        poses = [est(t) for t in fr]
        def occluded(o, X):
            d = X - o; L = np.linalg.norm(d)
            if L < 0.3: return True
            s = np.arange(0.15, L - 0.15, 0.05)
            return bool(in_occ(o + np.outer(s / L, d)).any())
        # landmarks: estimator's SLAM points + predicted features on mapped surfaces
        pts = [np.array(p) for p in jcs[k][2]]; n_slam = len(pts)
        for j in range(0, len(fr), 10):
            Re, pe = poses[j]; RC, oc = cam_pose(Re, pe, 0); D = DCg @ RC.T
            s = np.arange(0.3, 8.0, 0.05)
            P = oc[None, None] + D[:, None, :] * s[None, :, None]           # rays x samples x 3
            H = in_occ(P.reshape(-1, 3)).reshape(len(D), len(s))
            for i in np.nonzero(H.any(1))[0]: pts.append(P[i, np.argmax(H[i])])
        # dedupe predicted points at 0.1 m and cap at 1000 (seeded), keep all estimator landmarks
        pred_pts = np.array(pts[n_slam:]) if len(pts) > n_slam else np.zeros((0, 3))
        if len(pred_pts):
            _, ui = np.unique(np.floor(pred_pts / 0.1).astype(np.int64), axis=0, return_index=True); pred_pts = pred_pts[np.sort(ui)]
            if len(pred_pts) > 1000: pred_pts = pred_pts[np.random.default_rng(int(t0 * 100)).choice(len(pred_pts), 1000, replace=False)]
        pts = pts[:n_slam] + list(pred_pts)
        tid = f'{name}_{t0:.2f}'; open(f'{out}/jc/{tid}.cdr', 'wb').write(jcs[k][1])
        ft.write(f'TASK {tid} {out}/jc/{tid}.cdr\n')
        for t, (Re, pe) in zip(fr, poses):
            q = R.from_matrix(Re).as_quat(); ft.write(f'WP {t - fr[0]:.4f} {pe[0]:.5f} {pe[1]:.5f} {pe[2]:.5f} {q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}\n')
        ntr = 0
        for X in pts:
            lines = []
            for c in (0, 1):
                samp = []
                for j, (Re, pe) in enumerate(poses):
                    RC, oc = cam_pose(Re, pe, c); pc = RC.T @ (X - oc)
                    if not (0.3 < pc[2] < 8.0): continue
                    u, v = FX * pc[0] / pc[2] + CX, FY * pc[1] / pc[2] + CY
                    if not (2 <= u <= Wd - 2 and 2 <= v <= Hd - 2): continue
                    if j % 5 == 0 and occluded(oc, X): continue
                    samp.append((j, u, v, pc[2]))
                if len(samp) < 2: continue
                uv = np.array([(a[1], a[2]) for a in samp]); js = np.array([a[0] for a in samp])
                flow = float(np.median(np.linalg.norm(np.diff(uv, axis=0), axis=1) / np.maximum(np.diff(js) * 3, 1)))
                x = np.array([mu[0], np.log(np.median([a[3] for a in samp])), mu[2], np.log(flow + 0.1), np.log(1 / 15.0)])
                q2 = float(np.exp(b[0] + ((x - mu) / sd) @ b[1:]))
                parts = []; prev = None
                for a in samp:
                    parts.append(f'{a[0]} {1.0 if prev is None else (a[0] - prev) * 3:.1f}'); prev = a[0]
                lines.append(f'TRKC {c} {X[0]:.5f} {X[1]:.5f} {X[2]:.5f} {q2:.6g} {len(samp)} {" ".join(parts)}')
            if lines: ft.write('\n'.join(lines) + '\n'); ntr += 1
        ft.write('END\n')
        fm.write(json.dumps({'id': tid, 'flight': name, 't0': t0, 'landmarks': ntr, 'slam_landmarks_t0': n_slam, 'map_voxels': int(len(Vall))}) + '\n')
    print(name, 'windows', len(wins))
