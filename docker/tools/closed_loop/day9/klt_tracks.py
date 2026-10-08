#!/usr/bin/env python3
"""klt_tracks.py <replay_ov_dir> <flight_dir> <out.jsonl> (day 8 step 2): one record per cam0 KLT track with
its ground-truth error sequence and the candidate explanatory variables of a random-walk error model.

Truth as day7/klt_error.py (first observation's ray cast into scenes.py geometry from the GT camera pose,
later true pixels by projection). Per track (>= 4 frames, airborne GT z > 0.5 m):
  e_k (px, k = 0..n-1; e_0 = 0), increments d_k = e_k - e_(k-1)
  q2      mean squared increment per frame (px^2, both axes averaged) = the random-walk rate if the model holds
  min_eig Shi-Tomasi minimum eigenvalue of the 15x15 structure tensor at the first observation (image read from
          the bag, /camera/infra1/image_rect_raw), grad_energy (mean |grad|^2) and patch std
  depth   GT range to the point (m); cos_view = |cos| of the angle between the viewing ray and the surface normal
  flow    median image motion per frame along the track (px)
  dt      median frame interval (s); surface kind (textured / plain / dim / floor / floor_dim)."""
import json, os, re, sys
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
sys.path.insert(0, '/out/cl')
from scenes import SCENES
PLAIN = {"corr_n0", "corr_n1", "outer_w1", "outer_n0", "outer_w2", "part0_n0", "part0_n1", "nw_desk", "nw_shelf", "nw_cab"}
DIM = {"outer_s0", "outer_w0", "part0_s0", "part0_s1", "corr_s0", "sw_cab", "sw_shelf"}
FX, FY, CX, CY = 446.802773, 446.802773, 424.0, 240.0
R_CI = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]); P_CI = np.array([0.0424, 0.01174, -0.00552])
od, rd, outp = sys.argv[1], sys.argv[2], sys.argv[3]
fname = os.path.basename(os.path.dirname(od.rstrip('/'))) if os.path.basename(od.rstrip('/')) == 'ov' else os.path.basename(od.rstrip('/'))
world = next(m.group(1) for l in open(f'{rd}/bringup.log') for m in [re.search(r'world=(\w+)', l)] if m)
solids = SCENES[world][0]; names = [s[0] for s in solids]
LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
g = np.loadtxt(f'{rd}/gt_imu.txt'); g = g[np.unique(g[:, 0], return_index=True)[1]]
sl = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
def cam(t):
    Ri = sl([t])[0].as_matrix(); p = np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    return Ri @ R_CI.T, p + Ri @ (-R_CI.T @ P_CI), p[2]
def cast(o, d):
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1 = (LO - o) * inv; t2 = (HI - o) * inv; tmin = np.minimum(t1, t2); tn = tmin.max(1); tf = np.maximum(t1, t2).min(1)
        tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf); k = int(np.argmin(tb))
        tz = -o[2] / d[2] if d[2] < -1e-9 else np.inf
    if tz < tb[k]:
        X = o + tz * d; kind = 'floor_dim' if (world == 'office_plain' and X[0] < 6 and X[1] < 0) else 'floor'
        return X, kind, abs(d[2])
    if not np.isfinite(tb[k]): return None, None, None
    ax = int(np.argmax(tmin[k])); nrm = np.zeros(3); nrm[ax] = 1.0
    n = names[k]; kind = 'plain' if (world == 'office_plain' and n in PLAIN) else ('dim' if (world == 'office_plain' and n in DIM) else 'textured')
    return o + tb[k] * d, kind, abs(d @ nrm)
tr = defaultdict(list)
for l in open(f'{od}/openvins.log', errors='ignore'):
    if l.startswith('[TRK]'):
        p = l.split(); t = float(p[1])
        if g[0, 0] < t < g[-1, 0]: tr[int(p[2])].append((t, float(p[3]), float(p[4])))
first = defaultdict(list)
for fid, obs in tr.items():
    obs.sort()
    if len(obs) >= 4: first[round(obs[0][0], 4)].append(fid)
# image texture at the first observation
tex = {}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{rd}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/infra1/image_rect_raw']))
while r.has_next():
    _, b, _ = r.read_next(); m = deserialize_message(b, Image); t = round(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, 4)
    if t not in first: continue
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).astype(float)
    gx = ndimage.sobel(img, 1) / 8.0; gy = ndimage.sobel(img, 0) / 8.0
    for fid in first[t]:
        u, v = tr[fid][0][1], tr[fid][0][2]; iu, iv = int(round(u)), int(round(v))
        if not (7 <= iu < m.width - 7 and 7 <= iv < m.height - 7): continue
        sx = gx[iv - 7:iv + 8, iu - 7:iu + 8]; sy = gy[iv - 7:iv + 8, iu - 7:iu + 8]
        a, bb, c = (sx * sx).mean(), (sx * sy).mean(), (sy * sy).mean()
        tex[fid] = (float((a + c) / 2 - np.sqrt(((a - c) / 2) ** 2 + bb * bb)), float(a + c), float(img[iv - 7:iv + 8, iu - 7:iu + 8].std()))
fo = open(outp, 'a'); n = 0
for fid, obs in tr.items():
    if len(obs) < 4 or fid not in tex: continue
    t0, u0, v0 = obs[0]; R0, o0, z0 = cam(t0)
    if z0 < 0.5: continue
    dG = R0 @ np.array([(u0 - CX) / FX, (v0 - CY) / FY, 1.0]); dG /= np.linalg.norm(dG)
    X, kind, cosv = cast(o0, dG)
    if X is None or np.linalg.norm(X - o0) > 12: continue
    E = []
    for t, u, v in obs:
        Rk, ok_, zk = cam(t); pc = Rk.T @ (X - ok_)
        if pc[2] < 0.2 or zk < 0.5: break
        E.append((u - (FX * pc[0] / pc[2] + CX), v - (FY * pc[1] / pc[2] + CY)))
    if len(E) < 4: continue
    E = np.array(E); D = np.diff(E, axis=0)
    uv = np.array([(u, v) for _, u, v in obs[:len(E)]]); flow = float(np.median(np.linalg.norm(np.diff(uv, axis=0), axis=1)))
    rec = {'flight': fname, 'world': world, 'kind': kind, 'n': len(E), 'dt': round(float(np.median(np.diff([o[0] for o in obs[:len(E)]]))), 4),
           'q2': float(np.mean(D ** 2)), 'e1': float(np.linalg.norm(E[1])), 'e_abs': [round(float(x), 3) for x in np.linalg.norm(E, axis=1)[:31]],
           'min_eig': tex[fid][0], 'grad_energy': tex[fid][1], 'patch_std': tex[fid][2], 'depth': float(np.linalg.norm(X - o0)), 'cos_view': float(cosv), 'flow': flow}
    fo.write(json.dumps(rec) + '\n'); n += 1
print(fname, world, 'tracks', n)
