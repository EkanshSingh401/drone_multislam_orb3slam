#!/usr/bin/env python3
"""Diagnostics 3-5 for the OpenVINS overconfidence (PATCHES s54 draft).

    python3 geom_timing.py <rundir> [--pairs N] [--gap K]

Run INSIDE the sim container. Uses only the bag + gt.tum + the generator's
rig constants (copied below, they ARE the SDF).

  4  stamp intervals: /camera/imu and both image topics (header stamps)
  3  projection: stereo-triangulate corners in cam0/cam1 at frame i (depth from
     disparity), move them to world with GT pose at t_i, project into cam0 at
     frame i+K with GT pose at t_{i+K}, compare with the KLT track. Repeated
     for principal point (424,240) [SDF/config] and (423.5,239.5) [pixel-centre
     convention].
  5  same residual with GT sampled at (image stamp + delta), delta scanned:
     a render pose != stamp pose shows up as a nonzero best delta / a
     velocity-correlated residual at delta=0.
"""
import argparse, json, sys
import numpy as np, cv2
from scipy.spatial.transform import Rotation as R, Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

F = 446.802773; B = 0.095
P_CAM0_BASE = np.array([0.12, 0.0475, 0.242])
R_OPT_FROM_BODY = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)

ap = argparse.ArgumentParser()
ap.add_argument("rundir"); ap.add_argument("--pairs", type=int, default=150)
ap.add_argument("--gap", type=int, default=3)
a = ap.parse_args()

gt = np.loadtxt(f"{a.rundir}/gt.tum")
gt = gt[np.r_[True, np.diff(gt[:, 0]) > 0]]
slerp = Slerp(gt[:, 0], R.from_quat(gt[:, 4:8]))
def pose(t):  # base_link in world
    return slerp(t).as_matrix(), np.array([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])

# airborne window: same rule as s51 (3 s after GT z > 0.5 m, until touchdown)
z = gt[:, 3]; up = np.where(z > 0.5)[0]
t0 = gt[up[0], 0] + 3.0
last_hi = up[-1]; td = last_hi + np.argmax(z[last_hi:] < 0.05); t1 = gt[td, 0]

info = rosbag2_py.Info().read_metadata(f"{a.rundir}/flight.bag", "")
rd = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=f"{a.rundir}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
want = ["/camera/imu", "/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw"]
rd.set_filter(rosbag2_py.StorageFilter(topics=want))
st = {k: [] for k in want}; imgs = {0: {}, 1: {}}
while rd.has_next():
    topic, data, trecv = rd.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    st[topic].append(ts)
    if topic != "/camera/imu" and t0 <= ts <= t1:
        c = 0 if "infra1" in topic else 1
        imgs[c][round(ts, 6)] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width).copy()

out = {"window": [t0, t1]}
# ---- 4: intervals
for k in want:
    d = np.diff(np.array(st[k])) * 1e3
    u, n = np.unique(np.round(d, 3), return_counts=True)
    top = sorted(zip(n, u), reverse=True)[:6]
    out[k] = {"n": len(st[k]), "mean_ms": d.mean(), "min": d.min(), "max": d.max(),
              "hist_ms": {f"{v:g}": int(c) for c, v in top},
              "frac_on_4ms_grid": float(np.mean(np.abs(np.array(st[k]) / 0.004 - np.round(np.array(st[k]) / 0.004)) < 1e-3))}

# ---- 3/5: GT reprojection
common = sorted(set(imgs[0]) & set(imgs[1]))
idx = np.linspace(0, len(common) - 1 - a.gap, min(a.pairs, len(common) - a.gap)).astype(int)
lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 0.01))
obs = []  # (ti, tj, u0, v0, disparity, uj, vj)
dys = []
for i in idx:
    ti, tj = common[i], common[i + a.gap]
    L, Rr, Lj = imgs[0][ti], imgs[1][ti], imgs[0].get(tj)
    if Lj is None: continue
    p0 = cv2.goodFeaturesToTrack(L, 300, 0.01, 12)
    if p0 is None: continue
    pr, s1, _ = cv2.calcOpticalFlowPyrLK(L, Rr, p0, None, **lk)
    pb, s1b, _ = cv2.calcOpticalFlowPyrLK(Rr, L, pr, None, **lk)
    pj, s2, _ = cv2.calcOpticalFlowPyrLK(L, Lj, p0, None, **lk)
    pjb, s2b, _ = cv2.calcOpticalFlowPyrLK(Lj, L, pj, None, **lk)
    for q0, qr, qb, qj, qjb, a1, a2, a3, a4 in zip(p0[:, 0], pr[:, 0], pb[:, 0], pj[:, 0], pjb[:, 0], s1, s1b, s2, s2b):
        if not (a1 and a2 and a3 and a4): continue
        if np.linalg.norm(qb - q0) > 0.1 or np.linalg.norm(qjb - q0) > 0.1: continue
        disp = q0[0] - qr[0]; dys.append(qr[1] - q0[1])
        if disp < 3: continue  # > ~14 m: depth too poor
        obs.append((ti, tj, q0[0], q0[1], disp, qj[0], qj[1]))
obs = np.array(obs)
out["stereo_dy_px"] = {"median": float(np.median(dys)), "p95_abs": float(np.percentile(np.abs(dys), 95))}
out["n_obs"] = len(obs); out["depth_median_m"] = float(np.median(F * B / obs[:, 4]))

T_cache = {}
def T_opt(t):
    k = round(t, 7)
    if k not in T_cache:
        Rwb, pwb = pose(t)
        T_cache[k] = (Rwb, pwb + Rwb @ P_CAM0_BASE)
    return T_cache[k]

def resid(delta, cx, cy):
    e = []
    for ti, tj, u, v, d, uj, vj in obs:
        Z = F * B / d
        Xo = np.array([(u - cx) * Z / F, (v - cy) * Z / F, Z])
        Rwb, pc = T_opt(ti + delta); Xw = pc + Rwb @ (R_OPT_FROM_BODY.T @ Xo)
        Rwb2, pc2 = T_opt(tj + delta); Xo2 = R_OPT_FROM_BODY @ (Rwb2.T @ (Xw - pc2))
        e.append([F * Xo2[0] / Xo2[2] + cx - uj, F * Xo2[1] / Xo2[2] + cy - vj])
    return np.array(e)

res = {}
for name, (cx, cy) in {"sdf_424_240": (424.0, 240.0), "centre_423.5_239.5": (423.5, 239.5)}.items():
    rows = []
    for dms in range(-12, 13, 1):
        e = resid(dms * 1e-3, cx, cy); n = np.linalg.norm(e, axis=1)
        rows.append((dms, float(np.sqrt(np.mean(n ** 2))), float(np.median(n)), e.mean(0).round(3).tolist()))
    res[name] = rows
out["reproj"] = res
print(json.dumps(out, indent=1, default=float))
