#!/usr/bin/env python3
"""drift_windows.py <out.jsonl> <flight_dir>... (day 6 step 4): accumulated drift per metre over windows.

Per flight (closed-loop run dirs: ov_state_est.txt, ov_state_std.txt, gt_imu.txt, flight.bag, executor.log),
explore phase only, windows of W s (default 20, step 10; env DW_W, DW_STEP):
  drift   relative pose error over the window (frame-invariant, no alignment):
          E = (T_gt(t0)^-1 T_gt(t1))^-1 (T_ov(t0)^-1 T_ov(t1));  pos = |t(E)| (m), yaw = |yaw(E)| (deg)
  per m   divided by the GT path length in the window (windows with < 1 m path are kept, flagged)
  cov     growth of OpenVINS's own sigma over the window: d sqrt(tr P_pos), d sigma_yaw (from the std file;
          yaw sigma = sigma_theta about gravity, diagonal approximation)
  predictors  path length, mean speed, rotation (sum |d yaw|) and mean rate, SLAM landmarks in state
          (/openvins/joint_covariance num_slam_features, mean), and from emulated depth at the GT camera
          pose every 0.5 s against scenes.py geometry (validation / office / office_plain only): median
          range of returning rays, fraction of rays returning within 8 m, fraction of returning rays that
          hit a plain or dim surface (office_plain; gen_office_world PLAIN / DIM).
Prints a short summary; one JSON line per window to <out.jsonl>."""
import json, os, re, sys
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from scenes import SCENES
W = float(os.environ.get('DW_W', 20)); STEP = float(os.environ.get('DW_STEP', 10))
PLAIN = {"corr_n0", "corr_n1", "outer_w1", "outer_n0", "outer_w2", "part0_n0", "part0_n1", "nw_desk", "nw_shelf", "nw_cab"}
DIM = {"outer_s0", "outer_w0", "part0_s0", "part0_s1", "corr_s0", "sw_cab", "sw_shelf"}
FX, CX, CY = 446.802773, 424.0, 240.0
u, v = np.meshgrid(np.arange(8, 848, 16), np.arange(8, 480, 16))
DC = np.stack([(u.ravel() - CX) / FX, (v.ravel() - CY) / FX, np.ones(u.size)], 1); DC /= np.linalg.norm(DC, axis=1)[:, None]
R_CI = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0.0]]); P_CI = np.array([0.0424, 0.01174, -0.00552])

def world_of(d):
    for l in open(f'{d}/bringup.log'):
        m = re.search(r'world=(\w+)', l)
        if m: return m.group(1)
    return None

def raycast(world, R_ItoG, p_I):
    solids, _ = SCENES[world]
    names = [s[0] for s in solids]
    LO = np.array([[cx - sx / 2, cy - sy / 2, cz - sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    HI = np.array([[cx + sx / 2, cy + sy / 2, cz + sz / 2] for _, cx, cy, cz, sx, sy, sz in solids])
    R_GC = R_ItoG @ R_CI.T; o = p_I + R_ItoG @ (-R_CI.T @ P_CI); D = DC @ R_GC.T
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / np.where(np.abs(D) < 1e-12, 1e-12, D)
        t1 = (LO[None] - o) * inv[:, None]; t2 = (HI[None] - o) * inv[:, None]
        tn = np.minimum(t1, t2).max(2); tf = np.maximum(t1, t2).min(2)
        tb = np.where(tf >= np.maximum(tn, 0), np.maximum(tn, 0), np.inf)
        k = tb.argmin(1); tbm = tb[np.arange(len(D)), k]
        tz = np.where(D[:, 2] < -1e-9, -o[2] / D[:, 2], np.inf)
    rng = np.minimum(tbm, tz); floor = tz < tbm
    hit = rng <= 8.0
    plain = np.array([(not f) and (names[i] in PLAIN or names[i] in DIM) for i, f in zip(k, floor)])
    if world == 'office_plain':      # the dim floor tile (x < 6, y < 0)
        P = o + D * rng[:, None]; plain |= floor & (P[:, 0] < 6) & (P[:, 1] < 0)
    return (float(np.median(rng[hit])) if hit.any() else np.nan, float(hit.mean()),
            float(plain[hit].mean()) if (world == 'office_plain' and hit.any()) else 0.0)

def jc_counts(d):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    if '/openvins/joint_covariance' not in names: return np.zeros((0, 2))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/openvins/joint_covariance'])); T = get_message(names['/openvins/joint_covariance']); out = []
    while r.has_next():
        _, b, _ = r.read_next(); m = deserialize_message(b, T)
        if m.stage in ('', 'post'): out.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.num_slam_features))
    return np.array(out)

def flight(d, fo):
    name = os.path.basename(d.rstrip('/')); world = world_of(d)
    try:
        e = np.loadtxt(f'{d}/ov_state_est.txt'); sd = np.loadtxt(f'{d}/ov_state_std.txt'); g = np.loadtxt(f'{d}/gt_imu.txt')
    except Exception:
        return 0
    g = g[np.unique(g[:, 0], return_index=True)[1]]
    ex = [json.loads(l) for l in open(f'{d}/executor.log') if l.startswith('{')]
    tex = [r['t'] for r in ex if r['phase'] == 'explore']
    if len(tex) < 2: return 0
    a0, a1 = tex[0], tex[-1]
    jc = jc_counts(d)
    sl = Slerp(g[:, 0], R.from_quat(g[:, 4:8]))
    def gt_T(t):   # interpolated at exactly t (OpenVINS state times are ~15 Hz; nearest-sample pairing
                   # would add ~1 deg of fake yaw error at 0.5 rad/s)
        t = min(max(t, g[0, 0]), g[-1, 0])
        return sl([t])[0], np.array([np.interp(t, g[:, 0], g[:, k]) for k in (1, 2, 3)])
    def ov_T(t):
        k = min(np.searchsorted(e[:, 0], t), len(e) - 1); return R.from_quat(e[k, 1:5]), e[k, 5:8], k
    n = 0
    for t0 in np.arange(a0, a1 - W, STEP):
        t1 = t0 + W
        Ro0, po0, k0 = ov_T(t0); Ro1, po1, k1 = ov_T(t1)
        if abs(e[k0, 0] - t0) > 0.2 or abs(e[k1, 0] - t1) > 0.2: continue
        t0, t1 = e[k0, 0], e[k1, 0]
        Rg0, pg0 = gt_T(t0); Rg1, pg1 = gt_T(t1)
        dRg, dpg = Rg0.inv() * Rg1, Rg0.inv().apply(pg1 - pg0); dRo, dpo = Ro0.inv() * Ro1, Ro0.inv().apply(po1 - po0)
        Re = dRg.inv() * dRo; te = dRg.inv().apply(dpo - dpg)
        yaw_err = abs(np.degrees(Re.as_euler('zyx')[0]))
        s = g[(g[:, 0] >= t0) & (g[:, 0] <= t1)]
        path = float(np.sum(np.linalg.norm(np.diff(s[:, 1:4], axis=0), axis=1)))
        yaws = np.unwrap(R.from_quat(s[:, 4:8]).as_euler('zyx')[:, 0]); rot = float(np.sum(np.abs(np.diff(yaws))))
        # covariance growth (std file: t, sig_th(3), sig_p(3), ...)
        j0 = min(np.searchsorted(sd[:, 0], t0), len(sd) - 1); j1 = min(np.searchsorted(sd[:, 0], t1), len(sd) - 1)
        sp0, sp1 = np.linalg.norm(sd[j0, 4:7]), np.linalg.norm(sd[j1, 4:7])
        gI0 = Rg0.inv().apply([0, 0, 1.0]); gI1 = Rg1.inv().apply([0, 0, 1.0])
        sy0 = float(np.sqrt(np.sum((gI0 * sd[j0, 1:4]) ** 2))); sy1 = float(np.sqrt(np.sum((gI1 * sd[j1, 1:4]) ** 2)))
        rec = {'flight': name, 'world': world, 't0': round(float(t0), 2), 'path_m': round(path, 3), 'speed': round(path / W, 3),
               'rot_rad': round(rot, 3), 'rate': round(rot / W, 4), 'drift_pos_m': round(float(np.linalg.norm(te)), 4), 'drift_yaw_deg': round(float(yaw_err), 4),
               'dsig_pos_m': round(float(sp1 - sp0), 5), 'dsig_yaw_deg': round(float(np.degrees(sy1 - sy0)), 5),
               # day 8 (ceiling check): the filter's sigmas at both window ends (per axis, m / rad)
               'sig_p0': [round(float(x), 6) for x in sd[j0, 4:7]], 'sig_p1': [round(float(x), 6) for x in sd[j1, 4:7]],
               'sig_y0': round(sy0, 7), 'sig_y1': round(sy1, 7)}
        if path >= 1.0:
            rec.update({'drift_pos_per_m': rec['drift_pos_m'] / path, 'drift_yaw_per_m': rec['drift_yaw_deg'] / path,
                        'dsig_pos_per_m': rec['dsig_pos_m'] / path, 'dsig_yaw_per_m': rec['dsig_yaw_deg'] / path})
        if len(jc):
            k = (jc[:, 0] >= t0) & (jc[:, 0] <= t1); rec['n_slam'] = float(jc[k, 1].mean()) if k.any() else None
        if world in SCENES:
            vals = []
            for t in np.arange(t0, t1, 0.5):
                Rg, pg = gt_T(t); vals.append(raycast(world, Rg.as_matrix(), pg))
            vals = np.array(vals); rec['range_med'] = round(float(np.nanmedian(vals[:, 0])), 3)
            rec['return_frac'] = round(float(vals[:, 1].mean()), 3); rec['plain_frac'] = round(float(vals[:, 2].mean()), 3)
        fo.write(json.dumps(rec) + '\n'); n += 1
    return n

if __name__ == '__main__':
    fo = open(sys.argv[1], 'a'); tot = 0
    for d in sys.argv[2:]:
        try:
            k = flight(d, fo); tot += k; fo.flush()
        except Exception as ex:
            print('skip', d, ex)
    print('windows', tot)
