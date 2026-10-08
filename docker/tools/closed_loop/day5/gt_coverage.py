#!/usr/bin/env python3
"""gt_coverage.py <flight_dir> <world> [--real-depth] [--no-return-free] (day 5; from day4/gt_coverage.py)

DAY 5 CHANGES: geometry from ../scenes.py (validation | office); rays with NO RETURN (no hit within the
depth camera's 20 m far clip; with --real-depth: NaN/inf/<= 0.05 m pixels) are UNKNOWN, matching the
fixed mapper (no_return_as_free false); --no-return-free restores the day-4 rule. Coverage CURVE every
1 s from t_explore: GT-pose known volume clipped to the scene envelope / scenes.reachable_m3 -> time to
50% / 90% and normalized area under the curve over [0, 180] s (coverage_metrics()).

Day-4 description follows.
coverage that cannot count estimator divergence.

The flight bags carry no depth images, so the depth camera is EMULATED: rays of cam0 (the depth
camera's pose, mapper intrinsics 446.8 / 424 / 240, 848x480, every `STEP`-th pixel) are cast
against the validation scene's ground-truth geometry (clearance.py SOLIDS + floor z = 0) from the
GROUND-TRUTH camera pose. The ranges are then inserted like octomap_mapper does (free along the ray
up to min(hit, 8 m), the hit voxel occupied if within 8 m, no-return rays free to 8 m, 0.5 m free
sphere at the vehicle), resolution 0.1 m, at RATE Hz, into two maps:
  A  at the GT poses                     -> coverage with perfect localization
  B  at the OpenVINS poses (/ov_msckf/odomimu) -> emulation of what the mapper did (validation of the
     emulation against the recorded /octomap_mapper/coverage stream)
Known volume = free + occupied voxels. A is also reported clipped to the room (x -4..5.5,
y -5.5..5.5, z 0..4: inside the walls, below their tops). Times: t_explore + {60, 120, 180} s and final.
Prints one JSON line."""
import json, os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scenes import solids as scene_solids, envelope as scene_envelope, reachable_m3
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot

STEP, RATE, RES, RMAX, DS = 6, 2.0, 0.10, 8.0, 0.05
FX, FY, CX, CY, W, H = 446.802773, 446.802773, 424.0, 240.0, 848, 480
T_CI = np.array([0, -1, 0, 0.0424, 0, 0, -1, 0.01174, 1, 0, 0, -0.00552, 0, 0, 0, 1.0]).reshape(4, 4)  # cam0 <- IMU (mapper default)
FAR = 20.0   # depth camera far clip (d455 model.sdf)
WORLD = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith('--') else 'validation'
SOLIDS = [x[1:] for x in scene_solids(WORLD)]
X_MIN, X_MAX, Y_MIN, Y_MAX, WALL_H = scene_envelope(WORLD)
REACH = reachable_m3(WORLD)
NR_FREE = '--no-return-free' in sys.argv
LO = np.array([[cx-sx/2, cy-sy/2, cz-sz/2] for cx, cy, cz, sx, sy, sz in SOLIDS])
HI = np.array([[cx+sx/2, cy+sy/2, cz+sz/2] for cx, cy, cz, sx, sy, sz in SOLIDS])
u, v = np.meshgrid(np.arange(STEP / 2, W, STEP), np.arange(STEP / 2, H, STEP))
DIR_C = np.stack([(u.ravel() - CX) / FX, (v.ravel() - CY) / FY, np.ones(u.size)], 1)
NORM_C = np.linalg.norm(DIR_C, axis=1)
DIR_C = DIR_C / NORM_C[:, None]
SAMP = np.arange(DS / 2, RMAX + 1e-9, DS)

def first_hit(o, D):
    """distance along unit rays D from o to the first solid / floor (inf if none)."""
    with np.errstate(divide='ignore', invalid='ignore'):
        Ds = np.where(np.abs(D) < 1e-12, 1e-12, D)  # exact-zero components gave 0 * inf = NaN in the slab test
        inv = 1.0 / Ds
        t1 = (LO[None] - o[None, None]) * inv[:, None]; t2 = (HI[None] - o[None, None]) * inv[:, None]
        tn = np.minimum(t1, t2).max(axis=2); tf = np.maximum(t1, t2).min(axis=2)
        hit = (tf >= np.maximum(tn, 0))
        tb = np.where(hit, np.maximum(tn, 0), np.inf).min(1)
        tz = np.where(D[:, 2] < -1e-9, -o[2] / D[:, 2], np.inf)
    return np.minimum(tb, tz)

class Grid:
    def __init__(self):
        self.lo = np.array([-30.0, -30.0, -2.0]); self.n = np.array([600, 600, 140])
        self.g = np.zeros(self.n, bool)
    def mark(self, P):
        k = np.floor((P - self.lo) / RES).astype(np.int64)
        ok = np.all((k >= 0) & (k < self.n), 1); k = k[ok]
        self.g[k[:, 0], k[:, 1], k[:, 2]] = True
    def volume(self, clip=False):
        if not clip: return float(self.g.sum() * RES ** 3)
        c = self.lo + (np.indices(self.n).reshape(3, -1).T + 0.5) * RES if False else None
        i0 = np.floor((np.array([X_MIN, Y_MIN, 0.0]) - self.lo) / RES).astype(int)  # envelope incl. inner wall halves
        i1 = np.ceil((np.array([X_MAX, Y_MAX, WALL_H]) - self.lo) / RES).astype(int)
        return float(self.g[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]].sum() * RES ** 3)

def insert(grid, R_GC, p_GC, rng):
    D = DIR_C @ R_GC.T
    if not NR_FREE:                                       # no return -> unknown (day 5)
        k = np.isfinite(rng) & (rng <= FAR); D, rng = D[k], rng[k]
    r = np.minimum(rng, RMAX)
    m = SAMP[None, :] < r[:, None]                       # free samples before the hit / max range
    P = p_GC[None, None] + D[:, None] * SAMP[None, :, None]
    grid.mark(P[m])
    h = rng <= RMAX
    grid.mark(p_GC[None] + D[h] * rng[h, None])           # occupied endpoint
    b = np.mgrid[-5:6, -5:6, -5:6].reshape(3, -1).T * RES
    grid.mark(p_GC[None] + b[np.linalg.norm(b, axis=1) <= 0.5])

def pose_cam(R_IG_q, p):  # quaternion x y z w of R_ItoG, IMU position
    R_ItoG = Rot.from_quat(R_IG_q).as_matrix()
    R_CtoI = T_CI[:3, :3].T; p_CinI = -R_CtoI @ T_CI[:3, 3]
    return R_ItoG @ R_CtoI, p + R_ItoG @ p_CinI

def main():
    d = sys.argv[1]; name = os.path.basename(d.rstrip('/'))
    rec = []
    for l in open(f'{d}/executor.log'):
        l = re.sub(r'\x1b\[[0-9;]*m', '', l).strip()
        if l.startswith('{'):
            try: j = json.loads(l); rec.append((j['t'], j['phase']))
            except Exception: pass
    t_ex = next((t for t, p in rec if p == 'explore'), None)
    if t_ex is None: print(json.dumps({'flight': name, 'error': 'no explore'})); return
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    names = {t.name: t.type for t in r.get_all_topics_and_types()}
    real = '--real-depth' in sys.argv  # validation: use the recorded depth images (RECORD_SENSORS flights)
    tops = ['/ov_msckf/odomimu', '/octomap_mapper/coverage'] + (['/camera/depth/image_rect_raw'] if real else [])
    r.set_filter(rosbag2_py.StorageFilter(topics=tops)); ty = {t: get_message(names[t]) for t in tops}
    od, cov, depth = [], [], []
    last_d = -1e9
    while r.has_next():
        tp, raw, _ = r.read_next()
        if tp == '/camera/depth/image_rect_raw':
            m = deserialize_message(raw, ty[tp]); td = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            if td - last_d >= 1.0 / RATE:
                z = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)[int(STEP / 2)::STEP, int(STEP / 2)::STEP].ravel()
                depth.append((td, z.copy())); last_d = td
            continue
        m = deserialize_message(raw, ty[tp])
        if tp == tops[0]:
            o, p = m.pose.pose.orientation, m.pose.pose.position
            od.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        else: cov.append(list(m.data[:2]))
    od, cov = np.array(od), np.array(cov)
    gt = np.loadtxt(f'{d}/gt_imu.txt'); gt = gt[np.unique(gt[:, 0], return_index=True)[1]]
    # frames: GT camera rays vs the OV-pose insertion share the camera-frame ranges
    t0 = max(gt[0, 0], od[0, 0]); t1 = min(gt[-1, 0], od[-1, 0], cov[-1, 0] if len(cov) else 1e9)
    A, B = Grid(), Grid(); out = {'flight': name, 'depth': 'recorded' if real else 'emulated'}
    marks = {60: t_ex + 60, 120: t_ex + 120, 180: t_ex + 180}
    curve = []; next_c = t_ex
    ts = [x[0] for x in depth if t0 <= x[0] < t1] if real else np.arange(t0, t1, 1.0 / RATE)
    dz = {x[0]: x[1] for x in depth}
    for t in ts:
        ig = np.searchsorted(gt[:, 0], t); io = np.searchsorted(od[:, 0], t)
        if ig >= len(gt) or io >= len(od): break
        g, o = gt[ig], od[io]
        if g[3] < 0.3: pass  # on the ground the mapper still integrates; keep it
        Rg, pg = pose_cam(g[4:8], g[1:4])
        if real:  # z-depth -> range along the unit ray; no return (inf, nan, <= 0.05) -> beyond max range
            z = dz[t]; rng = np.where(np.isfinite(z) & (z > 0.05), z * NORM_C, np.inf)
        else:
            rng = first_hit(pg, DIR_C @ Rg.T)
        insert(A, Rg, pg, rng)
        Ro, po = pose_cam(o[4:8], o[1:4])
        insert(B, Ro, po, rng)
        if t >= next_c:
            rc = cov[min(np.searchsorted(cov[:, 0], t), len(cov) - 1), 1] if len(cov) else None
            curve.append((round(t - t_ex, 2), round(A.volume(True), 1), round(A.volume(), 1), round(float(rc), 1) if rc is not None else None))
            next_c = t_ex + len(curve) * 1.0
        for k, tm in list(marks.items()):
            if t >= tm:
                rc = cov[min(np.searchsorted(cov[:, 0], tm), len(cov) - 1), 1] if len(cov) else None
                out[f'@{k}'] = {'recorded': round(rc, 1) if rc is not None else None, 'gt_pose': round(A.volume(), 1),
                                'gt_pose_room': round(A.volume(True), 1), 'emulated_ov_pose': round(B.volume(), 1)}
                del marks[k]
    for k, tm in marks.items():  # flight ended before: carry final
        out[f'@{k}'] = None
    out['final'] = {'recorded': round(float(cov[-1, 1]), 1) if len(cov) else None, 'gt_pose': round(A.volume(), 1),
                    'gt_pose_room': round(A.volume(True), 1), 'emulated_ov_pose': round(B.volume(), 1)}
    out['world'] = WORLD; out['reachable_m3'] = round(REACH, 1); out['no_return'] = 'free' if NR_FREE else 'unknown'
    out['curve'] = curve   # (t - t_explore, gt_pose_envelope, gt_pose, recorded)
    out['metrics'] = coverage_metrics([c[0] for c in curve], [c[1] for c in curve], REACH)
    print(json.dumps(out))


def coverage_metrics(t, v, reach, horizon=180.0):
    """t50 / t90: first time the GT-pose envelope coverage reaches 50 / 90 % of the reachable volume
    (None if never within the curve); auc: mean coverage fraction over [0, horizon] s, the last value
    carried forward past the end of the curve (a flight that ends early keeps what it mapped)."""
    t, f = np.asarray(t, float), np.asarray(v, float) / reach
    if len(t) == 0: return None
    res = {}
    for q in (0.5, 0.9):
        k = np.nonzero(f >= q)[0]; res[f't{int(q * 100)}_s'] = round(float(t[k[0]]), 1) if len(k) and t[k[0]] <= horizon else None
    g = np.arange(0.0, horizon + 1e-9, 1.0)
    fg = np.interp(g, t, f, left=f[0], right=f[-1])
    res['auc'] = round(float(getattr(np, 'trapezoid', getattr(np, 'trapz', None))(fg, g) / horizon), 4)
    res['frac@180'] = round(float(fg[-1]), 4)
    return res


if __name__ == "__main__":
    main()
