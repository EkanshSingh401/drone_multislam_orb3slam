#!/usr/bin/env python3
"""IMU vs ground truth over the s51 airborne window (PATCHES s54). Run in the sim container.

    python3 imu_vs_gt.py <rundir> [--json]

No GT differentiation. GT stamps (bag receive time mapped through /clock) are
snapped to the 4 ms physics grid. Over consecutive GT knots a<m<b:
  accel   2*p[a,m,b] (second divided difference of GT position of point P)
          == integral of M(s) * (R(s) f(s) + g) ds,  M = unit hat on [a,b] peaking at m
  gyro    Log(R_a^T R_b) == integral_a^b w(s) ds  (body frame)
IMU samples are linearly interpolated between their stamps (+ delta for the time
scan). Residuals are compared with the same operator applied to white noise of the
SDF per-sample sigma (= the OpenVINS densities at 200 Hz, s53), so 'ratio' = 1
means the IMU error is exactly the modelled white noise.
P candidates: base_link origin, d455_link origin, IMU sensor position.
"""
import json, sys
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

G = np.array([0, 0, -9.81]); STEP = 0.004; FINE = 0.0005
P = {"base_origin": np.zeros(3), "d455_link": np.array([0.12, 0, 0.242]),
     "imu_sensor": np.array([0.12 - 0.00552, 0.0051, 0.242 - 0.01174])}
SIG_G, SIG_A = 0.0022627417, 0.0282842712          # SDF per-sample stddev
DENS_G, DENS_A, RW_G, RW_A = 1.6e-4, 2e-3, 2e-6, 3e-4  # OpenVINS config

rd = sys.argv[1]
gt = np.loadtxt(f"{rd}/gt.tum")
gt[:, 0] = np.round(gt[:, 0] / STEP) * STEP
gt = gt[np.r_[True, np.diff(gt[:, 0]) > 1e-6]]
z = gt[:, 3]; up = np.where(z > 0.5)[0]
t0 = gt[up[0], 0] + 3.0; td = up[-1] + np.argmax(z[up[-1]:] < 0.05); t1 = gt[td, 0]

info = rosbag2_py.Info().read_metadata(f"{rd}/flight.bag", "")
r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=f"{rd}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/camera/imu"]))
im = []
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Imu)
    im.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.x, m.angular_velocity.y,
               m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
im = np.array(im)

k = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t1)]
tk = k[:, 0]; Rk = R.from_quat(k[:, 4:8]); slerp = Slerp(gt[:, 0], R.from_quat(gt[:, 4:8]))
tg = np.arange(tk[0], tk[-1] + FINE / 2, FINE); Rg = slerp(tg).as_matrix()
gi = np.round((tk - tg[0]) / FINE).astype(int)          # knot -> fine index
gap = np.round(np.diff(tk) * 1e3); okg = np.isin(gap, [12, 16, 20]); oka = okg[:-1] & okg[1:]
a_, m_, b_ = gi[:-2], gi[1:-1], gi[2:]; ta, tm, tb = tk[:-2], tk[1:-1], tk[2:]

def cum(y):  # cumulative trapezoid along axis 0
    return np.vstack([np.zeros((1,) + y.shape[1:]), np.cumsum((y[1:] + y[:-1]) * FINE / 2, 0)])

def hat_int(y):
    """integral of M(s) y(s) over each triple, M unit-area hat (vectorised)."""
    S0, S1 = cum(y), cum(y * tg[:, None])
    I = lambda S, i, j: S[j] - S[i]
    L = (I(S1, a_, m_) - ta[:, None] * I(S0, a_, m_)) / (tm - ta)[:, None]
    Rr = (tb[:, None] * I(S0, m_, b_) - I(S1, m_, b_)) / (tb - tm)[:, None]
    return 2.0 * (L + Rr) / (tb - ta)[:, None]

def interp(t_s, y):
    return np.column_stack([np.interp(tg, t_s, y[:, j]) for j in range(y.shape[1])])

def accel_resid(f, ts, p_body):
    pk = k[:, 1:4] + Rk.apply(p_body)
    dd = 2 * ((pk[2:] - pk[1:-1]) / (tb - tm)[:, None] - (pk[1:-1] - pk[:-2]) / (tm - ta)[:, None]) / (tb - ta)[:, None]
    aw = np.einsum("nij,nj->ni", Rg, interp(ts, f)) + G
    e = dd - hat_int(aw)
    return np.einsum("nji,nj->ni", Rg[m_], e)[oka]           # world -> body at m, clean triples

def gyro_resid(w, ts):
    S = cum(interp(ts, w)); pred = S[gi[1:]] - S[gi[:-1]]
    meas = (Rk[:-1].inv() * Rk[1:]).as_rotvec()
    return ((meas - pred) / np.diff(tk)[:, None])[okg]         # rate units, clean intervals

rng = np.random.default_rng(0)
def mc(fun, sig, *args):
    return np.std(fun(rng.normal(0, sig, (len(im), 3)), im[:, 0], *args), 0)

out = {"clean_frac_acc": oka.mean(), "clean_frac_gyr": okg.mean(), "run": rd.split("/")[-1], "window": [t0, t1], "n_knots": len(tk), "knot_dt_ms": np.unique(np.round(np.diff(tk) * 1e3)).tolist()}
# accel: lever-arm point
exp_a = mc(lambda f, ts, p: accel_resid(f, ts, p) - accel_resid(0 * f, ts, p), SIG_A, P["imu_sensor"])
out["accel_expected_white_std"] = exp_a
for name, p in P.items():
    e = accel_resid(im[:, 4:7], im[:, 0], p)
    out[f"accel_{name}"] = {"std": e.std(0), "ratio": e.std(0) / exp_a, "mean": e.mean(0), "p99abs": np.percentile(np.abs(e), 99, 0)}
exp_g = mc(lambda w, ts: gyro_resid(w, ts) - gyro_resid(0 * w, ts), SIG_G)
eg = gyro_resid(im[:, 1:4], im[:, 0])
out["gyro_expected_white_std"] = exp_g
out["gyro"] = {"std": eg.std(0), "ratio": eg.std(0) / exp_g, "mean": eg.mean(0), "p99abs": np.percentile(np.abs(eg), 99, 0)}
# time offset scan (IMU stamp + delta)
scan = []
for dms in np.arange(-6, 6.01, 0.25):
    ea = accel_resid(im[:, 4:7], im[:, 0] + dms * 1e-3, P["imu_sensor"]); egg = gyro_resid(im[:, 1:4], im[:, 0] + dms * 1e-3)
    scan.append([float(dms), float(np.median(np.linalg.norm(ea, axis=1))), float(np.median(np.linalg.norm(egg, axis=1)))])
out["offset_scan_ms_accrms_gyrrms"] = scan
sc = np.array(scan); out["best_offset_ms"] = {"accel": sc[sc[:, 1].argmin(), 0], "gyro": sc[sc[:, 2].argmin(), 0]}
# bias (5 s bins) and Allan deviation from bin means, at the best point / zero offset
best = min(P, key=lambda n: np.linalg.norm(out[f"accel_{n}"]["std"]))
ea = accel_resid(im[:, 4:7], im[:, 0], P[best])
def bins(e, tau):
    tt = tm[oka] if len(e) == oka.sum() else tk[:-1][okg]
    idx = np.floor((tt - tt[0]) / tau).astype(int)
    return np.array([e[idx == i].mean(0) for i in range(idx.max()) if np.any(idx == i)])
out["bias_5s_accel"] = bins(ea, 5.0).round(4).tolist(); out["bias_5s_gyro"] = bins(eg, 5.0).round(5).tolist()
ad = []
for tau in [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10]:
    ba, bg = bins(ea, tau), bins(eg, tau)
    if len(ba) < 4: continue
    ad.append([tau, np.sqrt(0.5 * np.mean(np.diff(ba, axis=0) ** 2, 0)).tolist(), np.sqrt(0.5 * np.mean(np.diff(bg, axis=0) ** 2, 0)).tolist(),
               DENS_A / np.sqrt(tau), DENS_G / np.sqrt(tau), RW_A * np.sqrt(tau / 3), RW_G * np.sqrt(tau / 3)])
out["allan"] = {"cols": "tau, adev_acc_xyz, adev_gyr_xyz, white_acc, white_gyr, rw_acc, rw_gyr", "rows": ad}
out["best_point"] = best
np.savez(f"/out/diag_s56/resid_{out['run']}.npz", ea=ea, eg=eg, tm=tm, tk=tk, im=im, ta=ta, tb=tb)
print(json.dumps(out, default=lambda o: np.round(o, 6).tolist() if isinstance(o, np.ndarray) else float(o)))
