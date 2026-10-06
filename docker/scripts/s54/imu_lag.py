#!/usr/bin/env python3
"""IMU stamp lag per sample class (PATCHES s56). Run in the sim container.
    imu_lag.py <rundir>
GT (snapped to the 4 ms grid, mis-snaps repaired as in synth_imu.py) -> rotation
spline -> body rate w(t) and angular acceleration. Over the airborne window,
gyro(stamp) - w(stamp) ~= -lag * wdot(stamp) (first order), so per class
lag = -sum(r.wdot)/sum(wdot.wdot). Also a direct scan per class: the shift d that
minimises |gyro(stamp) - w(stamp + d)|. Classes: preceding interval (4/8 ms) and
phase in the 4,4,4,8 cycle (sample index since the last 8 ms gap).
"""
import sys, json, numpy as np
from scipy.spatial.transform import Rotation as R, RotationSpline
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
STEP = 0.004; rd = sys.argv[1]
gt = np.loadtxt(f"{rd}/gt.tum"); gt[:, 0] = np.round(gt[:, 0] / STEP) * STEP
gt = gt[np.r_[True, np.diff(gt[:, 0]) > 1e-6]]
t = gt[:, 0].copy()
for i in range(2, len(t) - 2):  # leave-one-out mis-snap repair on position
    nb = [i - 2, i - 1, i + 1, i + 2]
    c = [np.polyfit(t[nb] - t[i], gt[nb, k], 3) for k in (1, 2, 3)]
    err = [np.linalg.norm([np.polyval(cc, d) - gt[i, k + 1] for k, cc in enumerate(c)]) for d in (0, -STEP, STEP)]
    j = int(np.argmin(err))
    if j and err[j] < 0.3 * err[0] and t[i - 1] < t[i] + (-STEP, STEP)[j - 1] < t[i + 1]:
        t[i] += (-STEP, STEP)[j - 1]
gt[:, 0] = t; gt = gt[np.r_[True, np.diff(gt[:, 0]) > 1e-6]]
rs = RotationSpline(gt[:, 0], R.from_quat(gt[:, 4:8]))
def wbody(ts, eps=5e-4):
    return (rs(ts - eps).inv() * rs(ts + eps)).as_rotvec() / (2 * eps)
z = gt[:, 3]; up = np.where(z > 0.5)[0]; t0 = gt[up[0], 0] + 3; t1 = gt[up[-1] + np.argmax(z[up[-1]:] < 0.05), 0]
info = rosbag2_py.Info().read_metadata(f"{rd}/flight.bag", "")
r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=f"{rd}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/camera/imu"]))
im = []
while r.has_next():
    _, d, _ = r.read_next(); m = deserialize_message(d, Imu)
    im.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
im = np.array(im)
dt = np.r_[np.nan, np.diff(im[:, 0])]
phase = np.zeros(len(im), int); k = 0
for i in range(1, len(im)):
    k = 0 if dt[i] > 0.006 else k + 1; phase[i] = k
sel = (im[:, 0] > t0) & (im[:, 0] < t1)
ts = im[sel, 0]; g = im[sel, 1:4]; w = wbody(ts); wd = (wbody(ts + 1e-3) - wbody(ts - 1e-3)) / 2e-3
res = g - w
out = {"run": rd.split("/")[-1], "n": int(sel.sum())}
def lag(mask):
    rr, dd = res[mask].ravel(), wd[mask].ravel()
    keep = np.abs(rr) < 5 * 1.4826 * np.median(np.abs(rr))     # robust: drop GT outliers
    reg = -float(np.sum(rr[keep] * dd[keep]) / np.sum(dd[keep] ** 2))
    scan = []
    for d_ms in np.arange(-6, 6.01, 0.25):
        e = g[mask] - wbody(ts[mask] + d_ms * 1e-3); scan.append((float(np.median(np.abs(e))), float(d_ms)))
    return {"n": int(mask.sum()), "lag_reg_ms": round(reg * 1e3, 3), "best_shift_ms": min(scan)[1]}
out["all"] = lag(np.ones(len(ts), bool))
for lab, m in (("after_4ms", np.isclose(dt[sel], 0.004, atol=1e-4)), ("after_8ms", np.isclose(dt[sel], 0.008, atol=1e-4))):
    out[lab] = lag(m)
for p in range(4):
    out[f"phase{p}"] = lag(phase[sel] == p)
print(json.dumps(out))
