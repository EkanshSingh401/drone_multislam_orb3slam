#!/usr/bin/env python3
"""imu_allan.py <bag> [t_skip_s] (day 5 step 4a): what noise does the simulated IMU actually generate?

Static recording (vehicle on the ground, disarmed). Overlapping Allan deviation per axis of the
gyro and of the accelerometer (mean removed) from /camera/imu, against a SYNTHETIC reference with the
SAME stamps: white noise at the SDF per-sample sigma plus gz-sensors' bias recursion
(GaussianNoiseModel.cc: sigma_b_d = sqrt(-sb^2 tau/2 expm1(-2dt/tau)), bias = exp(-dt/tau) bias + N)
driven by the OpenVINS-configured random-walk densities, 8 realisations. Equal curves = the sensor
generates what OpenVINS assumes. Also fits the classical white density N = adev(tau)*sqrt(tau) at
tau = 0.1 s and the random walk K = adev(tau)*sqrt(3/tau) at the largest usable tau.
Prints one JSON line."""
import json, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

SIG_D = {"gyro": 0.0022627417, "acc": 0.0282842712}          # SDF per-sample sigma
DENS = {"gyro": 1.6e-4, "acc": 2e-3}; RW = {"gyro": 2e-6, "acc": 3e-4}  # OpenVINS config
TAU_GM = 3600.0

def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/camera/imu"]))
    t, w, a = [], [], []
    while r.has_next():
        _, d, _ = r.read_next(); m = deserialize_message(d, Imu)
        t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(w), np.array(a)

def adev(x, t, taus):
    """overlapping Allan deviation on the (irregular) stamps: cumulative integral, uniform resample."""
    dt = np.diff(t); th = np.cumsum(np.r_[0, x[1:] * dt])    # integral of the sample-and-hold signal
    T0 = 0.005; tu = np.arange(t[0], t[-1], T0); thu = np.interp(tu, t, th)
    out = []
    for tau in taus:
        m = int(round(tau / T0))
        if 2 * m >= len(tu) - 1: out.append(np.nan); continue
        d = thu[2 * m:] - 2 * thu[m:-m] + thu[:-2 * m]
        out.append(np.sqrt(np.mean(d ** 2) / (2 * (m * T0) ** 2)))
    return np.array(out)

def synth(t, kind, rng):
    n = len(t); dt = np.r_[0.005, np.diff(t)]
    x = rng.normal(0, SIG_D[kind], n); b = 0.0; out = np.empty(n)
    sb = RW[kind]
    phi = np.exp(-dt / TAU_GM); sbd = np.sqrt(-sb * sb * TAU_GM / 2 * np.expm1(-2 * dt / TAU_GM))
    e = rng.normal(0, 1, n)
    for i in range(n):
        b = phi[i] * b + sbd[i] * e[i]; out[i] = x[i] + b
    return out

def main():
    t, w, a = read(sys.argv[1])
    skip = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    k = t > t[0] + skip; t, w, a = t[k], w[k], a[k]
    T = t[-1] - t[0]
    taus = np.array([0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500])
    taus = taus[taus < T / 4]
    rng = np.random.default_rng(1)
    dts = np.diff(t); res = {"bag": sys.argv[1], "duration_s": round(T, 1), "n": len(t),
                             "dt_hist_ms": {str(round(v * 1e3)): int(c) for v, c in zip(*np.unique(np.round(dts, 3), return_counts=True))},
                             "taus": taus.tolist()}
    for kind, X in (("gyro", w), ("acc", a)):
        X = X - X.mean(0)
        meas = np.array([adev(X[:, j], t, taus) for j in range(3)])
        syn = np.array([[adev(synth(t, kind, rng), t, taus) for _ in range(8)]]).reshape(8, -1)
        sm, ss = syn.mean(0), syn.std(0)
        i01 = int(np.argmin(np.abs(taus - 0.1))); il = len(taus) - 1
        res[kind] = {"adev_meas_xyz": np.round(meas, 9).tolist(), "adev_synth_mean": np.round(sm, 9).tolist(), "adev_synth_sd": np.round(ss, 9).tolist(),
                     "ratio_meas_over_synth_xyz": np.round(meas / sm, 3).tolist(),
                     "white_density_fit_xyz": np.round(meas[:, i01] * np.sqrt(taus[i01]), 7).tolist(), "white_density_cfg": DENS[kind],
                     "rw_fit_xyz_at_tau%g" % taus[il]: np.round(meas[:, il] * np.sqrt(3 / taus[il]), 8).tolist(),
                     "rw_fit_synth_at_tau%g" % taus[il]: round(float(sm[il] * np.sqrt(3 / taus[il])), 8), "rw_cfg": RW[kind],
                     "mean_xyz": np.round((w if kind == "gyro" else a).mean(0), 5).tolist()}
    print(json.dumps(res))

if __name__ == "__main__":
    main()
