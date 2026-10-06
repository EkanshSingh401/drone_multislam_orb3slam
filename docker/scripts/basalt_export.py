#!/usr/bin/env python3
"""Export a flight bag to EuRoC/ASL layout + Basalt calibration (PATCHES s58). Run in the sim container.

    basalt_export.py <rundir> <outdir>

Writes <outdir>/mav0/{cam0,cam1}/data/<ns>.png + data.csv, imu0/data.csv,
state_groundtruth_estimate0/data.csv (GT moved to the IMU, v/biases zero),
<outdir>/basalt_calib.json, <outdir>/calib_conversion.txt and frames.txt.

Only stereo pairs (cam0 and cam1 with the identical header stamp) are written,
the same pairing the OpenVINS serial runner uses, so "frames in the bag" for the
consumed-frames check is the pair count printed here.

Calibration and IMU noise come from the GENERATED configs that OpenVINS and
ORB-SLAM3 use (docker/sim/config_sim_only, copied into the image as
/opt/config_sim_only), not re-typed:
  * T_imu_cam  = inverse(T_cam_imu) from kalibr_imucam_chain.yaml
  * pinhole fx fy cx cy from the same file (zero distortion)
  * Basalt's *_noise_std and *_bias_std are CONTINUOUS-time densities
    (basalt-headers calibration.hpp: sigma_d = sigma_c * sqrt(rate)), the same
    convention as Kalibr, so the kalibr_imu_chain.yaml values map 1:1.
"""
import json, os, sys
import numpy as np
import cv2, yaml
from scipy.spatial.transform import Rotation as R
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu

rd, out = sys.argv[1], sys.argv[2]
CFG = "/opt/config_sim_only"
P_BASE_IMU = np.array([0.12 - 0.00552, 0.0051, 0.242 - 0.01174])  # as ov_prep.py


def load_yaml(p):
    txt = open(p).read().replace("%YAML:1.0", "")
    return yaml.safe_load(txt)


cc = load_yaml(f"{CFG}/kalibr_imucam_chain.yaml"); ic = load_yaml(f"{CFG}/kalibr_imu_chain.yaml")["imu0"]
# PyYAML (YAML 1.1) reads "2e-06" as a STRING; force every scalar used to float.
for k in ("accelerometer_noise_density", "gyroscope_noise_density", "accelerometer_random_walk",
          "gyroscope_random_walk", "update_rate"):
    ic[k] = float(ic[k])
T_i_c, intr, res = [], [], []
for cam in ("cam0", "cam1"):
    T_c_i = np.array(cc[cam]["T_cam_imu"], float); Tic = np.linalg.inv(T_c_i)
    q = R.from_matrix(Tic[:3, :3]).as_quat()
    T_i_c.append({"px": Tic[0, 3], "py": Tic[1, 3], "pz": Tic[2, 3], "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]})
    fx, fy, cx, cy = map(float, cc[cam]["intrinsics"])
    assert not any(cc[cam]["distortion_coeffs"]), "pinhole export assumes zero distortion"
    intr.append({"camera_type": "pinhole", "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy}})
    res.append(list(cc[cam]["resolution"]))
rate = float(ic["update_rate"])
calib = {"value0": {
    "T_imu_cam": T_i_c, "intrinsics": intr, "resolution": res,
    "vignette": [], "calib_accel_bias": [0.0] * 9, "calib_gyro_bias": [0.0] * 12,
    "imu_update_rate": rate,
    "accel_noise_std": [ic["accelerometer_noise_density"]] * 3,
    "gyro_noise_std": [ic["gyroscope_noise_density"]] * 3,
    "accel_bias_std": [ic["accelerometer_random_walk"]] * 3,
    "gyro_bias_std": [ic["gyroscope_random_walk"]] * 3,
    "T_mocap_world": {"px": 0, "py": 0, "pz": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1},
    "T_imu_marker": {"px": 0, "py": 0, "pz": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1},
    "mocap_time_offset_ns": 0, "mocap_to_imu_offset_ns": 0,
    "cam_time_offset_ns": int(round(float(cc["cam0"].get("timeshift_cam_imu", 0.0)) * 1e9))}}
os.makedirs(out, exist_ok=True)
json.dump(calib, open(f"{out}/basalt_calib.json", "w"), indent=2)
with open(f"{out}/calib_conversion.txt", "w") as f:
    f.write("source: /opt/config_sim_only/kalibr_imucam_chain.yaml, kalibr_imu_chain.yaml (generated)\n")
    f.write("convention: Basalt *_noise_std / *_bias_std are continuous-time (calibration.hpp:\n"
            "  sigma_d = sigma_c*sqrt(rate)), identical to Kalibr -> copied 1:1\n")
    for k_b, k_k in (("accel_noise_std", "accelerometer_noise_density"), ("gyro_noise_std", "gyroscope_noise_density"),
                     ("accel_bias_std", "accelerometer_random_walk"), ("gyro_bias_std", "gyroscope_random_walk")):
        f.write(f"  {k_b:16s} = {ic[k_k]:<10g} (kalibr {k_k})\n")
    f.write(f"  imu_update_rate  = {rate:g}\n")
    for i, cam in enumerate(("cam0", "cam1")):
        f.write(f"  {cam}: T_imu_cam = inv(T_cam_imu) -> p {[round(T_i_c[i][k], 6) for k in ('px', 'py', 'pz')]} "
                f"q(xyzw) {[round(T_i_c[i][k], 6) for k in ('qx', 'qy', 'qz', 'qw')]}; pinhole {cc[cam]['intrinsics']}\n")
    f.write(f"  cam_time_offset_ns = {calib['value0']['cam_time_offset_ns']} (timeshift_cam_imu)\n")

info = rosbag2_py.Info().read_metadata(f"{rd}/flight.bag", "")
r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=f"{rd}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
L, Rt, I = "/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw", "/camera/imu"
r.set_filter(rosbag2_py.StorageFilter(topics=[L, Rt, I]))
for d in ("cam0/data", "cam1/data", "imu0", "state_groundtruth_estimate0"):
    os.makedirs(f"{out}/mav0/{d}", exist_ok=True)
imgs = {L: {}, Rt: {}}; imu = []
while r.has_next():
    t, data, _ = r.read_next()
    if t == I:
        m = deserialize_message(data, Imu); ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        imu.append((ns, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    else:
        m = deserialize_message(data, Image); ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        imgs[t][ns] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
pairs = sorted(set(imgs[L]) & set(imgs[Rt]))
for cam, top in (("cam0", L), ("cam1", Rt)):
    with open(f"{out}/mav0/{cam}/data.csv", "w") as f:
        f.write("#timestamp [ns],filename\n")
        for ns in pairs:
            cv2.imwrite(f"{out}/mav0/{cam}/data/{ns}.png", imgs[top][ns]); f.write(f"{ns},{ns}.png\n")
imu.sort()
with open(f"{out}/mav0/imu0/data.csv", "w") as f:
    f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
    for row in imu:
        f.write(f"{row[0]}," + ",".join(f"{v:.9g}" for v in row[1:]) + "\n")
gt = np.loadtxt(f"{rd}/gt.tum"); Rg = R.from_quat(gt[:, 4:8]); p = gt[:, 1:4] + Rg.apply(P_BASE_IMU)
with open(f"{out}/mav0/state_groundtruth_estimate0/data.csv", "w") as f:
    f.write("#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,b_w_x,b_w_y,b_w_z,b_a_x,b_a_y,b_a_z\n")
    for t, pp, q in zip(gt[:, 0], p, gt[:, 4:8]):
        f.write(f"{int(round(t * 1e9))},{pp[0]:.9f},{pp[1]:.9f},{pp[2]:.9f},{q[3]:.9f},{q[0]:.9f},{q[1]:.9f},{q[2]:.9f}" + ",0" * 9 + "\n")
with open(f"{out}/frames.txt", "w") as f:
    f.write(f"cam0_in_bag={len(imgs[L])} cam1_in_bag={len(imgs[Rt])} stereo_pairs={len(pairs)} imu={len(imu)}\n")
print(f"basalt_export: {len(pairs)} stereo pairs (cam0 {len(imgs[L])}, cam1 {len(imgs[Rt])}), {len(imu)} IMU -> {out}")
