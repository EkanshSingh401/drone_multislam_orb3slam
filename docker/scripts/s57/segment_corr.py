#!/usr/bin/env python3
"""Per-segment orientation NEES vs perception quality (PATCHES s57). Run in the sim container.
    segment_corr.py <rundir> <replay_dir> [--seg 5]
Frame k (k-th [TRACK] line of the serial run's DEBUG log) has the k-th stereo-pair
stamp of the bag (cam0 & cam1 header stamps present in both; the serial runner
pairs exactly these). [FI]/[MSCKF] lines after TRACK k belong to frame k.
Segments of --seg s over the s51 GT window. Per segment: mean ori / rp NEES (diag,
per state; and full where /openvins/joint_covariance exists), median attempt depth,
tri reject rate, MSCKF used/in, tracked cam0, GT angular speed, GT speed.
Prints one JSON line per segment."""
import sys, re, json, subprocess, numpy as np
from scipy.spatial.transform import Rotation as R
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
rd, od = sys.argv[1], sys.argv[2]; SEG = float(sys.argv[sys.argv.index("--seg") + 1]) if "--seg" in sys.argv else 5.0
ANSI = re.compile(r"\x1b\[[0-9;]*m")
info = rosbag2_py.Info().read_metadata(f"{rd}/flight.bag", "")
r = rosbag2_py.SequentialCompressionReader() if (info.compression_mode or "").upper() == "FILE" else rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=f"{rd}/flight.bag", storage_id=""), rosbag2_py.ConverterOptions("", ""))
L, Rr = "/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw"
r.set_filter(rosbag2_py.StorageFilter(topics=[L, Rr]))
st = {L: [], Rr: []}
from rclpy.serialization import deserialize_message as dm
from std_msgs.msg import Header
while r.has_next():
    t, d, _ = r.read_next(); m = dm(d, Image); st[t].append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
pairs = np.array(sorted(set(st[L]) & set(st[Rr]))) * 1e-9
frames = []  # per frame: dict
cur = None
for line in open(f"{od}/openvins.log", errors="replace"):
    line = ANSI.sub("", line)
    if "[TRACK]" in line and "VioManager" in line:
        m = re.search(r"cam0=(\d+)", line); cur = {"trk": int(m.group(1)), "z": [], "tri": 0, "rej": 0, "in": 0, "used": 0}; frames.append(cur)
    elif cur is None:
        continue
    elif "[FI] ok" in line or "[FI] reject=tri" in line:
        cur["tri"] += 1; cur["rej"] += "reject=tri" in line
        m = re.search(r"z=([-0-9.naif]+)", line)
        if m:
            try:
                z = float(m.group(1))
                if np.isfinite(z) and z > 0: cur["z"].append(z)
            except ValueError: pass
    elif "[MSCKF] in=" in line:
        m = re.search(r"in=(\d+) used=(\d+)", line)
        if m: cur["in"] += int(m.group(1)); cur["used"] += int(m.group(2))
assert len(frames) == len(pairs), (len(frames), len(pairs))
ft = pairs
nd = np.loadtxt(f"{od}/nees_dump_diag.txt"); nf = np.loadtxt(f"{od}/nees_dump_full.txt")
gt = np.loadtxt(f"{rd}/gt.tum")
w0, w1 = nd[0, 0], nd[-1, 0]
gq = R.from_quat(gt[:, 4:8]); gdt = np.diff(gt[:, 0])
angspd = np.r_[np.linalg.norm((gq[:-1].inv() * gq[1:]).as_rotvec(), axis=1) / gdt, np.nan]
spd = np.r_[np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1) / gdt, np.nan]
for a in np.arange(w0, w1 - SEG / 2, SEG):
    b = a + SEG
    fi = [f for f, t in zip(frames, ft) if a <= t < b]
    z = np.concatenate([f["z"] for f in fi]) if fi else np.array([])
    tri = sum(f["tri"] for f in fi); rej = sum(f["rej"] for f in fi); inn = sum(f["in"] for f in fi); used = sum(f["used"] for f in fi)
    sd = (nd[:, 0] >= a) & (nd[:, 0] < b); sf = (nf[:, 0] >= a) & (nf[:, 0] < b); sg = (gt[:, 0] >= a) & (gt[:, 0] < b)
    print(json.dumps({"run": rd.split("/")[-1], "t0": round(a, 2), "n": int(sd.sum()),
        "ori_diag": float(nd[sd, 2].mean()), "rp_diag": float(nd[sd, 3].mean()),
        "ori_full": float(nf[sf, 2].mean()) if sf.any() else None, "rp_full": float(nf[sf, 3].mean()) if sf.any() else None,
        "depth_med": float(np.median(z)) if len(z) else None, "tri_rej": rej / tri if tri else None,
        "msckf_used_frac": used / inn if inn else None, "tracked": float(np.mean([f["trk"] for f in fi])) if fi else None,
        "gt_angspd": float(np.nanmedian(angspd[sg])), "gt_speed": float(np.nanmedian(spd[sg]))}))
