#!/usr/bin/env python3
# jc_landmarks.py <run_dir>: per joint-covariance message, number of SLAM landmarks and their
# distance from the IMU, sampled at each planner decision time (day 3 step 1 context).
import sys, json, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
d = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=d + '/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/openvins/joint_covariance', '/exploration_planner/decision']))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
last = None
while r.has_next():
    tp, raw, t = r.read_next()
    m = deserialize_message(raw, types[tp])
    if tp == '/openvins/joint_covariance':
        if m.stage in ('', 'post'): last = m
    elif last is not None:
        dec = json.loads(m.data)
        p = np.array(last.imu_pose.p_iing)
        L = np.array([l.p_fing for l in last.landmarks]) if len(last.landmarks) else np.zeros((0, 3))
        dist = np.linalg.norm(L - p, axis=1) if len(L) else np.array([])
        nr = [c.get('n_real', 0) for c in dec['candidates']]
        print(f"t={dec['t']:.1f} nlm={len(L)} dist med={np.median(dist) if len(dist) else float('nan'):.1f} "
              f"max={dist.max() if len(dist) else float('nan'):.1f} ncand={len(nr)} cand_with_real={sum(1 for x in nr if x>0)} n_real={nr}")
