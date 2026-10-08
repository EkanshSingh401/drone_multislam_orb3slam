#!/usr/bin/env python3
"""single_cycle.py <rec_dir> (day 5 step 5b): model vs realized for ONE update cycle, same replay run.
For each JC snapshot (stamp t_k, 'post') of the recorded replay, the next update cycle t_c of the stage
log: one dense waypoint at the odometry pose at t_c, the landmarks in that JC (gain_ladder dense_win
rung). Writes tasks + realized (prop = ld_prop(t_c) - ld_post(t_k), meas = ld_prop(t_c) - ld_init(t_c)
and its msckf/slam split) to <rec_dir>/cyc/."""
import json, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
D = sys.argv[1]; O = f'{D}/cyc'; os.makedirs(f'{O}/jc', exist_ok=True)
st = {}
for p in (l.split() for l in open(f'{D}/stages.txt')):
    st.setdefault(round(float(p[1]), 6), {})[p[2]] = float(p[3])
tc = np.array(sorted(t for t, v in st.items() if all(k in v for k in ('propagated', 'msckf', 'slam', 'init', 'post'))))
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{D}/replay.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
names = {t.name: t.type for t in r.get_all_topics_and_types()}; tops = ['/openvins/joint_covariance', '/ov_msckf/odomimu']
r.set_filter(rosbag2_py.StorageFilter(topics=tops)); ty = {t: get_message(names[t]) for t in tops}
jc, od = [], []
while r.has_next():
    tp, raw, _ = r.read_next(); m = deserialize_message(raw, ty[tp]); t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    if tp == tops[0]:
        if m.stage in ('', 'post'): jc.append((t, bytes(raw), [int(L.feature_id) for L in m.landmarks]))
    else:
        o, p = m.pose.pose.orientation, m.pose.pose.position; od.append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w))
od = np.array(od); ft = open(f'{O}/tasks.txt', 'w'); fm = open(f'{O}/meta.jsonl', 'w'); n = 0
for k, (t, raw, ids) in enumerate(jc):
    if t < 60: continue                                   # skip init / ground
    key = round(t, 6)
    if key not in st or 'post' not in st[key]: continue
    i = np.searchsorted(tc, t + 1e-6)
    if i >= len(tc) or tc[i] - t > 0.2: continue
    c = tc[i]; j = min(np.searchsorted(od[:, 0], c), len(od) - 1)
    if abs(od[j, 0] - c) > 0.01: continue
    tid = f'c{k}'; open(f'{O}/jc/{tid}.cdr', 'wb').write(raw); x = od[j]
    ft.write(f"TASK {tid} {O}/jc/{tid}.cdr {x[1]} {x[2]} 0 0 0.5 1.5 0.5 0.6\n")
    ft.write(f"WP {c - t:.4f} {x[1]:.5f} {x[2]:.5f} {x[3]:.5f} {x[4]:.7f} {x[5]:.7f} {x[6]:.7f} {x[7]:.7f} 1 {len(ids)} {' '.join(map(str, ids))}\nEND\n")
    s, s0 = st[round(c, 6)], st[key]
    fm.write(json.dumps({'id': tid, 't': t, 'dt': c - t, 'post_k': s0['post'], 'prop': s['propagated'] - s0['post'],
                         'meas': s['propagated'] - s['init'], 'msckf': s['propagated'] - s['msckf'], 'slam': s['msckf'] - s['slam']}) + '\n'); n += 1
print(n, 'cycles')
