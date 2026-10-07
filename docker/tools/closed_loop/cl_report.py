#!/usr/bin/env python3
"""cl_report.py <flight_dir> : coverage over time from /octomap_mapper/coverage (PATCHES s64).
Prints known (observed) volume in m^3 at fixed times after the start of exploration, plus totals."""
import sys, json, rosbag2_py, numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray, String
d = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/octomap_mapper/coverage', '/offboard_executor/log']))
cov, t_explore = [], None
while r.has_next():
    t, b, _ = r.read_next()
    if t == '/octomap_mapper/coverage':
        cov.append(list(deserialize_message(b, Float64MultiArray).data))
    else:
        rec = json.loads(deserialize_message(b, String).data)
        if rec['phase'] == 'explore' and t_explore is None: t_explore = rec['t']
c = np.array(cov)
out = {'t_explore_start': t_explore}
for dt in (0, 30, 60, 90, 120, 150, 180):
    k = np.searchsorted(c[:, 0], t_explore + dt)
    if k < len(c): out[f'known_m3@{dt}s'] = round(float(c[k, 1]), 1)
out['known_m3_final'] = round(float(c[-1, 1]), 1); out['occupied_m3_final'] = round(float(c[-1, 3]), 2)
print(json.dumps(out))
