#!/usr/bin/env python3
"""arm_flags.py <flight_dir>: PX4 status / failsafe flags during the arm phase (day 6 step 5)."""
import sys, json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
d = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
names = {t.name: t.type for t in r.get_all_topics_and_types()}
tops = [t for t in ['/px4_1/fmu/out/vehicle_status_v1', '/px4_1/fmu/out/failsafe_flags', '/px4_1/fmu/out/estimator_status_flags'] if t in names]
r.set_filter(rosbag2_py.StorageFilter(topics=tops)); T = {t: get_message(names[t]) for t in tops}
last = {}; n = 0
while r.has_next() and n < 60:
    tp, b, _ = r.read_next(); m = deserialize_message(b, T[tp])
    if tp.endswith('vehicle_status_v1'):
        s = f"arming {m.arming_state} nav {m.nav_state} preflight_ok {m.pre_flight_checks_pass} failsafe {m.failsafe}"
    else:
        s = ' '.join(k for k in m.get_fields_and_field_types() if isinstance(getattr(m, k), bool) and getattr(m, k))
    if last.get(tp) != s:
        print(f"{m.timestamp * 1e-6:.1f} {tp.split('/')[-1][:22]}: {s}"); last[tp] = s; n += 1
