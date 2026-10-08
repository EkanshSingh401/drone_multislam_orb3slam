#!/usr/bin/env python3
"""land_flags.py <flight_dir>: PX4 land detector + local position (z, vz, z_valid) after the land command."""
import sys, json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
d = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
names = {t.name: t.type for t in r.get_all_topics_and_types()}
tops = ['/px4_1/fmu/out/vehicle_land_detected', '/px4_1/fmu/out/vehicle_local_position_v1', '/px4_1/fmu/out/vehicle_status_v1']
r.set_filter(rosbag2_py.StorageFilter(topics=tops)); T = {t: get_message(names[t]) for t in tops}
nav18 = None; last = None; lp = None
while r.has_next():
    tp, b, _ = r.read_next(); m = deserialize_message(b, T[tp]); t = m.timestamp * 1e-6
    if tp.endswith('status_v1'):
        if m.nav_state == 18 and nav18 is None: nav18 = t; print(f'{t:.1f} AUTO_LAND')
        if nav18 and m.arming_state == 1 and last != 'disarmed': print(f'{t:.1f} DISARMED'); last = 'disarmed'
    elif tp.endswith('local_position_v1'):
        lp = (round(m.z, 2), round(m.vz, 2), m.z_valid, m.v_z_valid)
    elif nav18:
        s = f"landed {m.landed} maybe {m.maybe_landed} gc {m.ground_contact} freefall {m.freefall} lpos(z,vz,zv,vzv) {lp}"
        if s.split(' lpos')[0] != (last or '').split(' lpos')[0] and t < nav18 + 40: print(f'{t:.1f} {s}'); last = s
