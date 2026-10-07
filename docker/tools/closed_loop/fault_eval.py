#!/usr/bin/env python3
"""fault_eval.py <flight_dir> : PX4 response to an injected VIO fault + landing timeline (overnight Stage 1b/1c)."""
import sys, re, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, EstimatorStatusFlags, FailsafeFlags, VehicleStatus, VehicleLandDetected
d = sys.argv[1]
fl = open(f'{d}/fault.log').read()
fs = re.search(r'FAULT \w+ start t=([0-9.]+)', fl); fe = re.search(r'FAULT \w+ end t=([0-9.]+)', fl)
T = {'/ov_msckf/odomimu': Odometry, '/ov_msckf/odomimu_px4': Odometry, '/px4_1/fmu/out/vehicle_local_position_v1': VehicleLocalPosition,
     '/px4_1/fmu/out/estimator_status_flags': EstimatorStatusFlags, '/px4_1/fmu/out/failsafe_flags': FailsafeFlags,
     '/px4_1/fmu/out/vehicle_status_v1': VehicleStatus, '/px4_1/fmu/out/vehicle_land_detected': VehicleLandDetected}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=list(T)))
ov, lp, ev, ff, vs, ld = [], [], [], [], [], []
while r.has_next():
    t, b, tr = r.read_next(); m = deserialize_message(b, T[t]); tr *= 1e-9
    if t == '/ov_msckf/odomimu': ov.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, tr, m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    elif t.endswith('local_position_v1'): lp.append((tr, m.y, m.x, -m.z, m.xy_valid))   # NED -> OpenVINS ENU-like
    elif t.endswith('status_flags'): ev.append((tr, m.cs_ev_pos, m.cs_ev_vel, m.cs_ev_hgt, m.cs_ev_yaw))
    elif t.endswith('failsafe_flags'): ff.append((tr, m.local_position_invalid, m.local_velocity_invalid, m.position_accuracy_low))
    elif t.endswith('vehicle_status_v1'): vs.append((tr, m.nav_state, m.failsafe, m.arming_state))
    elif t.endswith('land_detected'): ld.append((tr, m.landed, m.maybe_landed, m.ground_contact))
ov, lp = np.array(ov), np.array(lp)
# map sim stamp -> receive time with the OpenVINS stream
def recv_of(ts): return np.interp(ts, ov[:, 0], ov[:, 1])
def at(arr, tr, cols): k = np.argmin(np.abs(arr[:, 0] - tr)); return arr[k, cols]
if fs:
    a = recv_of(float(fs.group(1))); b = recv_of(float(fe.group(1))) if fe else a + 6
    print(f"fault window recv [{a:.1f}, {b:.1f}] s")
    for tt in np.arange(a - 2, b + 8, 1.0):
        o = ov[np.argmin(np.abs(ov[:, 1] - tt)), 2:5]; p = at(lp, tt, [1, 2, 3])
        evf = [x for x in ev if abs(x[0] - tt) < 0.6]; e = evf[-1][1:] if evf else None
        ffs = [x for x in ff if x[0] <= tt]; f = ffs[-1][1:] if ffs else None
        v = [x for x in vs if x[0] <= tt][-1][1:3]
        print(f"  {tt - a:+5.1f}s true(OV) ({o[0]:6.2f},{o[1]:6.2f},{o[2]:5.2f})  PX4 ({p[0]:6.2f},{p[1]:6.2f},{p[2]:5.2f})  "
              f"|PX4-true| {np.linalg.norm(p - o):5.2f}  ev_pos/vel/hgt/yaw {e}  fs(lpos_inv,lvel_inv,acc_low) {f}  nav {v[0]} failsafe {v[1]}")
# landing
land = [x for x in vs if True]
arm = [(x[0], x[3]) for x in vs]; dis = [t for (t, a2), (t0, a1) in zip(arm[1:], arm[:-1]) if a1 == 2 and a2 != 2]
print('disarm events (recv s):', [round(x, 1) for x in dis])
for t, l, mb, gc in ld:
    pass
chg, prev = [], None
for t, l, mb, gc in ld:
    if (l, mb, gc) != prev: chg.append((round(t, 2), l, mb, gc)); prev = (l, mb, gc)
print('land_detected changes (recv s, landed, maybe, contact):', chg[-6:])
