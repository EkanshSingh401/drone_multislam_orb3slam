#!/usr/bin/env python3
"""flight_summary.py <flight_dir> <world> : one JSON line with every Stage-2 metric (overnight protocol).
Runs inside the sim container. Requires cl_flight.sh outputs."""
import sys, json, subprocess, numpy as np, rosbag2_py, os
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
d, world = sys.argv[1], sys.argv[2]
def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True); return p.stdout
out = {'flight': os.path.basename(d), 'world': world}
ev = json.loads([l for l in run(['python3', '/out/cl/cl_eval.py', d]).splitlines() if l.startswith('{')][-1])
out['path_m'] = ev['gt_path_m']
g = json.loads([l for l in run(['python3', '/opt/scripts/gt_window_eval.py', f'{d}/gt.tum', f'ov={d}/ov_base.tum']).splitlines() if l.startswith('{')][-1])
out['ate_m'] = round(g['ov'].get('ate', float('nan')), 4); out['ate_max_m'] = round(g['ov'].get('ate_max', float('nan')), 4)
out['sim3'] = round(g['ov'].get('sim3_scale', float('nan')), 4)
cov = json.loads([l for l in run(['python3', '/out/cl/cl_report.py', d]).splitlines() if l.startswith('{')][-1])
out.update({k: v for k, v in cov.items() if k.startswith('known')})
cl = run(['python3', '/out/cl/clearance.py', f'{d}/gt.tum', world]) if world == 'validation' else ''
out['min_clearance_m'] = float(cl.split('min clearance ')[1].split(' m')[0]) if 'min clearance' in cl else None
out['contact'] = ('CONTACT' in cl) if cl else None
ne = json.loads([l for l in run(['python3', '/out/cl/cl_nees.py', d]).splitlines() if l.startswith('{')][-1])
out['nees_full_ori'] = ne['full_sub']['ori']; out['nees_full_rp'] = ne['full_sub']['rp']; out['nees_full_pos'] = ne['full_sub']['pos']
out['nees_diag_ori'] = ne['diag_all']['ori']
wd = open(f'{d}/watchdog.log').read(); out['watchdog_terminated'] = 'TERMINATED' in wd
ex = open(f'{d}/executor.log').read()
out['arm_failed'] = 'failed to arm' in ex; out['guard_holds'] = ex.count('ESDF guard')
out['landed_ok'] = 'landed and disarmed' in ex; out['touchdown_by'] = 'px4' if 'by=px4' in ex else ('openvins' if 'by=openvins' in ex else None)
out['gate_open'] = 'GATE OPEN' in open(f'{d}/gate.log').read() if os.path.exists(f'{d}/gate.log') else None
# planner decisions: number, chosen-candidate real/virtual gain shares
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag', storage_id=''), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/exploration_planner/decision'])); shares, ng, dip, cvg = [], 0, [], []
while r.has_next():
    _, b, _ = r.read_next(); m = json.loads(deserialize_message(b, String).data)
    if m['chosen'] < 0: continue
    ng += 1; c = m['candidates'][m['chosen']]
    if c.get('dI_pose') is not None: dip.append(c['dI_pose'])
    if c.get('coverage_gain') is not None: cvg.append(c['coverage_gain'])
    dr, dv = c.get('delta_real'), c.get('delta_virt')
    if dr is not None and dv is not None and dr + dv > 0: shares.append(dr / (dr + dv))
out['goals'] = ng
out['chosen_dI_pose_median'] = round(float(np.median(dip)), 3) if dip else None
out['chosen_coverage_gain_median'] = round(float(np.median(cvg)), 2) if cvg else None
out['real_share_median'] = round(float(np.median(shares)), 3) if shares else None
out['real_share_iqr'] = [round(float(np.percentile(shares, 25)), 3), round(float(np.percentile(shares, 75)), 3)] if shares else None
kf = out.get('known_m3_final'); out['known_m3_per_m'] = round(kf / out['path_m'], 2) if kf and out['path_m'] else None
out['ate_per_m'] = round(out['ate_m'] / out['path_m'], 5) if out['path_m'] else None
print(json.dumps(out))
