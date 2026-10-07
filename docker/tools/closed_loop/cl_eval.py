#!/usr/bin/env python3
"""cl_eval.py <flight_dir> : GT + OpenVINS trajectories, path extent, divergence time (PATCHES s64)."""
import sys, json, subprocess, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
d=sys.argv[1]
subprocess.run(['python3','/opt/scripts/bag_to_tum.py',f'{d}/flight.bag','--tf-topic','/ground_truth/pose_info','--tf-index','0','--time-from-clock','/clock','--out',f'{d}/gt.tum'],capture_output=True)
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=f'{d}/flight.bag',storage_id=''),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/ov_msckf/odomimu'])); rows=[]
while r.has_next():
    _,b,_=r.read_next(); m=deserialize_message(b,Odometry); p=m.pose.pose.position; q=m.pose.pose.orientation
    rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
a=np.array(rows); a=a[np.r_[True,np.diff(a[:,0])>0]]
P=np.array([0.12-0.00552,0.0051,0.242-0.01174]); a[:,1:4]-=R.from_quat(a[:,4:8]).apply(P)
np.savetxt(f'{d}/ov_base.tum',a,fmt='%.9f')
g=np.loadtxt(f'{d}/gt.tum')
print(json.dumps({'gt_x':[round(g[:,1].min(),2),round(g[:,1].max(),2)],'gt_y':[round(g[:,2].min(),2),round(g[:,2].max(),2)],'gt_zmax':round(g[:,3].max(),2),
  'gt_path_m':round(float(np.sum(np.linalg.norm(np.diff(g[:,1:4],axis=0),axis=1))),1),'ov_poses':len(a),'ov_max_abs':round(float(np.abs(a[:,1:4]).max()),1)}))
