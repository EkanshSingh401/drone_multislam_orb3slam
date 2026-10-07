#!/usr/bin/env bash
# run_decomp.sh <rundir> <outdir> <scene> : serial OpenVINS (DEBUG, stage snapshots) + ig_decomposition (PATCHES s62)
set -eo pipefail
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
source /out/build_s60/jazzy/install/setup.bash
rd=$1; od=$2; [[ -e $od ]] && { echo "exists $od" >&2; exit 2; }; mkdir -p $od
python3 /opt/scripts/check_no_publishers.py > /dev/null
ros2 bag record -s mcap -o $od/jc.bag /openvins/joint_covariance > $od/record.log 2>&1 &
sleep 4
/out/build_s60/jazzy/install/ov_msckf/lib/ov_msckf/ros2_serial_msckf --ros-args -r __ns:=/ov_msckf \
  -p config_path:=/opt/config_sim_only/openvins_estimator_config.yaml -p path_bag:=$rd/flight.bag \
  -p verbosity:=DEBUG -p joint_cov_rate:=0.0 -p joint_cov_stages:=true -p save_total_state:=true \
  -p filepath_est:=$od/ov_state_est.txt -p filepath_std:=$od/ov_state_std.txt -p filepath_gt:=$od/ov_state_gt.txt \
  > $od/openvins.log 2>&1 || true
sleep 5; pkill -TERM -f "[b]ag record -s mcap -o $od/jc.bag" || true; sleep 3
n_rec=$(ros2 bag info $od/jc.bag | grep -o 'Count: [0-9]*' | head -1); n_st=$(grep -vc '^#' $od/ov_state_est.txt)
echo "recorded joint cov: $n_rec ; OpenVINS updates (state rows): $n_st"
read w0 w1 < <(python3 -c "
import numpy as np; g=np.loadtxt('$rd/gt.tum'); t,z=g[:,0],g[:,3]; h=z-np.median(z[:max(5,len(z)//50)])
up=np.where(h>0.5)[0]; to=t[np.argmax(h>0.05)]; td=t[up[-1]+np.argmax(h[up[-1]:]<0.05)]; print(to+3.0, td)")
python3 -c "
import rosbag2_py, sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
p='$rd/flight.bag'; i=rosbag2_py.Info().read_metadata(p,'')
r=rosbag2_py.SequentialCompressionReader() if (i.compression_mode or '').upper()=='FILE' else rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=p,storage_id=''),rosbag2_py.ConverterOptions('','')); r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/imu']))
with open('$od/imu.txt','w') as f:
  while r.has_next():
    _,d,_=r.read_next(); m=deserialize_message(d,Imu); a,w=m.linear_acceleration,m.angular_velocity
    f.write('%.9f %.9g %.9g %.9g %.9g %.9g %.9g\\n'%(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,w.x,w.y,w.z,a.x,a.y,a.z))
" 2>&1 | grep -v rosbag2_ || true
/out/build_s60/jazzy/install/ig_prediction_validation/lib/ig_prediction_validation/ig_decomposition \
  $od/jc.bag $od/imu.txt $od/ov_state_est.txt $od/openvins.log $rd/gt.tum 1.0 1.0 $w0 $w1 $3 $od/records.jsonl /out/diag_s63/usage_model.txt > $od/segments.jsonl
echo "segments: $(wc -l < $od/segments.jsonl)"
