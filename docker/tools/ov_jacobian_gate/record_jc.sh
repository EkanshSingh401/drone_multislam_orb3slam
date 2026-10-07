#!/usr/bin/env bash
# record_jc.sh <rundir> <outdir>: serial OpenVINS (s60 build) + record /openvins/joint_covariance
set -eo pipefail
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
source /out/build_s60/jazzy/install/setup.bash
rd=$1; od=$2; [[ -e $od ]] && { echo "exists $od" >&2; exit 2; }; mkdir -p $od
echo "ov_msckf from: $(ros2 pkg prefix ov_msckf)"
python3 /opt/scripts/check_no_publishers.py
ros2 bag record -s mcap -o $od/jc.bag /openvins/joint_covariance > $od/record.log 2>&1 &
RP=$!; sleep 4
/out/build_s60/jazzy/install/ov_msckf/lib/ov_msckf/ros2_serial_msckf --ros-args -r __ns:=/ov_msckf \
  -p config_path:=/opt/config_sim_only/openvins_estimator_config.yaml -p path_bag:=$rd/flight.bag \
  -p verbosity:=INFO > $od/openvins.log 2>&1 || true
sleep 3; pkill -TERM -f "[b]ag record -s mcap -o $od/jc.bag" || true; sleep 2
grep "SERIAL.: done" $od/openvins.log
ros2 bag info $od/jc.bag | grep -E 'Count|joint'
