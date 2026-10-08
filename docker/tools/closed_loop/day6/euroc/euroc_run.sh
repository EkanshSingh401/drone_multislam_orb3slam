#!/usr/bin/env bash
# euroc_run.sh <seq> <track_frequency> <domain>: OpenVINS serial runner on EuRoC with OpenVINS's EuRoC config,
# our tracking/feature/update settings (day-5 sim config keys) and the given track_frequency; joint-cov dump.
set -o pipefail
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
E=/out/euroc; s=$1; f=$2; export ROS_DOMAIN_ID=$3; od=$E/run/${s}_tf$f; [[ -e $od ]] && exit 2; mkdir -p $od/cfg
cp /root/ws_offboard_control/src/open_vins/config/euroc_mav/*.yaml $od/cfg/
OURS=/out/cl/runs/d6_office_frontier_1/ov_config/openvins_estimator_config.yaml
for k in use_klt num_pts fast_threshold grid_x grid_y min_px_dist knn_ratio histogram_method max_clones max_slam max_slam_in_update \
         max_msckf_in_update feat_rep_msckf feat_rep_slam up_msckf_sigma_px up_msckf_chi2_multipler up_slam_sigma_px up_slam_chi2_multipler use_fej integration; do
  v=$(grep -E "^$k:" $OURS | head -1 | sed -E "s/^$k:[ ]*//; s/[ ]*#.*//")
  sed -i -E "s|^$k:.*|$k: $v|" $od/cfg/estimator_config.yaml
  grep -qE "^$k: $v$" $od/cfg/estimator_config.yaml || { echo "failed $k"; exit 3; }
done
sed -i -E "s|^track_frequency:.*|track_frequency: $f|" $od/cfg/estimator_config.yaml
python3 /out/diag_s54/joint_cov_dump.py $od/jc_imu.txt --idle 30 > $od/jc_dump.log 2>&1 &
DP=$!; sleep 3
/root/ws_offboard_control/install/ov_msckf/lib/ov_msckf/ros2_serial_msckf --ros-args -r __ns:=/ov_msckf \
  -p config_path:=$od/cfg/estimator_config.yaml -p path_bag:=$E/seq/$s/ros2 -p verbosity:=DEBUG -p save_total_state:=true \
  -p filepath_est:=$od/ov_state_est.txt -p filepath_std:=$od/ov_state_std.txt -p filepath_gt:=$od/ov_state_gt.txt > $od/openvins.log 2>&1
wait $DP; echo "$s tf$f frames $(grep -c 'seconds for tracking' $od/openvins.log) $(tail -1 $od/jc_dump.log)"
