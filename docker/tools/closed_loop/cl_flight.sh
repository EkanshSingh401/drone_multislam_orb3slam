#!/usr/bin/env bash
# cl_flight.sh <frontier|ig> <outdir> : one closed-loop exploration flight in the validation scene (PATCHES s64)
set -o pipefail
TYPE=$1; OD=$2; [[ -e $OD ]] && { echo "exists $OD" >&2; exit 2; }; mkdir -p $OD
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
/opt/scripts/bringup_sim.sh --stop > $OD/stop.log 2>&1
pkill -f '[r]un_subscribe_msckf' ; pkill -f '[o]penvins_to_px4'; pkill -f '[o]ctomap_mapper'; pkill -f '[e]xploration_planner'; sleep 2
MODEL=gz_x500_d455 D455_DEPTH=1 WORLD=validation START_SLAM=0 SLAM_MODE=stereo_inertial /opt/scripts/bringup_sim.sh > $OD/bringup.log 2>&1 \
  || { echo "bringup failed"; tail -5 $OD/bringup.log; exit 3; }
P="/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param --instance 1"
(cd /tmp; for kv in EKF2_GPS_CTRL=0 EKF2_EV_CTRL=15 EKF2_HGT_REF=3 EKF2_MAG_TYPE=5 EKF2_EV_NOISE_MD=0 EKF2_EV_DELAY=0 \
  EKF2_EV_POS_X=0.11448 EKF2_EV_POS_Y=-0.0051 EKF2_EV_POS_Z=-0.23026 COM_ARM_WO_GPS=1; do $P set ${kv%%=*} ${kv#*=} >/dev/null; done)
mkdir -p $OD/ov_config && cp /opt/config_sim_only/*.yaml $OD/ov_config/ && echo 'init_wait_for_jerk: 0' >> $OD/ov_config/openvins_estimator_config.yaml
ros2 bag record -s mcap -o $OD/flight.bag /ground_truth/pose_info /clock /ov_msckf/odomimu /openvins/joint_covariance \
  /exploration_planner/decision /exploration_planner/goal /octomap_mapper/coverage /offboard_executor/log \
  /px4_1/fmu/out/vehicle_local_position_v1 /px4_1/fmu/out/vehicle_status_v1 > $OD/record.log 2>&1 &
sleep 3
/root/ws_offboard_control/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args -r __ns:=/ov_msckf \
  -p config_path:=$OD/ov_config/openvins_estimator_config.yaml -p use_sim_time:=true -p verbosity:=INFO \
  -p save_total_state:=true -p filepath_est:=$OD/ov_state_est.txt -p filepath_std:=$OD/ov_state_std.txt -p filepath_gt:=$OD/ov_state_gt.txt \
  > $OD/openvins.log 2>&1 &
ros2 run active_slam_planner openvins_to_px4 --ros-args -p enabled:=true -p odom_topic:=/ov_msckf/odomimu \
  -p px4_topic:=/px4_1/fmu/in/vehicle_visual_odometry -p use_sim_time:=true > $OD/converter.log 2>&1 &
timeout 8 ros2 topic list 2>/dev/null | grep -i land > $OD/land_topics.txt
sleep 8
/root/ws_offboard_control/install/active_slam_sim/lib/active_slam_sim/octomap_mapper --ros-args -p use_sim_time:=true -p slice_height:=1.5 -p max_range:=8.0 > $OD/mapper.log 2>&1 &
/root/ws_offboard_control/install/active_slam_sim/lib/active_slam_sim/exploration_planner --ros-args -p use_sim_time:=true -p planner_type:=$TYPE -p flight_height:=1.5 > $OD/planner.log 2>&1 &
python3 /out/cl/gt_watchdog.py > $OD/watchdog.log 2>&1 &
sleep 2
timeout 900 python3 /root/ws_offboard_control/install/active_slam_sim/lib/active_slam_sim/offboard_executor.py --ros-args -p flight_height:=1.5 -p max_mission_s:=180.0 > $OD/executor.log 2>&1
echo "executor rc=$?"
sleep 3
pkill -TERM -f "record -s mcap -o $OD/[f]light.bag"; sleep 3
pkill -f '[e]xploration_planner'; pkill -f '[o]ctomap_mapper'; pkill -f '[o]penvins_to_px4'; pkill -f '[r]un_subscribe_msckf'; pkill -f '[g]t_watchdog'
cat $OD/watchdog.log | grep TERMINATED
/opt/scripts/bringup_sim.sh --stop > $OD/stop2.log 2>&1
grep -E 'phase|explore ends|landed|failed|timed out' $OD/executor.log | tail -12
