#!/usr/bin/env bash
# Closed-loop step 5 (PATCHES s64): PX4 EKF2 on OpenVINS vision only, live OpenVINS, converter.
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
source /out/build_s60/jazzy/install/setup.bash
P="/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param --instance 1"
cd /tmp
# vision-only EKF2: no GNSS, no mag; EV position+velocity+yaw; height reference = vision
for kv in EKF2_GPS_CTRL=0 EKF2_EV_CTRL=15 EKF2_HGT_REF=3 EKF2_MAG_TYPE=5 EKF2_EV_NOISE_MD=0 EKF2_EV_DELAY=0 \
          EKF2_EV_POS_X=0.11448 EKF2_EV_POS_Y=-0.0051 EKF2_EV_POS_Z=-0.23026 COM_ARM_WO_GPS=1; do
  $P set ${kv%%=*} ${kv#*=} > /dev/null; echo "$(${P} show ${kv%%=*} | grep -E '^ *[x+*]' | head -1)"
done
# OpenVINS config: the generated one + initialize at rest (no jerk wait)
mkdir -p /out/cl/ov_config && cp /opt/config_sim_only/*.yaml /out/cl/ov_config/
grep -q '^init_wait_for_jerk' /out/cl/ov_config/openvins_estimator_config.yaml || echo 'init_wait_for_jerk: 0' >> /out/cl/ov_config/openvins_estimator_config.yaml
nohup /out/build_s60/jazzy/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args -r __ns:=/ov_msckf \
  -p config_path:=/out/cl/ov_config/openvins_estimator_config.yaml -p use_sim_time:=true -p verbosity:=INFO \
  > /out/cl/openvins_live.log 2>&1 &
nohup ros2 run active_slam_planner openvins_to_px4 --ros-args -p enabled:=true -p odom_topic:=/ov_msckf/odomimu \
  -p px4_topic:=/px4_1/fmu/in/vehicle_visual_odometry -p use_sim_time:=true > /out/cl/converter.log 2>&1 &
echo started
