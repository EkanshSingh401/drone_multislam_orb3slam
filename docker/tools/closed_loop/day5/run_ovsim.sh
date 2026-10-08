#!/usr/bin/env bash
# run_ovsim.sh <traj.txt> <outdir> [seed]: OpenVINS's own simulator on a flight-derived trajectory, our
# estimator config + rig calibration + IMU noise (config of d4_sens_lam0_1), ideal features 2-6 m away,
# camera 15 Hz (our effective tracking rate), IMU 200 Hz, no perturbation; joint-covariance dump.
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
tr=$1; od=$2; seed=${3:-0}; [[ -e $od ]] && exit 2; mkdir -p $od/cfg
cp /out/cl/runs/d4_sens_lam0_1/ov_config/*.yaml $od/cfg/
cat >> $od/cfg/openvins_estimator_config.yaml <<EOC
sim_seed_state_init: 0
sim_seed_preturb: 0
sim_seed_measurements: $seed
sim_do_perturbation: false
sim_traj_path: "$tr"
sim_distance_threshold: 0.3
sim_freq_cam: 15
sim_freq_imu: 200
sim_min_feature_gen_dist: 2.0
sim_max_feature_gen_dist: 6.0
EOC
python3 /out/diag_s54/joint_cov_dump.py $od/jc_imu.txt --idle 30 > $od/jc_dump.log 2>&1 &
DP=$!; sleep 3
timeout 3000 /root/ws_offboard_control/install/ov_msckf/lib/ov_msckf/run_simulation --ros-args -r __ns:=/ov_msckf \
  -p config_path:=$od/cfg/openvins_estimator_config.yaml -p verbosity:=INFO -p save_total_state:=true \
  -p filepath_est:=$od/ov_state_est.txt -p filepath_std:=$od/ov_state_std.txt -p filepath_gt:=$od/ov_state_gt.txt > $od/openvins.log 2>&1
wait $DP; tail -1 $od/jc_dump.log
