#!/usr/bin/env bash
# nees_replay.sh <flight_dir> <outdir>  (day 5 step 4b): serial OpenVINS replay (stock binary; env
# OV_SET / OV_IMU_RW etc. pass through to replay_estimator for variants) + joint-covariance dump ->
# full-covariance roll/pitch/yaw NEES per sample (day4/nees_yaw.py --dump). Run with ROS_DOMAIN_ID=77.
set -o pipefail
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash
rd=$1; od=$2; [[ -e $od ]] && { echo exists; exit 2; }; mkdir -p $od
python3 /out/diag_s54/joint_cov_dump.py $od/jc_imu.txt --idle 30 > $od/jc_dump.log 2>&1 &
DP=$!; sleep 3
/opt/scripts/replay_estimator.sh --bag $rd/flight.bag --estimator openvins_serial --out $od > $od.log 2>&1
wait $DP
python3 /opt/scripts/ov_prep.py $od --gt-bag $rd/flight.bag > $od/prep.log 2>&1
python3 /out/diag_s54/full_cov_est.py $od $od/jc_imu.txt > $od/fullcov.log 2>&1
python3 /out/cl/day4/nees_yaw.py $od $rd/gt.tum --est est_cov_full.txt --dump $od/nees_dump.txt
