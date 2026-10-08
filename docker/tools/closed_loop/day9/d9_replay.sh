#!/usr/bin/env bash
# d7_replay.sh <flight> <track_frequency> <domain> (day 7 steps 1-3): serial OpenVINS replay of a sensor flight
# with the scratch build (stage logdet + NIS + tracked-feature logging), joint-cov dump, full-cov NEES dump,
# and a drift-ready directory (links to the flight's GT and logs).
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; source /out/cl/day5/ovws/install/setup.bash
f=$1; tf=$2; export ROS_DOMAIN_ID=$3; rd=/out/cl/runs/$f; od=/out/cl/day9/rep/${f}_tf$tf
[[ -e $od ]] && exit 2; mkdir -p $od
python3 /out/diag_s54/joint_cov_dump.py $od/jc_imu.txt --idle 30 > $od/jc_dump.log 2>&1 &
DP=$!; sleep 3
OV_APPEND="init_wait_for_jerk: 0" OV_SET="track_frequency=$tf" OV_STAGE_LOGDET=1 OV_NIS_LOG=1 OV_TRK_LOG=1 WS_OV=/out/cl/day5/ovws/install/ov_msckf/lib/ov_msckf \
  /out/cl/day5/replay_estimator_d5.sh --bag $rd/flight.bag --estimator openvins_serial --out $od/ov > $od/replay.log 2>&1
wait $DP
python3 /opt/scripts/ov_prep.py $od/ov --gt-bag $rd/flight.bag > $od/prep.log 2>&1
python3 /out/diag_s54/full_cov_est.py $od/ov $od/jc_imu.txt > $od/fullcov.log 2>&1
python3 /out/cl/day4/nees_yaw.py $od/ov $rd/gt.tum --est est_cov_full.txt --dump $od/nees_dump.txt > $od/nees.json 2>&1
for x in executor.log bringup.log gt_imu.txt gt.tum flight.bag; do ln -sf $rd/$x $od/ov/$x; done
echo "$f tf$tf $(grep -c '^\[NIS\]' $od/ov/openvins.log) NIS, $(grep -c '^\[TRK\]' $od/ov/openvins.log) TRK; $(cat $od/nees.json | head -c 200)"
