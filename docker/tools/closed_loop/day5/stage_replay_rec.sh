#!/usr/bin/env bash
# stage_replay_rec.sh <flight_dir> <outdir>: stage_replay.sh + records the replay's JC and odometry.
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; source /out/cl/day5/ovws/install/setup.bash
mkdir -p "$2"
ros2 bag record -s mcap -o "$2/replay.bag" /openvins/joint_covariance /ov_msckf/odomimu /ov_msckf/points_msckf > "$2/rec.log" 2>&1 &
sleep 3
OV_STAGE_LOGDET=1 WS_OV=/out/cl/day5/ovws/install/ov_msckf/lib/ov_msckf /out/cl/day5/replay_estimator_d5.sh --bag "$1/flight.bag" --estimator openvins_serial --out "$2/ov" > "$2/replay.log" 2>&1
sleep 3; pkill -INT -f "$(basename $2)/[r]eplay.bag"; sleep 4
grep '^\[STAGE\]' "$2/ov/openvins.log" > "$2/stages.txt"; wc -l < "$2/stages.txt"
