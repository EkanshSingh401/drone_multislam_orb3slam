#!/usr/bin/env bash
# stage_replay.sh <flight_dir> <outdir> [extra env passed through]: serial OpenVINS replay with the
# day-5 stage-logdet patch (scratch build), log lines '[STAGE] t stage ld6 ld3 ld2 n nslam nclones'.
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; source /out/cl/day5/ovws/install/setup.bash
OV_STAGE_LOGDET=1 WS_OV=/out/cl/day5/ovws/install/ov_msckf/lib/ov_msckf /out/cl/day5/replay_estimator_d5.sh --bag "$1/flight.bag" --estimator openvins_serial --out "$2" > "$2.log" 2>&1
grep '^\[STAGE\]' "$2/openvins.log" > "$2/stages.txt"; wc -l < "$2/stages.txt"
