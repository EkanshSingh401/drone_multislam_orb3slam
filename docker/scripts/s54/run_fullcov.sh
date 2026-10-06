#!/usr/bin/env bash
# Serial OpenVINS replay + joint-covariance dump + NEES, diagonal AND full (PATCHES s55).
# Runs INSIDE the sim container from /out/diag_s54/.
#   run_fullcov.sh <rundir> <outdir>      (outdir must not exist)
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/ws_offboard_control/install/setup.bash
rd="$1"; od="$2"; D=/out/diag_s54
[[ -e "${od}" ]] && { echo "exists: ${od}" >&2; exit 2; }
mkdir -p "${od}"
python3 "${D}/joint_cov_dump.py" "${od}/jc_imu.txt" --idle 30 > "${od}/jc_dump.log" 2>&1 &
DP=$!
sleep 3
/opt/scripts/replay_estimator.sh --bag "${rd}/flight.bag" --estimator openvins_serial --out "${od}" > "${od}.log" 2>&1
wait "${DP}"; cat "${od}/jc_dump.log"
python3 /opt/scripts/ov_prep.py "${od}" --gt-bag "${rd}/flight.bag" > /dev/null 2>&1
python3 "${D}/full_cov_est.py" "${od}" "${od}/jc_imu.txt"
python3 "${D}/nees_window.py" "${od}" "${rd}/gt.tum" --json | tee "${od}_nees_diag.json"
python3 "${D}/nees_window.py" "${od}" "${rd}/gt.tum" --est est_cov_full.txt | tee "${od}_nees_full.json"
