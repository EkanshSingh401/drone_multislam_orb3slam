#!/usr/bin/env bash
# Swap test driver (PATCHES s54), runs INSIDE the sim container.
#   run_swap.sh <run> <variant>     variant: cfg | gz
# Writes a NEW bag and a NEW replay dir; refuses if either already exists.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
r="$1"; v="$2"; rd=/out/eval/20261004-${r}
B=/out/diag_s54/synth/${r}_${v}; od=/out/replay/20261004-${r}_ovser_synth_${v}
[[ -e "${B}" || -e "${od}" ]] && { echo "exists: ${B} or ${od}" >&2; exit 2; }
mkdir -p /out/diag_s54/synth
python3 /out/diag_s54/synth_imu.py "${rd}" "${B}" "${v}" 2>&1 | grep -v rosbag2_
/opt/scripts/replay_estimator.sh --bag "${B}" --estimator openvins_serial --out "${od}" > "${od}.log" 2>&1
grep 'SERIAL\]: done' "${od}.log" || true
python3 /opt/scripts/ov_prep.py "${od}" --gt-bag "${rd}/flight.bag" > /dev/null 2>&1
python3 /opt/scripts/nees_window.py "${od}" "${rd}/gt.tum" --json | tee "${od}_nees.json"
