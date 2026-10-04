#!/usr/bin/env bash
# Window-for-window comparison of two estimators on the SAME bag (run in sim).
#
#   /opt/scripts/eval_window_matched.sh --orb /out/replay/<run>_stereo_inertial \
#                                       --ov  /out/replay/<run>_openvins
#
# Both estimators are evaluated over ONE time window, derived once, with the
# same alignment and the same association tolerance. Anything else is not a
# comparison:
#
#   * The window is ORB-SLAM3's converged window -- from VIBA 2 completion to
#     the end -- because that is the only window in which ORB-SLAM3's published
#     stream is free of retroactive corrections (PATCHES.md s34, s39).
#   * OpenVINS initialises EARLIER and has no equivalent transient, so letting
#     it use its own full window would compare a filter's steady state against a
#     smoother's post-transient segment over different amounts of flight. The
#     window is therefore taken from the ORB-SLAM3 run and IMPOSED on OpenVINS.
#   * The window start comes from the clock beacons via viba_window.py, measured,
#     not extrapolated from the real-time factor.
#
# Also reports the largest online pose discontinuity for each, split at the same
# instant. That is where the two are expected to differ most: OpenVINS performs
# no retroactive global correction, so its stream should show no step, while
# ORB-SLAM3's showed up to 8.4 m (25.9 m on one run) right before VIBA 2.
set -eo pipefail

ORB=""; OV=""; TOL="0.05"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --orb) ORB="$2"; shift 2 ;;
        --ov)  OV="$2"; shift 2 ;;
        --tol) TOL="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ -n "${ORB}" && -n "${OV}" ]] || { echo "need --orb and --ov" >&2; exit 2; }
source /opt/ros/"${ROS_DISTRO}"/setup.bash >/dev/null 2>&1 || true

# ---- 1. derive the window ONCE, from the ORB-SLAM3 run ---------------------
LOG="${ORB}/orb_slam3.log"
[[ -f "${LOG}" ]] || { echo "no ${LOG}" >&2; exit 3; }
T_START=$(python3 /opt/scripts/viba_window.py "${LOG}" --emit-t-start || true)
if [[ -z "${T_START}" ]]; then
    echo "eval_window_matched: cannot locate VIBA 2 on the simulation timeline." >&2
    echo "  The ORB-SLAM3 log has no clock beacons, so there is no window to" >&2
    echo "  impose. Re-run with the current build; do NOT substitute a guess --" >&2
    echo "  an unmatched window would silently favour one estimator." >&2
    exit 4
fi
echo "============================================================"
echo "WINDOW-MATCHED COMPARISON"
echo "  window start : sim t = ${T_START} s  (ORB-SLAM3 VIBA 2 completion)"
echo "  alignment    : SE(3) Umeyama, scale NOT fitted (both are metric)"
echo "  association  : nearest neighbour within ${TOL} s"
echo "============================================================"

# ---- 2. evaluate both over that window -------------------------------------
one() {  # one <label> <dir> <est-file>
    local label="$1" dir="$2" est="$3"
    if [[ ! -s "${dir}/${est}" ]]; then
        printf "  %-16s MISSING (%s)\n" "${label}" "${dir}/${est}"
        return
    fi
    rm -f "${dir}/ape_matched.zip" "${dir}/rpe_matched.zip"
    evo_ape tum "${dir}/gt.tum" "${dir}/${est}" \
        -a --t_max_diff "${TOL}" --t_start "${T_START}" \
        --save_results "${dir}/ape_matched.zip" > "${dir}/ape_matched.txt" 2>&1 || true
    evo_rpe tum "${dir}/gt.tum" "${dir}/${est}" \
        -a --t_max_diff "${TOL}" --delta 1 --delta_unit m --t_start "${T_START}" \
        --save_results "${dir}/rpe_matched.zip" > "${dir}/rpe_matched.txt" 2>&1 || true
    local A R D
    A=$(grep -E "^\s+rmse" "${dir}/ape_matched.txt" | head -1 | awk '{print $2}')
    R=$(grep -E "^\s+rmse" "${dir}/rpe_matched.txt" | head -1 | awk '{print $2}')
    D=$(python3 /opt/scripts/pose_discontinuity.py "${dir}" --est "${est}" \
            --split-at "${T_START}" --json 2>/dev/null \
        | python3 -c 'import json,sys; d=json.load(sys.stdin); print(f"{d["overall"]["max_m"]:.4f} {d["converged"]["max_m"]:.4f}")' \
        2>/dev/null || echo "nan nan")
    printf "  %-16s ATE %-10s RPE %-10s  max-step(all) %-9s max-step(window) %s\n" \
        "${label}" "${A:-nan}" "${R:-nan}" "${D%% *}" "${D##* }"
}

echo
printf "  %-16s %-14s %-14s %-23s %s\n" ESTIMATOR "ATE rmse (m)" "RPE rmse (m)" "worst online jump (m)" ""
one "ORB-SLAM3 SI" "${ORB}" "est_orbslam3.tum"
one "OpenVINS"     "${OV}"  "est_openvins.tum"
echo
echo "  Both rows use the SAME window, alignment and tolerance."
echo "  'max-step(all)' spans the whole stream including start-up, which is what"
echo "  a planner actually consumes; 'max-step(window)' is within the matched"
echo "  window only. A filter with no retroactive correction should show no"
echo "  meaningful difference between the two columns."
echo "============================================================"
