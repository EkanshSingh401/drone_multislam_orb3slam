#!/usr/bin/env bash
# Phase 3: ORB-SLAM3 replayed on recorded sensor bags (run on the HOST; PATCHES s52).
#
#   PLAY_RATE=0.5 ./docker/scripts/phase3_orbslam3.sh <experiment_dir> [si|st|both]
#
# si = stereo-inertial, st = stereo-only. Per run and mode:
#   /out/replay/<run>_orb_<mode>_r<rate>/   replay output (orb_slam3.log with [ORBEV]
#                                           lines, out.bag), plus
#     est.tum         replayed estimate (/robot_pose_slam, as published)
#     cam_stamps.txt  left-image stamps of the input bag (dropped-frame check)
#     orb_events.json orb_events.py report (init/VIBA/resets, usable time, ATE)
set -eo pipefail

EXP="${1:?experiment dir}"; MODES="${2:-both}"; RATE="${PLAY_RATE:-0.5}"
COMPOSE="docker compose -f docker/compose.yaml"
sim() { ${COMPOSE} exec -T sim "$@"; }
[[ -f "${EXP}/rundirs.txt" ]] || { echo "no ${EXP}/rundirs.txt" >&2; exit 2; }
mapfile -t RUNS < <(grep -oE '/out/eval/[0-9-]+' "${EXP}/rundirs.txt")
case "${MODES}" in both) MODES="st si" ;; esac
ROS="source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash"

for rd in "${RUNS[@]}"; do
    run=$(basename "${rd}")
    sim bash -c "test -s ${rd}/cam_stamps.txt || { ${ROS}; python3 /opt/scripts/cam_stamps.py ${rd}/flight.bag ${rd}/cam_stamps.txt; }"
    for m in ${MODES}; do
        est=$([[ "${m}" == si ]] && echo stereo_inertial || echo stereo)
        od="/out/replay/${run}_orb_${m}_r${RATE}"
        echo "== ${run} ${est} rate ${RATE}"
        sim bash -c "rm -rf ${od}"
        sim env PLAY_RATE="${RATE}" /opt/scripts/replay_estimator.sh \
            --bag "${rd}/flight.bag" --estimator "${est}" --out "${od}" \
            > "docker/out/replay/$(basename "${od}").log" 2>&1 || echo "  replay exit $?"
        sim bash -c "${ROS}; cp ${rd}/cam_stamps.txt ${od}/; \
            python3 /opt/scripts/bag_to_tum.py ${od}/out.bag --pose-topic /robot_pose_slam --out ${od}/est.tum >/dev/null; \
            python3 /opt/scripts/orb_events.py ${od} ${rd}/gt.tum > ${od}/orb_events.json; \
            cat ${od}/orb_events.json" || echo "  post-processing failed"
    done
done
