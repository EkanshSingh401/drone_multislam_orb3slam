#!/usr/bin/env bash
# Phase 3: OpenVINS on recorded sensor bags by deterministic replay (run on the HOST).
#
#   ./docker/scripts/phase3_openvins.sh determinism <experiment_dir>
#   ./docker/scripts/phase3_openvins.sh replay      <experiment_dir> [gravity_mag]
#   ./docker/scripts/phase3_openvins.sh evaluate    <experiment_dir> [gravity_mag]
#   ./docker/scripts/phase3_openvins.sh jointcov    <experiment_dir>
#   ./docker/scripts/phase3_openvins.sh topicspread <experiment_dir> [n=3]
#
# Every mode except topicspread uses ros2_serial_msckf (estimator
# openvins_serial), the deterministic bag-reading runner (PATCHES.md s47).
# topicspread replays ONE bag n times through the live topic node
# (run_subscribe_msckf) to measure the run-to-run spread to expect on hardware.
#
# <experiment_dir> is a run_experiment.sh output recorded with RECORD_SENSORS=1
# and RIG=d455, e.g. docker/out/experiment_d455_pathA_<stamp>. Its rundirs.txt
# names the live runs; each live run's flight.bag is the replay input and its
# orb_slam3.log (live stereo-inertial, same data) supplies the converged window.
#
# Outputs go to /out/replay/<run>_ovser_g<gravity>[_<tag>]/ (serial runner) or
# /out/replay/<run>_ovtopic_g<gravity>_<tag>/ (topic replay).
#
# determinism  replays the FIRST bag twice at the default gravity and requires
#              the OpenVINS outputs to be byte-identical. Nothing else in phase 3
#              means anything if this fails, so it exits non-zero on a mismatch.
# replay       replays every bag once at the given gravity (default 9.81).
# evaluate     per replay: window start from the live ORB-SLAM3 log, ov_prep,
#              eval_window_matched.sh, NEES via ov_eval posyaw on that window.
# jointcov     a THIRD replay of the first bag with check_joint_cov.py sampling
#              /openvins/joint_covariance -- separate from the determinism pair so
#              the extra subscriber cannot perturb it.
set -eo pipefail

MODE="${1:?mode}"; EXP="${2:?experiment dir}"; G="${3:-9.81}"
COMPOSE="docker compose -f docker/compose.yaml"
[[ -f "${EXP}/rundirs.txt" ]] || { echo "no ${EXP}/rundirs.txt" >&2; exit 2; }
mapfile -t RUNS < <(grep -oE '/out/eval/[0-9-]+' "${EXP}/rundirs.txt")
(( ${#RUNS[@]} > 0 )) || { echo "no runs listed in ${EXP}/rundirs.txt" >&2; exit 2; }
sim() { ${COMPOSE} exec -T sim "$@"; }

replay_one() {  # replay_one <rundir> <gravity> <outdir> [estimator]
    local rd="$1" g="$2" od="$3" est="${4:-openvins_serial}"
    sim bash -c "test -f ${rd}/flight.bag/metadata.yaml" \
        || { echo "  ${rd}: no complete flight.bag, skipping"; return 1; }
    sim bash -c "rm -rf ${od}"
    sim env OV_GRAVITY_MAG="${g}" /opt/scripts/replay_estimator.sh \
        --bag "${rd}/flight.bag" --estimator "${est}" --out "${od}" \
        > "docker/out/replay/$(basename "${od}").log" 2>&1
    sim cat "${od}/frames.txt" | grep -E 'published_infra1|processed' | tr '\n' ' '; echo
}

outdir()  { echo "/out/replay/$(basename "$1")_ovser_g${2}${3:+_$3}"; }
outdirT() { echo "/out/replay/$(basename "$1")_ovtopic_g${2}_$3"; }
mkdir -p docker/out/replay

case "${MODE}" in
determinism)
    rd="${RUNS[0]}"
    for tag in det1 det2; do
        echo "== ${tag}: $(basename "${rd}") g=${G}"
        replay_one "${rd}" "${G}" "$(outdir "${rd}" "${G}" "${tag}")"
    done
    a="$(outdir "${rd}" "${G}" det1)"; b="$(outdir "${rd}" "${G}" det2)"
    echo "== comparing OpenVINS outputs byte for byte"
    ok=1
    # PASS/FAIL on OpenVINS's own per-update state (save_total_state), which is
    # what phase 3 evaluates, plus the frame accounting.
    for f in ov_state_est.txt ov_state_std.txt; do
        if sim cmp -s "${a}/${f}" "${b}/${f}"; then
            echo "  IDENTICAL  ${f}  ($(sim bash -c "wc -l < ${a}/${f}") lines)"
        else
            echo "  DIFFERENT  ${f}"; ok=0
            sim bash -c "diff <(cut -d' ' -f1-8 ${a}/${f}) <(cut -d' ' -f1-8 ${b}/${f}) | head -6" || true
        fi
    done
    for f in frames.txt serial_summary.txt; do
        sim bash -c "diff ${a}/${f} ${b}/${f}" && echo "  IDENTICAL  ${f}" || { echo "  DIFFERENT  ${f}"; ok=0; }
    done
    (( ok )) && echo "DETERMINISM: PASS (estimator state bit-identical)" || { echo "DETERMINISM: FAIL"; exit 1; }
    ;;
replay)
    for rd in "${RUNS[@]}"; do
        echo "== $(basename "${rd}") g=${G}"
        replay_one "${rd}" "${G}" "$(outdir "${rd}" "${G}")" || true
    done
    ;;
evaluate)
    for rd in "${RUNS[@]}"; do
        od="$(outdir "${rd}" "${G}")"
        sim bash -c "test -s ${od}/ov_state_est.txt" || { echo "== $(basename "${rd}"): no replay at g=${G}"; continue; }
        T=$(sim python3 /opt/scripts/viba_window.py "${rd}/orb_slam3.log" --emit-t-start || true)
        if [[ -z "${T}" ]]; then
            echo "== $(basename "${rd}"): no VIBA 2 in the live ORB-SLAM3 log; no window to impose, skipping"
            continue
        fi
        echo "== $(basename "${rd}") g=${G}  window t >= ${T}"
        sim bash -c "source /opt/ros/jazzy/setup.bash; \
            python3 /opt/scripts/ov_prep.py ${od} --t-start ${T} --gt-bag ${rd}/flight.bag" \
            || { echo "  ov_prep failed for $(basename "${od}"), skipping"; continue; }
        sim /opt/scripts/eval_window_matched.sh --orb "${rd}" --ov "${od}" | tee "docker/out/replay/$(basename "${od}")_matched.txt"
        sim bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; \
            ros2 run ov_eval error_singlerun posyaw ${od}/gt_imu_window.txt ${od}/est_cov_window.txt" \
            > "docker/out/replay/$(basename "${od}")_nees.txt" 2>&1 || true
        grep -iE 'nees|rmse' "docker/out/replay/$(basename "${od}")_nees.txt" | sed 's/\x1b\[[0-9;]*m//g' | head -8
        # Scene degradation, measured per run (PATCHES.md s49).
        sim python3 /opt/scripts/ov_scene_stats.py "${od}/openvins.log" \
            | tee "docker/out/replay/$(basename "${od}")_scene.txt" || true
        sim python3 /opt/scripts/ov_scene_stats.py "${od}/openvins.log" --json \
            > "docker/out/replay/$(basename "${od}")_scene.json" 2>/dev/null || true
    done
    ;;
jointcov)
    rd="${RUNS[0]}"; od="$(outdir "${rd}" 9.81 jointcov)"
    echo "== joint covariance: serial run on $(basename "${rd}"), sampled early / mid / late"
    sim bash -c "rm -rf ${od}"
    sim /opt/scripts/replay_estimator.sh --bag "${rd}/flight.bag" --estimator openvins_serial --out "${od}" \
        > "docker/out/replay/$(basename "${od}").log" 2>&1 &
    RP=$!
    J="docker/out/replay/$(basename "${od}")_jointcov.jsonl"; : > "${J}"
    # The serial runner covers a path A bag in ~23 s wall (init at ~4 s), so
    # these offsets land early, mid and late in the flight.
    for wait_s in 8 6 6; do
        sleep "${wait_s}"
        sim bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; \
            timeout 20 python3 /opt/scripts/check_joint_cov.py --messages 2 --json" >> "${J}" 2>&1 || true
    done
    wait "${RP}" || true
    echo "results in ${J}"
    ;;
topicspread)
    rd="${RUNS[0]}"; n="${G:-3}"; [[ "${n}" =~ ^[0-9]+$ ]] || n=3
    echo "== topic-replay spread: $(basename "${rd}") x ${n} through run_subscribe_msckf"
    for k in $(seq 1 "${n}"); do
        replay_one "${rd}" 9.81 "$(outdirT "${rd}" 9.81 "r${k}")" openvins
    done
    ;;
*) echo "mode must be determinism|replay|evaluate|jointcov|topicspread" >&2; exit 2 ;;
esac
