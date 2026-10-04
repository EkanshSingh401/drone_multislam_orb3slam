#!/usr/bin/env bash
# Phase 3: OpenVINS on recorded sensor bags by deterministic replay (run on the HOST).
#
#   ./docker/scripts/phase3_openvins.sh determinism <experiment_dir>
#   ./docker/scripts/phase3_openvins.sh replay      <experiment_dir> [gravity_mag]
#   ./docker/scripts/phase3_openvins.sh evaluate    <experiment_dir> [gravity_mag]
#   ./docker/scripts/phase3_openvins.sh jointcov    <experiment_dir>
#
# <experiment_dir> is a run_experiment.sh output recorded with RECORD_SENSORS=1
# and RIG=d455, e.g. docker/out/experiment_d455_pathA_<stamp>. Its rundirs.txt
# names the live runs; each live run's flight.bag is the replay input and its
# orb_slam3.log (live stereo-inertial, same data) supplies the converged window.
#
# Replay outputs go to /out/replay/<run>_openvins_g<gravity>[_<tag>]/.
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

replay_one() {  # replay_one <rundir> <gravity> <outdir>
    local rd="$1" g="$2" od="$3"
    sim bash -c "test -f ${rd}/flight.bag/metadata.yaml" \
        || { echo "  ${rd}: no complete flight.bag, skipping"; return 1; }
    sim bash -c "rm -rf ${od}"
    sim env OV_GRAVITY_MAG="${g}" /opt/scripts/replay_estimator.sh \
        --bag "${rd}/flight.bag" --estimator openvins --out "${od}" \
        > "docker/out/replay/$(basename "${od}").log" 2>&1
    sim cat "${od}/frames.txt" | tr '\n' ' '; echo
}

outdir() { echo "/out/replay/$(basename "$1")_openvins_g${2}${3:+_$3}"; }
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
    # PASS/FAIL is decided on OpenVINS's own per-update state (save_total_state),
    # which is what phase 3 evaluates. The published /odomimu stream
    # (ov_est_cov.txt) is propagated at IMU rate from whatever state exists when
    # each IMU message is handled, so it depends on callback interleaving and is
    # NOT expected to be bit-identical (PATCHES.md s46); it is reported below for
    # information only.
    for f in ov_state_est.txt ov_state_std.txt; do
        if sim cmp -s "${a}/${f}" "${b}/${f}"; then
            echo "  IDENTICAL  ${f}  ($(sim bash -c "wc -l < ${a}/${f}") lines)"
        else
            echo "  DIFFERENT  ${f}"; ok=0
            sim bash -c "diff <(cut -d' ' -f1-8 ${a}/${f}) <(cut -d' ' -f1-8 ${b}/${f}) | head -6" || true
        fi
    done
    for f in frames.txt; do
        sim bash -c "diff ${a}/${f} ${b}/${f}" && echo "  IDENTICAL  ${f}" || { echo "  DIFFERENT  ${f}"; ok=0; }
    done
    echo "  (info) /odomimu stream, not part of the criterion:"
    sim python3 -c "
import numpy as np
L=lambda p:{l.split()[0]:np.array(l.split()[1:],float) for l in open(p) if not l.startswith('#')}
A,B=L('${a}/ov_est_cov.txt'),L('${b}/ov_est_cov.txt'); c=set(A)&set(B)
d=[np.linalg.norm(A[k][:3]-B[k][:3]) for k in c if not np.array_equal(A[k],B[k])]
print('    %d of %d common rows differ; max position diff %.3g m' % (len(d), len(c), max(d) if d else 0))
"
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
        sim bash -c "test -s ${od}/ov_est_cov.txt" || { echo "== $(basename "${rd}"): no replay at g=${G}"; continue; }
        T=$(sim python3 /opt/scripts/viba_window.py "${rd}/orb_slam3.log" --emit-t-start || true)
        if [[ -z "${T}" ]]; then
            echo "== $(basename "${rd}"): no VIBA 2 in the live ORB-SLAM3 log; no window to impose, skipping"
            continue
        fi
        echo "== $(basename "${rd}") g=${G}  window t >= ${T}"
        sim python3 /opt/scripts/ov_prep.py "${od}" --t-start "${T}"
        sim /opt/scripts/eval_window_matched.sh --orb "${rd}" --ov "${od}" | tee "docker/out/replay/$(basename "${od}")_matched.txt"
        sim bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; \
            ros2 run ov_eval error_singlerun posyaw ${od}/gt_imu_window.txt ${od}/est_cov_window.txt" \
            > "docker/out/replay/$(basename "${od}")_nees.txt" 2>&1 || true
        grep -iE 'nees|rmse' "docker/out/replay/$(basename "${od}")_nees.txt" | sed 's/\x1b\[[0-9;]*m//g' | head -8
    done
    ;;
jointcov)
    rd="${RUNS[0]}"; od="$(outdir "${rd}" 9.81 jointcov)"
    echo "== joint covariance replay: $(basename "${rd}")"
    sim bash -c "rm -rf ${od}"
    sim /opt/scripts/replay_estimator.sh --bag "${rd}/flight.bag" --estimator openvins --out "${od}" \
        > "docker/out/replay/$(basename "${od}").log" 2>&1 &
    RP=$!
    # sample early, mid and late in the flight
    for wait_s in 40 40 40; do
        sleep "${wait_s}"
        sim bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; \
            timeout 60 python3 /opt/scripts/check_joint_cov.py --messages 3 --json" \
            >> "docker/out/replay/$(basename "${od}")_jointcov.jsonl" 2>&1 || true
    done
    wait "${RP}" || true
    echo "results in docker/out/replay/$(basename "${od}")_jointcov.jsonl"
    ;;
*) echo "mode must be determinism|replay|evaluate|jointcov" >&2; exit 2 ;;
esac
