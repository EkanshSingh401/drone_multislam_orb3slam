#!/usr/bin/env bash
# Repeat the scripted flight N times and aggregate ATE/RPE (run on the HOST).
#
#   ./docker/scripts/run_experiment.sh 5              # phase-1 x500_depth rig
#   RIG=d455 ./docker/scripts/run_experiment.sh 5     # phase-2 D455 stereo-inertial
#
# Each run gets a genuinely fresh stack. bringup_sim.sh --stop verifies every
# process is gone (escalating to SIGKILL) before the next run starts, so runs
# cannot compete with a surviving gz server. The COVINS backend is restarted too
# where it is used, otherwise keyframes from a previous run stay in its map and
# the exported trajectory mixes runs.
set -eo pipefail

N="${1:-${N:-5}}"
COMPOSE="docker compose -f docker/compose.yaml"
SIDE="${SIDE:-3}"; ALT="${ALT:-2.0}"; LEG_TIME="${LEG_TIME:-6}"; SETTLE="${SETTLE:-8}"
RIG="${RIG:-depth}"

case "${RIG}" in
  depth)
    # Phase 1: x500_depth, ORB-SLAM3 RGB-D, COVINS backend.
    RIG_ENV="MODEL=gz_x500_depth SLAM_MODE=rgbd"
    EVAL_ENV="EST_TOPIC=/uav_1/robot_pose_slam"
    USE_COVINS=1
    ;;
  d455)
    # Phase 2: D455-mirror, ORB-SLAM3 STEREO-INERTIAL, depth camera OFF.
    # Neither stereo-inertial nor OpenVINS consumes depth, and rendering it is a
    # third 848x480 pass per frame -- pure real-time-factor cost. IR stays
    # 848x480 because resolution changes feature counts.
    RIG_ENV="MODEL=gz_x500_d455 D455_DEPTH=0 SLAM_MODE=stereo_inertial"
    # The stereo-inertial node runs in the root namespace.
    EVAL_ENV="EST_TOPIC=/robot_pose_slam"
    USE_COVINS=0
    ;;
  *) echo "RIG must be 'depth' or 'd455'" >&2; exit 2 ;;
esac

OUT="docker/out/experiment_${RIG}_$(date +%Y%m%d-%H%M%S)"
mkdir -p "${OUT}"

# --- guard: the container must be running the CURRENT scripts ----------------
# Rebuilding the image is not enough; the container keeps the old image until it
# is recreated. That silently produced a run with a stale record_and_eval.sh.
echo "--- verifying container scripts match the host ---"
STALE=0
for f in record_and_eval.sh bringup_sim.sh rate_monitor.py bag_to_tum.py fly_path.py analyze_bag.py; do
    H=$(md5 -q "docker/scripts/$f" 2>/dev/null || md5sum "docker/scripts/$f" | cut -d' ' -f1)
    C=$(${COMPOSE} exec -T sim md5sum "/opt/scripts/$f" 2>/dev/null | cut -d' ' -f1)
    if [[ "$H" != "$C" ]]; then echo "  STALE  $f"; STALE=1; else echo "  ok     $f"; fi
done
if [[ ${STALE} -ne 0 ]]; then
    echo
    echo "Rebuild AND recreate:"
    echo "  docker build --platform linux/arm64 -f docker/sim/Dockerfile -t drone-sim:jazzy-arm64 ."
    echo "  docker compose -f docker/compose.yaml up -d --force-recreate sim"
    exit 1
fi

echo "=== experiment: rig=${RIG}  ${N} runs  side=${SIDE}m alt=${ALT}m leg=${LEG_TIME}s -> ${OUT} ==="
echo "    rig env  : ${RIG_ENV}"
echo "    eval env : ${EVAL_ENV}"
echo "    covins   : ${USE_COVINS}"

for i in $(seq 1 "${N}"); do
    echo
    echo "################ RUN ${i}/${N} (${RIG}) ################"

    if [[ "${USE_COVINS}" == "1" ]]; then
        ${COMPOSE} restart covins-backend >/dev/null 2>&1
        sleep 12
        ${COMPOSE} exec -d covins-backend /opt/scripts/run_backend.sh
        for _ in $(seq 1 90); do
            ${COMPOSE} exec -T covins-backend netstat -ltn 2>/dev/null | grep -q ':9033' && break
            sleep 2
        done
        echo "run ${i}: backend listening"
    fi

    # Verified teardown, then a fresh gz server.
    # Tolerate a non-zero --stop: it is informational here, and `set -e` must
    # not abort the whole experiment over it.
    ${COMPOSE} exec -T sim /opt/scripts/bringup_sim.sh --stop 2>&1 | tail -1 \
        | sed "s/^/run ${i}: /" || true
    ${COMPOSE} exec -T sim rm -f /out/logs/bringup.log /out/logs/orb_slam3.log /out/covins/KF_0_ftum.csv
    ${COMPOSE} exec -d sim bash -c "${RIG_ENV} /opt/scripts/bringup_sim.sh > /out/logs/bringup.log 2>&1; echo EXIT=\$? >> /out/logs/bringup.log"

    for _ in $(seq 1 150); do
        ${COMPOSE} exec -T sim grep -qE 'stack up|FAILED|EXIT=' /out/logs/bringup.log 2>/dev/null && break
        sleep 10
    done
    if ! ${COMPOSE} exec -T sim grep -q 'stack up' /out/logs/bringup.log 2>/dev/null; then
        echo "run ${i}: BRINGUP FAILED, skipping"
        ${COMPOSE} exec -T sim tail -12 /out/logs/bringup.log | sed 's/^/    /'
        ${COMPOSE} exec -T sim cp /out/logs/bringup.log "/out/logs/bringup_fail_${i}.log" 2>/dev/null || true
        continue
    fi
    echo "run ${i}: stack up ($(${COMPOSE} exec -T sim grep -oE 'model gz_[a-z0-9_]+' /out/logs/bringup.log | head -1))"

    # Let SLAM settle and ROS 2 discovery converge before flying.
    sleep 30

    ${COMPOSE} exec -T sim env SIDE="${SIDE}" ALT="${ALT}" LEG_TIME="${LEG_TIME}" \
        SETTLE="${SETTLE}" ${EVAL_ENV} /opt/scripts/record_and_eval.sh \
        > "${OUT}/run${i}_flight.log" 2>&1 || echo "run ${i}: record_and_eval returned $?"

    RUNDIR=$(grep -oE '/out/eval/[0-9-]+' "${OUT}/run${i}_flight.log" | head -1)
    if [[ -z "${RUNDIR}" ]]; then
        echo "run ${i}: no rundir produced, skipping evaluation"
        continue
    fi
    echo "run ${i}: rundir ${RUNDIR}"

    if [[ "${USE_COVINS}" == "1" ]]; then
        ${COMPOSE} exec -T covins-backend /opt/scripts/export_covins_trajectory.sh \
            > "${OUT}/run${i}_covins_export.log" 2>&1 || echo "run ${i}: covins export returned $?"
    fi

    ${COMPOSE} exec -T sim env ${EVAL_ENV} /opt/scripts/record_and_eval.sh --eval-only "${RUNDIR}" \
        > "${OUT}/run${i}_eval.log" 2>&1 || echo "run ${i}: eval returned $?"

    # Post-hoc RTF/path from the bag -- the only trustworthy source; see
    # analyze_bag.py for why both live alternatives are not.
    ${COMPOSE} exec -T sim bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash; \
        source /root/ws_offboard_control/install/setup.bash; \
        python3 /opt/scripts/analyze_bag.py '${RUNDIR}/flight.bag' --json" \
        < /dev/null > "${OUT}/run${i}_bag.json" 2>/dev/null || true

    B=$(basename "${RUNDIR}")
    cp -f "docker/out/eval/${B}/rates.log"    "${OUT}/run${i}_rates.log"    2>/dev/null || true
    cp -f "docker/out/eval/${B}/imu_init.txt" "${OUT}/run${i}_imu_init.txt" 2>/dev/null || true
    ${COMPOSE} exec -T sim cp /out/logs/orb_slam3.log "${RUNDIR}/orb_slam3.log" 2>/dev/null || true
    cp -f "docker/out/eval/${B}/orb_slam3.log" "${OUT}/run${i}_slam.log" 2>/dev/null || true
    echo "${RUNDIR}" >> "${OUT}/rundirs.txt"

    if [[ -f "${OUT}/run${i}_imu_init.txt" ]]; then
        echo "run ${i}: $(tr '\n' ' ' < "${OUT}/run${i}_imu_init.txt")"
    fi
done

echo
echo "=== aggregating ==="
python3 docker/scripts/aggregate_results.py "${OUT}" | tee "${OUT}/SUMMARY.txt"
echo
echo "results in ${OUT}"
