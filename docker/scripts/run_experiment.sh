#!/usr/bin/env bash
# Repeat the scripted flight N times and aggregate ATE/RPE (run on the HOST).
#
# Each run gets a FRESH stack: Gazebo, PX4, the bridge and ORB-SLAM3 are all
# restarted, and the COVINS backend is restarted too so its map starts empty.
# Without that, keyframes from a previous run stay in the backend's map and the
# exported trajectory is a mixture of runs.
#
#   ./docker/scripts/run_experiment.sh 5
#   N=5 SIDE=3 ALT=2.0 LEG_TIME=6 ./docker/scripts/run_experiment.sh
set -eo pipefail

N="${1:-${N:-5}}"
COMPOSE="docker compose -f docker/compose.yaml"
SIDE="${SIDE:-3}"; ALT="${ALT:-2.0}"; LEG_TIME="${LEG_TIME:-6}"; SETTLE="${SETTLE:-8}"

# RIG=depth  -> phase-1 x500_depth, ORB-SLAM3 RGB-D, COVINS backend
# RIG=d455   -> phase-2 D455-mirror, ORB-SLAM3 STEREO-INERTIAL, depth camera OFF
RIG="${RIG:-depth}"
case "${RIG}" in
  depth)
    RIG_ENV="MODEL=gz_x500_depth SLAM_MODE=rgbd"
    EVAL_ENV="EST_TOPIC=/uav_1/robot_pose_slam"
    USE_COVINS=1 ;;
  d455)
    # D455_DEPTH=0 selects the depth-free model: neither stereo-inertial nor
    # OpenVINS consumes depth, and a third 848x480 render pass is pure RTF cost.
    RIG_ENV="MODEL=gz_x500_d455 D455_DEPTH=0 SLAM_MODE=stereo_inertial"
    # The stereo-inertial node runs in the root namespace.
    EVAL_ENV="EST_TOPIC=/robot_pose_slam"
    # COVINS is a phase-1 concern; the stereo-inertial baseline does not use it.
    USE_COVINS=0 ;;
  *) echo "RIG must be depth or d455" >&2; exit 2 ;;
esac
OUT="docker/out/experiment_$(date +%Y%m%d-%H%M%S)"
mkdir -p "${OUT}"

# --- guard: the container must be running the CURRENT scripts ---------------
# Rebuilding the image is not enough; the container keeps running the old image
# until it is recreated. That silently produced a run with a stale
# record_and_eval.sh whose bag-stop fix was missing, and the run hung.
echo "--- verifying container scripts match the host ---"
STALE=0
for f in record_and_eval.sh bringup_sim.sh rate_monitor.py bag_to_tum.py fly_path.py; do
    H=$(md5 -q "docker/scripts/$f" 2>/dev/null || md5sum "docker/scripts/$f" | cut -d' ' -f1)
    C=$(${COMPOSE} exec -T sim md5sum "/opt/scripts/$f" 2>/dev/null | cut -d' ' -f1)
    if [[ "$H" != "$C" ]]; then
        echo "  STALE  $f  (host $H != container $C)"
        STALE=1
    else
        echo "  ok     $f"
    fi
done
if [[ ${STALE} -ne 0 ]]; then
    echo
    echo "Container scripts are out of date. Rebuild AND recreate:"
    echo "  docker build --platform linux/arm64 -f docker/sim/Dockerfile -t drone-sim:jazzy-arm64 ."
    echo "  docker compose -f docker/compose.yaml up -d --force-recreate sim"
    exit 1
fi

echo "=== experiment: rig=${RIG} ${N} runs, side=${SIDE}m alt=${ALT}m leg=${LEG_TIME}s -> ${OUT} ==="
echo "    rig env : ${RIG_ENV}"
echo "    eval env: ${EVAL_ENV}"

for i in $(seq 1 "${N}"); do
    echo
    echo "################ RUN ${i}/${N} ################"

    # --- fresh backend (empty COVINS map), phase-1 rig only ---
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

    # --- fresh sim stack ---
    ${COMPOSE} exec -T sim /opt/scripts/bringup_sim.sh --stop >/dev/null 2>&1 || true
    ${COMPOSE} exec -T sim rm -f /out/logs/bringup.log /out/covins/KF_0_ftum.csv
    ${COMPOSE} exec -d sim bash -c '/opt/scripts/bringup_sim.sh > /out/logs/bringup.log 2>&1; echo "EXIT=$?" >> /out/logs/bringup.log'
    for _ in $(seq 1 120); do
        ${COMPOSE} exec -T sim grep -qE 'stack up|FAILED|EXIT=' /out/logs/bringup.log 2>/dev/null && break
        sleep 10
    done
    if ! ${COMPOSE} exec -T sim grep -q 'stack up' /out/logs/bringup.log 2>/dev/null; then
        echo "run ${i}: BRINGUP FAILED, skipping"
        ${COMPOSE} exec -T sim tail -5 /out/logs/bringup.log | sed 's/^/    /'
        continue
    fi
    echo "run ${i}: stack up"
    # Let ORB-SLAM3 settle AND let ROS 2 discovery converge before flying.
    # fly_path.py now retries its namespace autodetect, but giving discovery a
    # head start avoids burning that budget on every run.
    sleep 30

    # --- flight + record ---
    ${COMPOSE} exec -T sim env SIDE="${SIDE}" ALT="${ALT}" LEG_TIME="${LEG_TIME}" \
        SETTLE="${SETTLE}" ${EVAL_ENV} /opt/scripts/record_and_eval.sh \
        > "${OUT}/run${i}_flight.log" 2>&1 || echo "run ${i}: record_and_eval returned $?"

    RUNDIR=$(grep -oE '/out/eval/[0-9-]+' "${OUT}/run${i}_flight.log" | head -1)
    echo "run ${i}: rundir ${RUNDIR}"

    # --- COVINS trajectory, then re-evaluate including it ---
    if [[ "${USE_COVINS}" == "1" ]]; then
        ${COMPOSE} exec -T covins-backend /opt/scripts/export_covins_trajectory.sh \
            > "${OUT}/run${i}_covins_export.log" 2>&1 || echo "run ${i}: covins export returned $?"
    fi
    ${COMPOSE} exec -T sim env ${EVAL_ENV} /opt/scripts/record_and_eval.sh --eval-only "${RUNDIR}" \
        > "${OUT}/run${i}_eval.log" 2>&1 || echo "run ${i}: eval returned $?"

    # post-hoc RTF/path from the bag -- the only trustworthy source (see
    # analyze_bag.py for why the live alternatives are not)
    ${COMPOSE} exec -T sim bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash; \
        source /root/ws_offboard_control/install/setup.bash; \
        python3 /opt/scripts/analyze_bag.py '${RUNDIR}/flight.bag' --json" \
        < /dev/null > "${OUT}/run${i}_bag.json" 2>/dev/null || true
    cp -f "docker/out/eval/$(basename "${RUNDIR}")/imu_init.txt" "${OUT}/run${i}_imu_init.txt" 2>/dev/null || true

    cp -f "docker/out/eval/$(basename "${RUNDIR}")/rates.log" "${OUT}/run${i}_rates.log" 2>/dev/null || true
    echo "${RUNDIR}" >> "${OUT}/rundirs.txt"
done

echo
echo "=== aggregating ==="
python3 docker/scripts/aggregate_results.py "${OUT}" | tee "${OUT}/SUMMARY.txt"
echo
echo "results in ${OUT}"
