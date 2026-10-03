#!/usr/bin/env bash
# Repeat the scripted flight N times and aggregate ATE/RPE (run on the HOST).
#
#   ./docker/scripts/run_experiment.sh 5              # phase-1 x500_depth rig
#   RIG=d455 ./docker/scripts/run_experiment.sh 5     # phase-2 D455 stereo-inertial
#   RIG=d455_stereo ./docker/scripts/run_experiment.sh 3   # phase-3 stereo-only
#   PATH_VERSION=B RIG=d455 ./docker/scripts/run_experiment.sh 5
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
# PATH_VERSION labels the scripted trajectory. Results from different path
# versions are NOT comparable and must never be pooled into one mean:
#   A  straight constant-velocity legs at fixed altitude (the original)
#   B  yaw turns, altitude changes, varied acceleration (~3 min)
# Path A leaves accelerometer bias and inertial scale weakly observable, which
# is why B exists. See docker/OPEN_ISSUES.md s2.
PATH_VERSION="${PATH_VERSION:-A}"

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
    # We do not EVALUATE COVINS on this rig, but the backend must still be
    # listening. The vendored ORB_SLAM3 always constructs the COVINS
    # Communicator, and covins_comm's ConnectToServer() returns the literal 2 as
    # its error code -- which ORB-SLAM3 then uses as newfd_. File descriptor 2
    # is stderr, so the comm thread "connects" to stderr and later close()s it,
    # corrupting the process's standard error. Keeping the backend reachable
    # avoids that path entirely.
    USE_COVINS=0
    ;;
  d455_stereo)
    # Phase 3 bisection rig: the SAME D455 hardware, ORB-SLAM3 in STEREO-ONLY
    # mode. No IMU, and therefore no visual-inertial BA -- see PATCHES.md s35.
    # Everything else is identical to the d455 case, which is what makes this a
    # bisection rather than a separate experiment.
    RIG_ENV="MODEL=gz_x500_d455 D455_DEPTH=0 SLAM_MODE=stereo"
    EVAL_ENV="EST_TOPIC=/robot_pose_slam"
    # Same reason as the d455 case: the vendored ORB_SLAM3 always constructs the
    # COVINS Communicator, and an unreachable backend sends it down the
    # ConnectToServer -> "return 2" -> close(stderr) path.
    USE_COVINS=0
    ;;
  *) echo "RIG must be 'depth', 'd455' or 'd455_stereo'" >&2; exit 2 ;;
esac

OUT="docker/out/experiment_${RIG}_path${PATH_VERSION}_$(date +%Y%m%d-%H%M%S)"
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

# A manifest, written BEFORE the first run, so a result directory can always be
# traced back to the code and configuration that produced it. Without this the
# only record of which path version a number came from was my memory of which
# command I typed.
{
    echo "experiment:     $(basename "${OUT}")"
    echo "started:        $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "rig:            ${RIG}"
    echo "path_version:   ${PATH_VERSION}"
    echo "runs:           ${N}"
    echo "flight:         side=${SIDE}m alt=${ALT}m leg_time=${LEG_TIME}s settle=${SETTLE}s"
    echo "rig_env:        ${RIG_ENV}"
    echo "eval_env:       ${EVAL_ENV}"
    echo "use_covins:     ${USE_COVINS}"
    echo "git_commit:     $(git rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "git_dirty:      $(test -n "$(git status --porcelain 2>/dev/null)" && echo yes || echo no)"
    echo "sim_image:      $(docker image inspect drone-sim:jazzy-arm64 --format '{{.Id}}' 2>/dev/null || echo unknown)"
    echo "world_gravity:  $(${COMPOSE} exec -T sim grep -ohE '<gravity>[^<]*</gravity>' \
                              /opt/PX4-Autopilot/Tools/simulation/gz/worlds/forest.sdf 2>/dev/null | head -1)"
} > "${OUT}/MANIFEST.txt"

echo "=== experiment: rig=${RIG} path=${PATH_VERSION}  ${N} runs  side=${SIDE}m alt=${ALT}m leg=${LEG_TIME}s -> ${OUT} ==="
sed 's/^/    /' "${OUT}/MANIFEST.txt"

for i in $(seq 1 "${N}"); do
    echo
    echo "################ RUN ${i}/${N} (${RIG}) ################"

    # Always start the backend -- see the note in the d455 case above.
    if true; then
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
    echo "${PATH_VERSION}" > "docker/out/eval/${B}/path_version.txt" 2>/dev/null || true

    if [[ -f "${OUT}/run${i}_imu_init.txt" ]]; then
        echo "run ${i}: $(tr '\n' ' ' < "${OUT}/run${i}_imu_init.txt")"
    fi
done

echo
echo "=== aggregating ==="
python3 docker/scripts/aggregate_results.py "${OUT}" | tee "${OUT}/SUMMARY.txt"
echo
echo "results in ${OUT}"
