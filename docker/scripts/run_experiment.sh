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

echo "=== experiment: ${N} runs, side=${SIDE}m alt=${ALT}m leg=${LEG_TIME}s -> ${OUT} ==="

for i in $(seq 1 "${N}"); do
    echo
    echo "################ RUN ${i}/${N} ################"

    # --- fresh backend (empty COVINS map) ---
    ${COMPOSE} restart covins-backend >/dev/null 2>&1
    sleep 12
    ${COMPOSE} exec -d covins-backend /opt/scripts/run_backend.sh
    for _ in $(seq 1 90); do
        ${COMPOSE} exec -T covins-backend netstat -ltn 2>/dev/null | grep -q ':9033' && break
        sleep 2
    done
    echo "run ${i}: backend listening"

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
        SETTLE="${SETTLE}" /opt/scripts/record_and_eval.sh \
        > "${OUT}/run${i}_flight.log" 2>&1 || echo "run ${i}: record_and_eval returned $?"

    RUNDIR=$(grep -oE '/out/eval/[0-9-]+' "${OUT}/run${i}_flight.log" | head -1)
    echo "run ${i}: rundir ${RUNDIR}"

    # --- COVINS trajectory, then re-evaluate including it ---
    ${COMPOSE} exec -T covins-backend /opt/scripts/export_covins_trajectory.sh \
        > "${OUT}/run${i}_covins_export.log" 2>&1 || echo "run ${i}: covins export returned $?"
    ${COMPOSE} exec -T sim /opt/scripts/record_and_eval.sh --eval-only "${RUNDIR}" \
        > "${OUT}/run${i}_eval.log" 2>&1 || echo "run ${i}: eval returned $?"

    cp -f "docker/out/eval/$(basename "${RUNDIR}")/rates.log" "${OUT}/run${i}_rates.log" 2>/dev/null || true
    echo "${RUNDIR}" >> "${OUT}/rundirs.txt"
done

echo
echo "=== aggregating ==="
python3 docker/scripts/aggregate_results.py "${OUT}" | tee "${OUT}/SUMMARY.txt"
echo
echo "results in ${OUT}"
