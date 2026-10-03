#!/usr/bin/env bash
# Dedicated stream-rate measurement pass (run on the HOST).
#
# Separate from the flight runs on purpose. Subscribing to the image topics
# perturbs the simulator (see rate_monitor.py): with image subscriptions active
# the real-time factor dropped from 0.289 to ~0.026 on this host. So:
#
#   * flights measure RTF from /clock only, which is cheap and faithful;
#   * this pass measures the image/IMU rates, and its RTF is a LOWER BOUND.
#
# No flight and no SLAM -- the drone sits on the ground. That isolates the
# sensor pipeline from controller and SLAM load.
#
#   ./docker/scripts/measure_streams.sh depth    # phase-1 x500_depth rig
#   ./docker/scripts/measure_streams.sh d455     # phase-2 D455-mirror rig
set -eo pipefail

RIG="${1:-depth}"
SECS="${SECS:-150}"
COMPOSE="docker compose -f docker/compose.yaml"
OUT="docker/out/streams_${RIG}_$(date +%Y%m%d-%H%M%S)"
mkdir -p "${OUT}"

case "${RIG}" in
    depth) MODEL=gz_x500_depth; MONFLAGS="--images" ;;
    d455)  MODEL=gz_x500_d455;  MONFLAGS="--images --stereo" ;;
    *) echo "usage: $0 [depth|d455]" >&2; exit 2 ;;
esac

echo "=== stream measurement: rig=${RIG} model=${MODEL} ${SECS}s -> ${OUT} ==="

${COMPOSE} exec -T sim /opt/scripts/bringup_sim.sh --stop >/dev/null 2>&1 || true
${COMPOSE} exec -T sim rm -f /out/logs/bringup.log
${COMPOSE} exec -d sim bash -c "MODEL=${MODEL} START_SLAM=0 /opt/scripts/bringup_sim.sh > /out/logs/bringup.log 2>&1; echo EXIT=\$? >> /out/logs/bringup.log"

echo "--- waiting for bringup ---"
for _ in $(seq 1 150); do
    ${COMPOSE} exec -T sim grep -qE 'stack up|FAILED|EXIT=' /out/logs/bringup.log 2>/dev/null && break
    sleep 10
done
${COMPOSE} exec -T sim cp /out/logs/bringup.log /out/logs/bringup_${RIG}.log 2>/dev/null || true
cp -f "docker/out/logs/bringup.log" "${OUT}/bringup.log" 2>/dev/null || true

if ! ${COMPOSE} exec -T sim grep -q 'stack up' /out/logs/bringup.log 2>/dev/null; then
    echo "BRINGUP FAILED for ${RIG}:"
    ${COMPOSE} exec -T sim tail -25 /out/logs/bringup.log | sed 's/^/    /'
    exit 1
fi
echo "--- stack up; letting the sensor pipeline reach steady state ---"
sleep 25

echo "--- measuring for ${SECS}s (this pass perturbs the sim; RTF is a lower bound) ---"
# NOTE: `docker compose exec` bypasses the image ENTRYPOINT, so ROS is not
# sourced in the exec'd shell -- rclpy import fails without this.
${COMPOSE} exec -T sim bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash; \
    source /root/ws_offboard_control/install/setup.bash; \
    python3 -u /opt/scripts/rate_monitor.py ${MONFLAGS} \
      --duration ${SECS} --window 20 --out /out/logs/streams_${RIG}.csv" \
    2>&1 | tee "${OUT}/rates.log"

echo
echo "--- also: clock-only RTF for the same idle rig (unperturbed baseline) ---"
${COMPOSE} exec -T sim bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash; \
    source /root/ws_offboard_control/install/setup.bash; \
    python3 -u /opt/scripts/rate_monitor.py --duration 60 --window 20" \
    2>&1 | sed -n '/RUN SUMMARY/,$p' | tee "${OUT}/rtf_clean.log"

echo
echo "results in ${OUT}"
