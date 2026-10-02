#!/usr/bin/env bash
# Measure what the stack is ACTUALLY doing (run INSIDE the sim container).
#
# Reports, in order:
#   1. Gazebo real-time factor
#   2. measured RGB / depth / clock rates via `ros2 topic hz`
#   3. whether ORB-SLAM3 initialised and is still tracking
#   4. whether the COVINS backend is reachable and receiving keyframes
#
# Nothing here is inferred from configuration -- every number is measured.
set -o pipefail    # no -u: ROS setup.bash reads unset vars

WORLD="${WORLD:-forest}"
NAMESPACE="${NAMESPACE:-uav_1}"
WINDOW="${WINDOW:-15}"       # seconds per rate measurement
LOGDIR="${LOGDIR:-/out/logs}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS:-/root/ws_offboard_control}/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="${PX4_DIR:-/opt/PX4-Autopilot}/Tools/simulation/gz/models:${PX4_DIR:-/opt/PX4-Autopilot}/Tools/simulation/gz/worlds"

hr() { printf '%s\n' "------------------------------------------------------------"; }

echo
hr; echo "1. GAZEBO REAL-TIME FACTOR  (world=${WORLD})"; hr
# WorldStatistics carries real_time_factor; average a few samples.
gz topic -e -t "/world/${WORLD}/stats" -n 12 2>/dev/null \
  | awk '
      /real_time_factor/ { gsub(/[^0-9.]/,"",$2); if ($2+0 > 0) { s+=$2; n++ } }
      /iterations/       { it=$2 }
      END {
        if (n > 0) printf "   real_time_factor: mean %.3f over %d samples\n", s/n, n;
        else       print  "   real_time_factor: NO SAMPLES (is the Gazebo server running?)";
        if (it)    printf "   iterations:       %s\n", it;
      }'
echo "   (1.0 = wall-clock speed. Software rendering via llvmpipe with no GPU"
echo "    passthrough is the dominant cost here.)"

echo
hr; echo "2. MEASURED TOPIC RATES  (${WINDOW}s window each)"; hr
measure_hz() {  # measure_hz <topic> <expected>
    local topic="$1" expected="$2" out rate
    if ! ros2 topic info "${topic}" >/dev/null 2>&1; then
        printf "   %-34s MISSING (topic does not exist)\n" "${topic}"; return
    fi
    out="$(timeout "${WINDOW}" ros2 topic hz "${topic}" --window 100 2>/dev/null \
           | grep -Eo 'average rate: [0-9.]+' | tail -1 | awk '{print $3}')"
    if [[ -z "${out}" ]]; then
        printf "   %-34s 0.00 Hz   NO MESSAGES (expected ~%s)\n" "${topic}" "${expected}"
    else
        rate="${out}"
        printf "   %-34s %6.2f Hz  (expected ~%s)\n" "${topic}" "${rate}" "${expected}"
    fi
}
measure_hz "/clock"                            "250 Hz"
measure_hz "/${NAMESPACE}/rgb/image_raw"        "30 Hz (Camera.fps in gazebo_rgbd.yaml)"
measure_hz "/${NAMESPACE}/depth/image"          "30 Hz"
measure_hz "/${NAMESPACE}/rgb/camera_info"      "30 Hz"
measure_hz "/${NAMESPACE}/depth/camera_info"    "30 Hz"
measure_hz "/ground_truth/pose_info"            "varies"

echo
hr; echo "3. ORB-SLAM3 STATE"; hr
echo "   nodes:"
ros2 node list 2>/dev/null | grep -i 'slam' | sed 's/^/     /' || echo "     (no SLAM node found)"

echo "   use_sim_time on each running node:"
for n in $(ros2 node list 2>/dev/null); do
    v="$(timeout 5 ros2 param get "$n" use_sim_time 2>/dev/null | grep -Eo 'True|False' | head -1)"
    [[ -n "$v" ]] && printf "     %-44s use_sim_time=%s\n" "$n" "$v"
done

echo "   pose output (publishing => tracking, silence => lost/not initialised):"
measure_hz "/${NAMESPACE}/robot_pose_slam" "tracking rate"

if [[ -f "${LOGDIR}/orb_slam3.log" ]]; then
    echo "   tracking-state lines from the ORB-SLAM3 log:"
    grep -iE 'map initialis|map initializ|Tracking lost|relocaliz|RESET|First KF|not enough|Fail' \
        "${LOGDIR}/orb_slam3.log" 2>/dev/null | tail -12 | sed 's/^/     /' \
        || echo "     (no tracking-state lines yet)"
    echo "   COVINS handshake as seen by the agent:"
    grep -iE 'server_ip|port|connect|Connected|covins|Communicator' \
        "${LOGDIR}/orb_slam3.log" 2>/dev/null | tail -12 | sed 's/^/     /' \
        || echo "     (nothing yet)"
fi

echo
hr; echo "4. COVINS BACKEND"; hr
COVINS_HOST="$(grep -E '^sys\.server_ip' /root/.local/config/config_comm.yaml 2>/dev/null | sed "s/.*'\(.*\)'.*/\1/")"
COVINS_PORT="$(grep -E '^sys\.port'      /root/.local/config/config_comm.yaml 2>/dev/null | sed "s/.*'\(.*\)'.*/\1/")"
echo "   agent is configured to reach: ${COVINS_HOST:-?}:${COVINS_PORT:-?}"
echo "   (this comes from /root/.local/config/config_comm.yaml, whose path is"
echo "    baked into covins_comm at compile time from __FILE__)"

if command -v getent >/dev/null && [[ -n "${COVINS_HOST:-}" ]]; then
    ip="$(getent hosts "${COVINS_HOST}" | awk '{print $1}' | head -1)"
    echo "   DNS: ${COVINS_HOST} -> ${ip:-UNRESOLVED}"
fi
if [[ -n "${COVINS_HOST:-}" && -n "${COVINS_PORT:-}" ]]; then
    if timeout 5 bash -c "cat < /dev/null > /dev/tcp/${COVINS_HOST}/${COVINS_PORT}" 2>/dev/null; then
        echo "   TCP ${COVINS_HOST}:${COVINS_PORT}: OPEN"
    else
        echo "   TCP ${COVINS_HOST}:${COVINS_PORT}: CLOSED/unreachable"
        echo "   -> the backend node is probably not running; start it with"
        echo "      docker compose exec covins-backend /opt/scripts/run_backend.sh"
    fi
fi
echo
echo "   Keyframe receipt must be read from the BACKEND's stdout, not from here:"
echo "     docker compose exec covins-backend grep -iE 'agent|keyframe|KF' /out/logs/covins_backend.log | tail"
echo "   comm.start_sending_after_kf = $(grep -E '^comm\.start_sending_after_kf' /root/.local/config/config_comm.yaml 2>/dev/null | awk '{print $2}')"
echo "   comm.kf_buffer_withold     = $(grep -E '^comm\.kf_buffer_withold'      /root/.local/config/config_comm.yaml 2>/dev/null | awk '{print $2}')"
echo "   The agent withholds the first start_sending_after_kf keyframes AND the"
echo "   most recent kf_buffer_withold ones, so expect a delay (and some camera"
echo "   motion) before the backend reports anything at all."
echo
