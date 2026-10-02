#!/usr/bin/env bash
# Bring up the single-UAV stack inside the sim container:
#   Gazebo Harmonic server (headless) -> PX4 SITL (x500_depth) ->
#   Micro XRCE-DDS agent -> ros_gz_bridge -> static TF -> ORB-SLAM3 RGB-D
#
# Every ROS 2 node runs with use_sim_time:=true.
#
# Usage:  /opt/scripts/bringup_sim.sh [--world forest] [--namespace uav_1]
#         /opt/scripts/bringup_sim.sh --stop
set -eo pipefail   # no -u: ROS setup.bash reads unset vars

WORLD="${WORLD:-forest}"
NAMESPACE="${NAMESPACE:-uav_1}"
PX4_INSTANCE="${PX4_INSTANCE:-1}"
MODEL="${MODEL:-gz_x500_depth}"
MODEL_POSE="${MODEL_POSE:-0,0,0.25}"
PX4_DIR="${PX4_DIR:-/opt/PX4-Autopilot}"
WS="${WS:-/root/ws_offboard_control}"
LOGDIR="${LOGDIR:-/out/logs}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --world)     WORLD="$2"; shift 2 ;;
        --namespace) NAMESPACE="$2"; shift 2 ;;
        --stop)
            echo "stopping stack..."
            pkill -f covins_backend_node 2>/dev/null || true
            pkill -f 'orb_slam3_ros2_wrapper' 2>/dev/null || true
            pkill -f parameter_bridge 2>/dev/null || true
            pkill -f static_transform_publisher 2>/dev/null || true
            pkill -f MicroXRCEAgent 2>/dev/null || true
            pkill -f 'bin/px4' 2>/dev/null || true
            pkill -f 'gz sim' 2>/dev/null || true
            pkill -f 'ruby.*gz' 2>/dev/null || true
            sleep 2; echo "stopped."; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "${LOGDIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS}/install/setup.bash"

# Gazebo environment. PX4 generates exactly the right one at build time, so
# source that rather than hand-rolling it. It sets three things that all matter:
#
#   GZ_SIM_RESOURCE_PATH      models (x500_depth) + worlds (forest.sdf)
#   GZ_SIM_SYSTEM_PLUGIN_PATH PX4's own gz plugins (GstCamera, OpticalFlow,
#                             MotorFailure, ...) -- without it the model SDF
#                             logs "Failed to load system plugin
#                             [MotorFailurePlugin]: Could not find shared library"
#   GZ_SIM_SERVER_CONFIG_PATH the server config listing the Gazebo systems
#
# That last one is essential and easy to miss: PX4's world SDFs contain ZERO
# <plugin> entries (verified: `grep -c "<plugin" forest.sdf` == 0), so every
# system -- including gz-sim-sensors-system, which is what actually renders the
# RGB and depth cameras -- comes from the server config. Running a bare
# `gz sim -s forest.sdf` loads the stock config instead, the Sensors system
# never starts, and no camera topics are ever created. The world and the drone
# come up looking perfectly healthy, which makes this a confusing failure.
GZ_ENV="${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh"
if [[ -f "${GZ_ENV}" ]]; then
    # shellcheck disable=SC1090
    source "${GZ_ENV}"
else
    echo "[bringup] WARNING ${GZ_ENV} not found; falling back to manual paths" >&2
    export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
    export GZ_SIM_SERVER_CONFIG_PATH="${PX4_DIR}/src/modules/simulation/gz_bridge/server.config"
fi

log()  { echo "[bringup $(date +%H:%M:%S)] $*"; }
fail() { echo "[bringup] FAILED: $*" >&2; exit 1; }

wait_for() {  # wait_for <description> <timeout_s> <command...>
    local desc="$1" timeout="$2"; shift 2
    local i
    for ((i = 0; i < timeout * 2; i++)); do
        if "$@" >/dev/null 2>&1; then log "ok: ${desc}"; return 0; fi
        sleep 0.5
    done
    return 1
}

# -----------------------------------------------------------------------------
# 1. Gazebo server, headless by construction
# -----------------------------------------------------------------------------
# `gz sim -s` is the server only: no GUI, no window, no GPU needed. This is the
# headless requirement, and it is also why PX4 runs with PX4_GZ_STANDALONE=1
# below (PX4 must not try to start its own Gazebo).
log "starting Gazebo server (world=${WORLD}, headless)"
log "  server config: ${GZ_SIM_SERVER_CONFIG_PATH:-<stock>}"
# --headless-rendering makes the Sensors system render offscreen, which is what
# we want with no GPU passthrough (it still goes through Mesa llvmpipe; the
# container provides OpenGL 4.5 via Xvfb on $DISPLAY as a fallback).
nohup gz sim -s -r -v 2 --headless-rendering --render-engine ogre2 \
    "${WORLD}.sdf" > "${LOGDIR}/gz_server.log" 2>&1 &
wait_for "gz world '${WORLD}' is up" 120 \
    bash -c "gz topic -l 2>/dev/null | grep -q '^/world/${WORLD}/'" \
    || { tail -30 "${LOGDIR}/gz_server.log"; fail "Gazebo server did not come up"; }

# -----------------------------------------------------------------------------
# 2. PX4 SITL
# -----------------------------------------------------------------------------
log "starting PX4 SITL (instance ${PX4_INSTANCE}, model ${MODEL})"
cd "${PX4_DIR}"
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL="${MODEL}" \
PX4_GZ_WORLD="${WORLD}" \
PX4_GZ_MODEL_POSE="${MODEL_POSE}" \
nohup "${PX4_DIR}/build/px4_sitl_default/bin/px4" -i "${PX4_INSTANCE}" \
    > "${LOGDIR}/px4.log" 2>&1 &

# PX4 names the spawned model "<model>_<instance>", e.g. x500_depth_1, which is
# exactly what multi_slam/config/gz_bridge.yaml subscribes to.
EXPECTED_MODEL="x500_depth_${PX4_INSTANCE}"
wait_for "model ${EXPECTED_MODEL} spawned" 180 \
    bash -c "gz topic -l 2>/dev/null | grep -q '/world/${WORLD}/model/${EXPECTED_MODEL}/'" \
    || { tail -40 "${LOGDIR}/px4.log"; fail "PX4 did not spawn ${EXPECTED_MODEL}"; }

wait_for "camera sensor topics present" 180 \
    bash -c "gz topic -l 2>/dev/null | grep -q 'sensor/IMX214/image'" \
    || { tail -40 "${LOGDIR}/gz_server.log"; fail "RGB camera sensor never appeared"; }

# -----------------------------------------------------------------------------
# 3. Micro XRCE-DDS agent (PX4 uORB <-> ROS 2, needed for offboard control)
# -----------------------------------------------------------------------------
log "starting MicroXRCEAgent on udp4:8888"
nohup MicroXRCEAgent udp4 -p 8888 > "${LOGDIR}/xrce.log" 2>&1 &
sleep 2

# -----------------------------------------------------------------------------
# 4. ros_gz_bridge  (RGB, depth, camera_info, points, /clock, ground truth)
# -----------------------------------------------------------------------------
BRIDGE_CFG="$(ros2 pkg prefix --share multi_slam)/config/gz_bridge.yaml"
[[ -f "${BRIDGE_CFG}" ]] || fail "bridge config not found at ${BRIDGE_CFG}"
log "starting ros_gz_bridge with ${BRIDGE_CFG}"
nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p "config_file:=${BRIDGE_CFG}" \
    -p use_sim_time:=true \
    > "${LOGDIR}/gz_bridge.log" 2>&1 &

# /clock must flow before anything with use_sim_time:=true starts, otherwise
# those nodes block on a zero clock and look hung.
wait_for "/clock is publishing" 90 \
    bash -c "timeout 10 ros2 topic echo /clock --once >/dev/null 2>&1" \
    || { tail -30 "${LOGDIR}/gz_bridge.log"; fail "/clock never arrived - use_sim_time nodes would hang"; }

wait_for "${NAMESPACE} RGB images flowing" 120 \
    bash -c "timeout 20 ros2 topic echo /${NAMESPACE}/rgb/image_raw --once >/dev/null 2>&1" \
    || { tail -30 "${LOGDIR}/gz_bridge.log"; fail "no RGB images on /${NAMESPACE}/rgb/image_raw"; }

wait_for "${NAMESPACE} depth images flowing" 120 \
    bash -c "timeout 20 ros2 topic echo /${NAMESPACE}/depth/image --once >/dev/null 2>&1" \
    || { tail -30 "${LOGDIR}/gz_bridge.log"; fail "no depth images on /${NAMESPACE}/depth/image"; }

# -----------------------------------------------------------------------------
# 5. Static frames
# -----------------------------------------------------------------------------
log "starting static frames for ${NAMESPACE}"
nohup ros2 launch multi_slam static_frames.launch.py \
    robot_namespace:="${NAMESPACE}" use_sim_time:=true \
    > "${LOGDIR}/static_frames.log" 2>&1 &
sleep 3

# -----------------------------------------------------------------------------
# 6. ORB-SLAM3 RGB-D agent (connects to the COVINS backend over TCP on start)
# -----------------------------------------------------------------------------
log "starting ORB-SLAM3 RGB-D for ${NAMESPACE} (loads the ORB vocabulary; this takes ~30-60 s)"
nohup ros2 launch orb_slam3_ros2_wrapper rgbd.launch.py \
    robot_namespace:="${NAMESPACE}" use_sim_time:=true \
    > "${LOGDIR}/orb_slam3.log" 2>&1 &

wait_for "ORB-SLAM3 node registered" 300 \
    bash -c "ros2 node list 2>/dev/null | grep -qi 'ORB_SLAM3\|orb_slam3'" \
    || { tail -40 "${LOGDIR}/orb_slam3.log"; fail "ORB-SLAM3 node never appeared"; }

log "----------------------------------------------------------------"
log "stack up. logs in ${LOGDIR}/"
log "COVINS target: $(grep -E '^sys\.(server_ip|port)' /root/.local/config/config_comm.yaml 2>/dev/null | tr '\n' ' ')"
log "next: /opt/scripts/check_rates.sh"
log "----------------------------------------------------------------"
