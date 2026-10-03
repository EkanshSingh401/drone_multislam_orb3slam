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
# Which rig to bring up. Overridable so the same script serves both the phase-1
# x500_depth (RGB-D) drone and the phase-2 D455-mirror (stereo IR + depth + IMU)
# drone.
#   MODEL          PX4_SIM_MODEL, e.g. gz_x500_depth | gz_x500_d455
#   BRIDGE_CFG     ros_gz_bridge YAML (default: multi_slam's installed config)
#   PROBE_SENSOR   gz topic substring proving the cameras render
#   PROBE_ROS      ROS topics that must carry data before SLAM starts
#   START_SLAM     1 to launch ORB-SLAM3 RGB-D, 0 to skip (no stereo-inertial
#                  node exists in the wrapper yet -- see docker/PATCHES.md)
BRIDGE_CFG="${BRIDGE_CFG:-}"
PROBE_SENSOR="${PROBE_SENSOR:-}"
PROBE_ROS="${PROBE_ROS:-}"
START_SLAM="${START_SLAM:-1}"
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
            pkill -f px4_gcs.py 2>/dev/null || true
            pkill -f 'bin/px4' 2>/dev/null || true
            pkill -f 'gz sim' 2>/dev/null || true
            pkill -f 'ruby.*gz' 2>/dev/null || true
            sleep 2; echo "stopped."; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

# Defaults that depend on which rig was selected.
case "${MODEL}" in
    *x500_d455*)
        : "${PROBE_SENSOR:=sensor/infra1/image}"
        : "${PROBE_ROS:=/camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/depth/image_rect_raw /camera/imu}"
        : "${BRIDGE_CFG:=/opt/config_sim_only/gz_bridge_d455.yaml}"
        ;;
    *)
        : "${PROBE_SENSOR:=sensor/IMX214/image}"
        : "${PROBE_ROS:=/${NAMESPACE}/rgb/image_raw /${NAMESPACE}/depth/image}"
        ;;
esac

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
# -d = daemon mode: do NOT start the interactive pxh shell. Without it PX4 writes
# its "pxh> " prompt plus an ANSI clear-line to the redirected log on every loop;
# a ~45 min run produced a 72 MB px4.log of pure prompt spam.
#
# The env vars below MUST stay contiguous with the nohup line. A comment placed
# between a backslash-continued assignment list and the command silently breaks
# the continuation: bash then treats the assignments as a standalone statement
# and runs px4 with NONE of them, which makes PX4 log "No autostart ID found"
# and fall back to the SIH simulator (no Gazebo model, no sensors at all).
# PX4_PARAM_<NAME> is applied by PX4's rcS via `param set` (see rcS: "Allow
# overriding parameters via env variables"). Two are required to arm in OFFBOARD
# on a headless SITL rig with no RC transmitter and no ground station:
#
#   COM_RCL_EXCEPT=4   bitmask of modes exempt from RC-loss failsafe; bit 2 is
#                      Offboard. Without it, arming is blocked by the
#                      manual_control_signal_lost failsafe flag.
#   NAV_DLL_ACT=0      disable the datalink-loss action, which otherwise blocks
#                      arming via gcs_connection_lost.
#
# Both were confirmed necessary by measurement, not guessed: px4_arm_test.py
# showed OFFBOARD being accepted (nav_state 4 -> 14) while arming failed with
# exactly those two flags set, plus auto_mission_missing (irrelevant here --
# offboard needs no mission).
env PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART=4001 \
    PX4_PARAM_COM_RCL_EXCEPT=4 \
    PX4_PARAM_NAV_DLL_ACT=0 \
    PX4_SIM_MODEL="${MODEL}" \
    PX4_GZ_WORLD="${WORLD}" \
    PX4_GZ_MODEL_POSE="${MODEL_POSE}" \
    nohup "${PX4_DIR}/build/px4_sitl_default/bin/px4" -d -i "${PX4_INSTANCE}" \
    > "${LOGDIR}/px4.log" 2>&1 &

# PX4 names the spawned model "<model>_<instance>", e.g. x500_depth_1, which is
# exactly what multi_slam/config/gz_bridge.yaml subscribes to.
# Derive the spawned model name from MODEL rather than hardcoding x500_depth.
# PX4 spawns "<PX4_SIM_MODEL with the gz_ prefix stripped>_<instance>", so
# gz_x500_d455 -i 1 -> x500_d455_1. Hardcoding it made the D455 rig wait for a
# model that never appears.
EXPECTED_MODEL="${MODEL#gz_}_${PX4_INSTANCE}"
wait_for "model ${EXPECTED_MODEL} spawned" 180 \
    bash -c "gz topic -l 2>/dev/null | grep -q '/world/${WORLD}/model/${EXPECTED_MODEL}/'" \
    || { tail -40 "${LOGDIR}/px4.log"; fail "PX4 did not spawn ${EXPECTED_MODEL}"; }

wait_for "camera sensor topics present (${PROBE_SENSOR})" 240 \
    bash -c "gz topic -l 2>/dev/null | grep -q '${PROBE_SENSOR}'" \
    || { tail -40 "${LOGDIR}/gz_server.log"; fail "camera sensor ${PROBE_SENSOR} never appeared"; }

# -----------------------------------------------------------------------------
# 2b. MAVLink GCS (required before anything else touches PX4's MAVLink)
# -----------------------------------------------------------------------------
# PX4 refuses to arm with "Preflight Fail: No connection to the GCS" whenever
# NAV_DLL_ACT > 0, and the x500 airframe leaves it at 2. Attaching a minimal GCS
# satisfies that check and lets us set NAV_DLL_ACT=0 over MAVLink.
#
# This MUST start before any other MAVLink client: PX4's MAVLink instance locks
# onto the first partner's source address:port and never re-learns, so a second
# client is simply ignored. Starting it here claims the slot.
log "starting MAVLink GCS (satisfies the GCS arming check)"
nohup python3 -u /opt/scripts/px4_gcs.py --serve > "${LOGDIR}/px4_gcs.log" 2>&1 &
wait_for "GCS attached to PX4" 120 \
    bash -c "grep -qE 'from system [0-9]+' '${LOGDIR}/px4_gcs.log'" \
    || log "WARNING: GCS did not attach; arming will likely be denied"

# -----------------------------------------------------------------------------
# 3. Micro XRCE-DDS agent (PX4 uORB <-> ROS 2, needed for offboard control)
# -----------------------------------------------------------------------------
log "starting MicroXRCEAgent on udp4:8888"
nohup MicroXRCEAgent udp4 -p 8888 > "${LOGDIR}/xrce.log" 2>&1 &
sleep 2

# -----------------------------------------------------------------------------
# 4. ros_gz_bridge  (RGB, depth, camera_info, points, /clock, ground truth)
# -----------------------------------------------------------------------------
if [[ -z "${BRIDGE_CFG}" ]]; then
    BRIDGE_CFG="$(ros2 pkg prefix --share multi_slam)/config/gz_bridge.yaml"
fi
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

for t in ${PROBE_ROS}; do
    wait_for "data flowing on ${t}" 180 \
        bash -c "timeout 25 ros2 topic echo ${t} --once >/dev/null 2>&1" \
        || { tail -30 "${LOGDIR}/gz_bridge.log"; fail "no data on ${t}"; }
done

# -----------------------------------------------------------------------------
# 5. Static frames
# -----------------------------------------------------------------------------
log "starting static frames for ${NAMESPACE}"
nohup ros2 launch multi_slam static_frames.launch.py \
    robot_namespace:="${NAMESPACE}" use_sim_time:=true \
    > "${LOGDIR}/static_frames.log" 2>&1 &

# Verify, do not just sleep. These three static_transform_publisher processes
# previously aborted with SIGABRT about a second after launch (a bool parameter
# passed as a string), leaving /tf_static empty -- and because this step only
# slept, bringup still reported success with no TF tree at all.
wait_for "static TF published on /tf_static" 90 \
    bash -c "ros2 topic info /tf_static 2>/dev/null | grep -qE 'Publisher count: [1-9]'" \
    || { tail -20 "${LOGDIR}/static_frames.log"; fail "static TF publishers did not come up"; }

# -----------------------------------------------------------------------------
# 6. ORB-SLAM3 RGB-D agent (connects to the COVINS backend over TCP on start)
# -----------------------------------------------------------------------------
if [[ "${START_SLAM}" != "1" ]]; then
    log "----------------------------------------------------------------"
    log "stack up (START_SLAM=0, no SLAM node). logs in ${LOGDIR}/"
    log "next: /opt/scripts/rate_monitor.py  or  /opt/scripts/check_rates.sh"
    log "----------------------------------------------------------------"
    exit 0
fi

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
