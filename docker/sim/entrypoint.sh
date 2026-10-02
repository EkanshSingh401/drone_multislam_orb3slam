#!/usr/bin/env bash
# Entrypoint for the ROS 2 simulation container.
#  - sources ROS 2 + the colcon workspace
#  - points the COVINS agent transport at the backend container
#  - brings up a headless X server with VNC/noVNC for RViz
# NOTE: deliberately no `set -u`. ROS 2's /opt/ros/<distro>/setup.bash reads
# AMENT_TRACE_SETUP_FILES and friends without defaulting them, so `set -u`
# aborts the entrypoint with "AMENT_TRACE_SETUP_FILES: unbound variable".
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="${WS:-/root/ws_offboard_control}"
COVINS_PREFIX="${COVINS_PREFIX:-/root/.local}"

# --- COVINS server address ---------------------------------------------------
# config_comm.hpp bakes this file's path in at compile time (derived from
# __FILE__), so the frontend always reads exactly this file. Rewriting the YAML
# here is what aims the agent at the backend container; there is no env var or
# ROS parameter for it. ConnectToServer() uses getaddrinfo(), so a Compose
# service name works as well as an IP.
CONF_COMM="${COVINS_PREFIX}/config/config_comm.yaml"

# If Compose mounted the shared config (read-only) from the host, take a
# writable copy into the baked location. Both containers mount the SAME host
# file, which is what keeps the agent's and the backend's comm parameters
# (start_sending_after_kf, kf_buffer_withold, ...) in agreement.
if [[ -f /opt/covins-config/config_comm.yaml ]]; then
    mkdir -p "$(dirname "${CONF_COMM}")"
    cp /opt/covins-config/config_comm.yaml "${CONF_COMM}"
fi

if [[ -n "${COVINS_SERVER_IP:-}" ]]; then
    if [[ -f "${CONF_COMM}" ]]; then
        sed -i -E "s|^sys\.server_ip:.*|sys.server_ip: '${COVINS_SERVER_IP}'|" "${CONF_COMM}"
    else
        echo "entrypoint: WARNING ${CONF_COMM} is missing." >&2
        echo "entrypoint: covins_params would initialise silently from a missing file." >&2
    fi
fi
if [[ -f "${CONF_COMM}" ]]; then
    echo "entrypoint: COVINS agent -> $(grep -E '^sys\.(server_ip|port)' "${CONF_COMM}" | tr '\n' ' ')"
fi

# --- Headless display --------------------------------------------------------
# No GPU passthrough on macOS Docker, so everything renders through Mesa
# llvmpipe. Gazebo itself runs headless (gz sim -s, server only); this X server
# exists for RViz and other GUI tools.
if [[ "${START_VNC:-1}" == "1" ]]; then
    DISPLAY_NUM="${DISPLAY#:}"
    if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        Xvfb "${DISPLAY}" -screen 0 "${VNC_GEOMETRY:-1600x900x24}" -nolisten tcp >/tmp/xvfb.log 2>&1 &
        for _ in $(seq 1 40); do
            xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1 && break
            sleep 0.25
        done
        fluxbox >/tmp/fluxbox.log 2>&1 &
        x11vnc -display "${DISPLAY}" -forever -shared -nopw -quiet \
               -rfbport "${VNC_PORT:-5901}" >/tmp/x11vnc.log 2>&1 &
        websockify --web=/usr/share/novnc "${NOVNC_PORT:-6901}" \
                   "localhost:${VNC_PORT:-5901}" >/tmp/novnc.log 2>&1 &
        echo "entrypoint: noVNC on http://localhost:${NOVNC_PORT:-6901}/vnc.html (VNC :${DISPLAY_NUM} / port ${VNC_PORT:-5901})"
    fi
fi

# --- ROS environment ---------------------------------------------------------
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
fi

exec "$@"
