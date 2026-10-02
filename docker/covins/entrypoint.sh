#!/usr/bin/env bash
# Entrypoint for the COVINS backend container (ROS Melodic).
#  - sources ROS 1 + the catkin workspace
#  - optionally starts roscore (the backend node needs a master)
#  - brings up a headless X server with VNC/noVNC for COVINS RViz
set -euo pipefail

CATKIN_WS="${CATKIN_WS:-/root/covins_ws}"

# --- Headless display for COVINS RViz ----------------------------------------
if [[ "${START_VNC:-1}" == "1" ]]; then
    if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        Xvfb "${DISPLAY}" -screen 0 "${VNC_GEOMETRY:-1600x900x24}" -nolisten tcp >/tmp/xvfb.log 2>&1 &
        for _ in $(seq 1 40); do
            xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1 && break
            sleep 0.25
        done
        fluxbox >/tmp/fluxbox.log 2>&1 &
        x11vnc -display "${DISPLAY}" -forever -shared -nopw -quiet \
               -rfbport "${VNC_PORT:-5902}" >/tmp/x11vnc.log 2>&1 &
        websockify --web=/usr/share/novnc "${NOVNC_PORT:-6902}" \
                   "localhost:${VNC_PORT:-5902}" >/tmp/novnc.log 2>&1 &
        echo "covins-entrypoint: noVNC on http://localhost:${NOVNC_PORT:-6902}/vnc.html"
    fi
fi

# --- ROS environment ---------------------------------------------------------
# shellcheck disable=SC1090,SC1091
source /opt/ros/melodic/setup.bash
source "${CATKIN_WS}/devel/setup.bash"

# The backend node is a roscpp node and needs a master. Starting it here keeps
# the backend self-contained on the Compose network.
if [[ "${START_ROSCORE:-1}" == "1" ]]; then
    if ! rostopic list >/dev/null 2>&1; then
        echo "covins-entrypoint: starting roscore"
        roscore >/tmp/roscore.log 2>&1 &
        for _ in $(seq 1 60); do
            rostopic list >/dev/null 2>&1 && break
            sleep 0.5
        done
        rostopic list >/dev/null 2>&1 \
            && echo "covins-entrypoint: roscore up" \
            || { echo "covins-entrypoint: roscore FAILED to come up" >&2; tail -20 /tmp/roscore.log >&2; }
    fi
fi

# Report the comm settings the backend will actually use. start_sending_after_kf
# is the usual reason the backend looks idle right after an agent connects.
CONF_COMM="${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml"
if [[ -f "${CONF_COMM}" ]]; then
    echo "covins-entrypoint: $(grep -E '^(sys\.port|comm\.start_sending_after_kf|comm\.kf_buffer_withold)' "${CONF_COMM}" | tr '\n' ' ')"
fi

exec "$@"
