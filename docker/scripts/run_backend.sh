#!/usr/bin/env bash
# Start the COVINS backend node (run INSIDE the covins-backend container).
#
#   /opt/scripts/run_backend.sh            # start the backend node
#   /opt/scripts/run_backend.sh --rviz     # also start COVINS RViz on the VNC display
set -eo pipefail   # no -u: ROS setup.bash reads unset vars

CATKIN_WS="${CATKIN_WS:-/root/covins_ws}"
LOGDIR="${LOGDIR:-/out/logs}"
WITH_RVIZ=0
[[ "${1:-}" == "--rviz" ]] && WITH_RVIZ=1

mkdir -p "${LOGDIR}"
source /opt/ros/melodic/setup.bash
source "${CATKIN_WS}/devel/setup.bash"

if ! rostopic list >/dev/null 2>&1; then
    echo "run_backend: no ROS master reachable at ${ROS_MASTER_URI:-unset}" >&2
    echo "run_backend: the container entrypoint normally starts roscore (START_ROSCORE=1)" >&2
    exit 1
fi

if [[ ${WITH_RVIZ} -eq 1 ]]; then
    RVIZ_CFG="${CATKIN_WS}/src/covins/covins_backend/config/covins.rviz"
    echo "run_backend: starting COVINS RViz with ${RVIZ_CFG}"
    nohup rviz -d "${RVIZ_CFG}" > "${LOGDIR}/covins_rviz.log" 2>&1 &
fi

echo "run_backend: starting covins_backend_node (listening on TCP $(grep -E '^sys\.port' "${CATKIN_WS}/src/covins/covins_comm/config/config_comm.yaml" | tr -d "sys.port: '"))"
echo "run_backend: logging to ${LOGDIR}/covins_backend.log"
echo "run_backend: NOTE comm.start_sending_after_kf delays the first keyframe -- the"
echo "             agent must build that many keyframes locally before it sends anything."

# stdbuf keeps the backend's progress visible in the log in real time.
exec stdbuf -oL -eL rosrun covins_backend covins_backend_node 2>&1 \
    | tee "${LOGDIR}/covins_backend.log"
