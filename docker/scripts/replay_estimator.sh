#!/usr/bin/env bash
# Replay a recorded sensor bag through ONE estimator (run INSIDE the sim container).
#
#   /opt/scripts/replay_estimator.sh --bag /out/eval/<run>/flight.bag \
#        --estimator stereo_inertial|stereo|openvins [--out /out/replay/<name>]
#
# Why replay instead of a second live flight: two live flights differ in
# scheduling, in how many frames the renderer manages, and in PX4's exact
# trajectory. Replaying one recorded bag gives every estimator BYTE-IDENTICAL
# input, so a difference in the result is a difference in the estimator. The bag
# must therefore have been recorded with RECORD_SENSORS=1.
#
# Playback runs at the bag's own recorded pacing (normal rate). The bag carries
# /clock, so it is replayed as an ordinary topic and every node with
# use_sim_time follows it -- `ros2 bag play --clock` is deliberately NOT used,
# because that would add a SECOND /clock publisher competing with the recorded
# one.
set -eo pipefail

BAG=""; EST=""; OUTDIR=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag)       BAG="$2"; shift 2 ;;
        --estimator) EST="$2"; shift 2 ;;
        --out)       OUTDIR="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ -n "${BAG}" && -n "${EST}" ]] || { echo "need --bag and --estimator" >&2; exit 2; }
[[ -d "${BAG}" ]] || { echo "bag not found: ${BAG}" >&2; exit 2; }
OUTDIR="${OUTDIR:-/out/replay/$(basename "$(dirname "${BAG}")")_${EST}}"
mkdir -p "${OUTDIR}"

source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /root/ws_offboard_control/install/setup.bash
WS_OV="/root/ws_offboard_control/install/ov_msckf/lib/ov_msckf"

hr() { printf '%s\n' "------------------------------------------------------------"; }
log() { echo "[replay $(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------------------
# 1. The bag must actually contain the sensor streams.
# ---------------------------------------------------------------------------
# A bag recorded without RECORD_SENSORS=1 has the pose topics but no images, and
# the estimator would sit there receiving nothing while playback ran to
# completion -- producing an empty trajectory and a confusing "no samples"
# error several minutes later. Check up front.
hr; log "bag contents"
python3 /opt/scripts/bag_to_tum.py "${BAG}" --list --out /dev/null \
    | tee "${OUTDIR}/bag_topics.txt"
PUB_L=$(awk '$1=="/camera/infra1/image_rect_raw"{print $NF}' "${OUTDIR}/bag_topics.txt")
PUB_R=$(awk '$1=="/camera/infra2/image_rect_raw"{print $NF}' "${OUTDIR}/bag_topics.txt")
PUB_IMU=$(awk '$1=="/camera/imu"{print $NF}' "${OUTDIR}/bag_topics.txt")
if [[ -z "${PUB_L}" || "${PUB_L}" == "0" ]]; then
    echo "replay: bag has no /camera/infra1/image_rect_raw messages." >&2
    echo "  Re-record it with RECORD_SENSORS=1 -- replay needs the raw streams." >&2
    exit 3
fi
log "published: infra1=${PUB_L} infra2=${PUB_R} imu=${PUB_IMU}"

case "${EST}" in
  stereo_inertial) EST_TOPIC="/robot_pose_slam"; LAUNCH="stereo_inertial.launch.py" ;;
  stereo)          EST_TOPIC="/robot_pose_slam"; LAUNCH="stereo_d455.launch.py" ;;
  openvins)        EST_TOPIC="/ov_msckf/poseimu";  LAUNCH="" ;;
  *) echo "estimator must be stereo_inertial, stereo or openvins" >&2; exit 2 ;;
esac

# ---------------------------------------------------------------------------
# 2. Start the estimator and wait for it to be ready.
# ---------------------------------------------------------------------------
hr; log "starting estimator: ${EST}"
if [[ "${EST}" == "openvins" ]]; then
    # DEBUG verbosity on purpose: the only per-frame signal OpenVINS emits is
    # PRINT_DEBUG("[TIME]: %.4f seconds for tracking") in VioManager, and
    # counting those lines is how "frames processed" is measured directly from
    # the estimator rather than inferred from its output rate.
    # run_subscribe_msckf is invoked DIRECTLY, not through subscribe.launch.py,
    # because the launch file forwards only its own declared arguments and the
    # output filepaths are not among them.
    #
    # That matters: with save_total_state true, ROS2Visualizer reads
    # filepath_est / filepath_std from ROS PARAMETERS, not from the estimator
    # YAML (the YAML copies are read only by run_simulation). Absent those
    # parameters it falls back to "state_estimate.txt", whose parent_path() is
    # EMPTY, and then
    #     boost::filesystem::create_directories(parent_path())
    # throws on an empty path and the node aborts before it starts:
    #     terminate called after throwing an instance of
    #     'boost::filesystem::filesystem_error'
    #       what(): create_directories: Invalid argument [generic:22]
    # So the paths must be absolute AND have a directory component. Pointing
    # them into the run directory also keeps each replay's state files with its
    # own results.
    #
    # The node is created with allow_undeclared_parameters(true) and
    # automatically_declare_parameters_from_overrides(true)
    # (run_subscribe_msckf.cpp:62-65), so -p overrides are picked up by
    # has_parameter() without the launch file declaring them.
    # The config actually used is copied into the run directory, together with
    # the two kalibr files it references by RELATIVE path, so every replay keeps
    # an exact record of its estimator settings. OV_GRAVITY_MAG overrides ONLY
    # gravity_mag (PATCHES.md s44: the 9.80 vs 9.81 replay comparison); the
    # edit is asserted so a renamed key fails loudly instead of silently
    # replaying the default.
    OV_CFG_DIR="${OUTDIR}/ov_config"
    mkdir -p "${OV_CFG_DIR}"
    cp /opt/config_sim_only/openvins_estimator_config.yaml \
       /opt/config_sim_only/kalibr_imu_chain.yaml \
       /opt/config_sim_only/kalibr_imucam_chain.yaml "${OV_CFG_DIR}/"
    if [[ -n "${OV_GRAVITY_MAG:-}" ]]; then
        sed -i -E "s|^gravity_mag:.*|gravity_mag: ${OV_GRAVITY_MAG}|" \
            "${OV_CFG_DIR}/openvins_estimator_config.yaml"
        grep -qx "gravity_mag: ${OV_GRAVITY_MAG}" "${OV_CFG_DIR}/openvins_estimator_config.yaml" \
            || { echo "replay: failed to set gravity_mag=${OV_GRAVITY_MAG}" >&2; exit 2; }
    fi
    log "OpenVINS $(grep -E '^gravity_mag:' "${OV_CFG_DIR}/openvins_estimator_config.yaml")"
    nohup "${WS_OV}/run_subscribe_msckf" --ros-args \
        -r __ns:=/ov_msckf \
        -p use_sim_time:=true \
        -p config_path:="${OV_CFG_DIR}/openvins_estimator_config.yaml" \
        -p verbosity:=DEBUG \
        -p max_cameras:=2 \
        -p use_stereo:=true \
        -p save_total_state:=true \
        -p filepath_est:="${OUTDIR}/ov_state_est.txt" \
        -p filepath_std:="${OUTDIR}/ov_state_std.txt" \
        -p filepath_gt:="${OUTDIR}/ov_state_gt.txt" \
        > "${OUTDIR}/openvins.log" 2>&1 &
    EST_PID=$!
    for _ in $(seq 1 120); do
        ros2 node list 2>/dev/null | grep -qi "ov_msckf" && break
        sleep 1
    done
    ros2 node list 2>/dev/null | grep -qi "ov_msckf" \
        || { tail -40 "${OUTDIR}/openvins.log"; echo "replay: ov_msckf never appeared" >&2; exit 4; }
    # Fail loudly if the joint-covariance publisher was compiled out: the phase-3
    # covariance checks depend on it and a warning in a log is easy to miss.
    if grep -q "NOT compiled in" "${OUTDIR}/openvins.log"; then
        echo "replay: WARNING joint covariance publisher is NOT compiled in." >&2
        echo "  active_slam_msgs was absent when ov_msckf was built." >&2
        echo "  JOINT_COV_MISSING" > "${OUTDIR}/WARNINGS.txt"
    fi
    nohup python3 -u /opt/scripts/ov_pose_to_file.py \
        --topic /ov_msckf/odomimu --out "${OUTDIR}/ov_est_cov.txt" \
        > "${OUTDIR}/ov_pose_to_file.log" 2>&1 &
    OVREC_PID=$!
else
    nohup ros2 launch orb_slam3_ros2_wrapper "${LAUNCH}" use_sim_time:=true \
        > "${OUTDIR}/orb_slam3.log" 2>&1 &
    EST_PID=$!
    for _ in $(seq 1 300); do
        ros2 node list 2>/dev/null | grep -qi "ORB_SLAM3" && break
        sleep 1
    done
    ros2 node list 2>/dev/null | grep -qi "ORB_SLAM3" \
        || { tail -40 "${OUTDIR}/orb_slam3.log"; echo "replay: ORB-SLAM3 never appeared" >&2; exit 4; }
    OVREC_PID=""
fi
log "estimator up (pid ${EST_PID})"

# ---------------------------------------------------------------------------
# 3. Record the estimator's output on the replayed timeline.
# ---------------------------------------------------------------------------
# Ground truth and /clock come straight out of the input bag during playback, so
# recording them again here keeps estimate and truth on ONE timeline and lets
# the existing bag_to_tum + evo path be reused unchanged.
ros2 bag record -s mcap -o "${OUTDIR}/out.bag" \
    "${EST_TOPIC}" /ground_truth/pose_info /clock \
    > "${OUTDIR}/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 4

# ---------------------------------------------------------------------------
# 4. Play the bag at its recorded rate.
# ---------------------------------------------------------------------------
hr; log "playing bag (normal rate; /clock comes from the bag)"
ros2 bag play "${BAG}" 2>&1 | tee "${OUTDIR}/bag_play.log" || true
log "playback finished"
sleep 5

# ---------------------------------------------------------------------------
# 5. Stop everything. Same escalation ladder as record_and_eval.sh: rosbag2
#    ignores SIGINT and does not always stop on SIGTERM, and `wait` on a live
#    child never returns.
# ---------------------------------------------------------------------------
stop_pid() {  # stop_pid <pid> <name>
    local pid="$1" name="$2"
    [[ -n "${pid}" ]] || return 0
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do kill -0 "${pid}" 2>/dev/null || return 0; sleep 1; done
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in $(seq 1 15); do kill -0 "${pid}" 2>/dev/null || return 0; sleep 1; done
    echo "    ${name} ignored SIGINT and SIGTERM, sending SIGKILL"
    kill -KILL "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do kill -0 "${pid}" 2>/dev/null || return 0; sleep 1; done
    echo "    WARNING ${name} survived SIGKILL; not waiting on it" >&2
}
stop_pid "${BAG_PID}" "recorder"
pkill -TERM -f "ros2 bag record" 2>/dev/null || true
stop_pid "${OVREC_PID}" "ov_pose_to_file"
stop_pid "${EST_PID}" "estimator"
pkill -TERM -f "run_subscribe_msckf|orb_slam3_ros2_wrapper/(stereo_inertial|stereo)" 2>/dev/null || true
sleep 3
if [[ ! -f "${OUTDIR}/out.bag/metadata.yaml" ]]; then
    log "no metadata.yaml, reindexing output bag"
    ros2 bag reindex "${OUTDIR}/out.bag" -s mcap >/dev/null 2>&1 || true
fi

# ---------------------------------------------------------------------------
# 6. Frames processed vs frames published.
# ---------------------------------------------------------------------------
hr; log "frames processed vs published"
{
  echo "published_infra1=${PUB_L}"
  echo "published_infra2=${PUB_R}"
  echo "published_imu=${PUB_IMU}"
  if [[ "${EST}" == "openvins" ]]; then
      # One "seconds for tracking" line per frame OpenVINS actually tracked.
      PROC=$(grep -c "seconds for tracking" "${OUTDIR}/openvins.log" || true)
      echo "processed_frames=${PROC}"
      echo "source=openvins [TIME] tracking lines (VioManager PRINT_DEBUG)"
      # The throttle that would silently drop frames, stated explicitly.
      TF=$(grep -oE "track_frequency:[[:space:]]*[0-9.]+" \
            "${OV_CFG_DIR}/openvins_estimator_config.yaml" | grep -oE "[0-9.]+$")
      echo "track_frequency=${TF}"
      echo "min_allowed_gap_s=$(python3 -c "print(f'{1.0/${TF}:.6f}')")"
  else
      # The wrapper prints its own frame counter on shutdown.
      PROC=$(sed 's/\x1b\[[0-9;]*m//g' "${OUTDIR}/orb_slam3.log" \
             | grep -oE "frames=[0-9]+" | tail -1 | grep -oE "[0-9]+" || true)
      echo "processed_frames=${PROC:-unknown}"
      echo "source=wrapper shutdown counter"
  fi
} | tee "${OUTDIR}/frames.txt"

hr
log "replay output in ${OUTDIR}"
