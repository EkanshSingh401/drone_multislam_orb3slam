#!/usr/bin/env bash
# Record a scripted flight and evaluate ATE/RPE with evo (run INSIDE the sim container).
#
#   /opt/scripts/record_and_eval.sh                # record + fly + evaluate
#   /opt/scripts/record_and_eval.sh --eval-only DIR
#
# Produces, under /out/eval/<run>/:
#   flight.bag/        rosbag2 recording
#   est_orbslam3.tum   ORB-SLAM3 estimate  (from /uav_1/robot_pose_slam)
#   gt.tum             Gazebo ground truth (from /ground_truth/pose_info)
#   est_covins.tum     COVINS optimised keyframe trajectory, if exported
#   ape_*.zip rpe_*.zip + *.txt   evo results
set -o pipefail    # no -u: ROS setup.bash reads unset vars

NAMESPACE="${NAMESPACE:-uav_1}"
MODEL="${MODEL:-x500_depth_1}"
OUTROOT="${OUTROOT:-/out/eval}"
SIDE="${SIDE:-5}"
ALT="${ALT:-2.0}"
LEG_TIME="${LEG_TIME:-12}"
SETTLE="${SETTLE:-15}"       # seconds of hover before flying, to let SLAM initialise

EVAL_ONLY=""
[[ "${1:-}" == "--eval-only" ]] && EVAL_ONLY="${2:-}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS:-/root/ws_offboard_control}/install/setup.bash"

hr() { printf '%s\n' "============================================================"; }

if [[ -n "${EVAL_ONLY}" ]]; then
    RUNDIR="${EVAL_ONLY}"
else
    RUNDIR="${OUTROOT}/$(date +%Y%m%d-%H%M%S)"
    mkdir -p "${RUNDIR}"

    hr; echo "RECORDING + SCRIPTED FLIGHT -> ${RUNDIR}"; hr

    # Sanity-check the inputs before flying, so a failed run is diagnosable.
    for t in "/${NAMESPACE}/robot_pose_slam" "/ground_truth/pose_info"; do
        if ! ros2 topic info "$t" >/dev/null 2>&1; then
            echo "record_and_eval: required topic $t does not exist." >&2
            echo "  Is the stack up? Run /opt/scripts/bringup_sim.sh first." >&2
            exit 1
        fi
    done
    if ! timeout 20 ros2 topic echo "/${NAMESPACE}/robot_pose_slam" --once >/dev/null 2>&1; then
        echo "record_and_eval: WARNING no pose on /${NAMESPACE}/robot_pose_slam yet." >&2
        echo "  ORB-SLAM3 has probably not initialised. Recording anyway, but the" >&2
        echo "  estimate may be empty or start late." >&2
    fi

    echo "--> starting rosbag2"
    ros2 bag record -s mcap -o "${RUNDIR}/flight.bag" \
        "/${NAMESPACE}/robot_pose_slam" \
        "/ground_truth/pose_info" \
        /clock /tf /tf_static \
        > "${RUNDIR}/rosbag.log" 2>&1 &
    BAG_PID=$!
    sleep 5

    # Measure RTF and stream rates for THIS run. The gz stats topic's
    # real_time_factor is instantaneous and bimodal (0.032 vs 0.538 on identical
    # setups), so the only trustworthy figure is sim-time/wall-time over the
    # window, which rate_monitor computes from /clock.
    python3 -u /opt/scripts/rate_monitor.py --namespace "${NAMESPACE}" \
        --duration "${MONITOR_DURATION:-900}" --window 15 \
        --out "${RUNDIR}/rates.csv" > "${RUNDIR}/rates.log" 2>&1 &
    MON_PID=$!

    echo "--> hovering ${SETTLE}s to let ORB-SLAM3 initialise, then flying"
    python3 /opt/scripts/fly_path.py \
        --side "${SIDE}" --alt "${ALT}" \
        --leg-time "${LEG_TIME}" --settle-time "${SETTLE}" \
        2>&1 | tee "${RUNDIR}/fly_path.log"

    echo "--> flight done; stopping bag"
    sleep 3
    # ros2 bag record does NOT exit on SIGINT in this container; it needs
    # SIGTERM. Previously the recorder kept running for ~6.5 min of sim time
    # after landing, so ~88% of samples were of a stationary vehicle and the ATE
    # was computed mostly over a parked drone. Escalate and verify.
    kill -INT "${BAG_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do kill -0 "${BAG_PID}" 2>/dev/null || break; sleep 1; done
    if kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "    SIGINT ignored, sending SIGTERM"
        pkill -TERM -f "ros2 bag record" 2>/dev/null || true
        kill -TERM "${BAG_PID}" 2>/dev/null || true
    fi
    for _ in $(seq 1 30); do kill -0 "${BAG_PID}" 2>/dev/null || break; sleep 1; done
    kill -0 "${BAG_PID}" 2>/dev/null && echo "    WARNING recorder still alive"
    wait "${BAG_PID}" 2>/dev/null || true

    kill -TERM "${MON_PID}" 2>/dev/null || true
    wait "${MON_PID}" 2>/dev/null || true
    sleep 2

    # metadata.yaml is not written if the recorder is killed hard; regenerate.
    if [[ ! -f "${RUNDIR}/flight.bag/metadata.yaml" ]]; then
        echo "--> no metadata.yaml, reindexing"
        ros2 bag reindex "${RUNDIR}/flight.bag" -s mcap >/dev/null 2>&1 || true
    fi
    echo "--> measured rates for this run:"
    sed -n "/RUN SUMMARY/,/=====$/p" "${RUNDIR}/rates.log" 2>/dev/null | sed "s/^/    /"
fi

BAG="${RUNDIR}/flight.bag"
hr; echo "BAG CONTENTS"; hr
python3 /opt/scripts/bag_to_tum.py "${BAG}" --list --out /dev/null || true

hr; echo "EXTRACTING TRAJECTORIES (TUM)"; hr
python3 /opt/scripts/bag_to_tum.py "${BAG}" \
    --pose-topic "/${NAMESPACE}/robot_pose_slam" \
    --out "${RUNDIR}/est_orbslam3.tum"
EST_OK=$?

# Ground truth comes out by INDEX, not by name. ros_gz_bridge's
# gz.msgs.Pose_V -> TFMessage conversion discards the entity names, so every
# frame_id/child_frame_id in the recorded stream is empty and a name filter
# matches nothing. Gazebo orders <world>/dynamic_pose/info as
# [model, link, link, ...], so index 0 is the model's world pose.
#
# Verified rather than assumed: --variance shows index 0 sweeping the commanded
# path while indices 1..6 are constant link offsets (index 6 sits at
# (0.12, 0.03, 0.24), matching the camera extrinsics in the wrapper params).
echo "--> ground-truth index check"
python3 /opt/scripts/bag_to_tum.py "${BAG}" \
    --tf-topic /ground_truth/pose_info --variance --out /dev/null || true

python3 /opt/scripts/bag_to_tum.py "${BAG}" \
    --tf-topic /ground_truth/pose_info --tf-index "${GT_TF_INDEX:-0}" \
    --time-from-clock /clock \
    --out "${RUNDIR}/gt.tum"
GT_OK=$?

if [[ ${EST_OK} -ne 0 || ${GT_OK} -ne 0 ]]; then
    echo "record_and_eval: could not extract both trajectories; stopping before evo." >&2
    exit 4
fi

# -----------------------------------------------------------------------------
# evo
# -----------------------------------------------------------------------------
# -a = SE(3) Umeyama alignment. Required, and not a thumb on the scale: the
# ORB-SLAM3 map frame and the Gazebo world frame have different origins and
# orientations, so an unaligned ATE would just measure that offset. Scale is NOT
# estimated (-s omitted) because RGB-D SLAM is metric -- letting evo fit scale
# would hide real scale error.
run_evo() {  # run_evo <label> <est.tum>
    local label="$1" est="$2"
    [[ -s "${est}" ]] || { echo "  (no ${label} trajectory, skipping)"; return; }

    hr; echo "ATE (evo_ape) -- ${label} vs ground truth"; hr
    evo_ape tum "${RUNDIR}/gt.tum" "${est}" \
        -a --t_max_diff 0.05 ${EVO_WINDOW:-} \
        --save_results "${RUNDIR}/ape_${label}.zip" \
        2>&1 | tee "${RUNDIR}/ape_${label}.txt"

    hr; echo "RPE (evo_rpe) -- ${label}, 1 m delta"; hr
    evo_rpe tum "${RUNDIR}/gt.tum" "${est}" \
        -a --t_max_diff 0.05 --delta 1 --delta_unit m ${EVO_WINDOW:-} \
        --save_results "${RUNDIR}/rpe_${label}.zip" \
        2>&1 | tee "${RUNDIR}/rpe_${label}.txt"
}

run_evo "orbslam3" "${RUNDIR}/est_orbslam3.tum"

# -----------------------------------------------------------------------------
# COVINS optimised trajectory
# -----------------------------------------------------------------------------
# The backend writes <output_dir>/KF_<client_id>_ftum.csv in TUM format
# (sys.trajectory_format: 'TUM'), via Map::WriteKFsToFile(). Export it with
# /opt/scripts/export_covins_trajectory.sh in the backend container first; this
# picks it up from the shared /out mount.
hr; echo "COVINS OPTIMISED TRAJECTORY"; hr
COVINS_TUM="$(ls /out/covins/KF_*_ftum.csv 2>/dev/null | head -1)"
if [[ -n "${COVINS_TUM}" ]]; then
    echo "found ${COVINS_TUM}"
    # Strip any header/comment lines; evo wants bare numeric rows.
    # Sort by timestamp and drop non-monotonic duplicates. COVINS writes
    # keyframes in map order, not chronological order (observed: first row
    # t=64.7, last row t=10.4), and evo rejects unordered stamps.
    grep -E '^[-0-9]' "${COVINS_TUM}" | tr ',' ' ' \
        | sort -g -k1,1 | awk '!seen[$1]++' > "${RUNDIR}/est_covins.tum"
    echo "  $(wc -l < "${RUNDIR}/est_covins.tum") keyframe poses"
    echo "  span: $(head -1 "${RUNDIR}/est_covins.tum" | awk '{printf "%.2f", $1}')"\
         "-> $(tail -1 "${RUNDIR}/est_covins.tum" | awk '{printf "%.2f", $1}') s"
    run_evo "covins" "${RUNDIR}/est_covins.tum"
else
    echo "No COVINS trajectory found at /out/covins/KF_*_ftum.csv."
    echo "Export it from the backend container:"
    echo "  docker compose exec covins-backend /opt/scripts/export_covins_trajectory.sh"
    echo "(COVINS only writes keyframes it actually received -- check keyframe"
    echo " receipt first, and remember comm.start_sending_after_kf delays it.)"
fi

hr; echo "RESULTS IN ${RUNDIR}"; hr
ls -la "${RUNDIR}"
