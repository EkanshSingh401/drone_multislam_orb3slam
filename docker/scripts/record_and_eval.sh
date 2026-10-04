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
# Estimate topic. The phase-1 RGB-D node runs inside the uav_1 namespace; the
# phase-2 stereo-inertial node runs in the root namespace, so its pose lands on
# /robot_pose_slam.
EST_TOPIC="${EST_TOPIC:-/${NAMESPACE}/robot_pose_slam}"
# Which rig's monitor topic set to sample (clock-only either way).
MON_NS="${MON_NS:-${NAMESPACE}}"
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
    for t in "${EST_TOPIC}" "/ground_truth/pose_info"; do
        if ! ros2 topic info "$t" >/dev/null 2>&1; then
            echo "record_and_eval: required topic $t does not exist." >&2
            echo "  Is the stack up? Run /opt/scripts/bringup_sim.sh first." >&2
            exit 1
        fi
    done
    if ! timeout 20 ros2 topic echo "${EST_TOPIC}" --once >/dev/null 2>&1; then
        echo "record_and_eval: WARNING no pose on ${EST_TOPIC} yet." >&2
        echo "  ORB-SLAM3 has probably not initialised. Recording anyway, but the" >&2
        echo "  estimate may be empty or start late." >&2
    fi

    echo "--> starting rosbag2"
    # RECORD_SENSORS=1 additionally captures the raw sensor streams, which is
    # what makes an offline replay possible: both estimators can then be fed
    # BYTE-IDENTICAL input instead of two separate live flights that differ in
    # scheduling. It is off by default because it is expensive -- 848x480 mono8
    # at 30 Hz on two imagers is ~24 MB per simulated second, so a 60 s flight
    # is ~1.5 GB and a 3-minute path B run is ~4.5 GB.
    #
    # zstd file compression is used for those runs. It costs CPU at record time
    # but the recorder is not the bottleneck here (the renderer is), and it
    # roughly halves the footprint on mono8.
    SENSOR_TOPICS=()
    BAG_EXTRA=()
    if [[ "${RECORD_SENSORS:-0}" == "1" ]]; then
        SENSOR_TOPICS=(
            /camera/infra1/image_rect_raw /camera/infra1/camera_info
            /camera/infra2/image_rect_raw /camera/infra2/camera_info
            /camera/imu
        )
        BAG_EXTRA=(--compression-mode file --compression-format zstd)
        echo "    RECORD_SENSORS=1: also recording stereo IR + IMU (zstd)"
    fi
    # Disk-space preflight (PATCHES.md s45). A full disk does not stop rosbag2
    # with an error: it leaves a truncated .mcap.zstd ("file too small") that is
    # only discovered at evaluation time, after the flight is spent. Five path A
    # sensor bags were lost exactly that way on a 50 GB root partition. Refuse
    # to fly unless the output drive has 3x the expected bag size free: 1x for
    # the uncompressed mcap, 1x for the zstd copy written at close, 1x margin.
    #
    # Expected size = data rate x simulated recording length. The rate is
    # computed from the rig: 2 imagers x 848x480 mono8 x 30 Hz = 24.4 MB per
    # simulated second; without sensors the bag is pose/tf/clock only, budgeted
    # at 0.5 MB/s. The length is a deliberately generous per-path budget
    # (path A measures 65-72 s, path B ~3 min); EXPECTED_SIM_S overrides it.
    if [[ "${RECORD_SENSORS:-0}" == "1" ]]; then BAG_RATE_B=$((2 * 848 * 480 * 30)); else BAG_RATE_B=500000; fi
    case "${PATH_VERSION:-A}" in B|b) DEF_SIM_S=300 ;; *) DEF_SIM_S=120 ;; esac
    EXPECTED_SIM_S="${EXPECTED_SIM_S:-${DEF_SIM_S}}"
    NEED_B=$(( 3 * BAG_RATE_B * EXPECTED_SIM_S ))
    FREE_B=$(( $(df --output=avail -B1 "${RUNDIR}" | tail -1) ))
    awk -v n="${NEED_B}" -v f="${FREE_B}" -v m="$(df --output=target "${RUNDIR}" | tail -1)" \
        'BEGIN{printf "    disk preflight: need %.1f GB free (3 x %.1f GB expected), have %.1f GB on %s\n", n/1e9, n/3e9, f/1e9, m}'
    if (( FREE_B < NEED_B )); then
        echo "record_and_eval: REFUSING TO FLY -- not enough free disk for this bag." >&2
        echo "  Free space on the drive behind ${OUTROOT}, or set EXPECTED_SIM_S if" >&2
        echo "  this flight is genuinely shorter than the ${EXPECTED_SIM_S} s budget." >&2
        echo "DISK_PREFLIGHT_FAILED need=${NEED_B} free=${FREE_B}" > "${RUNDIR}/PREFLIGHT_FAILED.txt"
        exit 5
    fi

    ros2 bag record -s mcap -o "${RUNDIR}/flight.bag" \
        "${BAG_EXTRA[@]}" \
        "${EST_TOPIC}" \
        "/ground_truth/pose_info" \
        "${SENSOR_TOPICS[@]}" \
        /clock /tf /tf_static \
        > "${RUNDIR}/rosbag.log" 2>&1 &
    BAG_PID=$!
    sleep 5

    # Measure RTF and stream rates for THIS run. The gz stats topic's
    # real_time_factor is instantaneous and bimodal (0.032 vs 0.538 on identical
    # setups), so the only trustworthy figure is sim-time/wall-time over the
    # window, which rate_monitor computes from /clock.
    python3 -u /opt/scripts/rate_monitor.py --namespace "${MON_NS}" \
        --duration "${MONITOR_DURATION:-900}" --window 15 \
        --out "${RUNDIR}/rates.csv" > "${RUNDIR}/rates.log" 2>&1 &
    MON_PID=$!

    echo "--> hovering ${SETTLE}s to let ORB-SLAM3 initialise, then flying"
    python3 /opt/scripts/fly_path.py \
        --side "${SIDE}" --alt "${ALT}" \
        --leg-time "${LEG_TIME}" --settle-time "${SETTLE}" \
        --path-version "${PATH_VERSION:-A}" \
        2>&1 | tee "${RUNDIR}/fly_path.log"
    echo "${PATH_VERSION:-A}" > "${RUNDIR}/path_version.txt"

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
    # SIGKILL is the last resort and it must exist. Without it this escalation
    # ended at a warning and then blocked on `wait` for a process that was never
    # going to exit -- `wait` on a live child has no timeout. That wedged run 5
    # of the first stereo-inertial experiment for 36 minutes: the recorder kept
    # writing (the flight ended 18:50, the mcap was still growing at 19:26) and
    # the run's trajectory ended up ~10x longer than the flight, almost all of
    # it a parked vehicle, which makes its ATE meaningless. rosbag2 IGNORES
    # SIGINT here (/proc/<pid>/SigIgn has bit 2 set) and CATCHES SIGTERM
    # (SigCgt bit 15) -- so the handler exists and simply did not stop the
    # recording. SIGKILL costs only metadata.yaml, which the reindex below
    # regenerates.
    if kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "    SIGTERM ignored too, sending SIGKILL"
        pkill -KILL -f "ros2 bag record" 2>/dev/null || true
        kill -KILL "${BAG_PID}" 2>/dev/null || true
        for _ in $(seq 1 15); do kill -0 "${BAG_PID}" 2>/dev/null || break; sleep 1; done
    fi
    if kill -0 "${BAG_PID}" 2>/dev/null; then
        # Never `wait` on a process that survived SIGKILL: that blocks forever.
        echo "    ERROR recorder survived SIGKILL; NOT waiting on it." >&2
        echo "    This run's bag covers more than the flight -- treat its ATE" >&2
        echo "    as invalid rather than comparable." >&2
        touch "${RUNDIR}/INVALID_RECORDER_OVERRUN"
    else
        wait "${BAG_PID}" 2>/dev/null || true
    fi

    # SIGINT, not SIGKILL: the monitor traps it and prints the RUN SUMMARY that
    # carries this run's real-time factor. Killing it hard loses that.
    kill -INT "${MON_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do kill -0 "${MON_PID}" 2>/dev/null || break; sleep 1; done
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
    --pose-topic "${EST_TOPIC}" \
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
# evo's --save_results refuses to clobber an existing archive: it PROMPTS, and
# on a non-tty the prompt hits EOF and evo exits non-zero with a traceback --
# AFTER it has already printed the statistics. run_experiment.sh calls this
# script twice per run (the flight, then --eval-only), so the second pass always
# tripped it. The numbers survived (they are parsed from the .txt), but the
# non-zero exit was reported as a failure. Clear the archive first.
rm_stale() { rm -f "$@"; }

run_evo() {  # run_evo <label> <est.tum>
    local label="$1" est="$2"
    [[ -s "${est}" ]] || { echo "  (no ${label} trajectory, skipping)"; return; }
    rm_stale "${RUNDIR}/ape_${label}.zip" "${RUNDIR}/rpe_${label}.zip"

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
# Stereo-inertial only: evaluate again from IMU initialisation onwards.
# -----------------------------------------------------------------------------
# ORB-SLAM3 needs motion before it can initialise the IMU (scale, gravity
# direction, biases). Until then the estimate is effectively visual-only and
# drags the ATE down, so the post-init window is the figure that characterises
# stereo-inertial performance. Both are reported; neither alone is honest.
SLAM_LOG="${LOGDIR:-/out/logs}/orb_slam3.log"
if [[ -f "${SLAM_LOG}" ]]; then
    IMU_INIT_T=$(sed 's/\x1b\[[0-9;]*m//g' "${SLAM_LOG}" \
        | grep -oE 'IMU INITIALISED at frame t=[0-9.]+' | head -1 \
        | grep -oE '[0-9.]+$' || true)
    FIRST_T=$(sed 's/\x1b\[[0-9;]*m//g' "${SLAM_LOG}" \
        | grep -oE 'first stereo pair at t=[0-9.]+' | head -1 \
        | grep -oE '[0-9.]+$' || true)
    # Also record where the PUBLISHED estimate starts. On this rig it turns out
    # to begin several seconds AFTER IMU initialisation, because LocalMapping
    # keeps resetting the active map ("Not enough motion for initializing")
    # while the airframe is still parked, and the wrapper only publishes once
    # GetTrackingState()==2. The --t_start window then excludes nothing and the
    # full-flight and post-init metrics come out bit-identical. That is correct
    # but looks like a broken filter, so make it visible instead of puzzling.
    EST_FIRST_T=$(head -1 "${RUNDIR}/est_orbslam3.tum" 2>/dev/null | cut -d' ' -f1 || true)
    {
      echo "imu_init_stamp=${IMU_INIT_T:-none}"
      echo "first_frame_stamp=${FIRST_T:-none}"
      echo "est_first_stamp=${EST_FIRST_T:-none}"
      if [[ -n "${IMU_INIT_T}" && -n "${FIRST_T}" ]]; then
          echo "imu_init_delay_s=$(python3 -c "print(f'{${IMU_INIT_T}-${FIRST_T}:.3f}')")"
      fi
      if [[ -n "${IMU_INIT_T}" && -n "${EST_FIRST_T}" ]]; then
          python3 -c "
imu, est = ${IMU_INIT_T}, ${EST_FIRST_T}
print(f'est_starts_after_imu_init_s={est-imu:.3f}')
print('postinit_window_equals_full_flight=' + ('yes' if est >= imu else 'no'))"
      fi
    } > "${RUNDIR}/imu_init.txt"
    hr; echo "IMU INITIALISATION"; hr
    cat "${RUNDIR}/imu_init.txt" | sed 's/^/  /'
    if [[ -n "${IMU_INIT_T}" ]]; then
        rm_stale "${RUNDIR}/ape_orbslam3_postinit.zip" \
                 "${RUNDIR}/rpe_orbslam3_postinit.zip"
        hr; echo "ATE (evo_ape) -- orbslam3 AFTER IMU init (t >= ${IMU_INIT_T})"; hr
        evo_ape tum "${RUNDIR}/gt.tum" "${RUNDIR}/est_orbslam3.tum" \
            -a --t_max_diff 0.05 --t_start "${IMU_INIT_T}" \
            --save_results "${RUNDIR}/ape_orbslam3_postinit.zip" \
            2>&1 | tee "${RUNDIR}/ape_orbslam3_postinit.txt"
        hr; echo "RPE (evo_rpe) -- orbslam3 AFTER IMU init"; hr
        evo_rpe tum "${RUNDIR}/gt.tum" "${RUNDIR}/est_orbslam3.tum" \
            -a --t_max_diff 0.05 --delta 1 --delta_unit m --t_start "${IMU_INIT_T}" \
            --save_results "${RUNDIR}/rpe_orbslam3_postinit.zip" \
            2>&1 | tee "${RUNDIR}/rpe_orbslam3_postinit.txt"
    else
        echo "  (no IMU initialisation line in the SLAM log -- either this was an"
        echo "   RGB-D run, or stereo-inertial never initialised its IMU)"
    fi
fi

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
