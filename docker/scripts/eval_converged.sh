#!/usr/bin/env bash
# Converged-window ATE/RPE for one evaluated run (run INSIDE the sim container).
#
#   /opt/scripts/eval_converged.sh /out/eval/<run> [more rundirs...]
#
# The converged window starts when ORB-SLAM3's visual-inertial bundle adjustment
# has finished its second stage. Before that, the published pose stream is the
# PRE-refinement estimate plus the discontinuity where each correction lands
# (PATCHES.md s34), and the full-flight ATE mostly measures that transient: on a
# three-minute path B flight the full-flight figure was 9.005 m while the
# converged window over the remaining 167 s was 0.0535 m -- a factor of 168.
#
# The start time is MEASURED from the clock beacons the stereo-inertial node
# emits, via viba_window.py, not extrapolated from the real-time factor.
#
# It must be VIBA *2*, not VIBA 1: on that same run the post-VIBA-1 window still
# scored 5.851 m with a 24.6 m maximum, because VIBA 2's scale correction had yet
# to arrive. Cropping at the wrong stage is worse than not cropping.
#
# Runs with no VIBA markers at all (stereo-only, which has no IMU and therefore
# no visual-inertial BA) are reported as "converged == full flight" and skipped
# rather than failed: for those the full-flight number already IS the converged
# number.
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash >/dev/null 2>&1 || true

for RUNDIR in "$@"; do
    echo "============================================================"
    echo "CONVERGED WINDOW  ${RUNDIR}"
    echo "============================================================"
    if [[ ! -f "${RUNDIR}/est_orbslam3.tum" || ! -f "${RUNDIR}/gt.tum" ]]; then
        echo "  no trajectories in this rundir, skipping"
        continue
    fi
    LOG="${RUNDIR}/orb_slam3.log"
    if [[ ! -f "${LOG}" ]]; then
        echo "  no orb_slam3.log, cannot locate VIBA, skipping"
        continue
    fi

    # Stereo-only: no VIBA, so the full flight is already the converged window.
    if ! grep -q "VIBA" "${LOG}" 2>/dev/null; then
        echo "  no VIBA markers (stereo-only / non-inertial):"
        echo "  converged window == full flight; use the full-flight ATE as-is."
        { echo "converged_equals_full=yes"; echo "reason=no_viba_non_inertial"; } \
            > "${RUNDIR}/converged.txt"
        continue
    fi

    T_START=$(python3 /opt/scripts/viba_window.py "${LOG}" --emit-t-start || true)
    if [[ -z "${T_START}" ]]; then
        echo "  VIBA markers present but NOT placeable on the simulation timeline."
        echo "  This log has no clock beacons, so it predates the beacon build and"
        echo "  its converged window cannot be computed. Re-run, do not estimate:"
        echo "  extrapolating from the real-time factor is only as good as the RTF"
        echo "  being constant, and it is not."
        { echo "converged_equals_full=no"; echo "t_start=unavailable"; \
          echo "reason=no_clock_beacons_in_log"; } > "${RUNDIR}/converged.txt"
        continue
    fi
    echo "  converged window starts at sim t = ${T_START} s (end of VIBA 2)"

    # evo refuses to clobber an existing archive and PROMPTS, which EOFs on a
    # non-tty -- see PATCHES.md s33.
    rm -f "${RUNDIR}/ape_converged.zip" "${RUNDIR}/rpe_converged.zip"
    evo_ape tum "${RUNDIR}/gt.tum" "${RUNDIR}/est_orbslam3.tum" \
        -a --t_max_diff 0.05 --t_start "${T_START}" \
        --save_results "${RUNDIR}/ape_converged.zip" \
        2>&1 | tee "${RUNDIR}/ape_converged.txt" | grep -E "^\s+(rmse|mean|median|max|std)" || true
    evo_rpe tum "${RUNDIR}/gt.tum" "${RUNDIR}/est_orbslam3.tum" \
        -a --t_max_diff 0.05 --delta 1 --delta_unit m --t_start "${T_START}" \
        --save_results "${RUNDIR}/rpe_converged.zip" \
        2>&1 | tee "${RUNDIR}/rpe_converged.txt" | grep -E "^\s+(rmse|mean|max)" || true

    A=$(grep -E "^\s+rmse" "${RUNDIR}/ape_converged.txt" | head -1 | awk '{print $2}')
    R=$(grep -E "^\s+rmse" "${RUNDIR}/rpe_converged.txt" | head -1 | awk '{print $2}')
    {
      echo "converged_equals_full=no"
      echo "t_start=${T_START}"
      echo "ate_rmse=${A:-nan}"
      echo "rpe_rmse=${R:-nan}"
    } > "${RUNDIR}/converged.txt"
    echo "  -> converged ATE ${A:-nan} m | RPE ${R:-nan} m"
done
