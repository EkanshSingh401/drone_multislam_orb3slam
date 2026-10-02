#!/usr/bin/env bash
# Export COVINS's optimised keyframe trajectory (run INSIDE the covins-backend container).
#
# Calls the backend's /covins_gba service, whose callback runs a global bundle
# adjustment and then Map::WriteKFsToFile(). With
# sys.trajectory_format: 'TUM' that writes
#   <output_dir>/KF_<client_id>_ftum.csv     ->  "timestamp tx ty tz qx qy qz qw"
# which evo reads directly. Results are copied to /out/covins/ for the sim
# container to evaluate.
#
#   /opt/scripts/export_covins_trajectory.sh            # visual GBA + outlier rejection
#   /opt/scripts/export_covins_trajectory.sh --action 4 # visual GBA, no rejection
#   /opt/scripts/export_covins_trajectory.sh --action 100 --map-id 0   # PGO instead
set -euo pipefail

CATKIN_WS="${CATKIN_WS:-/root/covins_ws}"
OUTDIR="${CATKIN_WS}/src/covins/covins_backend/output"
DEST=/out/covins

# action 5 = VISUAL global BA with outlier rejection.
#
# Deliberately NOT 0/1: those are the visual-INERTIAL variants, and this setup
# feeds COVINS RGB-D keyframes with no IMU, so an inertial GBA would be
# optimising constraints that do not exist.
ACTION=5
MAP_ID=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --action) ACTION="$2"; shift 2 ;;
        --map-id) MAP_ID="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

source /opt/ros/melodic/setup.bash
source "${CATKIN_WS}/devel/setup.bash"

if ! rosservice list 2>/dev/null | grep -q '/covins_gba'; then
    echo "export: /covins_gba is not advertised." >&2
    echo "  The backend node must be running: /opt/scripts/run_backend.sh" >&2
    exit 1
fi

mkdir -p "${DEST}"
echo "export: calling /covins_gba (map_id=${MAP_ID}, action=${ACTION})"
echo "export: this blocks while the global bundle adjustment runs"
rosservice call /covins_gba "{map_id: ${MAP_ID}, action: ${ACTION}}" || {
    echo "export: service call failed. If the backend has received no keyframes," >&2
    echo "  there is no map to optimise -- check keyframe receipt first." >&2
    exit 1
}

echo "export: backend output dir contents:"
ls -la "${OUTDIR}" || true

shopt -s nullglob
found=0
for f in "${OUTDIR}"/KF_*_ftum.csv "${OUTDIR}"/KF_*_feuroc.csv; do
    cp -v "$f" "${DEST}/"
    found=1
done

if [[ ${found} -eq 0 ]]; then
    echo "export: no KF_*_f{tum,euroc}.csv produced." >&2
    echo "  Most likely the backend holds no keyframes for this map id." >&2
    echo "  Remember comm.start_sending_after_kf withholds the agent's first" >&2
    echo "  keyframes, and comm.kf_buffer_withold withholds the newest ones." >&2
    exit 3
fi

echo "export: copied to ${DEST}:"
ls -la "${DEST}"
for f in "${DEST}"/KF_*_ftum.csv; do
    echo "  $(basename "$f"): $(grep -cE '^[-0-9]' "$f") keyframe poses"
done
echo
echo "Now evaluate from the sim container:"
echo "  docker compose exec sim /opt/scripts/record_and_eval.sh --eval-only /out/eval/<run>"
