#!/usr/bin/env bash
# Apply the s56 IMU stamp correction (-2.0 ms) to existing bags. Runs INSIDE the sim container.
# Original kept as flight_gzstamp.bag; corrected written as flight.bag; marker IMU_STAMP_FIXED.txt.
set -eo pipefail; source /opt/ros/jazzy/setup.bash
for rd in "$@"; do
  [[ -f "$rd/IMU_STAMP_FIXED.txt" ]] && { echo "$rd: already fixed"; continue; }
  [[ -e "$rd/flight_gzstamp.bag" ]] || mv "$rd/flight.bag" "$rd/flight_gzstamp.bag"
  [[ -e "$rd/flight.bag" ]] && { echo "$rd: flight.bag exists but no marker -- stopping" >&2; exit 3; }
  python3 /opt/scripts/fix_imu_timing.py "$rd/flight_gzstamp.bag" "$rd/flight.bag" --gyro-lag-ms 2.0 2>&1 | grep -v rosbag2_
  echo "IMU stamps -2.0 ms by fix_imu_timing.py (PATCHES s56) from flight_gzstamp.bag, $(date -Is)" > "$rd/IMU_STAMP_FIXED.txt"
  echo "$rd: done"
done
