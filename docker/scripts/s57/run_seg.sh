#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
for rd in "$@"; do r=$(basename $rd); od=/out/replay/${r}_ovser_imufix_fc
  python3 /out/diag_s54/nees_window.py $od $rd/gt.tum --est est_cov.txt --dump $od/nees_dump_diag.txt > /dev/null
  python3 /out/diag_s54/nees_window.py $od $rd/gt.tum --est est_cov_full.txt --dump $od/nees_dump_full.txt > /dev/null
  python3 /out/diag_s57/segment_corr.py $rd $od 2>&1 | grep -v rosbag2_
done
