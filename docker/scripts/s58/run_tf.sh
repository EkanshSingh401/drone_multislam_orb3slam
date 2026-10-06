#!/usr/bin/env bash
# run_tf.sh <freq> <rundir...> : OpenVINS serial with track_frequency=<freq> (diagnostic, PATCHES s58)
source /opt/ros/jazzy/setup.bash
f=$1; shift
for rd in "$@"; do r=$(basename $rd); od=/out/replay/${r}_ovser_imufix_tf$f
  if [[ ! -e $od ]]; then
    OV_SET="track_frequency=$f" /opt/scripts/replay_estimator.sh --bag $rd/flight.bag --estimator openvins_serial --out $od > $od.log 2>&1
    python3 /opt/scripts/ov_prep.py $od --gt-bag $rd/flight.bag > /dev/null 2>&1
  fi
  python3 /out/diag_s54/nees_window.py $od $rd/gt.tum --est est_cov.txt --dump $od/nees_dump_diag.txt > /dev/null
  frames=$(grep -c 'VioManager.cpp:286 \[TRACK\]' $od/openvins.log)
  python3 /opt/scripts/gt_window_eval.py $rd/gt.tum ov=$od/est_openvins.tum 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(json.dumps({'run':'$r','tf':$f,'tracked_frames':$frames,'ate':d['ov'].get('ate'),'sim3':d['ov'].get('sim3_scale')}))"
done
