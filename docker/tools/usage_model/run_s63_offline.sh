#!/usr/bin/env bash
# Re-run both harnesses on saved data after the T_metric fix (PATCHES s62).
source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; source /out/build_s60/jazzy/install/setup.bash
B=/out/build_s60/jazzy/install/ig_prediction_validation/lib/ig_prediction_validation
for spec in "230119 validation step4d_230119" "230613 validation step4_20261006-230613" "231052 validation step4_20261006-231052" "231552 forest step4_20261006-231552" "232057 forest step4_20261006-232057" "232529 forest step4_20261006-232529"; do
  set -- $spec; rd=/out/eval/20261006-$1
  read w0 w1 < <(python3 -c "
import numpy as np; g=np.loadtxt('$rd/gt.tum'); t,z=g[:,0],g[:,3]; h=z-np.median(z[:max(5,len(z)//50)])
up=np.where(h>0.5)[0]; to=t[np.argmax(h>0.05)]; td=t[up[-1]+np.argmax(h[up[-1]:]<0.05)]; print(to+3.0, td)")
  s4=/out/diag_s62/d_$1
  # (s61 recompute done: segments_s61_fixed.jsonl)
  d=/out/diag_s62/d_$1
  $B/ig_decomposition $d/jc.bag $d/imu.txt $d/ov_state_est.txt $d/openvins.log $rd/gt.tum 1.0 1.0 $w0 $w1 $2 $d/records_s63.jsonl /out/diag_s63/usage_model.txt > $d/segments_s63.jsonl
  echo "$1: s63 $(wc -l < $d/segments_s63.jsonl) segs, records $(wc -l < $d/records_s63.jsonl)"
done
