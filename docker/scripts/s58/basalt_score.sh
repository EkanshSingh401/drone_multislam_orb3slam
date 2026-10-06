#!/usr/bin/env bash
# basalt_score.sh <run>: IMU-pose trajectory -> base_link TUM, frames check, GT-window ATE (+ OpenVINS for reference)
source /opt/ros/jazzy/setup.bash
r=$1; d=/out/basalt/$r; rd=/out/eval/$r
python3 - "$d" <<'PY'
import sys, numpy as np
from scipy.spatial.transform import Rotation as R
d=sys.argv[1]; a=np.loadtxt(f"{d}/trajectory.txt")
P=np.array([0.12-0.00552, 0.0051, 0.242-0.01174])
b=a.copy(); b[:,1:4]=a[:,1:4]-R.from_quat(a[:,4:8]).apply(P)
np.savetxt(f"{d}/est_basalt_base.tum", b, fmt="%.9f")
pairs=int(open(f"{d}/frames.txt").read().split("stereo_pairs=")[1].split()[0])
print(f"FRAMES consumed(poses)={len(a)} stereo_pairs_in_bag={pairs} {'OK' if len(a)==pairs else 'MISMATCH'}")
PY
python3 /opt/scripts/gt_window_eval.py $rd/gt.tum basalt=$d/est_basalt_base.tum ov=/out/replay/${r}_ovser_imufix_fc/est_openvins.tum 2>/dev/null
