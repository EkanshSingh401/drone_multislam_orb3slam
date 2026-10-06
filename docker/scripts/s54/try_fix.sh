#!/usr/bin/env bash
# try_fix.sh <run> <tag> [fix args...]   -> fixed bag, OV replay (timeshift 0), NEES; then online-dt check
set -eo pipefail; source /opt/ros/jazzy/setup.bash
r="$1"; tag="$2"; shift 2; rd=/out/eval/$r; B=/out/diag_s56/bags/${r}_${tag}
mkdir -p /out/diag_s56/bags; [[ -e "$B" ]] || python3 /out/diag_s56/fix_imu_timing.py "$rd/flight.bag" "$B" "$@" 2>&1 | grep -v rosbag2_
for mode in fixed calib; do
  od=/out/replay/${r}_ovser_${tag}_${mode}; [[ -e "$od" ]] && continue
  if [[ $mode == calib ]]; then export OV_SET="calib_cam_timeoffset=true"; else unset OV_SET; fi
  /opt/scripts/replay_estimator.sh --bag "$B" --estimator openvins_serial --out "$od" > "$od.log" 2>&1
  python3 /opt/scripts/ov_prep.py "$od" --gt-bag "$rd/flight.bag" > /dev/null 2>&1
  python3 /opt/scripts/nees_window.py "$od" "$rd/gt.tum" --json > "${od}_nees.json"
  python3 - "$od" "$mode" <<'PY'
import json,sys,numpy as np
od,mode=sys.argv[1:]; d=json.load(open(od+'_nees.json'))
s=f"{mode:6s} pos {d['pos']['mean']:.2f} ori {d['ori']['mean']:.1f} ({d['ori']['in95']:.2f}) rp {d['rp']['mean']:.1f} ({d['rp']['in95']:.2f}) tilt {d['rmse_tilt_deg']:.3f}"
if mode=='calib':
  a=np.loadtxt(od+'/ov_state_est.txt'); s+=f"  online dt final {a[-1,17]*1e3:+.2f} ms"
print(s)
PY
done
