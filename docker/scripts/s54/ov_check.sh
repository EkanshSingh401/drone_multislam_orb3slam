#!/usr/bin/env bash
# ov_check.sh <rundir> <tag> : OV serial (timeshift 0) + NEES diag; and online time-offset run
set -eo pipefail; source /opt/ros/jazzy/setup.bash
rd="$1"; tag="$2"; r=$(basename "$rd")
for mode in fixed calib; do
  od=/out/replay/${r}_ovser_${tag}_${mode}; [[ -e "$od" ]] && continue
  if [[ $mode == calib ]]; then export OV_SET="calib_cam_timeoffset=true"; else unset OV_SET; fi
  /opt/scripts/replay_estimator.sh --bag "$rd/flight.bag" --estimator openvins_serial --out "$od" > "$od.log" 2>&1
  python3 /opt/scripts/ov_prep.py "$od" --gt-bag "$rd/flight.bag" > /dev/null 2>&1
  python3 /opt/scripts/nees_window.py "$od" "$rd/gt.tum" --json > "${od}_nees.json"
  python3 - "$od" "$mode" "$r" <<'PY'
import json,sys,numpy as np
od,mode,r=sys.argv[1:]; d=json.load(open(od+'_nees.json'))
s=f"{r} {mode:6s} pos {d['pos']['mean']:.2f} ori {d['ori']['mean']:.1f} ({d['ori']['in95']:.2f}) rp {d['rp']['mean']:.1f} ({d['rp']['in95']:.2f}) tilt {d['rmse_tilt_deg']:.3f}"
if mode=='calib':
  a=np.loadtxt(od+'/ov_state_est.txt'); s+=f"  online dt final {a[-1,17]*1e3:+.2f} ms"
print(s)
PY
done
