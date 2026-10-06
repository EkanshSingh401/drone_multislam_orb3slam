#!/usr/bin/env bash
# diag (full window), diag on the joint-cov subset, full on the same subset (PATCHES s55)
source /opt/ros/jazzy/setup.bash
for od in "$@"; do
  r=$(basename "$od" | cut -d_ -f1)
  python3 - "$od" <<'PY'
import sys, numpy as np
od = sys.argv[1]
e = np.loadtxt(f"{od}/est_cov.txt"); f = np.loadtxt(f"{od}/est_cov_full.txt")
np.savetxt(f"{od}/est_cov_diagsub.txt", e[np.isin(np.round(e[:, 0], 5), np.round(f[:, 0], 5))], fmt="%.12g")
PY
  for est in est_cov.txt est_cov_diagsub.txt est_cov_full.txt; do
    python3 /out/diag_s54/nees_window.py "$od" "/out/eval/$r/gt.tum" --est $est | python3 -c "
import sys,json; d=json.loads(sys.stdin.read()); print('$r %-20s n %5d pos %6.2f ori %6.1f (%.2f) rp %6.1f (%.2f) ate %.4f tilt %.3f'%('$est',d['n'],d['pos']['mean'],d['ori']['mean'],d['ori']['in95'],d['rp']['mean'],d['rp']['in95'],d['rmse_pos_m'],d['rmse_tilt_deg']))"
  done
done
