#!/usr/bin/env python3
"""nis_analyze.py <openvins.log> [t_from t_to] (day 7 step 1): ground-truth-free consistency of OpenVINS.

Reads the scratch build's '[NIS] t type featid dof chi2 [r_u/sqrt(S_uu) r_v/sqrt(S_vv)]' lines
(ov_nis.patch, OV_NIS_LOG). type M = MSCKF feature (null-space projected), S = SLAM feature update.
  NIS / dof   chi2 / dof per feature update; 1.0 if the filter's innovation covariance is right
              (> 1: overconfident or measurement noise too small; < 1: underconfident). Median and mean,
              and the fraction above the chi2 95% quantile (5% if consistent; computed before gating).
  whiteness   SLAM features only: for each feature, consecutive updates (sorted by time) give pairs of
              normalized residuals (r_t, r_t+1); Pearson correlation pooled over features, per image axis.
              0 if the innovations are white; > 0 = errors repeat from one update to the next.
Prints JSON."""
import json, sys
from collections import defaultdict
import numpy as np
from scipy import stats
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else -1e18; t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e18
M, S = [], defaultdict(list)
for l in open(sys.argv[1], errors='ignore'):
    if not l.startswith('[NIS]'): continue
    p = l.split()
    t = float(p[1])
    if not (t0 <= t <= t1): continue
    dof, chi2 = int(p[4]), float(p[5])
    if p[2] == 'M': M.append((dof, chi2))
    else: S[int(p[3])].append((t, dof, chi2, float(p[6]), float(p[7])))
def nis(rows):
    if not rows: return None
    d = np.array([r[0] for r in rows]); c = np.array([r[1] for r in rows])
    q = stats.chi2.ppf(0.95, d)
    return {'n': len(rows), 'nis_per_dof_median': round(float(np.median(c / d)), 3), 'nis_per_dof_mean': round(float(np.mean(c / d)), 3),
            'frac_above_chi2_95': round(float(np.mean(c > q)), 3), 'dof_median': float(np.median(d))}
out = {'log': sys.argv[1], 'msckf': nis(M), 'slam': nis([(r[1], r[2]) for v in S.values() for r in v])}
pu, pv, gaps = [], [], []
for v in S.values():
    v.sort()
    for a, b in zip(v[:-1], v[1:]):
        gaps.append(b[0] - a[0]); pu.append((a[3], b[3])); pv.append((a[4], b[4]))
if pu:
    pu, pv = np.array(pu), np.array(pv)
    out['slam_whiteness'] = {'pairs': len(pu), 'features': len(S), 'median_gap_s': round(float(np.median(gaps)), 4),
                             'lag1_corr_u': round(float(np.corrcoef(pu[:, 0], pu[:, 1])[0, 1]), 3),
                             'lag1_corr_v': round(float(np.corrcoef(pv[:, 0], pv[:, 1])[0, 1]), 3),
                             'mean_norm_resid_u': round(float(pu[:, 0].mean()), 3), 'mean_norm_resid_v': round(float(pv[:, 0].mean()), 3),
                             'track_len_median': float(np.median([len(v) for v in S.values()]))}
print(json.dumps(out))
