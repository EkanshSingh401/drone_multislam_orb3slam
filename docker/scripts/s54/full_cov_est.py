#!/usr/bin/env python3
"""Build <replay>/est_cov_full.txt: est_cov.txt rows whose stamps appear in the
joint-covariance dump, with the FULL 3x3 orientation and position blocks
(PATCHES s55). Also reports how well the dump's diagonal matches ov_state_std.

    full_cov_est.py <replay_dir> <jc_imu.txt>
"""
import sys
import numpy as np
od, jcf = sys.argv[1], sys.argv[2]
est = np.loadtxt(f"{od}/est_cov.txt"); jc = np.loadtxt(jcf)
key = lambda t: np.round(t, 5)
idx = {k: i for i, k in enumerate(key(jc[:, 0]))}
keep = [i for i, t in enumerate(key(est[:, 0])) if t in idx]
iu = np.triu_indices(3)
rows, rel = [], []
for i in keep:
    C = jc[idx[key(est[i, 0])], 1:].reshape(6, 6)
    Pr, Pp = C[:3, :3], C[3:, 3:]
    diag_std = np.r_[np.diag(Pr), np.diag(Pp)]
    diag_file = est[i, [8, 11, 13, 14, 17, 19]]
    rel.append(np.max(np.abs(diag_std - diag_file) / diag_file))
    rows.append(np.r_[est[i, :8], Pr[iu], Pp[iu]])
np.savetxt(f"{od}/est_cov_full.txt", np.array(rows), fmt="%.12g")
rel = np.array(rel)
print(f"full_cov_est: {len(keep)}/{len(est)} states matched; diag vs std-file max rel diff "
      f"median {np.median(rel):.2e} max {rel.max():.2e}")
