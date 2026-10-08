#!/usr/bin/env python3
"""turn_in_place.py <flight_dir>... (day 4 step 2a): from nees_motion.npz (per-sample orientation
NEES, GT yaw rate / speed, features), the share of NEES growth in turn-in-place segments.
Segments: turn-in-place = yaw rate > 0.2 rad/s and speed < 0.1 m/s; translate-straight = speed >= 0.1
and yaw rate < 0.05; translate-turning = speed >= 0.1 and yaw rate >= 0.05; hover = rest.
Growth: per-sample increase of log NEES (smoothed over 1 s) summed per segment class, as a share of
all positive increments; plus median NEES per class."""
import sys, os, json, numpy as np
for d in sys.argv[1:]:
    z = np.load(f'{d}/nees_motion.npz'); yr, sp, ori = z['yaw_rate'], z['speed'], z['ori']
    cls = np.where((yr > 0.2) & (sp < 0.1), 'turn_in_place', np.where(sp >= 0.1, np.where(yr < 0.05, 'straight', 'translate_turning'), 'hover'))
    k = 10; lo = np.convolve(np.log(np.maximum(ori, 1e-6)), np.ones(k) / k, 'same'); inc = np.diff(lo, prepend=lo[0])
    pos = np.maximum(inc, 0); tot = pos.sum()
    out = {'flight': os.path.basename(d.rstrip('/'))}
    for c in ('turn_in_place', 'straight', 'translate_turning', 'hover'):
        m = cls == c
        out[c] = {'time_frac': round(float(m.mean()), 3), 'growth_share': round(float(pos[m].sum() / tot), 3) if tot else None,
                  'nees_med': round(float(np.median(ori[m])), 2) if m.any() else None}
    print(json.dumps(out))
