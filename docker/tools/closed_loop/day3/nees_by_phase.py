#!/usr/bin/env python3
"""nees_by_phase.py <flight_dir>... (day 3 step 4): orientation NEES (nees_motion.npz) split by
executor phase (explore / home / land), plus seconds per phase and seconds stationary in it.
Executor log 't' and the NEES stamps are both sim time."""
import sys, os, re, json, math, numpy as np
for d in sys.argv[1:]:
    rec = []
    for l in open(f'{d}/executor.log'):
        l = re.sub(r'\x1b\[[0-9;]*m', '', l).strip()
        if l.startswith('{'):
            try: j = json.loads(l); rec.append((j['t'], j['phase']))
            except Exception: pass
    z = np.load(f'{d}/nees_motion.npz'); t = z['t']
    ts = np.array([r[0] for r in rec]); phs = [r[1] for r in rec]
    idx = np.clip(np.searchsorted(ts, t, side='right') - 1, 0, len(ts) - 1)
    ph = np.array([phs[i] for i in idx])
    still = (z['yaw_rate'] < 0.05) & (z['speed'] < 0.1)
    out = {'flight': os.path.basename(d.rstrip('/'))}
    for p in ('explore', 'home', 'land'):
        m = ph == p
        if m.sum() >= 10:
            out[p] = {'n': int(m.sum()), 'frac_still': round(float(still[m].mean()), 2), 'nees_med': round(float(np.median(z['ori'][m])), 2),
                      'nees_mean': round(float(z['ori'][m].mean()), 2), 'yaw_rate': round(float(z['yaw_rate'][m].mean()), 3),
                      'speed': round(float(z['speed'][m].mean()), 3)}
    print(json.dumps(out))
