#!/usr/bin/env python3
"""Minimum ground-truth clearance to a generated scene (PATCHES s64; day 5: any world in scenes.py).
clearance.py <gt.tum> [world=validation]: distance from base_link to each solid (axis-aligned boxes,
scenes.py SOLIDS). The x500's rotor tips reach ~0.33 m from base_link
horizontally, so clearance < 0.33 m (with overlapping height) means contact."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenes import solids
SOLIDS = solids(sys.argv[2] if len(sys.argv) > 2 else "validation")
g = np.loadtxt(sys.argv[1]); P = g[:, 1:4]
air = P[:, 2] > 0.3
best = []
for n, cx, cy, cz, sx, sy, sz in SOLIDS:
    lo, hi = np.array([cx-sx/2, cy-sy/2, cz-sz/2]), np.array([cx+sx/2, cy+sy/2, cz+sz/2])
    d = np.linalg.norm(np.maximum(0, np.maximum(lo - P, P - hi)), axis=1)
    k = np.argmin(np.where(air, d, np.inf)); best.append((d[k], n, P[k]))
best.sort()
for d, n, p in best[:3]: print(f"  min clearance {d:.2f} m to {n} at gt ({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")
print("CONTACT (< 0.33 m)" if best[0][0] < 0.33 else "no contact (all >= 0.33 m)")
