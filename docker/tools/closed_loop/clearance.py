#!/usr/bin/env python3
"""Minimum ground-truth clearance to the validation scene (PATCHES s64).
clearance.py <gt.tum> [airborne_only]: distance from base_link to each solid (axis-aligned boxes,
from gen_validation_world.py SOLIDS). The x500's rotor tips reach ~0.33 m from base_link
horizontally, so clearance < 0.33 m (with overlapping height) means contact."""
import sys, numpy as np
WALL_H, WALL_T = 4.0, 0.2
X_MIN, X_MAX, Y_MIN, Y_MAX = -4.0, 5.5, -5.5, 5.5
SOLIDS = [("wall_w", X_MIN, 0.0, WALL_H/2, WALL_T, Y_MAX-Y_MIN, WALL_H), ("wall_e", X_MAX, 0.0, WALL_H/2, WALL_T, Y_MAX-Y_MIN, WALL_H),
  ("wall_s", (X_MIN+X_MAX)/2, Y_MIN, WALL_H/2, X_MAX-X_MIN, WALL_T, WALL_H), ("wall_n", (X_MIN+X_MAX)/2, Y_MAX, WALL_H/2, X_MAX-X_MIN, WALL_T, WALL_H),
  ("box_ne", 4.6, 4.2, 0.9, 1.4, 2.0, 1.8), ("box_e", 4.8, 0.5, 1.25, 1.0, 1.2, 2.5), ("box_se", 4.5, -4.4, 0.6, 1.6, 1.6, 1.2),
  ("box_nw", -3.2, 4.4, 1.1, 1.2, 1.6, 2.2), ("box_w", -3.4, -0.8, 0.75, 0.8, 1.6, 1.5), ("box_sw", -3.0, -4.6, 1.4, 1.6, 1.2, 2.8),
  ("box_n", 1.0, 4.8, 0.8, 1.8, 1.0, 1.6), ("box_s", 0.5, -4.8, 1.0, 1.4, 1.0, 2.0)]
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
