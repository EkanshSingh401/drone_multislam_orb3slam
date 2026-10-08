#!/usr/bin/env python3
"""scenes.py (day 5): ground-truth geometry of the generated closed-loop scenes, one source of truth.

Used by docker/sim/tools/gen_office_world.py (writes the office world from OFFICE) and by the GT
tools (gt_coverage.py, clearance, watchdog box). Gazebo ENU, metres, drone spawned at the origin.
SOLIDS: (name, centre x, y, z, size x, y, z) axis-aligned boxes; the floor is z = 0.
ENVELOPE: (x0, x1, y0, y1, z1) the inside of the outer walls, below the wall tops.
REACHABLE_M3: envelope volume minus the solids' volume inside it = the volume a perfect mapper could
know (air inside the building). Coverage fractions are known GT-pose volume clipped to the envelope /
REACHABLE_M3 (defined a priori, day 5 step 3a)."""

# ---------------- validation (gen_validation_world.py, PATCHES s49) ----------------
_VH, _VT = 4.0, 0.2
_VX0, _VX1, _VY0, _VY1 = -4.0, 5.5, -5.5, 5.5
VALIDATION = [
    ("wall_w", _VX0, 0.0, _VH / 2, _VT, _VY1 - _VY0, _VH), ("wall_e", _VX1, 0.0, _VH / 2, _VT, _VY1 - _VY0, _VH),
    ("wall_s", (_VX0 + _VX1) / 2, _VY0, _VH / 2, _VX1 - _VX0, _VT, _VH), ("wall_n", (_VX0 + _VX1) / 2, _VY1, _VH / 2, _VX1 - _VX0, _VT, _VH),
    ("box_ne", 4.6, 4.2, 0.9, 1.4, 2.0, 1.8), ("box_e", 4.8, 0.5, 1.25, 1.0, 1.2, 2.5), ("box_se", 4.5, -4.4, 0.6, 1.6, 1.6, 1.2),
    ("box_nw", -3.2, 4.4, 1.1, 1.2, 1.6, 2.2), ("box_w", -3.4, -0.8, 0.75, 0.8, 1.6, 1.5), ("box_sw", -3.0, -4.6, 1.4, 1.6, 1.2, 2.8),
    ("box_n", 1.0, 4.8, 0.8, 1.8, 1.0, 1.6), ("box_s", 0.5, -4.8, 1.0, 1.4, 1.0, 2.0)]
VALIDATION_ENVELOPE = (_VX0, _VX1, _VY0, _VY1, _VH)

# ---------------- office (day 5 step 3b) ----------------
# 30 x 20 m single-storey office/warehouse, 4 m walls, no ceiling (like validation). A 3.5 m corridor
# along x (y in [-1.75, 1.75]) with the spawn point in it; three rooms north and three south
# (10 x 8.25 m), each opening onto the corridor through a 3 m doorway and onto its neighbours through
# 3 m doorways in the partitions (loops). Openings are 3 m because the planner's speed-scaled path
# margin is r_path + 1 s * v = 1.3 m (day 2 step 5) and the executor holds at 0.45 m.
H, T = 4.0, 0.2
X0, X1, Y0, Y1 = -4.0, 26.0, -10.0, 10.0
YC = 1.75                  # corridor half-width
DOOR = 3.0

def _wx(name, y, xa, xb):   # wall along x at y from xa to xb
    return (name, (xa + xb) / 2, y, H / 2, xb - xa, T, H)

def _wy(name, x, ya, yb):   # wall along y at x from ya to yb
    return (name, x, (ya + yb) / 2, H / 2, T, yb - ya, H)

def _office():
    s = []
    # outer walls, split into ~10 m segments (one texture each: no stretched texels)
    for i, (a, b) in enumerate([(X0, 6.0), (6.0, 16.0), (16.0, X1)]):
        s += [_wx(f"outer_s{i}", Y0, a, b), _wx(f"outer_n{i}", Y1, a, b)]
    for i, (a, b) in enumerate([(Y0, -YC), (-YC, YC), (YC, Y1)]):
        s += [_wy(f"outer_w{i}", X0, a, b), _wy(f"outer_e{i}", X1, a, b)]
    # corridor walls with doorways centred at x = 1, 11, 21
    segs = [(X0, 1 - DOOR / 2), (1 + DOOR / 2, 11 - DOOR / 2), (11 + DOOR / 2, 21 - DOOR / 2), (21 + DOOR / 2, X1)]
    for i, (a, b) in enumerate(segs):
        s += [_wx(f"corr_n{i}", YC, a, b), _wx(f"corr_s{i}", -YC, a, b)]
    # partitions at x = 6 and 16 with doorways centred at y = +-6
    for j, x in enumerate((6.0, 16.0)):
        s += [_wy(f"part{j}_n0", x, YC, 6 - DOOR / 2), _wy(f"part{j}_n1", x, 6 + DOOR / 2, Y1),
              _wy(f"part{j}_s0", x, Y0, -6 - DOOR / 2), _wy(f"part{j}_s1", x, -6 + DOOR / 2, -YC)]
    # furniture against the walls, clear of the doorways
    s += [("nw_desk", -2.5, 8.8, 0.4, 2.0, 1.2, 0.8), ("nw_shelf", 3.5, 9.4, 1.0, 3.0, 0.8, 2.0), ("nw_cab", -3.3, 4.0, 0.6, 1.0, 1.6, 1.2),
          ("nc_shelf", 13.5, 9.3, 1.25, 3.0, 1.0, 2.5), ("nc_desk", 8.0, 9.3, 0.4, 2.0, 1.0, 0.8), ("nc_rack", 11.0, 5.0, 1.0, 3.0, 0.8, 2.0),
          ("ne_shelf", 25.3, 6.0, 1.0, 1.0, 3.0, 2.0), ("ne_cab", 20.0, 9.3, 0.6, 3.0, 1.0, 1.2),
          ("sw_cab", -2.5, -8.8, 0.6, 2.0, 1.2, 1.2), ("sw_shelf", 3.5, -9.3, 1.4, 3.0, 1.0, 2.8),
          ("sc_island", 11.0, -6.0, 0.4, 2.4, 1.2, 0.8), ("sc_shelf", 14.8, -9.3, 1.0, 2.0, 1.0, 2.0),
          ("se_shelf", 25.3, -6.0, 1.25, 1.0, 3.0, 2.5), ("se_desk", 19.0, -9.2, 0.8, 2.4, 1.2, 1.6),
          ("corr_planter", 25.3, 0.0, 0.6, 0.8, 0.8, 1.2)]
    return s

OFFICE = _office()
OFFICE_ENVELOPE = (X0, X1, Y0, Y1, H)

SCENES = {"validation": (VALIDATION, VALIDATION_ENVELOPE), "office": (OFFICE, OFFICE_ENVELOPE)}

def _inside_volume(solids, env):
    x0, x1, y0, y1, z1 = env
    v = 0.0
    for _, cx, cy, cz, sx, sy, sz in solids:
        dx = max(0.0, min(cx + sx / 2, x1) - max(cx - sx / 2, x0))
        dy = max(0.0, min(cy + sy / 2, y1) - max(cy - sy / 2, y0))
        dz = max(0.0, min(cz + sz / 2, z1) - max(cz - sz / 2, 0.0))
        v += dx * dy * dz
    return v      # boxes in this list do not overlap except at wall corners (ignored, < 0.1 m^3)

def reachable_m3(world):
    solids, env = SCENES[world]
    x0, x1, y0, y1, z1 = env
    return (x1 - x0) * (y1 - y0) * z1 - _inside_volume(solids, env)

def solids(world):
    return SCENES[world][0]

def envelope(world):
    return SCENES[world][1]

if __name__ == "__main__":
    for w in SCENES:
        print(w, len(solids(w)), "solids, envelope", envelope(w), "reachable %.0f m^3" % reachable_m3(w))
