#!/usr/bin/env python3
"""Generate the phase-3 VALIDATION scene: same rig, nearby textured structure.

    python3 docker/sim/tools/gen_validation_world.py           # write files
    python3 docker/sim/tools/gen_validation_world.py --check   # verify they match

Writes (all generated -- do not edit by hand):
    docker/sim/worlds/validation.sdf
    docker/sim/models/validation_scene/model.config
    docker/sim/models/validation_scene/model.sdf
    docker/sim/models/validation_scene/materials/textures/*.png

Why (PATCHES.md s49): in the forest world most features are 7-18 m away, and
OpenVINS rejects 99% of its triangulation attempts on the condition-number
check. This scene puts textured surfaces 2-6 m from the flight path so the
estimators can be validated where parallax is not the limiting factor. The
forest bags remain the degraded-scene condition.

Everything except the scenery is copied from PX4's forest.sdf: physics step,
update rate, gravity (-9.81, already aligned with ORB-SLAM3), magnetic field,
lighting, and spherical coordinates. Static models do not appear in
/world/<w>/dynamic_pose/info, so ground-truth index 0 is still the drone.

Layout, in Gazebo ENU (x = east, y = north), drone spawned at the origin:
    path A covers x in [0, 3],    y in [0, 3]     (3 m square)
    path B covers x in [-1.5, 1.5], y in [-3, 3]  (figure-eight lobes)
The interior is left EMPTY because path B's lobes cross it. Walls stand at
x = -4 and 5.5 and y = -5.5 and 5.5, 4 m high, so the walls the camera faces
are 2.5-6 m away. Textured boxes against the walls add nearer structure and
depth variation.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import os
import sys

import numpy as np

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    sys.exit("gen_validation_world: needs Pillow (python3-pil)")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
OUT_WORLD = os.path.join(ROOT, "docker/sim/worlds/validation.sdf")
OUT_MODEL = os.path.join(ROOT, "docker/sim/models/validation_scene")
WORLD_NAME = "validation"
TEX_PX = 1024

WALL_H = 4.0
WALL_T = 0.2
X_MIN, X_MAX, Y_MIN, Y_MAX = -4.0, 5.5, -5.5, 5.5

# (name, centre x, y, z, size x, y, z) -- walls, then boxes against the walls.
SOLIDS = [
    ("wall_w", X_MIN, 0.0, WALL_H / 2, WALL_T, Y_MAX - Y_MIN, WALL_H),
    ("wall_e", X_MAX, 0.0, WALL_H / 2, WALL_T, Y_MAX - Y_MIN, WALL_H),
    ("wall_s", (X_MIN + X_MAX) / 2, Y_MIN, WALL_H / 2, X_MAX - X_MIN, WALL_T, WALL_H),
    ("wall_n", (X_MIN + X_MAX) / 2, Y_MAX, WALL_H / 2, X_MAX - X_MIN, WALL_T, WALL_H),
    ("box_ne", 4.6, 4.2, 0.9, 1.4, 2.0, 1.8),
    ("box_e",  4.8, 0.5, 1.25, 1.0, 1.2, 2.5),
    ("box_se", 4.5, -4.4, 0.6, 1.6, 1.6, 1.2),
    ("box_nw", -3.2, 4.4, 1.1, 1.2, 1.6, 2.2),
    ("box_w",  -3.4, -0.8, 0.75, 0.8, 1.6, 1.5),
    ("box_sw", -3.0, -4.6, 1.4, 1.6, 1.2, 2.8),
    ("box_n",  1.0, 4.8, 0.8, 1.8, 1.0, 1.6),
    ("box_s",  0.5, -4.8, 1.0, 1.4, 1.0, 2.0),
]


def texture(seed: int) -> bytes:
    """High-contrast multi-scale noise plus random blobs and strokes.

    Corner-rich at every scale the camera sees between 1 and 6 m, and
    non-repeating (one seed per surface), so neither FAST nor KLT is starved
    and there is no aliasing between surfaces.
    """
    rng = np.random.default_rng(seed)
    img = np.zeros((TEX_PX, TEX_PX))
    for octave in (4, 8, 16, 32, 64, 128):
        g = rng.random((octave, octave))
        up = np.kron(g, np.ones((TEX_PX // octave, TEX_PX // octave)))
        img += up / octave ** 0.35
    img = (img - img.min()) / (img.max() - img.min())
    yy, xx = np.mgrid[0:TEX_PX, 0:TEX_PX]
    for _ in range(140):
        cx, cy, r = rng.integers(0, TEX_PX, 2).tolist() + [int(rng.integers(6, 60))]
        img[(xx - cx) ** 2 + (yy - cy) ** 2 < r * r] = rng.choice([0.05, 0.95])
    for _ in range(60):
        x0, y0 = rng.integers(0, TEX_PX, 2)
        w, h = rng.integers(4, 120, 2)
        img[y0:y0 + h, x0:x0 + w] = rng.choice([0.0, 1.0])
    buf = io.BytesIO()
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8), "L").save(buf, "PNG", optimize=False)
    return buf.getvalue()


def solid_link(name: str, x, y, z, sx, sy, sz, tex: str) -> str:
    return f"""    <link name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      <visual name="v">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://validation_scene/materials/textures/{tex}</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
"""


def build() -> dict[str, bytes]:
    files: dict[str, bytes] = {}
    links = []
    names = ["ground"] + [s[0] for s in SOLIDS]
    for i, n in enumerate(names):
        files[f"materials/textures/{n}.png"] = texture(1000 + i)
    links.append("""    <link name="ground">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="v">
        <geometry><plane><normal>0 0 1</normal><size>16 16</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://validation_scene/materials/textures/ground.png</albedo_map></metal></pbr>
        </material>
      </visual>
    </link>
""")
    for (n, x, y, z, sx, sy, sz) in SOLIDS:
        links.append(solid_link(n, x, y, z, sx, sy, sz, f"{n}.png"))
    files["model.sdf"] = ("<?xml version=\"1.0\"?>\n<!-- GENERATED by docker/sim/tools/gen_validation_world.py -- do not edit. -->\n"
                          "<sdf version=\"1.9\">\n  <model name=\"validation_scene\">\n    <static>true</static>\n"
                          + "".join(links) + "  </model>\n</sdf>\n").encode()
    files["model.config"] = b"""<?xml version="1.0"?>
<!-- GENERATED by docker/sim/tools/gen_validation_world.py -- do not edit. -->
<model>
  <name>validation_scene</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Phase-3 validation scene: textured structure 2-6 m from the flight path (SIMULATION ONLY).</description>
</model>
"""
    world = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- GENERATED by docker/sim/tools/gen_validation_world.py -- do not edit.
     Physics, gravity, magnetic field, lighting and spherical coordinates are
     copied from PX4's forest.sdf; only the scenery differs (PATCHES.md s49). -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <include>
      <uri>model://validation_scene</uri>
      <name>validation_scene</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
    files["__world__"] = world.encode()
    return files


def target(rel: str) -> str:
    return OUT_WORLD if rel == "__world__" else os.path.join(OUT_MODEL, rel)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    files = build()
    bad = 0
    for rel, data in files.items():
        p = target(rel)
        if a.check:
            ok = os.path.exists(p) and open(p, "rb").read() == data
            bad += not ok
            print(f"  {'ok   ' if ok else 'DIFF '} {os.path.relpath(p, ROOT)}")
        else:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            open(p, "wb").write(data)
            print(f"  wrote {os.path.relpath(p, ROOT)}  ({len(data)} B, sha {hashlib.sha256(data).hexdigest()[:10]})")
    if a.check:
        print("all generated files match" if not bad else f"{bad} file(s) differ -- regenerate")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
