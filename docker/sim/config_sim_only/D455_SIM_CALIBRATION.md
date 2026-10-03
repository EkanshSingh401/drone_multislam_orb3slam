# D455-mirror simulated rig — calibration and IMU noise units

**SIMULATION ONLY.** Everything in this directory describes the Gazebo
D455-*mirror* rig, not a physical D455. These files must never overwrite the
real sensor's Kalibr output — that is why they live in `config_sim_only/` rather
than alongside the hardware configs.

All of it is generated from one spec, `docker/sim/tools/gen_d455_sim.py`, which
emits the Gazebo SDF *and* these YAMLs. Regenerate and verify with:

```bash
python3 docker/sim/tools/gen_d455_sim.py            # print the derivation
python3 docker/sim/tools/gen_d455_sim.py --write    # regenerate everything
python3 docker/sim/tools/gen_d455_sim.py --check    # fail if files are stale
```

A single spec matters here because the SDF and the estimator config must agree
exactly. If they drift, a VIO run silently measures calibration error instead of
estimator error.

---

## 1. IMU noise: the unit conversion

This is the part that is easy to get wrong and impossible to notice afterwards.

| | Gazebo (`<noise>`) | OpenVINS / Kalibr |
|---|---|---|
| white noise | **per-sample standard deviation** (discrete) | **continuous-time noise density** |
| accel units | m/s² | m/s²/√Hz |
| gyro units | rad/s | rad/s/√Hz |

$$\sigma_{\text{discrete}} = \sigma_{\text{density}}\sqrt{f}
\qquad
\sigma_{\text{density}} = \frac{\sigma_{\text{discrete}}}{\sqrt{f}}$$

At **f = 200 Hz**, √f = **14.142136**. So putting a density straight into
Gazebo, or a discrete sigma straight into OpenVINS, misstates the noise by ~14×.
Nothing errors out; the filter simply mis-weights the IMU against the camera,
and you get a plausible-looking but wrong trajectory.

### Applied here

| quantity | declared (continuous) | → Gazebo (`<stddev>`, discrete) |
|---|---|---|
| accel noise density | `2.0e-3` m/s²/√Hz | `0.0282843` m/s² |
| gyro noise density | `1.6e-4` rad/s/√Hz | `0.00226274` rad/s |

### 2. Bias: Gazebo is *not* a plain random walk

Kalibr's `*_random_walk` is the density of a **random walk** (Brownian motion)
driving the bias. gz-sim instead models the bias as a **first-order
Gauss–Markov / Ornstein–Uhlenbeck** process, parameterised by

- `<dynamic_bias_stddev>` — the **stationary** standard deviation σ_b
- `<dynamic_bias_correlation_time>` — the correlation time τ

For an OU process `db/dt = −b/τ + w` with stationary standard deviation σ_b, the
driving noise density is

$$\sigma_{rw} = \sigma_b\sqrt{\tfrac{2}{\tau}}
\qquad\Longleftrightarrow\qquad
\sigma_b = \sigma_{rw}\sqrt{\tfrac{\tau}{2}}$$

The spec declares the **random-walk densities** (the physical quantity a
datasheet or Kalibr gives) and derives Gazebo's σ_b. Doing it in that direction
means the OpenVINS YAML carries the declared values exactly, with no round-trip
loss.

τ is set to **3600 s**, far longer than a ~60 s flight, so over a run the bias
behaves like a random walk rather than visibly mean-reverting.

| quantity | declared (continuous) | → Gazebo `<dynamic_bias_stddev>` |
|---|---|---|
| accel random walk | `3.0e-4` m/s³ | `0.0127279` m/s² |
| gyro random walk | `2.0e-6` rad/s² | `8.48528e-05` rad/s |

**Caveat, stated plainly:** OU with a finite τ is not identical to a true random
walk. Over 60 s with τ = 3600 s the difference is negligible, but for very long
runs the simulated bias will mean-revert in a way OpenVINS's random-walk model
does not expect.

---

## 3. Imagers

Mirrors a D455 in its usual VIO mode.

| | value |
|---|---|
| stereo IR resolution | **848 × 480** |
| rate | **30 Hz** |
| format | monochrome (`L8`) |
| stereo baseline | **95 mm** (D455; a D435 is 50 mm) |
| horizontal FOV | 87° |
| depth camera | 848 × 480 @ 30 Hz, `R_FLOAT32`, aligned to the left IR imager |
| depth clip | 0.2 – 20.0 m |

Intrinsics follow from the SDF with no estimation involved — a Gazebo pinhole
camera with horizontal FOV *h* and width *w* has

$$f_x = f_y = \frac{w/2}{\tan(h/2)}, \quad c_x = w/2, \quad c_y = h/2$$

giving **fx = fy = 446.802773**, **cx = 424.0**, **cy = 240.0**, and **zero
distortion**. That exactness is the point: it removes calibration error from the
comparison entirely.

---

## 4. Geometry and camera–IMU extrinsics

Body axes are ROS/REP-103: **+X forward, +Y left, +Z up**. Poses are relative to
the `d455_link` origin (the module mounting reference).

| element | position (m) relative to `d455_link` |
|---|---|
| `infra1` (left IR, cam0) | `( 0.0,  +0.0475, 0.0 )` |
| `infra2` (right IR, cam1) | `( 0.0,  −0.0475, 0.0 )` |
| depth | `( 0.0,  +0.0475, 0.0 )` |
| IMU | `( −0.00552, +0.00510, −0.01174 )` |
| module on airframe (`base_link` → `d455_link`) | `( 0.12, 0.0, 0.242 )` |

Resulting translations of each camera origin **in the IMU frame**:

| | translation (m) |
|---|---|
| IMU → cam0 | `( 0.00552,  0.0424,  0.01174 )` |
| IMU → cam1 | `( 0.00552, −0.0526,  0.01174 )` |

These are **realistic D455-*like* values chosen for this rig, not numbers
scraped from a vendor datasheet.** That is fine, and deliberate: because the SDF
is generated from this same spec, whatever is written here *is* the simulation's
ground-truth calibration by construction.

### T_cam_imu

`kalibr_imucam_chain.yaml` reports `T_cam_imu`, which takes points from the IMU
frame into the camera **optical** frame (z forward, x right, y down). Relative to
the Gazebo body frame (x forward, y left, z up) the optical frame is a pure axis
relabelling, since every sensor is mounted axis-aligned with the module:

```
optical x = −body y
optical y = −body z
optical z = +body x
```

so

```
R_cam_imu = [ 0 −1  0 ]
            [ 0  0 −1 ]
            [ 1  0  0 ]
```

and the translation is that rotation applied to the IMU origin expressed in the
camera body frame. The generator computes it; it is not hand-written.

---

## 5. Topic names

Deliberately identical to the Jetson/D455 stack, so OpenVINS configs and launch
files transfer unchanged:

| ROS 2 topic | content |
|---|---|
| `/camera/infra1/image_rect_raw` | left IR, mono8, 848×480 |
| `/camera/infra2/image_rect_raw` | right IR, mono8, 848×480 |
| `/camera/infra1/camera_info` | left intrinsics |
| `/camera/infra2/camera_info` | right intrinsics |
| `/camera/depth/image_rect_raw` | depth, 32FC1, metres |
| `/camera/depth/camera_info` | depth intrinsics |
| `/camera/imu` | IMU @ 200 Hz |

"`image_rect_raw`" is accurate rather than aspirational: the simulated cameras
have zero distortion, so the raw images *are* rectified.
