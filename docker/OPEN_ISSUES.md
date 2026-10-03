# Open issues

Known limitations and deferred work for the Dockerised ORB-SLAM3 / COVINS /
OpenVINS simulation. Each entry says what is wrong, why it matters, and what
would close it. Nothing here is a bug in the reproduction: these are
*fidelity* and *scope* gaps that change how results should be read.

---

## 1. Simulated cameras are unrealistically good relative to the IMU

**Status:** open, deliberately not fixed yet. Deferred by decision; do not
implement without asking.

The Gazebo IR cameras are:

- **noise-free** -- no photon shot noise, no read noise, no fixed-pattern noise;
- **blur-free** -- instantaneous exposure, so no motion blur however fast the
  airframe rotates, and no rolling-shutter skew;
- **exactly calibrated** -- `gen_d455_sim.py` generates the SDF and the
  estimator configs from one spec, so intrinsics, extrinsics and the stereo
  baseline carry *zero* calibration error;
- **perfectly synchronised** -- 1226 of 1226 stereo pairs share a bit-identical
  timestamp (PATCHES.md s31 area / `check_stereo_sync.py`), with no
  camera-to-IMU time offset.

The IMU, by contrast, is modelled with realistic continuous-time noise
densities and an Ornstein-Uhlenbeck bias process
(`config_sim_only/D455_SIM_CALIBRATION.md`).

**Why it matters.** The two sensors are not degraded equally, so the simulation
systematically favours vision over inertial. This is the most likely reason
stereo-only ORB-SLAM3 reaches 0.0138 m while stereo-inertial reaches
1.369 +/- 0.856 m on the same rig (PATCHES.md s35): with flawless images and a
known baseline, the IMU can only add error. On real hardware, where images are
noisy, motion-blurred and imperfectly calibrated, the balance is very
different -- which is precisely the regime where VIO earns its keep.

**Read results accordingly.** Absolute ATE numbers from this rig are optimistic
for vision-only methods and should not be used to argue that an IMU is
unhelpful in general, or to compare against published results on real datasets.
Relative comparisons between estimators fed *identical* input remain valid.

**What would close it**, roughly in order of value per unit effort:

1. Camera noise: Gazebo's `<noise>` on a camera sensor (Gaussian, per-pixel).
   Needs a realistic sigma for a D455 IR imager at the operating gain.
2. Motion blur / exposure: not natively supported by `gz-sim`'s camera sensor.
   Would need a post-process bridge node convolving along the projected optical
   flow, or multi-sample accumulation per frame -- expensive under llvmpipe.
3. Calibration error: perturb the estimator configs away from the SDF truth by a
   realistic Kalibr residual, deliberately breaking the single-source-of-truth
   invariant that `gen_d455_sim.py --verify` currently enforces. That check
   would need an explicit opt-out rather than being weakened.
4. Rolling shutter and camera-IMU time offset: a nonzero `timeshift_cam_imu`
   is already plumbed through the generated Kalibr files (currently 0.0).

Item 3 is the one that most changes conclusions, and also the one most likely
to be mistaken for a bug later -- so if it is ever done, it must be opt-in and
loudly labelled.

---

## 2. Gravity: the sim uses 9.81, Atlanta is ~9.795, the Jetson config says 9.81

**Status:** open for HARDWARE. Fix before the next hardware run. The simulation
side is settled and should be left alone.

Three different numbers are in play and only two of them agree for a good
reason:

| where | value | why |
|---|---|---|
| ORB-SLAM3 | 9.81 | hardcoded, `ImuTypes.h:46 GRAVITY_VALUE`, not configurable |
| this simulation | 9.81 | patched to MATCH ORB-SLAM3 (PATCHES.md s36) |
| Georgia Tech, actual | **~9.795** | local gravity at that latitude/elevation |
| Jetson OpenVINS config | 9.81 | **wrong on hardware -- off by ~0.015 m/s^2** |

**Why this is not a rounding detail.** In simulation, a **0.01 m/s^2** mismatch
between the world and ORB-SLAM3's constant was worth **68% of the ATE and 85% of
the run-to-run variance** (1.369 +/- 0.856 m -> 0.437 +/- 0.129 m, PATCHES.md
s36). The hardware error is **0.015 m/s^2**, i.e. half again larger, and in the
same direction of harm: it feeds the gravity-direction and accelerometer-bias
states the filter solves for, so it does not show up as a constant offset that
alignment removes. It shows up as inconsistent, run-dependent drift -- and it
will also bias NEES, because the covariance will not know about it.

**What to fix.** On the Jetson, set OpenVINS `gravity_mag: 9.795` (or the
properly computed local value -- see below). Nothing in this repo's simulation
configs should change: there, 9.81 is correct *by construction* because the
world is generated to match it, and `gen_d455_sim.py --verify` enforces that
agreement.

**ORB-SLAM3 on hardware cannot be fixed this way**, because 9.81 is a compiled
constant. Options, in order of preference: accept the ~0.015 m/s^2 error and say
so when reporting; or patch `GRAVITY_VALUE` and record it as a deviation. Do not
quietly do the latter -- it changes results and would otherwise be invisible.

**Getting the local value properly** rather than from memory: the WGS-84
Somigliana formula with a free-air correction,

    g(phi, h) = 9.7803267715 * (1 + 0.0052790414 sin^2(phi)
                                  + 0.0000232718 sin^4(phi)) - 3.086e-6 * h

with phi = latitude in radians and h = elevation in metres. For Atlanta
(phi ~= 33.78 deg, h ~= 320 m) that gives ~9.7953 m/s^2. Worth computing for the
actual test site rather than reusing this number.

## 3. Confirm the OpenVINS build on the Jetson's Humble container

**Status:** open, blocking the next hardware run.

`EkanshSingh401/open_vins` commit `06242b2` changes four includes in the
ROS-2-compiled sources to `__has_include` shims, because Jazzy removed the `.h`
spellings:

```cpp
#if __has_include(<image_transport/image_transport.hpp>)
#include <image_transport/image_transport.hpp>
#else
#include <image_transport/image_transport.h>
#endif
```

The four are `image_transport/image_transport`,
`tf2_geometry_msgs/tf2_geometry_msgs` and `cv_bridge/cv_bridge` in
`ROS2Visualizer.h`, plus `tf2_geometry_msgs/tf2_geometry_msgs` in
`ROSVisualizerHelper.h`'s `ROS_AVAILABLE == 2` branch.

The shim is designed to be distro-agnostic and the expectation is that Humble
takes the `.hpp` branch too -- Jazzy removed the old spelling rather than adding
the new one. **But that has only been verified on Jazzy.** Build `ov_msckf` in
the Jetson's Humble container and confirm it still compiles before the next
hardware run. If Humble lacks a `.hpp` for any of the four, the `#else` branch
covers it; the risk is not a missing header but an unnoticed behavioural
difference between two headers of the same name.

ROS 1 paths were deliberately left alone (`ROS1Visualizer.h`, the
`ROS_AVAILABLE == 1` branches, and the ROS-1-only test executables), so a ROS 1
build is unaffected.

## 4. The scripted flight paths are versioned; results are path-specific

**Status:** managed, not closed.

Path A (the original) uses straight constant-velocity legs at fixed altitude.
That leaves accelerometer bias and inertial scale weakly observable, which is a
plausible contributor to the stereo-inertial result above and to the 31-38
`Not enough motion for initializing` resets (PATCHES.md s33). Path B adds yaw
turns, altitude changes and varied acceleration.

Every experiment record carries its path version. **Numbers from different path
versions are not comparable** and must not be pooled into one mean.

---

## 5. `ros1_bridge` is not built

**Status:** open, out of scope. See PATCHES.md s30. Requires amd64 emulation;
the reproduction does not depend on it. Only needed to bring COVINS poses back
into ROS 2.

---

## 6. COVINS runs without an IMU

**Status:** open, inherent to the phase-1 configuration. COVINS is a
visual-*inertial* backend being fed RGB-D keyframes with no inertial data, so
its inertial machinery is unexercised. Use GBA `action: 4`/`5` (visual), never
`0`/`1`. See PATCHES.md and the README.

---

## 7. Single-agent COVINS does no place recognition

**Status:** open, configuration not fault. `placerec.inter_map_matches_only: 1`
in this repo's `config_backend.yaml` means a single agent has no second map to
match against, so expect no loop closures.
