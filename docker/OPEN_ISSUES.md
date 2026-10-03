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

## 2. The scripted flight paths are versioned; results are path-specific

**Status:** managed, not closed.

Path A (the original) uses straight constant-velocity legs at fixed altitude.
That leaves accelerometer bias and inertial scale weakly observable, which is a
plausible contributor to the stereo-inertial result above and to the 31-38
`Not enough motion for initializing` resets (PATCHES.md s33). Path B adds yaw
turns, altitude changes and varied acceleration.

Every experiment record carries its path version. **Numbers from different path
versions are not comparable** and must not be pooled into one mean.

---

## 3. `ros1_bridge` is not built

**Status:** open, out of scope. See PATCHES.md s30. Requires amd64 emulation;
the reproduction does not depend on it. Only needed to bring COVINS poses back
into ROS 2.

---

## 4. COVINS runs without an IMU

**Status:** open, inherent to the phase-1 configuration. COVINS is a
visual-*inertial* backend being fed RGB-D keyframes with no inertial data, so
its inertial machinery is unexercised. Use GBA `action: 4`/`5` (visual), never
`0`/`1`. See PATCHES.md and the README.

---

## 5. Single-agent COVINS does no place recognition

**Status:** open, configuration not fault. `placerec.inter_map_matches_only: 1`
in this repo's `config_backend.yaml` means a single agent has no second map to
match against, so expect no loop closures.
