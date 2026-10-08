# HANDOFF — phase 3 (OpenVINS vs ORB-SLAM3) on the amd64 simulation server

Written 2026-10-04 so that a fresh session can continue from this file alone.
Detailed history and reasoning live in `docker/PATCHES.md` (§41–§49 are this
host's work). Where this file and PATCHES disagree, PATCHES carries the
evidence; this file is the summary.

---

## 1. Where things stand

| | |
|---|---|
| Main repo | `EkanshSingh401/drone_multislam_orb3slam`, branch **`phase3/openvins-sim`** @ see `git log` (PATCHES §50–64; overnight run: DECISIONS.md, MORNING_REPORT.md). `amd64-gpu` no longer tracks it (local `68c013b`, origin `ad9b259`). History is linear on top of the Mac's final commit `4624988`. |
| OpenVINS fork | `EkanshSingh401/open_vins`, branch `openvins-integration` @ **`5b14b93`** (pushed), pinned by commit in `docker/sim/Dockerfile` (`OPEN_VINS_COMMIT`). |
| Other forks | `active_slam_msgs` `a0790aa`, `active_slam_planner` `f56e114`, `active_slam_information` `e225068` (pinned in the Dockerfile). |
| Who runs experiments | **This host only.** The Mac is retired from experiments. |
| Images | `drone-sim:jazzy-amd64`, `covins-backend:melodic-amd64`. Build sim with `docker compose -f docker/compose.yaml build sim` (only the last layers rebuild for script/config changes). The scripts are **baked into the image**: rebuild + `up -d --force-recreate sim` after editing anything in `docker/scripts/`. |

**Headline result so far** (bags of §55, IMU bias and stamp fixed §54–§56).
OpenVINS while airborne, GT-window ATE: **validation 1–3 cm** (path A
0.8–1.3, path B 2.0–2.9 cm), **forest 3–14 cm** (A 3.2–6.4, B 6.8–13.7 cm),
Sim(3) scale 0.99–1.00. Full-covariance orientation NEES 3.7–7.8 on validation,
11–27 in the forest (expected 3); position NEES ≤ 0.3 / 0.4–5.
(Before the §56 stamp fix: validation 2–5 cm, forest 5–21 cm.)

**The forest's earlier metre-scale errors were mostly the IMU bias bug, not
parallax.** On the old bags forest ATE was 0.11–4 m (A) and 0.25–153 m (B),
and it was attributed to distant features failing triangulation (§49, R8). With
only the Gazebo IMU bias corrected (§55) the same scene and paths gave 5–21 cm,
and 3–14 cm after the §56 stamp fix. The forest is still harder than validation
(roughly 3–5x the error; the
triangulation statistics of §49 still hold), but it no longer fails.

OpenVINS still **diverges after touchdown** when stationary (no parallax, ZUPT
off; §50). ORB-SLAM3 **stereo-inertial is broken** on this rig on every bag,
before and after the IMU bias fix (§52, §55; not re-run after the §56 stamp
fix), while stereo-only is cm-level. A time-boxed config pass (§56) found no
error. **Basalt** (upstream binary, ROS-free, stock EuRoC config; §58) is the
replacement stereo-inertial baseline: GT-window ATE validation 3–7 cm, forest
4–19 cm, all frames consumed.

**Comparison convention (§59).** OpenVINS is reported at two settings,
always labelled: **OpenVINS (all frames, 30 Hz)**, the default comparable to
Basalt, and **OpenVINS (every other frame, 15 Hz)**, our system setting
(`track_frequency: 20.0`). Held-out flights (§59), GT-window ATE:

| | validation (3) | forest (3) |
|---|---|---|
| OpenVINS, every other frame (15 Hz) — system setting | 1.1 cm | 2.0–2.1 cm |
| OpenVINS, all frames (30 Hz) | 1.2–1.7 cm | 5.6–9.6 cm |
| Basalt (all frames, stock config) | 3.1–3.6 cm | 4.6–5.0 cm |

**Planner phase (§60–61).** JointCovariance carries OpenVINS's linearization
point; the planner's Jacobians match OpenVINS's to 1e-15 (gate, §60). The C++
`fisher_ig_estimator` (active_slam_information) scores the metric map by
D-optimality and is launched by `active_slam.launch.py`. Prediction vs realized
gain over 1 s segments: 1.31x optimistic as built, 1.09x with OpenVINS's
propagation included (§61). Of the remaining ~10%: visible-but-unused
measurements over-predict ~3 nats/s (mostly features tracked in one camera only),
MSCKF + new landmarks under-predict ~0.6 nats/s; predicting exactly the used
measurements matches realized (§62). Closed loop in sim (§64): PX4 SITL on OpenVINS vision only,
OctoMap mapper behind the nvblox interface, frontier and IG planners: **both
complete autonomous flights without collision** (first milestone; no comparison
yet). Vision-only flight needs immediate disarm at touchdown (OpenVINS diverges
on the ground within ~1 s) — the same hazard applies on hardware.
**Note:** predicted vs realized is measured against OpenVINS's own covariance, so
it validates agreement with the filter's belief; NEES separately measures whether
that belief is true (orientation still overconfident, §58–§59).

**Closed loop: how to run (§64, image-built since the overnight Stage 0).**
`docker exec drone-sim /out/cl/cl_flight.sh <frontier|ig> /out/cl/<name>` (copy of
`docker/tools/closed_loop/cl_flight.sh`; brings up the depth rig in the validation
world, sets PX4 vision-only, starts OpenVINS live + converter + mapper + planner +
sim-only GT watchdog + executor, records `flight.bag`). Evaluate with
`cl_eval.py`, `cl_report.py`, `clearance.py`, `gt_window_eval.py`.

**Closed-loop comparison protocol (overnight Stage 2; written before running).**
Validation scene, stock config, no parameter changes between or after flights.
Planners: `frontier` (baseline) and `ig` (sigma_virtual = 1.0, p(used) 0.92).
5 flights each, alternated f,i,f,i,... Each flight: PX4 vision-only (EKF2 EV),
OpenVINS initialized at rest, VIO health gate, 180 s exploration budget from
reaching 1.5 m, then retrace home, land (vision hold below 0.4 m), disarm.
Metrics per flight (`flight_summary.py`): known volume at 0/30/60/90/120/150/180 s
and final, path length, known volume per metre flown, OpenVINS ATE on the GT
airborne window (and per metre), full-covariance NEES (ori, rp, pos) on the
joint-covariance subset, min GT clearance (contact < 0.33 m), sim-only watchdog
terminations, arm failures, ESDF guard holds, IG real-vs-virtual gain share of the
chosen candidate (median, IQR). Reported as median and range over the 5 flights;
no claim from a single flight. Any failure counts; flights are not rerun to replace
failures.

---

## 2. Host and environment

- Ubuntu 22.04, Ryzen 9 5950X (32 threads, SMT on, no CPU isolation), 64 GB,
  GTX 1080 Ti (Pascal, compute 6.1).
- NVIDIA driver **580.178.04**, the last branch that supports Pascal, **apt-held**
  (`apt-mark showhold`). Never upgrade it. Containers must use **CUDA 12.x**
  (CUDA 13 dropped sm_61). Isaac ROS nvblox is unsupported (needs Ampere+ / driver 595+).
- `nvidia-persistenced` runs with Ubuntu's `--no-persistence-mode`, so
  persistence mode reads Disabled. Harmless.
- Docker 29.4.3, nvidia-container-toolkit 1.19.0, default runtime `runc`; the
  sim service requests the GPU in compose. Docker root is `/mnt/data/docker`.
- **`docker/out` is a symlink to `/mnt/data/drone_sim_out`** (866 GB). The root
  partition is only 50 GB: sensor bags must never go there. A fresh clone needs
  the same symlink. `docker/out` is gitignored.
- Containers run as the host user (`user: ${UID:-1000}:${GID:-1000}`, HOME
  `/home/sim`); files in `docker/out` belong to `ekansh`.
- Gazebo renders on the GPU via EGL. RTF with the full stack is ~0.72–0.79
  during flights.
- `git push` over HTTPS needs the gh credential helper:
  `git -c credential.helper= -c credential.helper='!gh auth git-credential' push origin <branch>`.
- `sudo` needs a TTY password; the user runs sudo commands themselves in a
  separate terminal (the `!` prefix has no TTY).

---

## 3. Data on disk

**Current bags (PATCHES §55, IMU bias fixed; §56 IMU stamps corrected in place,
originals kept as `flight_gzstamp.bag`, marker `IMU_STAMP_FIXED.txt`): all `20261005-`.**
New recordings get the stamp fix at source (`imu_restamp.py`, raw on `/camera/imu_gz`).
validation A `experiment_d455_pathA_validation_20261005-154055`,
validation B `experiment_d455_pathB_validation_20261005-160305`,
forest A `experiment_d455_pathA_20261005-165226`,
forest B `experiment_d455_pathB_20261005-171524`. OpenVINS outputs:
`docker/out/replay/<run>_ovser_imufix_fc/` (current; `_g9.81_fc` = before the stamp fix) (with joint-covariance dump);
ORB-SLAM3 SI replays `<run>_orb_si_r0.5/`. The `20261004-` sets below were
recorded with the IMU bias walking 42x too fast (§54) and are **superseded**;
the s54 synthetic bags in `docker/out/diag_s54/synth/` are kept until the fix
is fully signed off.


Experiment directories (under `docker/out/`), each with `rundirs.txt` listing
its runs. Every run's sensor bag is `docker/out/eval/<run>/flight.bag`
(FILE-zstd mcap; reading it leaves an uncompressed `.mcap` beside it).

| condition | experiment dir | runs |
|---|---|---|
| validation, path A | `experiment_d455_pathA_validation_20261004-153547` | 193700 194115 194531 195005 195439 |
| validation, path B | `experiment_d455_pathB_validation_20261004-155800` | 195913 200912 201905 202940 204137 |
| forest, path A | `experiment_d455_pathA_20261004-013513` | **053636 (tuning bag — exclude from results)** 054056 054533 055010 055428 |
| forest, path B | `experiment_d455_pathB_20261004-015734` | 055847 060837 061845 062853 063858 |

All runs are prefixed `20261004-`. Live ORB-SLAM3 stereo-inertial ran during
recording (its log is `docker/out/eval/<run>/orb_slam3.log`, with clock beacons).
OpenVINS outputs are in `docker/out/replay/<run>_ovser_g9.81/` (serial runner),
with per-run summaries `docker/out/replay/<run>_ovser_g9.81_{matched.txt,full.json,scene.json,nees*.txt}`.

Junk to ignore: `experiment_d455_pathA_20261004-005410`, `..._012857`,
`experiment_d455_pathB_20261004-011037` (disk-full / test attempts) and
`experiment_d455_pathA_validation_20261004-152944` (single test flight; its run
`193057` is a valid validation-A flight but is not part of the 5-run set).

Also on disk: a 1-run forest stereo-only set (`experiment_d455_stereo_pathA_*`)
and older phase-1/2 material from the Mac era.

---

## 4. How to run things

All from the repo root on the host.

```bash
# record N flights with sensor bags (live ORB-SLAM3 stereo-inertial)
WORLD=validation ALT=1.5 RECORD_SENSORS=1 PATH_VERSION=A RIG=d455 ./docker/scripts/run_experiment.sh 5
#   WORLD=forest (default) | validation ;  PATH_VERSION=A | B
#   disk preflight refuses to fly (exit 5) unless 3x the expected bag size is free

# OpenVINS on recorded bags (deterministic serial runner), then evaluate
./docker/scripts/phase3_openvins.sh determinism <exp_dir>        # two runs, must be byte-identical
./docker/scripts/phase3_openvins.sh replay      <exp_dir> 9.81   # gravity_mag override via OV_GRAVITY_MAG
./docker/scripts/phase3_openvins.sh evaluate    <exp_dir> 9.81
./docker/scripts/phase3_openvins.sh jointcov    <exp_dir>
./docker/scripts/phase3_openvins.sh topicspread <exp_dir>        # live node, 3 replays (one-off)
python3 docker/scripts/phase3_table.py <exp_dir> 9.81 [--exclude 20261004-053636]
```

Key pieces:
- `ros2_serial_msckf` (in the fork): reads the bag directly, drives the live
  node's own `ROS2Visualizer` callbacks on one thread. **Bit-deterministic.**
  `run_subscribe_msckf` (the live node) is unchanged and is not deterministic
  under replay.
- `docker/scripts/replay_estimator.sh --estimator openvins_serial` wraps it, copies the config
  into `<outdir>/ov_config/` (with any `OV_GRAVITY_MAG` edit asserted), logs at DEBUG.
- `ov_prep.py`: converts OpenVINS IMU poses to **base_link** for ATE
  (IMU is 0.257 m from base_link), and ground truth to the **IMU** frame for NEES.
  Uses the per-update state files (`ov_state_est/std.txt`), not `/odomimu`.
- `ov_scene_stats.py`: median feature depth, triangulation and MSCKF rejection
  rates, tracked features — parsed from the fork's DEBUG `[FI] [MSCKF] [TRACK] [GRID]` lines.
- `ov_full_metrics.py`: OpenVINS full post-init ATE, Sim(3) scale, window coverage.
- Configs are **generated**: `docker/sim/tools/gen_d455_sim.py --write|--check`
  (rig, calibration, estimator configs, bridges) and
  `docker/sim/tools/gen_validation_world.py --write|--check` (validation scene).
  Never hand-edit `docker/sim/config_sim_only/*` or the validation world.

Current OpenVINS estimator settings that differ from the Mac era:
`init_imu_thresh 0.5` (measured, §46), `fast_threshold 5` (pre-registered rule, §48).
`try_zupt false`, `zupt_max_disparity 0.5`, `zupt_max_velocity 0.1`,
`zupt_chi2_multipler 0` (check what 0 means in `UpdaterZeroVelocity` before
enabling ZUPT), `gravity_mag 9.81`.

---

## 5. Results (with definitions)

**Definitions used below**
- *ATE*: RMSE of position error after SE(3) Umeyama alignment, **no scale**,
  nearest-neighbour association within 0.05 s, against Gazebo ground truth
  (model pose = base_link). OpenVINS is converted to base_link first.
- *Airborne segment*: from OpenVINS initialisation (≈0.4 s after takeoff) to
  touchdown (ground-truth z drops below 0.05 m after the last time it was above
  0.5 m). **Not yet the agreed GT window** — see next steps.
- *Matched window (deprecated)*: ORB-SLAM3 VIBA-2 completion to end of bag,
  imposed on both estimators (`eval_window_matched.sh`). Retired by decision;
  see §7.
- *Sim(3) scale*: scale from a Sim(3) alignment on the same span (1.0 = metric).
- *Median depth (all attempts)*: median anchor-frame depth over all
  triangulation attempts, accepted and rejected. *Tri reject*: triangulation
  rejects / attempts at OpenVINS's feature initializer (MSCKF + SLAM init).

**R1 (SUPERSEDED: old bags, IMU bias 42x; see headline and §55). OpenVINS, airborne segment, gravity_mag 9.81, fast_threshold 5** (§49)

| set | n | ATE per run (m) | mean | Sim(3) | depth (all) | tri reject |
|---|---|---|---|---|---|---|
| validation A | 5 | 0.030 0.021 0.037 0.029 0.030 | **0.029** | 0.986–1.001 | 3.9–4.5 m | 57–69% |
| validation B | 5 | 0.071 0.079 0.047 0.058 0.051 | **0.061** | 0.998–1.003 | 4.2–4.4 m | 59–64% |
| forest A (excl. 053636) | 4 | 4.06 0.79 0.11 0.41 | 1.34 (median 0.60) | 0.23–1.00 | 6.0–8.7 m | 72–91% |
| forest B | 5 | 0.36 0.25 2.57 0.28 152.6 | median 0.36 | 0.002–0.97 | 7.2–8.0 m | 85–94% |

**R2. OpenVINS full post-init ATE (includes post-landing), validation** (§49):
A 0.043 4.90 2.90 0.60 6.44; B 2.14 0.22 3.07 3.52 0.051 (m). The large values
are the post-landing divergence, not flight error.

**R3. ORB-SLAM3 stereo-inertial** — only on the deprecated matched window,
which is not a fair or converged window (it contains jumps up to 102 m). **No
trustworthy ORB-SLAM3 validation numbers exist yet.**

**R4. ORB-SLAM3 stereo-only, forest path A** (full flight; it has no transient):
amd64 0.030 0.038 0.031 0.037 m (n=4) vs Mac 0.0302 ± 0.0033 m (n=3) — the
machines agree (§41 results).

**R5. Determinism.** Serial runner: byte-identical state output across runs
(PASS). Live node under topic replay: one pair diverged up to 2.8 cm, a later
triple was bit-identical — intermittent (§46–47).

**R6. Joint covariance** (serial run, 281 messages, forest bag 053636; §47):
rank deficit at `max_eig·1e-12` exactly 6 in all 281; 6th-smallest/max eig
median 2e-16 vs 7th 4.5e-11; null space = (IMU pose − newest clone) to ≤1.4e-3 deg;
duplicate-row residual ≤1.2e-13 relative; published after the full update cycle
in both live and serial nodes. **PASS by those criteria.** Open: a checker sample
at t=83.956 showed dim 130 / 2.3e-9 where the recorded matrix is dim 132 / 3e-19.

**R7. Fork sanity.** OpenVINS's own simulator on the fork (stock `rpng_sim`,
296 m): position RMSE 0.046 m, orientation 0.18°, NEES 3.8 (pos) / 0.7 (ori).
The process segfaults during ROS teardown after finishing; harmless.

**R8. Triangulation diagnosis, forest 053636** (§49; the statistics stand, but they are NOT the cause of the old metre-scale forest errors -- that was the IMU bias, §55): 99% of triangulation
rejects fail `cond > 1e4` (median condA 1.1e5); MSCKF candidates ~1/update,
83% rejected, chi2 only 2%. fast_threshold sweep (30→5): cam0 tracked features
flat at 75–81 — threshold is not the limiter; grid cells never reach their cap.

**R9. Gravity 9.80 vs 9.81, forest path A** (OpenVINS, while it was not working):
differences ≤ 0.03 m. **Not a valid gravity result** — redo on validation (next steps).

---

## 6. Next steps (decided by the user, in this order)

1. **Enable `try_zupt`, disparity check on** (`zupt_max_disparity` > 0). Log
   every ZUPT event with ground-truth speed at that instant; confirm **no ZUPT
   fires while airborne, hover included**. Change it in `gen_d455_sim.py`, record
   the change in PATCHES. Check `zupt_chi2_multipler: 0` semantics first.
2. **Replace the evaluation window with a ground-truth window**: from **3 s after
   takeoff** (GT altitude above a threshold) **until touchdown**, identical for
   every estimator. Report each estimator's **time-to-usable-estimate**
   separately. Re-run ORB-SLAM3 *and* OpenVINS on **all validation and forest
   bags** with this window (ORB-SLAM3 can use the live run's `est_orbslam3.tum`
   and `gt.tum` in `docker/out/eval/<run>/`; consider replaying it on the bag).
   The **airborne segment is the primary number.**
3. **NEES on the same airborne window**, posyaw alignment; report **roll/pitch
   NEES separately** (independent of yaw alignment). The saved std file gives a
   diagonal covariance only; `/odomimu` rows carry the full 6x6 but are not
   deterministic.
4. **Gravity 9.80 vs 9.81 on all 10 validation bags by replay**
   (`OV_GRAVITY_MAG=9.80`); report the per-bag difference. Apply the rule:
   an effect counts only if it replicates across two independent run sets, or
   confidence intervals do not overlap.

Later / noted, not started:
- The 130 vs 132 joint-covariance mismatch (checker ran on the serial runner at
  DEBUG; the recording was at WARNING).
- Why tracked features plateau at ~78 when no grid cell is saturated.
- Real D455: apply the same fast_threshold rule; the per-threshold counts are in §48.

---

## 7. Retractions and corrections (do not reuse these numbers)

- **§62 "forest usage drops for viewing-angle change > 5° (0.81) and > 10° (0.62)"**:
  did not replicate. On path C (§63, 10x more large-angle records) usage beyond
  10° is 0.937, no lower than below 5° (0.922). The §62 bins were small (n=157 at
  > 10°). In sim, usage does not depend on viewing angle.

- **All OpenVINS position NEES before §55** (§53 table, its "vertical σ ≈ 2.4 mm",
  §54 position NEES): `ov_prep.py` read position σ from the wrong std-file
  columns (p_y, p_z, v_x). Fixed in §55. Orientation/roll-pitch NEES stand.
- **§53 "SDF OU bias matches the configured random walks"**: wrong, gz-sensors
  uses dynamic_bias_stddev as a density (§54).
- **All results on the `20261004-` bags**: IMU bias 42x too fast (§54); use §55.

- **Mac §36 gravity effect (68% / 85%)**: withdrawn by the Mac (§40); full-flight
  online ATE is not reproducible (two 5-run sets differed 4.9×).
- **Mac 0.0138 m stereo-only ATE**: a single lucky run; the 3-run value is
  0.0302 ± 0.0033 m.
- **"OpenVINS tilt error 8.9° rms / 46° max"** (this host): wrong — compared
  gravity directions across world frames 180° apart in yaw. Correct value:
  0.5° rms tilt, 0.7° full rotation.
- **"30–45 tracked features"**: an eyeball estimate from trackhist; the
  tracker's own count is ~75–81.
- **`ov_eval` orientation RMSE 15–33° and all NEES values so far**: artefacts of
  posyaw alignment fitted to near-stationary or diverged positions. Do not use.
- **Matched-window (VIBA-2) results for both estimators**: superseded by the GT
  window decision; ORB-SLAM3's "converged" window contained 2.9–102 m jumps.
- **"init_imu_thresh default is single-threaded subs"** wording in an early §46
  draft: corrected in place — the live node forces multi-threaded subscribers.
- **Determinism "PASS" on empty files** (before the init fix): both outputs were
  empty; the real PASS is the serial runner (§47).

---

## 8. Open issues

- OpenVINS post-landing divergence (to be addressed by ZUPT, step 1).
- **OpenVINS orientation NEES** after the bias (§55) and stamp (§56) fixes (§58:
  falls to ≲3 at track_frequency 15 Hz, and forest ATE improves 7.1 → 3.0 cm,
  consistent with correlated per-frame errors counted as independent; setting not
  adopted yet):
  full-covariance 3.7–7.8 validation, 11–27 forest (expected 3); roll/pitch
  4–19 / 15–61 (expected 2). Residual unattributed. Use full-covariance NEES
  (`docker/scripts/s54/run_fullcov.sh`); the diagonal understates orientation.
- **Slow segments ~2.5x more overconfident than fast at every frame rate** (§58:
  slow/fast orientation NEES ratio 2.2–2.8 at 30, ~21 and ~10 Hz tracking). A second
  effect, separate from correlated per-frame error; likely weak observability under
  low excitation (hover / slow rotation). Not pursued yet.
- Basalt's Sim(3) scale is +0.4 to +2.2% on every bag (§59): its calibration is
  verified honoured (pinhole, our intrinsics, time offset 0), so not a config
  mismatch; internal to Basalt, not pursued.
- **In sim, measurement usage is ~geometry-independent (§63)**: ~92% per camera,
  stereo-partner loss looks random, no viewing-angle effect on path C. A
  feature-quality predictor therefore cannot be learned from these images; it
  needs degraded images (simulated blur/noise/exposure, or real D455 data).
  p(used) stays in the predictor as a constant correction (~0.92).
- **Landmark usage model is sim-only (§63).** p(used) (additive logistic over
  binned viewing-angle change, depth, border distance, both-cameras term;
  `docker/out/diag_s63/usage_model.*`) was fitted on near-noise-free Gazebo images,
  where usage is ~92% and losing the stereo partner looks almost random. It must
  be refit on real D455 data (blur, exposure, texture). This analytic model is the
  BASELINE any learned feature-quality predictor has to beat (held-out Brier 0.0750
  vs 0.0756 for a constant; predicted/realized gain 1.03 with it vs 1.11 without).
- **Overnight 2026-10-07 (MORNING_REPORT.md, §65–§66):** (1) IG planner contacts
  (2/17 validation IG flights) — margins/planner need work before more comparisons;
  (2) post-landing disarm missed in 17/32 flights (sim watchdog disarmed on the ground):
  vision-only PX4 + OpenVINS ground divergence is a flyaway risk on hardware;
  (3) IG objective ~95% virtual-landmark gain at every sigma_l tried (0.01–3 m);
  (4) forest coverage 'decrease' was a reporting artifact (fixed, §68; use *_v2 results);
  (5) VIO jump/drift faults: PX4 follows them unless gated (vio_health_gate); slow drift
  needs a second sensor.
- **Day 2 (REPORT_2026-10-07.md, §68–§70):** sigma_l plumbing verified (flat curve is the
  log gain above a ~9 mm crossover); coverage report fixed; new `pose_cov` planner mode and λ
  sweep (§69); IG collisions traced to an ESDF-band blind spot for low obstacles under the
  vehicle (fixed: band 1.0 m below the slice) + speed-scaled margins (§70).
- **Day 3 (§71, REPORT day 3):** real-landmark visibility is low because candidates face unknown
  space (92% FOV failures; predictor validated vs GT); pose term split logged (a meas / b travel);
  new pose_cov candidate set `yaw_samples`/`path_spacing` (old = 1 / 0); NEES jump at λ ≥ 1 is the
  home phase (ESDF-guard hover stalls, no timeout; low yaw rate), not exploration. Step-5 run:
  `docs/day3/d3_step5.list` → results in `docs/day3/` (§72): new set → NEES 19–41 (low yaw rate).
- **Day 4 (§73):** recorded coverage is inflated 1.8–3.7x by the mapper's no-return-as-free handling
  (use `day4/gt_coverage.py` GT-geometry coverage on validation flights); NEES blow-up of the new
  set is caused by yaw sampling (low rotation -> roll/pitch over-confident); pose-gain prediction does
  not track realized covariance at goal scale (optimizer's curse shown with `random_pick`).
- **Day 5 (REPORT_2026-10-08.md, DECISIONS "Day 5", PATCHES §74):** (1) executor `guard_timeout_s`
  (10 s) lands in place after a continuous home-phase ESDF hold. (2) mapper `no_return_as_free`
  (default false): recorded coverage now within 3% of GT-pose coverage at 180 s; GT coverage tool
  `docker/tools/closed_loop/day5/gt_coverage.py <dir> <world>` (no-return = unknown, curve metrics
  t50/t90/AUC vs `scenes.reachable_m3`); flight_summary carries `gt_metrics`. (3) office scene
  (`gen_office_world.py`, geometry `docker/tools/closed_loop/scenes.py`, `WORLD=office GEOFENCE=40.0`):
  frontier covers 23–26% in 180 s. (4) IMU params match (in-flight check); roll/pitch overconfidence
  in slow flight is caused by the sim's VISION inputs (correlated per-frame tracking errors on
  noise-free renders): OpenVINS's own simulator on the same trajectories is consistent; 5 Hz tracking
  removes it (diagnostic only). Slow in-place rotation (0.05 rad/s) made live OpenVINS drift and jump
  in 3/3 scripted flights (gate/watchdog). (5) Predictor ladder (`gain_ladder`, built in the image):
  one-step prediction matches OpenVINS, multi-step from one snapshot over-counts ~2x by 1 s, worst
  under rotation; realized pose information is an equilibrium (SLAM 89%, MSCKF 9%). Step 6 skipped.
  Tools: scripted flights `TYPE=script` (`script_goals.py`), stage-logdet patch for a scratch OpenVINS
  build (`day5/ov_stage_logdet.patch`, `/out/cl/day5/ovws`), OpenVINS-simulator runs on flight GT
  (`day5/run_ovsim.sh`, `make_traj.py`, `sim_nees.py`).
  **Pass doubles as `40.0`, never `40`** in env/params (int -> InvalidParameterTypeException).
  **Do not pause run_batch by killing it mid-bringup** (orphaned imu_restamp -> duplicate /camera/imu).
- Gravity 9.80 vs 9.81 (§53): no ATE effect (sub-mm, sign flips between sets);
  scale +0.0005 consistently -- negligible.
- **ZUPT disabled** (`try_zupt: false`): disparity gating fires during hover
  (airborne accepts on all 10 validation bags, PATCHES §50). For hardware,
  gate ZUPT on PX4's landed state (`vehicle_land_detected`) instead of image
  disparity.
- ORB-SLAM3 stereo-inertial is broken on this rig (PATCHES §52; **still broken on
  the §55 bags with the IMU fixed**: 0.5–76 m, scale 0.01–0.91): replayed on
  the validation bags, stereo-only is 0.7–1.3 cm but stereo-inertial is
  0.7–90 m with Sim(3) scale 0.00–0.88 even after IMU init + VIBA 2 with no
  later reset. IMU delivery verified clean (0 gaps / 0 late). Next suspect:
  `orbslam3_d455_stereo_inertial.yaml` (Tbc, noise, units/axes).
  Replay with `PLAY_RATE=0.5 ./docker/scripts/phase3_orbslam3.sh <exp> [si|st|both]`.
- Simulated cameras are noise-free, blur-free and perfectly calibrated while the
  IMU is realistically noisy (`docker/OPEN_ISSUES.md` §1) — results favour vision.
- GPU drops to its P8 idle clock under light Gazebo load (costs ~0.85→0.75 RTF);
  fixing it needs sudo (locked clocks); not pursued.
- The OpenVINS joint-covariance checker (`check_joint_cov.py`) uses a 1e-9
  relative null threshold that over-counts the deficit; relative 1e-12 is the
  agreed criterion.

---

## 9. Operational gotchas (each one cost time)

- `replay_estimator.sh` now refuses (exit 4) if anything already publishes the
  sensor topics or /clock (§56). `run_experiment.sh` still leaves the LAST run's Gazebo/PX4 stack running. Run
  `docker compose -f docker/compose.yaml exec -T sim /opt/scripts/bringup_sim.sh --stop`
  before replaying on the default ROS domain, or replay with
  `docker exec -e ROS_DOMAIN_ID=77` (§55: it doubled every replay frame).

- **Never edit a script while it is executing** (bash reads incrementally; this
  killed a Mac experiment, §37). For scripts a running chain will call later,
  write a new file and `mv` it into place.
- `run_experiment.sh` refuses to start if the container's copies of six scripts
  differ from the host (md5 guard) — rebuild and recreate after edits.
- `pkill -f '<pattern>'` matches its own shell when the pattern appears in the
  command line; use the `[x]yz` bracket trick.
- `docker exec` does not source ROS: wrap with
  `bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; ..."`.
  Feeding a script on stdin needs `docker exec -i`.
- Recreating the sim container wipes its `/tmp` (scratch builds live there).
- PATCHES section numbers: check the highest `## N.` before adding one.
- Gazebo camera sensors render only when subscribed; a bare `gz sim -s` without
  PX4's server config never creates camera topics (§20).
- Static models are absent from `/world/<w>/dynamic_pose/info`; ground-truth
  index 0 is the drone in both worlds.
- OpenVINS `trackhist` images are stamped with wall time, not image time.
