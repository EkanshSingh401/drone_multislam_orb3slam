# DECISIONS (overnight run, started 2026-10-07 02:00 EDT)

Decisions made without the user, each with what / why / alternatives.

## Stage 0
- **Package built in its own Dockerfile step after the fork build** (`COPY active_slam_sim`, then
  `colcon build --packages-select active_slam_sim` with executable checks). Why: it links ov_msckf
  and active_slam_information, so it must come after them; a separate step keeps a failure local.
  Alt: adding it to the existing fork colcon invocation (couples an experimental package to the VIO build).
- **Closed-loop harness stays in `docker/tools/closed_loop/` and runs from `/out/cl/`** (copied),
  now using the image's binaries only. Alt: bake it into /opt/scripts (would trigger the md5 guard
  and require rebuilds for every harness edit during the night).

## Stage 1
- **1b: did not get PX4's land detector to confirm before the OpenVINS backup.** Shipped
  vision HOLD below 0.4 m + LNDMC_TRIG_TIME 0.3 + PX4-primary/OpenVINS-backup disarm.
  Why: disarm is reliable now (no watchdog intervention) and further PX4 land-detector work
  (thrust thresholds, LNDMC_* tuning) risked the night's schedule. Alt: tune LNDMC_* further,
  or ZUPT keyed on PX4 landed (needs PX4 to detect first — circular in vision-only mode).
- **1c: VIO health gate (step consistency, latched) rather than EKF2 gate tuning.** Why: with
  EV the only aiding source, EKF2 resets to vision on persistent rejection, so tighter EKF2
  gates cannot stop a jump; turning faults into dropouts uses PX4's existing failsafe. Alt:
  tighter EKF2_EVP_GATE (tested implicitly: 0.5 m gate did not prevent the jump reset).
- **Gate threshold 0.15 m per message** (OpenVINS publishes at ~140 Hz, normal step error is
  mm-level). Not tuned on evaluation data.

## Stage 2/3/4 scheduling
- **sigma_l = 1.0 in the Stage 3 sweep reuses 3 of the 5 Stage 2 IG flights** (identical
  configuration) instead of flying 3 more; saves ~30 min. Sweep values {0.01, 0.1, 0.3, 1.0, 3.0} m,
  chosen a priori to span "virtual negligible" (0.01 m: frontier voxels treated as already
  known) to "virtual dominates" (3 m), without looking at results.
- **Stages 2–4 run as one background chain** (one simulator); Stage 5 code is written while
  it runs. Order of flights alternates planners / sigma values to spread slow drifts.

## Stage 2–4 (made while running / analysing)
- **Failures were not rerun or replaced** (protocol). Counted separately: in-flight
  (sim-only watchdog terminated in the air, or GT clearance < 0.33 m = contact) vs
  post-landing watchdog disarms (vehicle on the ground; the touchdown disarm was missed).
- **Stage 3 sweep did not reach "virtual negligible"**: even at sigma_l = 0.01 m real landmarks
  are only ~9% of the chosen candidate's gain (log-det). Not extended during the night: the
  values were fixed a priori and changing them after seeing results would be tuning on
  evaluation data. Next sweep should add smaller sigma_l (1e-3, 1e-4) or scale virtual
  landmark count.
- **Forest clearance not computed**: the forest's tree geometry is not modelled in
  clearance.py; the sim-only watchdog used a generous box. Forest collision-freedom therefore
  rests on "no in-flight watchdog termination", not on GT clearance.

## Stage 5
- **Degradation applied offline to recorded clean bags, not in the live rig.** Why: the single
  simulator was busy with Stages 2–4 all night; motion blur is computed from the recorded IMU
  gyro, so the offline result is identical to what a live node using the same module would
  produce for the same flight. The module (`active_slam_sim_py/degrade.py`) is the rig-level
  component; inserting it live needs a bridge remap (clean topics → degrader → standard topics)
  with the clean rig as default. Alt: wire it live and re-record (costs ~1 h of sim time).
- **6 datasets = 2 existing flights × {mild, moderate, severe}**: validation path C (yaw swings,
  stresses blur) and forest path A. Clean baselines for both already have usage records (s62/s63).
- **Presets** (light, gain, exposure, read noise, e-/DN): mild (1, 1, 3 ms, 1.5, 4),
  moderate (0.5, 2, 10 ms, 2, 4), severe (0.25, 4, 20 ms, 2.5, 4) — chosen to bracket a real
  D455 IR stream from good indoor light to low light, not fitted to data.

## Day 2, step 1 — sigma_l plumbing
- **No bug, Stage 3 not rerun.** Evidence: (a) `ros2 param get` on the node returns the passed
  value; (b) in the Stage 3 decisions the virtual gain per virtual measurement rises
  1.7 → 5.2 → 7.1 → 9.6 → 12.7 nats for sigma_l 0.01 → 3 m, ~4.6 nats per decade = the
  2·ln(10) slope of a 2-D measurement's log det above the crossover; (c) new unit test matches
  log det(I + σ² H R⁻¹ Hᵀ) exactly over σ = 1e-4…10 m. Why the curve looked flat: the gain is
  logarithmic in σ_l above the crossover σ_l ≈ σ_px·Z/f ≈ 9 mm, and all swept values were ≥ 10 mm.
  The test's first closed form ignored pose uncertainty (R = s² I); the planner was right, the
  test reference was wrong — corrected to R = s² I + H_x Σ_x H_xᵀ.
- **Side finding logged, not acted on:** median real-landmark gain per candidate is 0.00 nats
  (most candidates predict no visible real SLAM landmark).

## Day 2, step 2 — forest coverage metric
- **Cause: a reporting artifact, not the mapper.** The recorded `/octomap_mapper/coverage`
  stream is monotone in all 32 flights. `cl_report.py` skipped time points past the end of the
  stream; most forest flights stop early (planner runs out of reachable frontiers), so the
  aggregate's medians at 120/180 s were over a different (smaller) subset of flights than at
  60 s. Fix: carry the last value forward past the end of the stream (mapped volume persists),
  record where the stream ended, and check monotonicity (`coverage_monotone`,
  `coverage_max_drop_m3`). All stages recomputed from the bags (`stage*_results_v2.jsonl`).
- `ended_early_at_s` is where the recorded coverage stream ends relative to the start of
  exploration (≈ landing/teardown), an upper bound on when exploration actually ended.

## Day 2, step 3 — pose-marginal objective
- **dI_pose is measured against the CURRENT pose covariance** (log det Σ_pose(now) − log det
  Σ⁺_pose(candidate)), so the propagation cost of travelling is included. Alt: measure against
  the propagated prior (would reward far candidates for the uncertainty their travel creates).
- **coverage_gain = ray-cast unknown volume in the candidate frustum** through a coarse
  (0.25 m, 10 m radius) known-space cloud the mapper now publishes at 1 Hz. Alt: frontier
  voxel count (no occlusion, no depth); a mapper service (synchronous coupling).
- **lambda grid {0, 0.01, 0.1, 1, 10} nats/m³ chosen from ONE probe flight**
  (`runs/d2_probe_posecov`, excluded from evaluation): dI_pose ≈ −8…+6 nats, coverage ≈ 0…110 m³,
  so λ·coverage ≈ dI_pose near λ = 0.1. The probe itself failed (OpenVINS diverged smoothly at
  ~12 s, the gate passed it, PX4 followed and the vehicle fell in open space): the Stage 1c
  undetectable-drift failure, logged as such.
- **Step 4 runs on the image built for step 3** (before step 5's ESDF-band and speed-margin
  changes), so it stays comparable to the Stage 2 frontier / IG references. Step 5's changes are
  committed with this step's code but only deployed (rebuild) for the step 5 flights.

## Day 3 (2026-10-07 evening)
- **Stale shell**: none found — no flight/sim/chain process on host or in the container; the previous
  session's background shell ended with it. Nothing stopped.
- **Step 1 diagnosed with new flights, not old bags**: old decisions lack candidate yaw and the
  coarse map is not recorded. Added per-candidate logging (yaw, visibility classes, occlusion vs
  occupied coarse cells, rays through unknown) with default (old) scoring; 3 diagnostic flights,
  excluded from all evaluations. The predictor itself was NOT changed (no occlusion added): GT
  shows it is right; the gap is the candidate set.
- **Step 2 split logged in both modes**; old mode keeps its R_delta = I approximation unchanged
  (old behavior); the new path mode maps waypoint errors with the propagated attitude (correct).
- **Step 3: new set implemented for pose_cov only**; `ig` (virtual-landmark log-det) and
  `frontier` keep the old set. 8 yaw samples (45°, half the 87° HFOV). Coverage gain stays at the
  endpoint (not path-integrated; not asked). Synthetic propagation tabulated (state is static under
  the synthetic IMU, so exact up to the 0.02 s grid).
- **Step 4 used only the 15 day-2 λ flights + GT; features = SLAM landmarks in the joint
  covariance** (tracked/MSCKF counts were not recorded). Step-5 flights now also record
  /ov_msckf/points_msckf and points_slam so step 4 can be replicated with used-feature counts.
- **Executor home-phase ESDF-guard stall NOT fixed before step 5** (keeps comparability with day 2
  and the protocol); step 5 additionally reports explore-phase-only NEES, defined before running.
- **Step 5 frontier baseline re-flown** (5 flights) on the current image rather than reusing day-1.

## Day 4 (2026-10-08)
- **Step 1: GT-pose coverage uses EMULATED depth** (ray cast against the validation scene's GT
  geometry from GT camera poses): closed-loop bags carry no depth. Emulation validated on two new
  sensor-recording flights (d4_sens_lam0_{1,2}): emulated and recorded ranges agree to mm where
  both return; recorded depth inserted at OpenVINS poses reproduces the recorded coverage within 7%.
  Found: real depth has no-return pixels (2–17% per frame, grazing floor and wall edges) that the
  mapper inserts as free space to 8 m, carving free space through floor and walls — the recorded
  metric is 1.8–3.7x the GT-geometry coverage in EVERY flight. Flags therefore split: recorded vs
  GT (all flights) and OpenVINS-pose vs GT-pose with the same depth (divergence only). Mapper not
  changed (no rerun of past results; fix belongs to a future step).
- **Step 2b: replays need images; closed-loop bags have none** -> 2 new diagnostic flights at the worst
  condition (λ = 0, new set) with RECORD_SENSORS=1 (stereo IR, IMU, depth), replayed with the serial
  runner at track_frequency 20 (system: every other frame) and 40 (all frames). Replay config lacks the
  live harness's `init_wait_for_jerk: 0` (both rates alike). Diagonal-covariance NEES (serial runner
  has no joint-covariance dump), compared with the live flight's diagonal NEES.
- **Step 3 skipped**: it was conditional on fast yaw. Evidence (2a–2c) points to the opposite: NEES
  grows in straight translation with little rotation, roll/pitch (not yaw) blows up, full frame rate
  does not help. A yaw-rate limit would reduce rotation further. Not flown.
- **Step 4 design**: realized gain exists only for flown candidates, so instead of "a random unchosen
  candidate" the planner flies a uniformly random scored candidate on 30% of decisions
  (`random_pick`, diagnostic only, seeds 1–3), λ = 0.01, new set, 3 flights; argmax vs random
  picks compared on predicted − realized pose gain.
- **Ablation: 6 flights invalid** (`PATH_SPACING=0` reached ROS as an integer; the planner threw
  InvalidParameterTypeException at startup and the vehicle only hovered). Excluded and re-flown as
  `d4_abl_{none,yaw}_r{1,2,3}` with `0.0`. Configuration error, not a flight failure.
- **Step 4 realized gain = log det change of OpenVINS's own IMU pose covariance** decision -> arrival
  (agreement with the filter's belief, as in §61); goals not reached before the next decision skipped.
