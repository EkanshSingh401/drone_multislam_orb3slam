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

## Day 5 (2026-10-08 night; user's answers to the day-4 decisions)
- **Step 1, executor guard timeout = land in place** (not replan). `guard_timeout_s` (default 10 s): a
  continuous ESDF-guard hold in the home phase longer than this commands NAV_LAND where the vehicle
  is; the 120 s home timeout now also applies while the guard holds (before, it sat in the branch the
  guard short-circuited, which is why day-3 holds lasted 314–460 s). Why land, not replan: the home
  path is the flown breadcrumb trail (known free), so a guard trip there means the map near the trail
  changed or the vehicle drifted; there is no planner in the home phase to ask, and landing in place
  is the bounded, simple failsafe. Alt: skip the blocked crumb / step away along the ESDF gradient.
- **Check flights**: `d5_guard_forced` INVALID (`MAX_MISSION=40` reached ROS as an integer, executor
  threw at startup — same trap as day 4; renamed `_INVALID_intparam`). `d5_guard_forced2`
  (hold_distance 1.5 m): guard tripped twice in home but drift released it each time (timer resets;
  120 s home timeout is the backstop for oscillation). `d5_guard_forced3` (hold_distance 3.5 m, so
  the hold is continuous in home): held 10.0 s, landed in place, PX4 detector disarmed. These are
  diagnostic flights with a deliberately wrong hold distance; not evaluations.
- **Harness**: `cl_flight.sh` gained `EXECUTOR_ARGS` (extra executor parameters).
- **Step 2, mapper**: no-return depth pixels are SKIPPED (unknown), param `no_return_as_free`
  (default false; true = old behaviour). Valid returns beyond max_range (8 m) are still free up to
  8 m (octomap max-range truncation, as nvblox's max integration distance).
- **Step 2, GT-pose coverage definition updated to match**: `day5/gt_coverage.py` (from day 4) treats
  rays with no hit within the depth camera's 20 m far clip as unknown (the room has no ceiling: sky
  rays). Checked on d4_sens_lam0_1 (recorded depth images available): old rule reproduces day 4
  exactly (225.6 m³); new rule emulated 155.9 vs recorded-depth-at-GT-poses 157.3 m³ (1%). "Recorded vs
  GT" compares /octomap_mapper/coverage with the GT-pose map (unclipped); flight summaries now carry
  GT-pose coverage (envelope-clipped) and the curve metrics.
- **Step 3a metric definitions (a priori)**: reachable volume = scene envelope (inside outer walls,
  below wall tops) minus solids (`scenes.reachable_m3`: validation 373 m³, office 2247 m³); coverage
  fraction = GT-pose known volume clipped to the envelope / reachable; t50/t90 = first time ≥ 50/90%
  (None if never); AUC = mean fraction over [0, 180] s, last value carried forward. Computed on GT-pose
  coverage, not the recorded stream.
- **Step 3b office scene**: 30 x 20 m, 4 m walls, no ceiling, 3.5 m corridor + 6 rooms, 3 m doorways
  (planner path margin at cruise = 1.3 m), 15 furniture boxes; textures/lighting as validation. Geometry
  in `scenes.py` (one source for generator, coverage, clearance, watchdog box). Executor geofence for
  the office: `GEOFENCE=40.0` (default 8 m would end exploration in the first room).
- **Step 5 method**: ladder computed INSIDE the planner's model (new tool `gain_ladder`, same functions
  as `score_path`), so rungs differ only in inputs (planned vs flown path, landmark persistence,
  waypoint density, used features). Gate: rung `plan_real` must equal the logged dI_prop + dI_meas_real.
  MSCKF features enter by a per-feature Schur complement onto the observing clones (null-space
  equivalent), observing clones = sliding window (11 clones) before the publish time of
  /ov_msckf/points_msckf, FOV-tested. New SLAM landmarks: 10 m prior, observed from one window before
  first appearance until last appearance. Propagation in every rung: the planner's static model.
- **Step 5b realized split**: needs per-stage covariance; JC topic is post-only and throttled in flight.
  Used a diagnostic patch (`day5/ov_stage_logdet.patch`, env OV_STAGE_LOGDET, default off, prints pose
  log det after propagate / MSCKF / SLAM / init / marginalization) in a SCRATCH build of ov_msckf
  (`/out/cl/day5/ovws`), run only in offline serial replays. The pinned fork is unchanged.
- **Step 3b office check flights**: d5_office_frontier_2 failed to arm (PX4 arming flake; counted as a
  failure, no data). Two office frontier flights appended (_4, _5) so the saturation check rests on ≥ 3
  flights; this is a check of the scene, not a planner comparison, so adding flights is not "re-flying
  to replace a failure" in a comparison.
- **Step 4a**: OpenVINS's loaded parameters read from its startup print (= config). Empirical check of
  what the sim generates = a 15-min static recording (Allan deviation vs a synthetic reference with
  the same stamps and gz-sensors' recursion at the configured densities). The in-flight IMU-vs-GT
  residual tool (s54) was run but is dominated by GT stamp artifacts (as §54 found) -> not used as
  evidence.
- **Step 4b scripted flights**: new `script_goals.py` (TYPE=script) replaces the planner with a moving
  goal: hover 20 s, square of straight legs at 0.3 m/s with heading held, hover, in-place rotation at
  0.05 / 0.15 / 0.4 rad/s, hovers between. Offline serial replay (system setting, every other frame) +
  JC dump -> full-covariance roll/pitch/yaw NEES per sample, binned by GT angular rate and by segment.
  "After fix" replays only if 4a finds a mismatch.
- **Step 4c**: the §54 swap test (GT-synthesized IMU) is inconclusive on these flights: the GT-spline IMU
  differs from the real accelerometer by MAD 0.04–0.10 m/s² (> the 0.028 white noise), so it carries
  its own error. Used instead: OpenVINS's own simulator (`run_simulation`) driven by the flights' GT
  trajectories (20 Hz, 0.25 s smoothing), with our estimator config, rig calibration and IMU noise;
  ideal features 2–6 m, camera 15 Hz, IMU 200 Hz. This isolates the filter (FEJ, bias/gravity
  coupling, init) from our sim's sensor generation.
- **CPU discipline**: d5_lowrot_1 lost track in flight (GT drifted 0.7 m while OpenVINS held still,
  watchdog cut the motors at 1.6 m) while I was running heavy offline jobs in the same container.
  OpenVINS showed no processing lag (3 ms behind), so starvation is not indicated, but from here on no
  heavy offline jobs run during flights. The flight is kept and reported (failures count).
- **Step 4a static test dropped as evidence**: the static recording is dominated by ground-contact
  jitter (and had a duplicate /camera/imu publisher from an orphaned imu_restamp left by the flight
  I killed to pause the batch). The in-flight long-span test (3 flights) is the 4a evidence. Pausing a
  batch by killing it mid-bringup left an orphan: next time pause between flights only.
- **Step 4c diagnostic replays at track_frequency 5** are diagnostics, not a fix; nothing adopted. They
  are not noise inflation either, but changing the system rate is the user's call (needs real-D455
  data first).
- **Step 6 skipped by its condition**: step 4 identified the cause (correlated vision errors at the
  system frame rate) but adopted no fix, so orientation NEES is not back to frontier-like ~4 for the
  pose-objective planners; comparing planners in the office on an estimator known to be overconfident
  in exactly the slow-motion regime those planners induce would not be interpretable.
- **Step 4c extra tests (after the main ones)**: mild image degradation (overnight module), half-pixel
  principal point (OV_CXCY), descriptor tracker (use_klt false) — all offline replays of the scripted
  flights, diagnostics only. Chosen to separate "noise-free renders" from "systematic correlated
  projection error"; results in the report.
- **Step 5 replication flights** used yaw_samples 1 (user: yaw sampling off by default), path 0.5, λ 0.01.

## Day 6 (2026-10-08 → 09; user's answers to the day-5 items)
- **Step 1, executor default for all planners (planners never command yaw rates; the executor does):**
  yaw keeps changing toward the goal yaw while translating (unchanged); when in place (< 5 cm from the
  position goal), a remaining yaw change < `turn_deadband` (0.3 rad) is not flown and a larger one is
  flown as one turn at the full `yaw_rate` 0.6 rad/s (frontier's turn rate) — no slow in-place yaw can
  be produced by any goal stream. The home phase's turn-then-move already turns at 0.6 rad/s.
- **Step 5, arming failure cause**: the executor sent ARM only on ticks with `int(tp*20) % 20 == 1`, a
  20 Hz wall-clock timer tested against sim time; at RTF ~1 (the lighter office scene) the index can
  skip 1 every second, so no ARM was ever sent (PX4 log: "Ready for takeoff!", no arm, no denial).
  Fix: ARM every 1 s by elapsed time + log PX4's command acks.
- **Step 5, landing**: two causes found for PX4's land detector never firing: (i) with vision as the
  height reference, PX4's height dead-reckons once vision is withheld below 0.4 m; (ii) the hold was
  keyed on OpenVINS height, which diverges upward on the ground and released the hold. Changes:
  EKF2_HGT_REF 0 (baro) by default in cl_flight (`PX4_HGT_REF` env; 3 = old), executor corrects its
  height setpoint by the learned PX4-minus-OpenVINS height offset (EMA, frozen near ground/landing),
  and the landing hold is latched. First attempt with baro alone (no offset) climbed short (PX4 baro
  1.5 m vs OpenVINS 1.25 m): 3 flights moved to `runs_invalid/` (configuration error).
- **Disk**: /out hit the batch's 50 GB floor; deleted my own regenerable day-5 derived bags (degraded,
  synthetic-IMU, replay recordings; 78 GB). Results computed from them are kept.
- **EuRoC**: direct DSpace bitstream URLs found (`euroc/fetch.sh`), but the ETH Research Collection
  rate-limits content downloads (HTTP 429, "access temporarily restricted"). Background loop probes
  once per 20 min. Kaggle mirror needs an account (none on this host).
- **Step 3 feature-poor variant `office_plain`**: same geometry; NE room walls, the x = 16 partitions,
  two north corridor segments and NE furniture near-uniform grey (±1 grey level); the south-centre
  room dim (emissive 0.15 vs 0.55).
- **Landing logic refined between batches** (each flight's version recorded in the report's table):
  baro without offset (invalid, 3) → + height offset → + latched hold → + PX4 ground_contact → + PX4
  local height within 0.15 m of takeoff and |vz| < 0.15 for 0.5 s. Later versions were deployed through
  the executor override file between flights (never during one).
- **EuRoC**: OpenVINS EuRoC config + our tracking/feature/update keys; calibration options stay EuRoC's
  (online calibration is dataset-appropriate for real data). track_frequency 21/10.5/5.25 to get exactly
  every 1st/2nd/4th frame at 20 Hz. NEES with a constant GT-body-to-IMU rotation removed (fitted after
  the world-yaw alignment). joint_cov_dump stamps have 12 significant digits (10 ms at epoch time):
  matched to estimates within 6 ms.
- **EuRoC verdict inconclusive → system rate unchanged, no sim-only workaround, render investigation not
  started**: the user's rule had two branches (real consistent → sim artifact; real same dependence →
  real); EuRoC's GT orientation error (0.3–1.0°) exceeds the filter σ (0.04–0.06°) 6–25x, so neither
  can be established. Adopting a sim-only workaround needs evidence it is sim-only.
- **Step 3 plain variant moved** before any plain-office flight (frontier never reaches x > 4 m).
- **Step 4 window**: 20 s, step 10 s, ≥ 1 m path; drift as relative pose error (no alignment); GT
  interpolated at OpenVINS state times.

## Day 7 (2026-10-10; user: is the cov-vs-drift mismatch a sim image artifact or real?)
- **Disk budget up front**: sensor flights were 20–25 GB (float32 depth dominates). Recorded WITHOUT depth
  (`RECORD_DEPTH=0`; the mapper still uses it live; steps 2–3 take depth from GT geometry): 5.6–8 GB each.
  Freed 157 GB of regenerable copies: uncompressed `.mcap` files that rosbag2 leaves beside file-compressed
  (`.zstd`) bags when they are read — deleted only where metadata.yaml references the `.zstd` original.
- **Step 1/2 instrumentation** in the scratch OpenVINS build (`/out/cl/day5/ovws`, patch in
  `docker/tools/closed_loop/day7/ov_nis.patch`): `OV_NIS_LOG` prints χ²/dof per MSCKF and SLAM feature update
  (before gating) and SLAM features' per-axis normalized residuals r/√S_ii (cam0); `OV_TRK_LOG` prints cam0
  tracked feature ids and pixels per frame. Default off. Compiled in a gap between flights.
- **Whiteness definition**: lag-1 correlation of each SLAM feature's normalized residual across its
  consecutive updates, pooled. MSCKF features are used once, so only their NIS is reported.
- **KLT truth**: 3D point from the first observation's ray cast into scenes.py geometry from the GT camera
  pose; later true pixels by projection. Measures what tracking adds after detection.
- **Sensor flight d7_sens_tex_fr_2 failed** (climb timeout at OpenVINS z 0.46 m; landed safely): PX4's baro
  height was ~1 m off at takeoff and the executor only learns the height offset once OpenVINS's base height
  exceeds 0.4 m. Counted, not re-flown (9 flights remain). Fix for later: learn while either height says
  airborne.
- **Replays use `init_wait_for_jerk: 0`** like the live harness (new OV_APPEND hook): without it the 5 Hz
  replay initialized only at the end of a flight. First replay batch discarded (configuration), rerun.
- **Rates**: sim 15 Hz = track_frequency 20 at 30 fps (system setting) vs 5 Hz; EuRoC 21/10.5/5.25.
- **Step 1 verdict = real**: EuRoC innovations are as time-correlated at 20 Hz (lag-1 0.76) as the sim's at
  15 Hz (0.69); rule "same pattern in both → real". Runs with < 1000 residual pairs (failed runs) excluded.
- **Step 5 branch = stop**: Spearman(σ growth, drift) at the consistent rate (5 Hz, NEES rp 3.2) is 0.03
  for position and 0.47 [0.23, 0.60] for yaw; bar ≥ 0.5 not met → nothing built, no rate change.

## Day 8 (2026-10-11; user direction: keep the information framework, replace the white-noise model)
- **Step 0**: height-offset learning starts when either PX4 (above its takeoff height) or OpenVINS (base) is
  above 0.4 m. Check flight d8_check_zoff: normal climb (offset 0.14 m learned during the climb), explore,
  executor disarm.
- **Step 1 ceiling**: per window σ_rel² = max(σ(t1)² − σ(t0)², 0) from the filter's marginal sigmas (the
  t0–t1 cross-covariance is not logged: random-walk approximation); 500 simulated draws per dataset.
- **Step 2 training data**: validation-scene sim flights only (7 sensor flights × 15 and 5 Hz replays).
  **EuRoC not used for fitting**: pixel truth needs camera poses to ~0.05°, EuRoC's GT orientation error is
  0.3–1° (day 6), i.e. 2–8 px — larger than the KLT error to be modelled. EuRoC remains the real-data
  evidence that the time correlation exists (day 7). Office and plain office fully held out.
- **Step 3**: random-walk information implemented as a planner option (`meas_model`, default `white`).
  The first predicted sample of an already-tracked landmark is treated as a fresh track start
  (approximation, documented in MATH_TO_CODE.md of active_slam_information, new pin e12c376).
- **Step 4 predictor** (`drift_model`, new tool): planner's linear model, IMU prior from the flight's JC at
  t0, static propagation, clones at 5 Hz, every tracked feature (TRK) as white 1 px or random-walk track
  (q² from the fitted model), landmark points from the GT ray cast (oracle geometry, moved into the
  estimate's frame) — so the comparison isolates the measurement model, not landmark estimation.
- **Per-metre metric retired as headline**: 1/path alone correlates 0.63–0.68 with position drift per metre;
  raw per-window drift has no shared normalization (raw path vs raw drift ≈ 0). Both reported, raw headline.
- **Step 2 targets/statistics robust**: per-track rate = median_k |e_k|²/(1.386 k) (mean squared increments
  were dominated by gross tracks); shape check = median over tracks, not RMS.

## Day 9 (2026-10-12; position primary, planner test decides)
- **Step 1 stereo**: scratch OpenVINS track log extended to cam1 (`[TRK1]`, same feature ids for stereo
  matches); all 32 sensor replays (held-out 18 + validation 14) rerun into `/out/cl/day9/rep`.
  drift_model: one random-walk track per camera per landmark (independent errors per camera), shared
  landmark columns, one Schur step. cam1 tracks use the cam0 track's predicted rate.
- **Step 2 training scenes** `texlevels_a`, `texlevels_b`: the VALIDATION geometry (not the office layouts)
  with each surface's texture contrast scaled to 1 / 0.3 / 0.1 / 0.03 / 0 (0 = ±1 grey-level noise, like
  office_plain's plain walls), two permutations so a level is not tied to a surface. Training only.
