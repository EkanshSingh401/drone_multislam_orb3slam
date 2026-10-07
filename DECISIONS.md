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
