# Morning report — overnight run 2026-10-07 (02:00–10:00 EDT)

Everything is committed and pushed on `phase3/openvins-sim`. Decisions: `DECISIONS.md`.
Raw per-flight results: `docs/overnight/stage{2,3,4}_results.jsonl`; tables:
`docs/overnight/aggregate.md`; details: `docker/PATCHES.md` §65–§66.
All numbers below are median [min–max] over flights; no claim rests on one flight.

## Stage 0 — housekeeping: DONE
`4e923e4` and the open_vins fork commit pushed; `active_slam_sim` built into the image
(own Dockerfile step with executable checks); fork pin updated. Confirmation flight from
the image: 180 s exploration, landed, disarmed, 0.80 m min clearance.

## Stage 1 — diagnostics: DONE (1b partial)
- **1a** IG objective split per candidate into gain from real SLAM landmarks vs virtual
  frontier landmarks (`delta_real`, `delta_virt`); per-flight shares below (Stage 2/3).
  **The IG objective is dominated by virtual landmarks**: chosen candidate's real share
  ≈ 4% at σ_l = 1.0 (validation), ≈ 0% in the forest.
- **1b** Why PX4's land detector never fires: in vision-only EKF2 its vertical-velocity
  criterion (|vz| < 0.25 m/s for LNDMC_TRIG_TIME) is fed by OpenVINS, which diverges
  within ~1 s of ground contact. Added: vision HOLD below 0.4 m, LNDMC_TRIG_TIME 0.3,
  PX4-primary / OpenVINS-backup disarm. **Partial**: PX4 confirmed landing in only 2 of the 32
  batch flights; the executor's disarm was missed in 17 of 32 and the sim-only watchdog
  disarmed on the ground (see "post-landing" below).
- **1c** Injected VIO faults (new injector). Without protection PX4 **follows** a 2 m jump
  (EKF resets to the only aiding source) and a 0.5 m/s drift (2.8 m displacement); a 5 s
  dropout fails safe (dead-reckon → position invalid → descent). New **VIO health gate**
  turns jumps into dropouts: with it the jump ends in a failsafe descent and landing in
  place. Slow self-consistent drift is undetectable from vision alone.
- **1d** Full-covariance NEES added to the closed-loop evaluation (`cl_nees.py`).

## Stage 2 — protocol + comparison, validation: DONE
Protocol written in HANDOFF before running. 5 flights per planner, 180 s budget.

| | frontier | IG (σ_l 1.0) |
|---|---|---|
| known volume @60 s (m³) | 1120 [1090–1190] | 1220 [1170–1250] |
| known volume @120 s (m³) | 1250 [1140–1300] | 1660 [1530–1830] |
| known volume @180 s (m³) | 1350 [1310–1450] | 2000 [1960–2070] |
| path (m) | 31.6 [30.9–42.9] | 99.5 [45–123] |
| known m³ per m flown | 44 [32–48] | 21 [18–44] |
| OpenVINS ATE (m) | 0.019 [0.010–0.025] | 0.034 [0.018–0.062] |
| ATE per m flown | 0.00058 [0.00032–0.00064] | 0.00040 [0.00027–0.00059] |
| NEES full ori / rp | 3.8 [3.4–4.8] / 8.3 [4.3–11.5] | 9.1 [5.7–87] / 20 [6.7–316] |
| min GT clearance (m) | 0.78 [0.76–0.85] | 0.70 [0.00–0.74] |
| in-flight failures | 0 | **1** (s2_ig_2: hit box_se at 1.19 m, watchdog terminated) |
| post-landing watchdog disarms | 3 | 2 |
| IG real-landmark gain share | — | 0.041 [0.031–0.046] |

IG explores ~50% more volume in the budget, but flies 3x farther, with roughly 2x the ATE
and higher, more variable orientation NEES; it had the only collision.

## Stage 3 — σ_l sweep (report only): DONE
3 flights each (σ_l = 1.0 reuses 3 Stage 2 flights). Plot: `docs/overnight/stage3_tradeoff.png`.

| σ_l (m) | known @180 s (m³) | path (m) | ATE (m) | NEES full ori | real share | contacts |
|---|---|---|---|---|---|---|
| frontier (ref) | 1350 [1310–1450] | 32 | 0.019 | 3.8 | — | 0 |
| 0.01 | 1920 [1700–1930] | 74 | 0.034 | 5.4 [3.6–10.7] | 0.094 | 0 |
| 0.1 | 1720 [1630–1740] | 66 | 0.027 | 5.8 [3.9–460] | 0.079 | 0 (min 0.34 m) |
| 0.3 | 1900 [1740–2030] | 67 | 0.037 | 5.4 [3.9–10.8] | 0.053 | 1 |
| 1.0 | 2000 [1960–2070] | 106 | 0.034 | 9.1 [5.7–87] | 0.041 | 1 (s2_ig_2) |
| 3.0 | 1970 [1790–2000] | 58 | 0.027 | 17.9 [5.3–19.3] | 0.031 | 0 (min 0.37 m) |

The real share falls monotonically with σ_l, but **the sweep never reached "virtual
negligible"** (9% real at 0.01 m). Coverage and ATE are flat within the spread across σ_l;
all IG settings sit at higher coverage and higher ATE/NEES than frontier. Orientation NEES
rises at the top of the sweep (σ_l = 3: 17.9). No "best" value picked.

## Stage 4 — forest: DONE, with caveats
5 flights per planner. **Forest results need care**:
- frontier coverage *decreases* over time (1880 → 1120 m³), which a known-volume metric
  should not do — suspect a mapper/coverage-measurement problem in the large forest scene;
- IG flights ended before 180 s (no @180 value); both planners stop after few goals
  (frontier 17, IG 8);
- no GT clearance check (tree geometry not modelled); no in-flight watchdog terminations.

| | frontier | IG |
|---|---|---|
| path (m) | 24 [4.9–27] | 47 [23–72] |
| ATE (m) | 0.047 [0.033–0.066] | 0.029 [0.026–0.056] |
| NEES full ori | 5.8 [3.3–6.5] | 7.9 [6.6–13.6] |
| IG real share | — | 0 [0–0.08] |
| in-flight failures | 0 | 0 |

## Stage 5 — image degradation: DONE (1 of 6 replays failed)
Degradation module (`active_slam_sim/active_slam_sim_py/degrade.py`: low light + gain,
shot + read noise, rotational motion blur from the gyro over the exposure) and offline bag
tool (`degrade_bag.py`); clean rig stays the default; not yet wired live (DECISIONS.md).
Preview: `docs/overnight/degrade_preview.png`. 6 datasets = validation path C and forest
path A × mild / moderate / severe; OpenVINS replayed (estimator config unchanged) with the
s62 instrumentation; **validation-moderate replay failed** (empty log, not investigated).

Usage of predicted-visible (frame, landmark, camera), by bin:

| scene / level | usage | records | by angular speed [0,.05,.2,.5,1,3] rad/s | by depth [0,2,4,8,12,30] m | by border [0,20,50,100,500] px |
|---|---|---|---|---|---|
| validation C clean | 0.935 | 70.8k | .93 .94 .94 .94 .92 | .88 .90 .94 .99 – | .73 .83 .97 .94 |
| validation C mild | 0.934 | 70.6k | .92 .95 .95 .95 .92 | .88 .95 .94 .88 – | .88 .94 .94 .93 |
| validation C severe | 0.901 | 64.9k | .90 .91 .91 .90 .87 | .79 .88 .91 .94 – | .74 .83 .89 .91 |
| forest A clean | 0.929 | 22.6k | .92 .94 .95 .90 .93 | – .89 .94 .94 .87 | .93 .94 .93 .92 |
| forest A mild | 0.906 | 19.8k | .91 .88 .90 .94 .88 | – .96 .93 .92 .83 | .81 .97 .91 .89 |
| forest A moderate | 0.861 | 15.8k | .86 .87 .87 – .76 | .66 .78 .88 .83 .85 | .85 .94 .86 .84 |
| forest A severe | 0.852 | 10.4k | .85 .88 .87 – – | .42 .86 .86 .89 .84 | .76 .85 .82 .86 |

**Answer: partly.** Degradation lowers usage (forest 0.93 → 0.85) and, more strongly, shrinks
the map (forest records −54% at severe: fewer SLAM landmarks). A geometry/motion dependence
starts to appear at moderate/severe in the forest — near features (< 2 m: 0.42–0.66) and fast
rotation (> 1 rad/s: 0.76) — but most bins stay within a few points of the level's mean, and
each condition is a single flight, so this is a direction, not a result. A feature-quality
predictor now has signal to learn from in degraded sim; it needs more flights per level and
the live rig wiring.

## What needs you
1. **Collisions with the IG planner** (validation: 2 contacts in 17 IG flights across Stages
   2–3 — s2_ig_2 and one σ_l = 0.3 flight — and 2 more IG flights within 0.04 m of the
   0.33 m contact line; 0 in 5 frontier flights). The IG planner flies longer, closer paths; the ESDF
   guard/margins were sufficient for frontier but not IG. Decide margins vs planner changes
   before any further comparison.
2. **Post-landing disarm still unreliable** (watchdog had to disarm on the ground in 17 of
   32 flights). The sim-only watchdog hides it; on hardware this is a flyaway risk. Needs a
   proper fix (ZUPT or a VIO freeze keyed on commanded landing, or PX4 detector work).
3. **The IG objective is ~95% virtual-landmark gain**: the planner is effectively a
   frontier-uncertainty planner. Decide whether to extend the σ_l sweep downward / change
   virtual landmark count before interpreting IG vs frontier as "information-driven".
4. **Forest coverage metric looks broken** (decreasing known volume): verify before using
   Stage 4 numbers.
5. Stage 5 needs more flights per degradation level (one each now) and the live wiring;
   the validation-moderate replay failure is unexplained.
6. Real drone requirements from Stage 1c (VIO health monitor, second position source,
   failsafe timing, touchdown handling) — see PATCHES §65.
