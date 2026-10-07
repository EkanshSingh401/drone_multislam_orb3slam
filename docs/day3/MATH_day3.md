# Day 3 — candidate set and pose-term split (math → code)

Code: `active_slam_sim/include/active_slam_sim/candidate_scoring.hpp`, planner
`active_slam_sim/src/exploration_planner.cpp` (`score_path`), tests
`active_slam_sim/test/test_candidate_scoring.cpp` (run in the image build).
Extends active_slam_information MATH_TO_CODE.md "pose-marginal objective" (day 2).

## Split of the pose term (step 2)

With Σ_pose(now) the current 6-DoF IMU pose covariance, Σ⁻ the pose covariance after
propagating over the travel time with no measurements, and Σ⁺ after the predicted
measurements:

    dI_pose = log det Σ_pose(now) − log det Σ⁺
            = [log det Σ⁻ − log det Σ⁺]  +  [log det Σ_pose(now) − log det Σ⁻]
            =        (a) dI_meas        +        (b) dI_prop

(a) ≥ 0 is what the views buy; (b) ≤ 0 is the travel/propagation cost. `dI_meas_real` is (a)
with the real SLAM landmarks only. Both modes log a, b, a_real per candidate.

## Yaw samples (step 3)

`yaw_samples = N`: each candidate position is scored at yaw_k = facing + 2πk/N, k = 0..N−1
(facing = towards the frontier tile centroid; k = 0 is the old candidate). N = 1 is the old set.

## Path integration (step 3)

The executor flies a straight line at `cruise_speed` v while turning toward the goal yaw at
most `yaw_rate` ω (both from t = 0). Waypoints at s = Δ, 2Δ, … < L − Δ/2 (Δ = `path_spacing`)
at t = s / v with yaw ψ(t) = ψ₀ + sign(Δψ) min(|Δψ|, ω t), plus the endpoint at
T = max(L / v, |Δψ| / ω). Δ = 0: endpoint only (old).

State: x = [x_cal | x_IMU (15) | clones | landmarks]. Propagation over a segment only acts on
the IMU block: S ← Φ S Φᵀ on the IMU rows/cols, S_II += Q. At waypoint k the IMU pose
(orientation, position) is stochastically cloned, c_k. A measurement of landmark f from
waypoint k: z = h(c_k, f) + v, linearized with H_c (via `predict_measurement`, current =
propagated attitude, candidate = waypoint pose, so the error-frame rotation R_δ is applied) and
H_f (landmark representation Jacobian, or identity for a virtual landmark with prior σ_l² I).

Because every step is linear at the nominal path and later propagation does not touch earlier
clones, propagate→clone→update in time order equals: build the joint prior with all clones
first, then one joint update with all measurements. Joint update in information form,
valid for singular S:

    J = Σ_k H_kᵀ V_k⁻¹ H_k
    S⁺ = S − S Hᵀ (H S Hᵀ + V)⁻¹ H S = (I + S J)⁻¹ S        (push-through identity)

Scored pose = endpoint clone (identical to the propagated IMU pose there, without duplicating it
in the scored block). Cost is O(n³) in the reduced state size, independent of the number of
measurements (hundreds along a path).

Reduction: only the IMU block (for propagation), the real-landmark columns the measurements
touch (landmark + anchor-clone columns of the representation Jacobian), the virtual landmarks
and the new clones are kept — the marginal of a Gaussian, exact.

Propagation tables: the synthetic IMU (level, unaccelerated, zero gyro) leaves the nominal state
unchanged, so OpenVINS's one-step F, Q_d (dt = 0.02 s) are the same every step; Φ_k = F^k and
Q_k = F Q_{k−1} Fᵀ + Q_d are tabulated once per decision.

## Tests (brute force)

| test | reference | result |
|---|---|---|
| 4-waypoint path posterior | sequential dense EKF (propagate, clone, Kalman update per waypoint) | equal to 1e-12 |
| same | library covariance-form `evaluate_reference` on the joint prior | equal to 1e-8 |
| reduction 69 → 36 states | full state | exact |
| single waypoint | old endpoint score (`InformationPrior` on the IMU block) | equal to 1e-12 |
| split | a + b = dI_pose, a ≥ 0, b ≤ 0 | holds |
| executor profile | hand values (9 waypoints + endpoint, turn-in-place T = Δψ/ω) | exact |
| visibility classifier | `predict_measurement` accept/reject, 4000 random points | 4000/4000 |
| segment occlusion | occupied cells occlude; unknown does not; landmark's own cell does not | pass |
