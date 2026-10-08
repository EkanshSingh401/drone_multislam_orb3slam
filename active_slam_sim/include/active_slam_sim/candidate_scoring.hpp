#pragma once
// candidate_scoring.hpp (day 3) -- pieces of the planner's candidate scoring that are
// testable without ROS:
//   classify_visibility   why a (landmark, camera) pair is or is not predicted visible
//                         (step 1 diagnosis; same tests as predict_measurement, in order)
//   executor_profile      the executor's straight-line, rate-limited-yaw motion sampled
//                         at ~0.5 m waypoints (step 3 path integration)
//   build_path_prior      propagate the joint covariance along the waypoint times and
//                         stochastic-clone the IMU pose at each waypoint
//   marginal              restrict a covariance to an index set (exact for a Gaussian)
//   pose_gain_split       dI_pose = (a) measurement gain + (b) propagation term (step 2),
//                         using the information form so that hundreds of path
//                         measurements cost O(n^3) in the reduced state size, not in m
//
// Path integration (MATH_TO_CODE.md, day 3): with clones c_1..c_K at the waypoints and
// measurements z_k = H_k x_{c_k} + v_k, the posterior after propagate-clone-update in time
// order equals the joint update of the augmented prior with all measurements, because the
// propagation only acts on the IMU block, the clones carry the correlations, and every
// step is linear (Jacobians at the nominal path). Tested against the sequential dense EKF.
//
// Joint update in information form (valid for singular Sigma):
//   J = sum_k H_k^T V_k^-1 H_k,   Sigma+ = (I + Sigma J)^-1 Sigma
// (push-through identity on Sigma - Sigma H^T (H Sigma H^T + V)^-1 H Sigma).

#include <cmath>
#include <functional>
#include <vector>

#include <Eigen/Dense>

#include "active_slam_information/information_gain.hpp"
#include "active_slam_information/view_geometry.hpp"
#include "active_slam_sim/coverage_gain.hpp"

namespace active_slam_sim {

namespace asi = active_slam_information;

enum class Vis { VISIBLE = 0, FOV = 1, RANGE = 2, OCCLUDED = 3 };

// FOV: behind the camera or projecting outside the image (minus border).
// RANGE: inside the image but nearer than min_depth or farther than max_depth.
// OCCLUDED: passes both, and an OCCUPIED coarse cell lies on the ray (unknown never
// occludes; *n_unknown counts unknown cells crossed). map == nullptr -> no occlusion test
// (as the deployed predictor). VISIBLE pairs are exactly those predict_measurement accepts
// with occluded == nullptr.
inline Vis classify_visibility(const asi::Pose& cand, const asi::CameraModel& cam, const Eigen::Vector3d& p_FinG,
                               const CoarseMap* map = nullptr, int* n_unknown = nullptr) {
  const Eigen::Vector3d p_C = cam.R_ItoC * (cand.R_GtoI * (p_FinG - cand.p_IinG)) + cam.p_IinC;
  const double Z = p_C.z();
  if (!(Z > 0)) return Vis::FOV;
  const double u = cam.fx * p_C.x() / Z + cam.cx, v = cam.fy * p_C.y() / Z + cam.cy;
  if (u < cam.border_px || u > cam.width - cam.border_px || v < cam.border_px || v > cam.height - cam.border_px) return Vis::FOV;
  if (!(Z > cam.min_depth) || Z > cam.max_depth) return Vis::RANGE;
  if (map && segment_occluded(*map, asi::camera_center_in_global(cand, cam), p_FinG, n_unknown)) return Vis::OCCLUDED;
  return Vis::VISIBLE;
}

struct Waypoint {
  double t = 0;    // time from now [s]
  asi::Pose pose;  // IMU pose (level, at flight height)
  double yaw = 0;
};

// The executor's explore motion (offboard_executor.py step_toward): position moves along
// the straight line at `speed`, yaw turns toward the goal yaw at most `yaw_rate`, both from
// t = 0. Waypoints every `spacing` metres of travel (s = spacing, 2 spacing, ... < L) and
// the endpoint, reached at T = max(L / speed, |dyaw| / yaw_rate). spacing <= 0: endpoint only.
inline std::vector<Waypoint> executor_profile(const Eigen::Vector2d& from, double yaw0, const Eigen::Vector2d& to, double yaw1,
                                              double height, double speed, double yaw_rate, double spacing) {
  const double L = (to - from).norm();
  const double dyaw = std::remainder(yaw1 - yaw0, 2 * M_PI);
  auto at = [&](double t) {
    Waypoint w;
    w.t = t;
    const double s = std::min(L, speed * t);
    const Eigen::Vector2d xy = L > 1e-9 ? Eigen::Vector2d(from + (to - from) * (s / L)) : to;
    w.yaw = yaw0 + std::copysign(std::min(std::fabs(dyaw), yaw_rate * t), dyaw);
    w.pose.R_GtoI = Eigen::AngleAxisd(w.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose();
    w.pose.p_IinG = Eigen::Vector3d(xy.x(), xy.y(), height);
    return w;
  };
  std::vector<Waypoint> out;
  if (spacing > 0)
    for (double s = spacing; s < L - 0.5 * spacing; s += spacing) out.push_back(at(s / speed));
  out.push_back(at(std::max(L / speed, std::fabs(dyaw) / yaw_rate)));
  return out;
}

// Propagate S (IMU block at imu_col) from t0 to t1, in place.
using PropagateFn = std::function<void(Eigen::MatrixXd& S, int imu_col, double t0, double t1)>;

struct PathPrior {
  Eigen::MatrixXd S;            // n0 + 6K: original state (IMU propagated to the end) + K clones
  std::vector<int> clone_cols;  // column of clone k (orientation 3, position 3), k = waypoint index
};

// Propagate along the waypoint times and clone the IMU pose (first 6 IMU columns:
// orientation, position) at each waypoint.
inline PathPrior build_path_prior(const Eigen::MatrixXd& S0, int imu_col, const std::vector<double>& times, const PropagateFn& prop) {
  PathPrior P;
  P.S = S0;
  double t = 0;
  for (double tk : times) {
    if (tk > t) prop(P.S, imu_col, t, tk);
    t = std::max(t, tk);
    const int n = (int)P.S.rows();
    Eigen::MatrixXd S2 = Eigen::MatrixXd::Zero(n + 6, n + 6);
    S2.topLeftCorner(n, n) = P.S;
    S2.block(n, 0, 6, n) = P.S.middleRows(imu_col, 6);
    S2.block(0, n, n, 6) = P.S.middleCols(imu_col, 6);
    S2.block(n, n, 6, 6) = P.S.block(imu_col, imu_col, 6, 6);
    P.S = 0.5 * (S2 + S2.transpose());
    P.clone_cols.push_back(n);
  }
  return P;
}

// Restrict to the listed indices (sorted, unique) -- the marginal of a Gaussian. Returns the
// sub-covariance and, through *map, old column -> new column (-1 if dropped).
inline Eigen::MatrixXd marginal(const Eigen::MatrixXd& S, const std::vector<int>& idx, std::vector<int>* map = nullptr) {
  const int m = (int)idx.size();
  Eigen::MatrixXd R(m, m);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < m; ++j) R(i, j) = S(idx[i], idx[j]);
  if (map) {
    map->assign(S.rows(), -1);
    for (int i = 0; i < m; ++i) (*map)[idx[i]] = i;
  }
  return R;
}

// Columns a set of measurements touches (each block spans J.cols() columns).
inline void touched_columns(const std::vector<asi::PredictedMeasurement>& meas, std::vector<char>& mark) {
  for (const auto& pm : meas)
    for (const auto& b : pm.blocks)
      for (int c = 0; c < b.J.cols(); ++c) mark[b.col + c] = 1;
}

// Re-index measurement blocks through an old->new column map (all touched columns kept).
inline std::vector<asi::PredictedMeasurement> reindex(const std::vector<asi::PredictedMeasurement>& meas, const std::vector<int>& map) {
  std::vector<asi::PredictedMeasurement> out = meas;
  for (auto& pm : out)
    for (auto& b : pm.blocks) b.col = map[b.col];  // contiguous blocks stay contiguous: kept columns are sorted
  return out;
}

inline double logdet_spd(const Eigen::MatrixXd& A, bool* ok = nullptr) {
  Eigen::LLT<Eigen::MatrixXd> l(0.5 * (A + A.transpose()));
  if (ok) *ok = l.info() == Eigen::Success;
  if (l.info() != Eigen::Success) return NAN;
  return 2.0 * l.matrixL().toDenseMatrix().diagonal().array().log().sum();
}

// Information of the measurements, J = H^T V^-1 H (n x n), accumulated block by block
// (each measurement touches a few blocks; a dense 2 x n row would cost m n^2).
inline Eigen::MatrixXd information(const std::vector<asi::PredictedMeasurement>& meas, int n) {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, n);
  for (const auto& pm : meas) {
    const double w = 1.0 / (pm.sigma * pm.sigma);
    for (const auto& bi : pm.blocks)
      for (const auto& bj : pm.blocks)
        J.block(bi.col, bj.col, bi.J.cols(), bj.J.cols()).noalias() += w * bi.J.transpose() * bj.J;
  }
  return J;
}

// Day 8 (MATH_TO_CODE.md "Random-walk tracking error"): information of ONE feature track whose
// measurement errors are a per-track random walk, e_0 ~ N(0, s0^2 I), e_k = e_(k-1) + d_k,
// d_k ~ N(0, q2 * frames_k I) (frames_k = camera frames between samples k-1 and k).
// Differencing (z_0, z_1 - z_0, ...) is invertible and whitens the noise exactly, so
//   J = H_0^T H_0 / s0^2 + sum_k (H_k - H_(k-1))^T (H_k - H_(k-1)) / (q2 frames_k).
// `track` holds the samples in time order (each a PredictedMeasurement, blocks at state columns,
// pm.sigma ignored). With the white model the same track gives sum_k H_k^T H_k / sigma^2.
// UNITS (day 9 fix): the Jacobians of asi::predict_measurement are in normalized image coordinates; s0 (px) and
// q2 (px^2) are scaled by px_to_meas = 1 / f (resp. its square). Day 8 called this without the scale.
inline Eigen::MatrixXd information_random_walk(const std::vector<asi::PredictedMeasurement>& track, int n, double s0,
                                               double q2, const std::vector<double>& frames, double px_to_meas = 1.0) {
  s0 *= px_to_meas; q2 *= px_to_meas * px_to_meas;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, n);
  for (size_t k = 0; k < track.size(); ++k) {
    std::vector<asi::JacobianBlock> blocks = track[k].blocks;
    double var = s0 * s0;
    if (k > 0) {
      for (auto b : track[k - 1].blocks) { b.J = -b.J; blocks.push_back(b); }
      var = q2 * std::max(1.0, k < frames.size() ? frames[k] : 1.0);
    }
    const double w = 1.0 / var;
    for (const auto& bi : blocks)
      for (const auto& bj : blocks)
        J.block(bi.col, bj.col, bi.J.cols(), bj.J.cols()).noalias() += w * bi.J.transpose() * bj.J;
  }
  return J;
}

// Day 9: STEREO random walk. One landmark seen by cam0 and cam1 at the same samples; per image axis the two
// cameras' errors are jointly Gaussian: first sample Cov = [[s0^2, s0^2 - sd^2/2], [., s0^2]] (sd = disparity
// spread), increments over m frames Cov = q2 m [[1, rho], [rho, 1]] (rho ~ 1: the cameras' KLT errors drift
// together, the disparity error barely grows). Differencing whitens the time correlation exactly, so
// J = sum_k sum_axis [r0 r1]_k^T C_k^-1 [r0 r1]_k with r_c = the (differenced) row of camera c on that axis.
// `c0`, `c1`: the samples of each camera, same length, same sample times.
inline Eigen::MatrixXd information_random_walk_stereo(const std::vector<asi::PredictedMeasurement>& c0,
                                                      const std::vector<asi::PredictedMeasurement>& c1, int n, double s0,
                                                      double sd, double q2, double rho, const std::vector<double>& frames,
                                                      double px_to_meas = 1.0) {
  s0 *= px_to_meas; sd *= px_to_meas; q2 *= px_to_meas * px_to_meas;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, n);
  auto diff_blocks = [](const std::vector<asi::PredictedMeasurement>& v, size_t k) {
    std::vector<asi::JacobianBlock> b = v[k].blocks;
    if (k > 0) for (auto x : v[k - 1].blocks) { x.J = -x.J; b.push_back(x); }
    return b;
  };
  for (size_t k = 0; k < c0.size(); ++k) {
    Eigen::Matrix2d C;
    if (k == 0) C << s0 * s0, s0 * s0 - sd * sd / 2, s0 * s0 - sd * sd / 2, s0 * s0;
    else { const double m = std::max(1.0, k < frames.size() ? frames[k] : 1.0); C << 1.0, rho, rho, 1.0; C *= q2 * m; }
    const Eigen::Matrix2d Wm = C.inverse();
    const std::vector<asi::JacobianBlock> B[2] = {diff_blocks(c0, k), diff_blocks(c1, k)};
    for (int a = 0; a < 2; ++a)
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
          for (const auto& bi : B[i])
            for (const auto& bj : B[j])
              J.block(bi.col, bj.col, bi.J.cols(), bj.J.cols()).noalias() += Wm(i, j) * bi.J.row(a).transpose() * bj.J.row(a);
  }
  return J;
}

struct PoseGainSplit {
  double logdet_now = NAN;    // log det Sigma_pose(now)
  double logdet_prior = NAN;  // log det Sigma_pose at the end, propagated, no measurements
  double logdet_post = NAN;   // ... after all measurements along the path
  double propagation() const { return logdet_now - logdet_prior; }   // (b) <= 0: travel cost
  double measurement() const { return logdet_prior - logdet_post; }  // (a) >= 0: what the views buy
  double total() const { return logdet_now - logdet_post; }          // dI_pose = (a) + (b)
  bool ok = false;
};

// S: prior (already propagated + cloned), Tpose: 6 x n scoring map of the pose,
// J: measurement information on the same state. Sigma+ = (I + S J)^-1 S.
inline PoseGainSplit pose_gain_split(double logdet_now, const Eigen::MatrixXd& S, const Eigen::MatrixXd& Tpose, const Eigen::MatrixXd& J) {
  PoseGainSplit g;
  g.logdet_now = logdet_now;
  bool ok1 = false, ok2 = false;
  g.logdet_prior = logdet_spd(Tpose * S * Tpose.transpose(), &ok1);
  const int n = (int)S.rows();
  const Eigen::MatrixXd Sp = (Eigen::MatrixXd::Identity(n, n) + S * J).partialPivLu().solve(S);
  g.logdet_post = logdet_spd(Tpose * Sp * Tpose.transpose(), &ok2);
  g.ok = ok1 && ok2;
  return g;
}

}  // namespace active_slam_sim
