// test_candidate_scoring (day 3): path-integrated pose gain vs brute force, reduction,
// single-waypoint equivalence with the old endpoint score, visibility classifier.
#include <cstdio>
#include <random>

#include "active_slam_sim/candidate_scoring.hpp"

using namespace active_slam_sim;
using Eigen::MatrixXd;
static int fails = 0, passes = 0;
#define CHECK(c, ...) do { if (c) ++passes; else { ++fails; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static std::mt19937 rng(7);
static double nrm() { static std::normal_distribution<double> d(0, 1); return d(rng); }
static MatrixXd randn(int r, int c) { MatrixXd M(r, c); for (int i = 0; i < r; ++i) for (int j = 0; j < c; ++j) M(i, j) = nrm(); return M; }

// linear time-invariant IMU model on the 15-block: x+ = F(dt) x + w, Q(dt)
struct Lin {
  MatrixXd A, Qc;
  void operator()(MatrixXd& S, int ic, double t0, double t1) const {
    const double dt = t1 - t0;
    const MatrixXd F = MatrixXd::Identity(15, 15) + A * dt;
    MatrixXd rows = F * S.middleRows(ic, 15); S.middleRows(ic, 15) = rows;
    MatrixXd cols = S.middleCols(ic, 15) * F.transpose(); S.middleCols(ic, 15) = cols;
    S.block(ic, ic, 15, 15) += Qc * dt;
  }
};

int main() {
  const int n0 = 45, ic = 6;  // a calibration block before the IMU, landmarks after
  MatrixXd Lr = randn(n0, n0 + 5);
  MatrixXd S0 = Lr * Lr.transpose() / n0 * 0.01;
  // make it rank deficient like OpenVINS (newest clone == IMU pose): copy the IMU pose into cols 21..26
  for (int i = 0; i < 6; ++i) { S0.row(21 + i) = S0.row(ic + i); S0.col(21 + i) = S0.col(ic + i); }
  for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) S0(21 + i, 21 + j) = S0(ic + i, ic + j);
  Lin lin; lin.A = 0.05 * randn(15, 15); MatrixXd q = randn(15, 15); lin.Qc = 1e-4 * q * q.transpose() / 15;
  const PropagateFn prop = [&](MatrixXd& S, int c, double a, double b) { lin(S, c, a, b); };
  const std::vector<double> times{0.7, 1.4, 2.1, 2.6};
  const int K = (int)times.size();

  // measurements per waypoint: 3 landmark observations, each touching the clone and a 3-col landmark block
  std::vector<std::vector<asi::PredictedMeasurement>> per(K);
  for (int k = 0; k < K; ++k)
    for (int j = 0; j < 3; ++j) {
      asi::PredictedMeasurement pm;
      pm.blocks.push_back({n0 + 6 * k, randn(2, 6)});
      pm.blocks.push_back({30 + 3 * ((k + j) % 4), randn(2, 3)});
      pm.sigma = 0.3 + 0.1 * j;
      per[k].push_back(pm);
    }

  // --- brute force: sequential propagate / clone / dense Kalman update ---
  MatrixXd S = S0;
  double t = 0;
  for (int k = 0; k < K; ++k) {
    lin(S, ic, t, times[k]); t = times[k];
    const int n = (int)S.rows();
    MatrixXd S2 = MatrixXd::Zero(n + 6, n + 6);
    S2.topLeftCorner(n, n) = S; S2.block(n, 0, 6, n) = S.middleRows(ic, 6); S2.block(0, n, n, 6) = S.middleCols(ic, 6);
    S2.block(n, n, 6, 6) = S.block(ic, ic, 6, 6); S = 0.5 * (S2 + S2.transpose());
    const MatrixXd H = asi::dense_jacobian(per[k], n + 6);
    MatrixXd V = MatrixXd::Zero(H.rows(), H.rows());
    for (int i = 0; i < (int)per[k].size(); ++i) V.block(2 * i, 2 * i, 2, 2) = per[k][i].sigma * per[k][i].sigma * MatrixXd::Identity(2, 2);
    const MatrixXd A = H * S * H.transpose() + V;
    S = S - S * H.transpose() * A.ldlt().solve(H * S);
    S = 0.5 * (S + S.transpose());
  }
  const int last = n0 + 6 * (K - 1);
  const double ld_bf = logdet_spd(S.block(last, last, 6, 6));
  const double ld_now = logdet_spd(S0.block(ic, ic, 6, 6));

  // --- fast: joint prior, all measurements at once, information form ---
  const PathPrior P = build_path_prior(S0, ic, times, prop);
  std::vector<asi::PredictedMeasurement> all;
  for (auto& v : per) all.insert(all.end(), v.begin(), v.end());
  const int N = (int)P.S.rows();
  MatrixXd Tp = MatrixXd::Zero(6, N); Tp.block(0, P.clone_cols.back(), 6, 6).setIdentity();
  const auto g = pose_gain_split(ld_now, P.S, Tp, information(all, N));
  std::printf("path (4 waypoints): sequential EKF %.12f  joint info-form %.12f\n", ld_bf, g.logdet_post);
  CHECK(g.ok && std::fabs(g.logdet_post - ld_bf) < 1e-8, "path-integrated posterior != sequential EKF (%.3e)", g.logdet_post - ld_bf);
  CHECK(std::fabs(g.total() - (g.measurement() + g.propagation())) < 1e-12, "split does not add up");
  CHECK(g.propagation() <= 1e-12 && g.measurement() >= -1e-12, "signs: propagation %.3g measurement %.3g", g.propagation(), g.measurement());
  // reference covariance-form update on the same joint prior (library reference path)
  const auto ref = asi::evaluate_reference(P.S, Tp, all);
  CHECK(ref.ok && std::fabs(ref.posterior_logdet - g.logdet_post) < 1e-8, "info form != covariance-form reference (%.3e)", ref.posterior_logdet - g.logdet_post);

  // --- reduction to touched columns + scored pose is exact ---
  std::vector<char> mark(N, 0);
  touched_columns(all, mark);
  for (int i = 0; i < 6; ++i) mark[P.clone_cols.back() + i] = 1;
  std::vector<int> idx, map;
  for (int i = 0; i < N; ++i) if (mark[i]) idx.push_back(i);
  const MatrixXd Sr = marginal(P.S, idx, &map);
  const auto allr = reindex(all, map);
  MatrixXd Tr = MatrixXd::Zero(6, Sr.rows()); Tr.block(0, map[P.clone_cols.back()], 6, 6).setIdentity();
  const auto gr = pose_gain_split(ld_now, Sr, Tr, information(allr, (int)Sr.rows()));
  std::printf("reduced %d -> %d states: %.12f\n", N, (int)Sr.rows(), gr.logdet_post);
  CHECK(gr.ok && std::fabs(gr.logdet_post - g.logdet_post) < 1e-9 && std::fabs(gr.logdet_prior - g.logdet_prior) < 1e-9, "reduction not exact");

  // --- one waypoint == the old endpoint score (InformationPrior on the IMU block, fast path) ---
  {
    const PathPrior P1 = build_path_prior(S0, ic, {2.6}, prop);
    const int N1 = (int)P1.S.rows();
    std::vector<asi::PredictedMeasurement> m1 = per[0];
    for (auto& pm : m1) pm.blocks[0].col = P1.clone_cols[0];
    MatrixXd Timu = MatrixXd::Zero(6, N1); Timu.block(0, ic, 6, 6).setIdentity();
    const asi::InformationPrior old(P1.S, Timu);
    const auto go = asi::evaluate(old, m1);
    MatrixXd Tc = MatrixXd::Zero(6, N1); Tc.block(0, P1.clone_cols[0], 6, 6).setIdentity();
    const auto gn = pose_gain_split(ld_now, P1.S, Tc, information(m1, N1));
    std::printf("single waypoint: old dI_pose %.12f  new %.12f\n", ld_now - go.posterior_logdet, gn.total());
    CHECK(go.ok && gn.ok && std::fabs((ld_now - go.posterior_logdet) - gn.total()) < 1e-8, "single waypoint differs from old endpoint score");
  }

  // --- executor profile ---
  {
    const auto w = executor_profile({0, 0}, 0.0, {3, 4}, M_PI / 2, 1.5, 0.5, 0.6, 0.5);
    CHECK(w.size() == 10, "waypoints: %zu (expected 9 at 0.5 m + endpoint)", w.size());
    CHECK(std::fabs(w.back().t - 10.0) < 1e-12 && std::fabs(w.back().yaw - M_PI / 2) < 1e-12, "endpoint t %.3f yaw %.3f", w.back().t, w.back().yaw);
    CHECK(std::fabs(w[0].yaw - 0.6) < 1e-12 && std::fabs(w[0].pose.p_IinG.x() - 0.3) < 1e-12, "first waypoint yaw %.3f x %.3f", w[0].yaw, w[0].pose.p_IinG.x());
    const auto turn = executor_profile({1, 1}, 0.0, {1, 1}, -M_PI / 2, 1.5, 0.5, 0.6, 0.5);
    CHECK(turn.size() == 1 && std::fabs(turn[0].t - (M_PI / 2) / 0.6) < 1e-12, "turn in place");
    const auto old = executor_profile({0, 0}, 0.0, {3, 4}, 0.3, 1.5, 0.5, 0.6, 0.0);
    CHECK(old.size() == 1, "spacing 0 must give the endpoint only");
  }

  // --- classify_visibility VISIBLE <=> predict_measurement accepts ---
  {
    asi::CameraModel cam;
    cam.R_ItoC << 0, -1, 0, 0, 0, -1, 1, 0, 0;  // IMU x forward -> camera z
    cam.p_IinC = Eigen::Vector3d(0.01, 0.02, -0.03);
    cam.width = 848; cam.height = 480; cam.max_depth = 20;
    asi::Pose cp; cp.R_GtoI = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose(); cp.p_IinG = {1, 2, 1.5};
    const asi::ConstantNoiseModel noise(1.0);
    int agree = 0, nvis = 0, n = 4000, cnt[4] = {0, 0, 0, 0};
    for (int i = 0; i < n; ++i) {
      asi::LandmarkLinearization lm;
      lm.p_FinG = cp.p_IinG + Eigen::Vector3d(nrm() * 12, nrm() * 12, nrm() * 3);
      if (i % 50 == 0) lm.p_FinG = cp.p_IinG + cp.R_GtoI.transpose() * Eigen::Vector3d(0.1, 0, 0);  // too near
      asi::PredictedMeasurement pm;
      const bool a = asi::predict_measurement(cp, cp, 0, cam, lm, noise, nullptr, &pm);
      const Vis v = classify_visibility(cp, cam, lm.p_FinG);
      cnt[(int)v]++;
      agree += (a == (v == Vis::VISIBLE));
      nvis += a;
    }
    std::printf("classifier: %d/%d agree (visible %d, fov %d, range %d)\n", agree, n, nvis, cnt[1], cnt[2]);
    CHECK(agree == n && cnt[2] > 0 && nvis > 100, "classifier disagrees with predict_measurement");
  }
  std::printf("%d passed, %d failed\n", passes, fails);
  return fails ? 1 : 0;
}
