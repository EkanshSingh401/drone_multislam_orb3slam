// ig_prediction_validation (PATCHES s61, planner step 4) -- predicted vs realized
// information gain over flight segments. No closed loop.
//
//   ig_prediction_validation <jc.bag> <imu.txt> <ov_state_est.txt> <gt.tum>
//                            <seg_len_s> <seg_every_s> <t_window0> <t_window1>
//
// Segment [t0, t1]: t0 = time of a recorded JointCovariance message M0, t1 = the
// first recorded message M1 at or after t0 + seg_len. Same marginal at both ends:
// IMU (15) + global XYZ of the SLAM landmarks present in BOTH M0 and M1.
//   L0 = log det of that marginal in M0, L1 = in M1 (realized).
//   realized gain   = L0 - L1
//   (a) as built    common prior M0; one predicted measurement per landmark per
//                   camera from every frame OpenVINS processes in (t0, t1]
//                   (state-file stamps = 15 Hz effective tracking), mapped onto the
//                   current IMU pose (v1, no process noise). pred_a = L0 - post_a.
//   (b) propagated  M0's covariance (message order) propagated with OpenVINS's own
//                   Propagator::predict_and_compute (Phi, Qd; rk4; config noise) over
//                   the segment's IMU data, a clone appended at each processed frame
//                   (OpenVINS augment_clone: copy of the IMU pose rows); each frame's
//                   measurements attach to its own clone. pred_b = L0 - post_b, and
//                   prop_only = L0 - log det(propagated marginal, no updates).
// Only SLAM-map landmarks are predicted: OpenVINS's MSCKF features and newly
// initialized SLAM features also inform the IMU state in reality, so they are a
// known source of realized > predicted.
// Output: one JSON line per segment.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "active_slam_information/ros/joint_cov_problem.hpp"
#include "active_slam_msgs/msg/joint_covariance.hpp"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateOptions.h"
#include "utils/NoiseManager.h"
#include "utils/sensor_data.h"

namespace asi = active_slam_information;
using Eigen::MatrixXd;

struct PropagatorAccess : public ov_msckf::Propagator {
  using ov_msckf::Propagator::Propagator;
  using ov_msckf::Propagator::predict_and_compute;
};

static double logdet_spd(const MatrixXd& A, bool* ok) {
  Eigen::LLT<MatrixXd> llt(A);
  *ok = llt.info() == Eigen::Success;
  if (!*ok) return NAN;
  return 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
}

int main(int argc, char** argv) {
  if (argc < 9) {
    std::fprintf(stderr, "usage: %s jc.bag imu.txt ov_state_est.txt gt.tum seg_len seg_every t_win0 t_win1\n", argv[0]);
    return 2;
  }
  const double seg_len = std::atof(argv[5]), seg_every = std::atof(argv[6]);
  const double tw0 = std::atof(argv[7]), tw1 = std::atof(argv[8]);

  // ---- joint covariance messages ----
  std::vector<active_slam_msgs::msg::JointCovariance> jc;
  {
    rosbag2_cpp::Reader r;
    r.open(argv[1]);
    rclcpp::Serialization<active_slam_msgs::msg::JointCovariance> ser;
    while (r.has_next()) {
      auto bm = r.read_next();
      if (bm->topic_name != "/openvins/joint_covariance") continue;
      active_slam_msgs::msg::JointCovariance m;
      rclcpp::SerializedMessage sm(*bm->serialized_data);
      ser.deserialize_message(&sm, &m);
      jc.push_back(m);
    }
  }
  auto stamp = [](const active_slam_msgs::msg::JointCovariance& m) { return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec; };
  // ---- IMU: CSV "t wx wy wz ax ay az" exported from the (stamp-corrected) bag ----
  std::vector<ov_core::ImuData> imu;
  {
    std::ifstream f(argv[2]);
    ov_core::ImuData d;
    while (f >> d.timestamp >> d.wm(0) >> d.wm(1) >> d.wm(2) >> d.am(0) >> d.am(1) >> d.am(2)) imu.push_back(d);
    std::sort(imu.begin(), imu.end(), [](auto& x, auto& y) { return x.timestamp < y.timestamp; });
  }
  // ---- OpenVINS state file (one row per processed frame): t q(4) p v bg ba ----
  std::map<double, Eigen::Matrix<double, 16, 1>> est;
  {
    std::ifstream f(argv[3]);
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      double t;
      Eigen::Matrix<double, 16, 1> x;
      ss >> t;
      for (int i = 0; i < 16; i++) ss >> x(i);
      est[t] = x;
    }
  }
  // ---- ground truth (base_link) for motion speed ----
  std::vector<std::array<double, 8>> gt;
  {
    std::ifstream f(argv[4]);
    std::array<double, 8> a;
    while (f >> a[0] >> a[1] >> a[2] >> a[3] >> a[4] >> a[5] >> a[6] >> a[7]) gt.push_back(a);
  }

  // ---- OpenVINS propagator, configured as the estimator (rk4, Kalibr IMU model, config noise) ----
  ov_msckf::NoiseManager nm;
  nm.sigma_w = 1.6e-4; nm.sigma_w_2 = nm.sigma_w * nm.sigma_w;
  nm.sigma_a = 2.0e-3; nm.sigma_a_2 = nm.sigma_a * nm.sigma_a;
  nm.sigma_wb = 2.0e-6; nm.sigma_wb_2 = nm.sigma_wb * nm.sigma_wb;
  nm.sigma_ab = 3.0e-4; nm.sigma_ab_2 = nm.sigma_ab * nm.sigma_ab;
  PropagatorAccess prop(nm, 9.81);
  const asi::ConstantNoiseModel noise(1.0);  // up_slam_sigma_px

  double next_t0 = tw0;
  for (size_t i0 = 0; i0 < jc.size(); ++i0) {
    const double t0 = stamp(jc[i0]);
    if (t0 < next_t0 || t0 > tw1 - seg_len) continue;
    size_t i1 = i0 + 1;
    while (i1 < jc.size() && stamp(jc[i1]) < t0 + seg_len - 1e-6) ++i1;
    if (i1 >= jc.size()) break;
    const double t1 = stamp(jc[i1]);
    if (t1 - t0 > 1.5 * seg_len) { next_t0 = t0 + seg_every; continue; }  // recording gap
    next_t0 = t0 + seg_every;

    const auto P0 = asi::problem_from_msg(jc[i0]);
    const auto P1 = asi::problem_from_msg(jc[i1]);
    if (!P0.ok || !P1.ok) continue;
    std::set<long long> common;
    for (auto& kv : P0.landmark_index) if (P1.landmark_index.count(kv.first)) common.insert(kv.first);
    const MatrixXd T0 = P0.T_metric(common), T1 = P1.T_metric(common);
    bool ok0, ok1;
    const double L0 = logdet_spd(T0 * P0.Sigma * T0.transpose(), &ok0);
    const double L1 = logdet_spd(T1 * P1.Sigma * T1.transpose(), &ok1);
    if (!ok0 || !ok1) continue;

    // frames OpenVINS processed in (t0, t1]
    std::vector<std::pair<double, asi::Pose>> frames;
    for (auto it = est.upper_bound(t0 + 1e-9); it != est.end() && it->first <= t1 + 1e-9; ++it) {
      asi::Pose p;
      p.R_GtoI = asi::jpl_quat_to_rot({it->second(0), it->second(1), it->second(2), it->second(3)});
      p.p_IinG = it->second.segment<3>(4);
      frames.push_back({it->first, p});
    }

    // (a) as built: common prior
    std::vector<asi::PredictedMeasurement> meas_a;
    for (auto& fr : frames) {
      auto m = asi::predict_all(P0, fr.second, noise, &common);
      meas_a.insert(meas_a.end(), m.begin(), m.end());
    }
    const asi::InformationPrior prior_a(P0.Sigma, T0);
    const auto g_a = asi::evaluate(prior_a, meas_a);

    // (b) OpenVINS propagation + clone per processed frame
    auto e0 = est.find(t0);
    double pred_b = NAN, prop_only = NAN, post_b = NAN;
    int n_meas_b = 0;
    if (e0 != est.end()) {
      ov_msckf::StateOptions so;
      so.num_cameras = 2;
      so.integration_method = ov_msckf::StateOptions::IntegrationMethod::RK4;
      auto st = std::make_shared<ov_msckf::State>(so);
      st->_imu->set_value(e0->second);
      st->_imu->set_fej(e0->second);
      MatrixXd S = P0.Sigma;
      const int ic = P0.imu_col;
      std::vector<std::pair<double, int>> clone_cols;
      double tcur = t0;
      for (auto& fr : frames) {
        auto readings = ov_msckf::Propagator::select_imu_readings(imu, tcur, fr.first, false);
        for (size_t k = 0; k + 1 < readings.size(); ++k) {
          Eigen::MatrixXd F, Qd;
          prop.predict_and_compute(st, readings[k], readings[k + 1], F, Qd);
          const int d = (int)F.rows();  // 15 (no IMU intrinsics)
          // Sigma <- Phi Sigma Phi^T + Q on the IMU block (cross terms follow)
          MatrixXd rows = F * S.middleRows(ic, d);
          S.middleRows(ic, d) = rows;
          MatrixXd cols = S.middleCols(ic, d) * F.transpose();
          S.middleCols(ic, d) = cols;
          S.block(ic, ic, d, d) += Qd;
        }
        tcur = fr.first;
        // augment clone: copy of IMU pose rows/cols (orientation, position)
        const int n = (int)S.rows();
        MatrixXd S2 = MatrixXd::Zero(n + 6, n + 6);
        S2.topLeftCorner(n, n) = S;
        S2.block(n, 0, 6, n) = S.middleRows(ic, 6);
        S2.block(0, n, n, 6) = S.middleCols(ic, 6);
        S2.block(n, n, 6, 6) = S.block(ic, ic, 6, 6);
        S = 0.5 * (S2 + S2.transpose());
        clone_cols.push_back({fr.first, n});
      }
      MatrixXd Tb = MatrixXd::Zero(T0.rows(), S.cols());
      Tb.leftCols(T0.cols()) = T0;
      bool okp;
      const double Lp = logdet_spd(Tb * S * Tb.transpose(), &okp);
      std::vector<asi::PredictedMeasurement> meas_b;
      for (size_t k = 0; k < frames.size(); ++k) {
        const asi::Pose& fp = frames[k].second;
        for (const auto& lm : P0.landmarks) {
          if (!common.count(lm.id)) continue;
          for (const auto& cam : P0.cameras) {
            asi::PredictedMeasurement pm;
            if (asi::predict_measurement(fp, fp, clone_cols[k].second, cam, lm, noise, nullptr, &pm)) meas_b.push_back(pm);
          }
        }
      }
      n_meas_b = (int)meas_b.size();
      const asi::InformationPrior prior_b(S, Tb);
      const auto g_b = asi::evaluate(prior_b, meas_b);
      if (okp && prior_b.ok() && g_b.ok) {
        prop_only = L0 - Lp;
        post_b = g_b.posterior_logdet;
        pred_b = L0 - post_b;
      }
    }

    // motion over the segment (GT, base_link)
    double ang = 0, dist = 0;
    {
      const std::array<double, 8>* prev = nullptr;
      for (auto& g : gt) {
        if (g[0] < t0 || g[0] > t1) continue;
        if (prev) {
          Eigen::Quaterniond qa((*prev)[7], (*prev)[4], (*prev)[5], (*prev)[6]), qb(g[7], g[4], g[5], g[6]);
          ang += qa.angularDistance(qb);
          dist += Eigen::Vector3d(g[1] - (*prev)[1], g[2] - (*prev)[2], g[3] - (*prev)[3]).norm();
        }
        prev = &g;
      }
    }
    std::printf(
        "{\"t0\":%.3f,\"t1\":%.3f,\"frames\":%zu,\"common_landmarks\":%zu,\"landmarks_M0\":%zu,\"meas_a\":%zu,\"meas_b\":%d,"
        "\"L0\":%.6f,\"L1\":%.6f,\"realized\":%.6f,\"pred_a\":%.6f,\"post_a\":%.6f,\"pred_b\":%.6f,\"post_b\":%.6f,"
        "\"prop_only\":%.6f,\"ang_speed\":%.5f,\"speed\":%.5f}\n",
        t0, t1, frames.size(), common.size(), P0.landmarks.size(), meas_a.size(), n_meas_b, L0, L1, L0 - L1,
        g_a.ok ? g_a.delta : NAN, g_a.ok ? g_a.posterior_logdet : NAN, pred_b, post_b, prop_only, ang / (t1 - t0),
        dist / (t1 - t0));
  }
  return 0;
}
