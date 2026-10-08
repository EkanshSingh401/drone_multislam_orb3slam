// drift_model (day 8 step 4): predicted relative-pose (drift) covariance over a window from the
// features actually tracked, under the white (independent sigma_px) or the random-walk measurement model
// (MATH_TO_CODE.md "Random-walk tracking error"), inside the planner's linear model:
//   prior   IMU block (15) of the flight's joint covariance at the window start
//   motion  the planner's synthetic static propagation (tabulated OpenVINS F, Qd)
//   clones  IMU pose cloned at the sampled frame times (waypoints)
//   tracks  each tracked feature: 3D point (input), its samples (waypoint indices, in order), per-track
//           random-walk rate q2 (px^2 per frame) and frames between samples; landmark eliminated by a
//           Schur complement with a 10 m prior
// Output per task: sqrt of the relative position covariance trace (m) and the relative yaw sigma (rad)
// between the first and last clone, for model white (sigma 1 px, the filter's value) and random_walk.
// Usage: drift_model <tasks.txt> <s0_px>
// tasks.txt:  TASK <id> <jc.cdr>   WP <t rel> x y z qx qy qz qw   TRK x y z q2 n k1 f1 k2 f2 ...   END
//   (k = waypoint index, f = camera frames since the previous sample of this track; f of the first = 1)
//   day 9 (stereo): "TRKC <cam> x y z q2 n k1 f1 ..." gives a track in camera <cam>; consecutive TRKC lines
//   with identical x y z are the same landmark (one per camera) and share its columns (one Schur step).
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "active_slam_information/information_gain.hpp"
#include "active_slam_information/ros/joint_cov_problem.hpp"
#include "active_slam_information/view_geometry.hpp"
#include "active_slam_msgs/msg/joint_covariance.hpp"
#include "active_slam_sim/candidate_scoring.hpp"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateOptions.h"
#include "utils/NoiseManager.h"
#include "utils/sensor_data.h"

namespace asi = active_slam_information;
using Eigen::MatrixXd;
using Eigen::Vector3d;

struct PropagatorAccess : public ov_msckf::Propagator {
  using ov_msckf::Propagator::Propagator;
  using ov_msckf::Propagator::predict_and_compute;
};
struct Trk { Vector3d p; double q2; std::vector<int> k; std::vector<double> f; int cam = 0; bool join = false; };
struct Task { std::string id, cdr; std::vector<active_slam_sim::Waypoint> W; std::vector<Trk> trk; };

static active_slam_msgs::msg::JointCovariance load_cdr(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  rclcpp::SerializedMessage sm(buf.size());
  auto& rm = sm.get_rcl_serialized_message();
  std::memcpy(rm.buffer, buf.data(), buf.size()); rm.buffer_length = buf.size();
  active_slam_msgs::msg::JointCovariance m;
  rclcpp::Serialization<active_slam_msgs::msg::JointCovariance>().deserialize_message(&sm, &m);
  return m;
}

int main(int argc, char** argv) {
  if (argc < 3) { std::cerr << "usage: drift_model tasks.txt s0_px\n"; return 2; }
  const double s0 = std::stod(argv[2]);
  std::ifstream in(argv[1]); std::vector<Task> tasks; std::string line;
  while (std::getline(in, line)) {
    std::istringstream s(line); std::string k; s >> k;
    if (k == "TASK") { Task t; s >> t.id >> t.cdr; tasks.push_back(t); }
    else if (k == "WP") {
      active_slam_sim::Waypoint w; double x, y, z, qx, qy, qz, qw; s >> w.t >> x >> y >> z >> qx >> qy >> qz >> qw;
      w.pose.R_GtoI = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix().transpose(); w.pose.p_IinG = Vector3d(x, y, z);
      tasks.back().W.push_back(w);
    } else if (k == "TRK" || k == "TRKC") {
      Trk t; int n; if (k == "TRKC") s >> t.cam;
      s >> t.p.x() >> t.p.y() >> t.p.z() >> t.q2 >> n;
      if (!tasks.back().trk.empty() && k == "TRKC" && (tasks.back().trk.back().p - t.p).norm() < 1e-9) t.join = true;
      for (int i = 0; i < n; ++i) { int kk; double ff; s >> kk >> ff; t.k.push_back(kk); t.f.push_back(ff); }
      tasks.back().trk.push_back(t);
    }
  }
  ov_msckf::NoiseManager nm;
  nm.sigma_w = 1.6e-4; nm.sigma_w_2 = nm.sigma_w * nm.sigma_w; nm.sigma_a = 2.0e-3; nm.sigma_a_2 = nm.sigma_a * nm.sigma_a;
  nm.sigma_wb = 2.0e-6; nm.sigma_wb_2 = nm.sigma_wb * nm.sigma_wb; nm.sigma_ab = 3.0e-4; nm.sigma_ab_2 = nm.sigma_ab * nm.sigma_ab;
  PropagatorAccess prop(nm, 9.81);
  const asi::ConstantNoiseModel noise(1.0);
  for (const auto& T : tasks) {
    const auto msg = load_cdr(T.cdr);
    const auto P = asi::problem_from_msg(msg);
    if (!P.ok || T.W.size() < 2) { std::cout << "ERR " << T.id << "\n"; continue; }
    const int ic = P.imu_col;
    MatrixXd S0 = P.Sigma.block(ic, ic, 15, 15);
    const double dt = 0.02; MatrixXd F1, Q1;
    {
      ov_msckf::StateOptions so; so.num_cameras = 2; auto st = std::make_shared<ov_msckf::State>(so);
      Eigen::Matrix<double, 16, 1> x = Eigen::Matrix<double, 16, 1>::Zero();
      for (int i = 0; i < 4; i++) x(i) = msg.imu_pose.q_gtoi[i];
      for (int i = 0; i < 3; i++) x(4 + i) = msg.imu_pose.p_iing[i];
      st->_imu->set_value(x); st->_imu->set_fej(x);
      ov_core::ImuData a, b; a.wm.setZero(); b.wm.setZero(); a.am = P.current.R_GtoI * Vector3d(0, 0, 9.81); b.am = a.am; a.timestamp = 0; b.timestamp = dt;
      prop.predict_and_compute(st, a, b, F1, Q1);
    }
    const int nsteps = (int)std::ceil((T.W.back().t + 1.0) / dt) + 1;
    std::vector<MatrixXd> Phi(nsteps + 1), Qk(nsteps + 1);
    Phi[0] = MatrixXd::Identity(15, 15); Qk[0] = MatrixXd::Zero(15, 15);
    for (int k = 1; k <= nsteps; ++k) { Phi[k] = F1 * Phi[k - 1]; Qk[k] = F1 * Qk[k - 1] * F1.transpose() + Q1; }
    const active_slam_sim::PropagateFn pf = [&](MatrixXd& S, int c, double t0, double t1) {
      const int k = std::min(nsteps, (int)std::lround(t1 / dt)) - std::min(nsteps, (int)std::lround(t0 / dt));
      if (k <= 0) return;
      MatrixXd rows = Phi[k] * S.middleRows(c, 15); S.middleRows(c, 15) = rows;
      MatrixXd cols = S.middleCols(c, 15) * Phi[k].transpose(); S.middleCols(c, 15) = cols;
      S.block(c, c, 15, 15) += Qk[k];
    };
    std::vector<double> times; for (const auto& w : T.W) times.push_back(w.t);
    const auto PP = active_slam_sim::build_path_prior(S0, 0, times, pf);
    const int n = (int)PP.S.rows();
    const int L = n;  // landmark placeholder columns appended per track (3)
    for (int model = 0; model < 2; ++model) {
      MatrixXd J = MatrixXd::Zero(n, n);
      int used = 0;
      MatrixXd Jt = MatrixXd::Zero(n + 3, n + 3); int nm_lm = 0;
      auto schur = [&]() {
        if (nm_lm >= 2) {
          const Eigen::Matrix3d A = Jt.block(n, n, 3, 3) + Eigen::Matrix3d::Identity() * 1e-2;
          J += Jt.topLeftCorner(n, n) - Jt.block(0, n, n, 3) * A.inverse() * Jt.block(n, 0, 3, n);
          ++used;
        }
        Jt.setZero(); nm_lm = 0;
      };
      for (const auto& t : T.trk) {
        if (!t.join) schur();
        asi::LandmarkLinearization lm; lm.is_virtual = true; lm.p_FinG = t.p; lm.rep = {{L, Eigen::Matrix3d::Identity()}};
        std::vector<asi::PredictedMeasurement> ms; std::vector<double> fr; int prev = -2;
        std::vector<std::vector<asi::PredictedMeasurement>> segs; std::vector<std::vector<double>> segf;
        const auto& camm = P.cameras[std::min<size_t>(t.cam, P.cameras.size() - 1)];
        for (size_t i = 0; i < t.k.size(); ++i) {
          asi::PredictedMeasurement pm;
          if (!asi::predict_measurement(P.current, T.W[t.k[i]].pose, PP.clone_cols[t.k[i]], camm, lm, noise, nullptr, &pm)) { prev = -2; continue; }
          if (t.k[i] != prev + 1 && !ms.empty()) { segs.push_back(ms); segf.push_back(fr); ms.clear(); fr.clear(); }
          fr.push_back(ms.empty() ? 1.0 : t.f[i]); ms.push_back(pm); prev = t.k[i];
        }
        if (!ms.empty()) { segs.push_back(ms); segf.push_back(fr); }
        for (size_t si = 0; si < segs.size(); ++si) {
          nm_lm += (int)segs[si].size();
          if (model == 0) Jt += active_slam_sim::information(segs[si], n + 3);
          else Jt += active_slam_sim::information_random_walk(segs[si], n + 3, s0, t.q2, segf[si]);
        }
      }
      schur();
      const MatrixXd Sp = (MatrixXd::Identity(n, n) + PP.S * J).partialPivLu().solve(PP.S);
      const int c0 = PP.clone_cols.front(), c1 = PP.clone_cols.back();
      MatrixXd D = MatrixXd::Zero(6, n); D.block(0, c1, 6, 6).setIdentity(); D.block(0, c0, 6, 6) -= MatrixXd::Identity(6, 6);
      const MatrixXd C = D * Sp * D.transpose();
      // yaw: local orientation error projected on gravity in the IMU frame (now-attitude, static model)
      const Vector3d gI = P.current.R_GtoI * Vector3d(0, 0, 1);
      const double syaw = std::sqrt(std::max(0.0, (double)(gI.transpose() * C.block(0, 0, 3, 3) * gI)));
      const double spos = std::sqrt(std::max(0.0, C.block(3, 3, 3, 3).trace()));
      std::cout << "DRIFT " << T.id << " " << (model == 0 ? "white" : "random_walk") << " " << spos << " " << syaw << " " << used << "\n";
    }
  }
  return 0;
}
