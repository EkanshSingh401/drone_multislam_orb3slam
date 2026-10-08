// gain_ladder (day 5 step 5a/5b): ladder of pose-gain predictors at goal scale, all inside the
// planner's own linear-Gaussian model (candidate_scoring.hpp + active_slam_information), so that
// only the INPUTS change from rung to rung:
//   plan_real     the planner's path (executor_profile to the chosen goal, path_spacing), the SLAM
//                 landmarks in the state at the decision, persistent (= logged dI_prop + dI_meas_real;
//                 GATE: must reproduce the log)
//   act_real      the flown path (OpenVINS odometry, same spacing by arc length), same landmarks
//   act_real_win  ... a landmark counts at a waypoint only while it is still in OpenVINS's state
//   dense_win     ... waypoints at OpenVINS's actual clone times (camera rate) instead of 0.5 m
//   oracle        dense_win + SLAM landmarks initialised during the window (from their first
//                 appearance in the state, plus their pre-init track) + the MSCKF features
//                 OpenVINS actually used (positions from /ov_msckf/points_msckf, observed by the
//                 clones of the sliding window before their use, FOV-tested), MSCKF features
//                 eliminated by a per-feature Schur complement (null-space equivalent)
//   oracle_slam / oracle_msckf   the oracle's SLAM-only and MSCKF-only parts (5b)
// Propagation everywhere: the planner's synthetic static model (tabulated OpenVINS F, Qd).
//
// Usage: gain_ladder <tasks.txt>   Output: one line per (task, rung):
//   RES <id> <rung> <ld_now> <ld_prior> <ld_post> <total> <n_meas> <n_wp>
// tasks.txt (written by docker/tools/closed_loop/day5/ladder_extract.py):
//   TASK <id> <jc.cdr path> <goal x> <goal y> <goal yaw> <drone yaw at decision> <spacing> <height> <speed> <yaw_rate>
//   WP <t rel> <x> <y> <z> <qx> <qy> <qz> <qw> <dense 0|1> <n ids> <id>...     (odometry pose, Hamilton q_ItoG)
//   NEWLM <id> <x> <y> <z> <t_first rel> <t_last rel>
//   MSCKF <x> <y> <z> <t_used rel>
//   END
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
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
using Eigen::Vector2d;
using Eigen::Vector3d;

struct PropagatorAccess : public ov_msckf::Propagator {
  using ov_msckf::Propagator::Propagator;
  using ov_msckf::Propagator::predict_and_compute;
};

struct WP {
  double t;
  asi::Pose pose;
  bool dense;
  std::set<long long> ids;
};
struct NewLm { long long id; Vector3d p; double t0, t1; };
struct Msckf { Vector3d p; double tu; };
struct Task {
  std::string id, cdr;
  double gx, gy, gyaw, yaw0, spacing, height, speed, yaw_rate;
  std::vector<WP> wps;
  std::vector<NewLm> newlm;
  std::vector<Msckf> msckf;
};

static active_slam_msgs::msg::JointCovariance load_cdr(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  rclcpp::SerializedMessage sm(buf.size());
  auto& rm = sm.get_rcl_serialized_message();
  std::memcpy(rm.buffer, buf.data(), buf.size());
  rm.buffer_length = buf.size();
  active_slam_msgs::msg::JointCovariance m;
  rclcpp::Serialization<active_slam_msgs::msg::JointCovariance>().deserialize_message(&sm, &m);
  return m;
}

int main(int argc, char** argv) {
  if (argc < 2) { std::cerr << "usage: gain_ladder tasks.txt\n"; return 2; }
  std::ifstream in(argv[1]);
  std::vector<Task> tasks;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream s(line);
    std::string k; s >> k;
    if (k == "TASK") {
      Task t; s >> t.id >> t.cdr >> t.gx >> t.gy >> t.gyaw >> t.yaw0 >> t.spacing >> t.height >> t.speed >> t.yaw_rate;
      tasks.push_back(t);
    } else if (k == "WP") {
      WP w; double x, y, z, qx, qy, qz, qw; int d, n;
      s >> w.t >> x >> y >> z >> qx >> qy >> qz >> qw >> d >> n;
      w.dense = d != 0;
      for (int i = 0; i < n; ++i) { long long id; s >> id; w.ids.insert(id); }
      w.pose.R_GtoI = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix().transpose();
      w.pose.p_IinG = Vector3d(x, y, z);
      tasks.back().wps.push_back(w);
    } else if (k == "NEWLM") {
      NewLm l; s >> l.id >> l.p.x() >> l.p.y() >> l.p.z() >> l.t0 >> l.t1; tasks.back().newlm.push_back(l);
    } else if (k == "MSCKF") {
      Msckf m; s >> m.p.x() >> m.p.y() >> m.p.z() >> m.tu; tasks.back().msckf.push_back(m);
    }
  }
  ov_msckf::NoiseManager nm;  // = the planner's
  nm.sigma_w = 1.6e-4; nm.sigma_w_2 = nm.sigma_w * nm.sigma_w;
  nm.sigma_a = 2.0e-3; nm.sigma_a_2 = nm.sigma_a * nm.sigma_a;
  nm.sigma_wb = 2.0e-6; nm.sigma_wb_2 = nm.sigma_wb * nm.sigma_wb;
  nm.sigma_ab = 3.0e-4; nm.sigma_ab_2 = nm.sigma_ab * nm.sigma_ab;
  PropagatorAccess prop(nm, 9.81);
  const double p_used = 0.92, sigma_px = 1.0, max_clones_window = 11.0 / 15.0;
  const asi::ConstantNoiseModel noise(sigma_px / std::sqrt(p_used));

  for (const auto& T : tasks) {
    const auto msg = load_cdr(T.cdr);
    const auto P = asi::problem_from_msg(msg);
    if (!P.ok) { std::cout << "ERR " << T.id << " " << P.why << "\n"; continue; }
    const int ic = P.imu_col;
    bool okn = false;
    const double ld_now = active_slam_sim::logdet_spd(P.Sigma.block(ic, ic, 6, 6), &okn);
    if (!okn) { std::cout << "ERR " << T.id << " ld_now\n"; continue; }
    // static propagation, tabulated (as score_path)
    const double dt = 0.02;
    MatrixXd F1, Q1;
    {
      ov_msckf::StateOptions so; so.num_cameras = 2;
      auto st = std::make_shared<ov_msckf::State>(so);
      Eigen::Matrix<double, 16, 1> x = Eigen::Matrix<double, 16, 1>::Zero();
      for (int i = 0; i < 4; i++) x(i) = msg.imu_pose.q_gtoi[i];
      for (int i = 0; i < 3; i++) x(4 + i) = msg.imu_pose.p_iing[i];
      st->_imu->set_value(x); st->_imu->set_fej(x);
      ov_core::ImuData a, b; a.wm.setZero(); b.wm.setZero();
      a.am = P.current.R_GtoI * Vector3d(0, 0, 9.81); b.am = a.am; a.timestamp = 0; b.timestamp = dt;
      prop.predict_and_compute(st, a, b, F1, Q1);
    }
    double Tmax = 30.0;
    for (const auto& w : T.wps) Tmax = std::max(Tmax, w.t + 1.0);
    const int nsteps = (int)std::ceil(Tmax / dt) + 1;
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
    const int N0 = (int)P.Sigma.rows();
    const int PLACE = 1 << 28;

    // evaluate: waypoints (pose, t), per-waypoint landmark filter, optional new landmarks + msckf
    struct In { std::vector<active_slam_sim::Waypoint> W; std::vector<const std::set<long long>*> keep;
                bool use_t0 = true, use_new = false, use_msckf = false; };
    auto eval = [&](const std::string& rung, const In& I) {
      const auto& W = I.W;
      if (W.empty()) return;
      std::vector<asi::PredictedMeasurement> m_real;
      int n_meas = 0;
      if (I.use_t0)
        for (size_t k = 0; k < W.size(); ++k)
          for (const auto& cam : P.cameras)
            for (const auto& lm : P.landmarks) {
              if (I.keep[k] && !I.keep[k]->count(lm.id)) continue;
              asi::PredictedMeasurement pm;
              if (asi::predict_measurement(P.current, W[k].pose, PLACE + 6 * (int)k, cam, lm, noise, nullptr, &pm)) m_real.push_back(pm);
            }
      // reduce to IMU + touched landmark columns
      std::vector<char> mark(N0, 0);
      for (int c = 0; c < 15; ++c) mark[ic + c] = 1;
      for (const auto& pm : m_real)
        for (const auto& b : pm.blocks)
          if (b.col < N0) for (int c = 0; c < b.J.cols(); ++c) mark[b.col + c] = 1;
      std::vector<int> idx, map;
      for (int c = 0; c < N0; ++c) if (mark[c]) idx.push_back(c);
      MatrixXd Sr = active_slam_sim::marginal(P.Sigma, idx, &map);
      // new landmarks (weak 10 m prior, global XYZ columns appended)
      std::vector<asi::LandmarkLinearization> nl;
      std::vector<std::pair<double, double>> life;
      if (I.use_new)
        for (const auto& l : T.newlm) {
          asi::LandmarkLinearization lm; lm.is_virtual = true; lm.p_FinG = l.p; lm.id = (int)l.id;
          nl.push_back(lm); life.push_back({l.t0 - max_clones_window, l.t1});  // pre-init track ~ one window
        }
      const int nr = (int)Sr.rows(), nn = (int)nl.size();
      if (nn) Sr = asi::augment_with_virtual_landmarks(Sr, nn, 10.0);
      for (int j = 0; j < nn; ++j) nl[j].rep = {{PLACE / 2 + 3 * j, Eigen::Matrix3d::Identity()}};
      std::vector<asi::PredictedMeasurement> m_new;
      for (size_t k = 0; k < W.size(); ++k)
        for (int j = 0; j < nn; ++j) {
          if (W[k].t < life[j].first || W[k].t > life[j].second) continue;
          for (const auto& cam : P.cameras) {
            asi::PredictedMeasurement pm;
            if (asi::predict_measurement(P.current, W[k].pose, PLACE + 6 * (int)k, cam, nl[j], noise, nullptr, &pm)) m_new.push_back(pm);
          }
        }
      std::vector<double> times;
      for (const auto& w : W) times.push_back(w.t);
      const auto PP = active_slam_sim::build_path_prior(Sr, map[ic], times, pf);
      auto remap = [&](std::vector<asi::PredictedMeasurement>& ms) {
        for (auto& pm : ms)
          for (auto& b : pm.blocks) {
            if (b.col >= PLACE) b.col = PP.clone_cols[(b.col - PLACE) / 6];
            else if (b.col >= PLACE / 2) b.col = nr + (b.col - PLACE / 2);
            else b.col = map[b.col];
          }
      };
      remap(m_real); remap(m_new);
      const int n = (int)PP.S.rows();
      MatrixXd J = active_slam_sim::information(m_real, n) + active_slam_sim::information(m_new, n);
      n_meas = (int)(m_real.size() + m_new.size());
      // MSCKF features: per-feature Schur complement onto the observing clones
      if (I.use_msckf) {
        for (const auto& f : T.msckf) {
          asi::LandmarkLinearization lm; lm.is_virtual = true; lm.p_FinG = f.p; lm.rep = {{PLACE / 2, Eigen::Matrix3d::Identity()}};
          std::vector<asi::PredictedMeasurement> ms;
          for (size_t k = 0; k < W.size(); ++k) {
            if (W[k].t > f.tu + 1e-6 || W[k].t < f.tu - max_clones_window) continue;
            for (const auto& cam : P.cameras) {
              asi::PredictedMeasurement pm;
              if (asi::predict_measurement(P.current, W[k].pose, PLACE + 6 * (int)k, cam, lm, noise, nullptr, &pm)) ms.push_back(pm);
            }
          }
          if (ms.size() < 3) continue;  // needs >= 2 views (x 2 rows) to constrain the feature
          // A = sum Hl' W Hl, B = Hx' W Hl, D = Hx' W Hx over the clone columns
          Eigen::Matrix3d A = Eigen::Matrix3d::Identity() * 1e-2;  // 10 m prior
          std::map<int, Eigen::Matrix<double, 6, 3>> B;
          std::map<std::pair<int, int>, Eigen::Matrix<double, 6, 6>> D;
          for (auto& pm : ms) {
            const double w = 1.0 / (pm.sigma * pm.sigma);
            Eigen::MatrixXd Hl; int cc = -1; Eigen::MatrixXd Hx;
            for (auto& b : pm.blocks) {
              if (b.col >= PLACE) { cc = PP.clone_cols[(b.col - PLACE) / 6]; Hx = b.J; }
              else if (b.col >= PLACE / 2) Hl = b.J;
            }
            if (cc < 0 || Hl.size() == 0) continue;
            A += w * Hl.transpose() * Hl;
            if (!B.count(cc)) B[cc].setZero();
            B[cc] += w * Hx.transpose() * Hl;
            auto key = std::make_pair(cc, cc);
            if (!D.count(key)) D[key].setZero();
            D[key] += w * Hx.transpose() * Hx;
          }
          const Eigen::Matrix3d Ai = A.inverse();
          for (auto& d : D) J.block(d.first.first, d.first.second, 6, 6) += d.second;
          for (auto& bi : B)
            for (auto& bj : B) J.block(bi.first, bj.first, 6, 6) -= bi.second * Ai * bj.second.transpose();
          n_meas += (int)ms.size();
        }
      }
      MatrixXd Tp = MatrixXd::Zero(6, n);
      Tp.block(0, PP.clone_cols.back(), 6, 6).setIdentity();
      const auto g = active_slam_sim::pose_gain_split(ld_now, PP.S, Tp, J);
      {  // roll/pitch-like block: local x/y orientation rows of the end pose (as the OV stage log's ld2)
        MatrixXd T2 = MatrixXd::Zero(2, n);
        T2.block(0, PP.clone_cols.back(), 2, 2).setIdentity();
        const auto g2 = active_slam_sim::pose_gain_split(active_slam_sim::logdet_spd(P.Sigma.block(ic, ic, 2, 2)), PP.S, T2, J);
        std::cout << "RP2 " << T.id << " " << rung << " " << g2.logdet_now << " " << g2.logdet_prior << " " << g2.logdet_post << "\n";
      }
      std::cout << "RES " << T.id << " " << rung << " " << ld_now << " " << g.logdet_prior << " " << g.logdet_post << " "
                << (g.ok ? g.total() : NAN) << " " << n_meas << " " << W.size() << "\n";
    };

    // ---- rung inputs ----
    const Vector2d from(P.current.p_IinG.x(), P.current.p_IinG.y());
    In plan;
    plan.W = active_slam_sim::executor_profile(from, T.yaw0, Vector2d(T.gx, T.gy), T.gyaw, T.height, std::max(T.speed, 0.05), T.yaw_rate, T.spacing);
    plan.keep.assign(plan.W.size(), nullptr);
    eval("plan_real", plan);
    // flown path: sparse (by arc length, the planner's spacing, plus the last pose) and dense (clone times)
    In act, actw, dense, oracle;
    double s = 0, next = T.spacing > 0 ? T.spacing : 1e9;
    for (size_t k = 0; k < T.wps.size(); ++k) {
      const auto& w = T.wps[k];
      if (k > 0) s += (w.pose.p_IinG - T.wps[k - 1].pose.p_IinG).head<2>().norm();
      active_slam_sim::Waypoint x; x.t = w.t; x.pose = w.pose;
      if (s >= next || k + 1 == T.wps.size()) {
        act.W.push_back(x); act.keep.push_back(nullptr);
        actw.W.push_back(x); actw.keep.push_back(&w.ids);
        while (next <= s) next += T.spacing;
      }
      if (w.dense) { dense.W.push_back(x); dense.keep.push_back(&w.ids); }
    }
    eval("act_real", act);
    eval("act_real_win", actw);
    eval("dense_win", dense);
    oracle = dense; oracle.use_new = true; oracle.use_msckf = true;
    eval("oracle", oracle);
    In os = dense; os.use_new = true;
    eval("oracle_slam", os);
    In om = dense; om.use_t0 = false; om.use_msckf = true;
    eval("oracle_msckf", om);
  }
  return 0;
}
