// exploration_planner (PATCHES s64) -- both planners, one pipeline.
//   planner_type: frontier  baseline: nearest reachable frontier cluster; ignores covariance
//                 ig        frontier candidates scored by predicted information gain
//                           (OpenVINS propagation + constant p(used) + virtual frontier
//                           landmarks), ranked by posterior log-det (MATH_TO_CODE.md)
// in   /ov_msckf/odomimu, ESDF slice (nvblox interface), ~frontiers of the mapper,
//      /openvins/joint_covariance (ig only)
// out  ~/goal (geometry_msgs/PoseStamped, OpenVINS global frame, IMU position, yaw
//      facing the frontier), ~/status (std_msgs/String: exploring|done), ~/decision
//      (std_msgs/String, one JSON line per replan: candidates and scores, for the log)
// Candidates: frontier voxels in the altitude band are clustered on a 2D grid; per
// cluster the viewpoint is the centroid pulled back toward the drone by `standoff`
// and further until it is in observed free space with ESDF distance >= r_safe; the
// straight path to it must keep ESDF distance >= r_path. Flight is at constant
// height (the ESDF slice height), like the nvblox slice planner on the Jetson.

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <atomic>
#include <chrono>
#include <random>
#include <thread>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "active_slam_information/information_gain.hpp"
#include "active_slam_sim/candidate_scoring.hpp"
#include "active_slam_sim/coverage_gain.hpp"
#include "active_slam_information/ros/joint_cov_problem.hpp"
#include "active_slam_information/view_geometry.hpp"
#include "active_slam_msgs/msg/joint_covariance.hpp"
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

struct Candidate {
  Vector2d xy;
  double yaw = 0, path_len = 0;
  int cluster_size = 0;
  Vector3d centroid = Vector3d::Zero();
  std::vector<Vector3d> frontier_pts;
  double score = NAN, delta = NAN, cost = 0, delta_real = NAN, delta_virt = NAN;
  int n_meas_real = 0, n_meas_virt = 0;
  double dI_pose = NAN, coverage_gain = NAN, score_pc = NAN;  // pose_cov mode terms (day 2)
  // day 3: dI_pose = dI_meas (a, measurement gain) + dI_prop (b, propagation/travel term);
  // dI_meas_real: (a) from real SLAM landmarks alone. vis: (landmark, camera) pairs at the
  // endpoint by class [visible, fov, range, occluded-by-occupied]; unk_vis: visible pairs
  // whose ray crosses unknown space (unknown does not occlude); lm_vis: landmarks with >= 1
  // visible pair (no occlusion test, as scored).
  double dI_meas = NAN, dI_prop = NAN, dI_meas_real = NAN;
  int vis[4] = {0, 0, 0, 0}, unk_vis = 0, lm_vis = 0, lm_vis_occ = 0, n_wp = 0;
  std::pair<int, int> tile{0, 0};
};

class ExplorationPlanner : public rclcpp::Node {
 public:
  ExplorationPlanner() : Node("exploration_planner") {
    type_ = declare_parameter("planner_type", std::string("frontier"));
    height_ = declare_parameter("flight_height", 1.5);  // OpenVINS-frame z of the IMU
    standoff_ = declare_parameter("standoff", 1.2);
    // Margins from the IMU position (planned point) to the nearest occupied voxel:
    // rotor tips reach ~0.33 m from base_link, the IMU sits 0.11 m ahead of it, plus
    // a voxel and estimation margin (s64: 0.5 m left ~6 cm and a home leg touched a box).
    r_safe_ = declare_parameter("r_safe", 1.0);
    r_path_ = declare_parameter("r_path", 0.8);
    // Speed-scaled margin (day 2 step 5): r_eff = r + margin_time * cruise_speed -- the
    // distance covered during a reaction time at the commanded speed is added to both the
    // path and the viewpoint clearance.
    margin_time_ = declare_parameter("margin_time", 1.0);
    min_cluster_ = declare_parameter("min_cluster", 6);
    cell_ = declare_parameter("cluster_cell", 1.5);
    speed_ = declare_parameter("cruise_speed", 0.5);
    reach_tol_ = declare_parameter("reach_tolerance", 0.35);
    goal_timeout_ = declare_parameter("goal_timeout", 30.0);
    p_used_ = declare_parameter("p_used", 0.92);       // constant usage correction (s63)
    sigma_px_ = declare_parameter("sigma_px", 1.0);
    sigma_l_ = declare_parameter("sigma_virtual", 1.0); // prior std of virtual frontier landmarks [m]
    max_virtual_ = declare_parameter("max_virtual_per_cluster", 8);
    done_after_ = declare_parameter("done_after_empty_replans", 6);
    grace_ = declare_parameter("grace_s", 10.0);
    max_attempts_ = declare_parameter("max_tile_attempts", 2);
    // Day 3 candidate set. yaw_samples 1 = old (one yaw, facing the frontier centroid);
    // N > 1 = N yaws per position, facing + k 2pi/N. path_spacing 0 = old (endpoint only);
    // > 0 = pose_cov gain integrated over waypoints this far apart along the executor's path.
    yaw_samples_ = declare_parameter("yaw_samples", 1);
    path_spacing_ = declare_parameter("path_spacing", 0.0);
    // day 8: measurement model of real landmarks in the path score. "white" (default, old behaviour):
    // independent sigma_px per sample. "random_walk": per-track random-walk KLT error (MATH_TO_CODE.md
    // "Random-walk tracking error"): first sample rw_sigma0 px, then rw_q2 px^2 per camera frame at
    // rw_frame_rate frames/s (the tracking rate).
    meas_model_ = declare_parameter("meas_model", std::string("white"));
    rw_sigma0_ = declare_parameter("rw_sigma0", 0.3);
    rw_q2_ = declare_parameter("rw_q2", 0.02);
    rw_frame_rate_ = declare_parameter("rw_frame_rate", 15.0);
    yaw_rate_ = declare_parameter("yaw_rate", 0.6);  // the executor's yaw rate limit
    threads_ = declare_parameter("score_threads", 8);
    // Day 4 optimizer's-curse check (diagnostic only): with this probability fly a uniformly
    // random scored candidate instead of the argmax; logged as "pick":"random".
    random_pick_ = declare_parameter("random_pick", 0.0);
    rng_.seed((unsigned)declare_parameter("random_seed", 1));  // airborne this long before "done" may be declared
    const auto esdf_topic = declare_parameter("esdf_topic", std::string("/nvblox_node/static_esdf_pointcloud"));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter("odom_topic", std::string("/ov_msckf/odomimu")), 10,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          odom_ = m;
          const auto& q = m->pose.pose.orientation;
          drone_yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        });
    sub_esdf_ = create_subscription<sensor_msgs::msg::PointCloud2>(esdf_topic, 2, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { on_esdf(m); });
    sub_front_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter("frontier_topic", std::string("/octomap_mapper/frontiers")), 2,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { frontiers_ = m; });
    lambda_ = declare_parameter("lambda", 0.1);  // pose_cov: nats per m^3 of expected new mapped volume
    sub_coarse_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/octomap_mapper/occupancy_coarse", 2, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { on_coarse(m); });
    if (type_ == "ig" || type_ == "pose_cov")
      sub_jc_ = create_subscription<active_slam_msgs::msg::JointCovariance>(
          "/openvins/joint_covariance", 10, [this](active_slam_msgs::msg::JointCovariance::SharedPtr m) {
            if (m->stage.empty() || m->stage == "post") jc_ = m;
          });
    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/goal", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    pub_dec_ = create_publisher<std_msgs::msg::String>("~/decision", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { tick(); });
    ov_msckf::NoiseManager nm;
    nm.sigma_w = 1.6e-4; nm.sigma_w_2 = nm.sigma_w * nm.sigma_w;
    nm.sigma_a = 2.0e-3; nm.sigma_a_2 = nm.sigma_a * nm.sigma_a;
    nm.sigma_wb = 2.0e-6; nm.sigma_wb_2 = nm.sigma_wb * nm.sigma_wb;
    nm.sigma_ab = 3.0e-4; nm.sigma_ab_2 = nm.sigma_ab * nm.sigma_ab;
    prop_ = std::make_unique<PropagatorAccess>(nm, 9.81);
    RCLCPP_INFO(get_logger(), "exploration_planner up: type=%s height=%.2f standoff=%.2f r_safe=%.2f sigma_virtual=%.6g p_used=%.3f lambda=%.6g yaw_samples=%d path_spacing=%.2f",
                type_.c_str(), height_, standoff_, r_safe_, sigma_l_, p_used_, lambda_, yaw_samples_, path_spacing_);
  }

 private:
  void on_coarse(const sensor_msgs::msg::PointCloud2::SharedPtr m) {
    coarse_.clear();
    sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x"), y(*m, "y"), z(*m, "z"), o(*m, "occ");
    for (; x != x.end(); ++x, ++y, ++z, ++o) coarse_.set(*x, *y, *z, *o > 0.5f);
  }

  // ---------------- ESDF slice lookup ----------------
  void on_esdf(const sensor_msgs::msg::PointCloud2::SharedPtr m) {
    std::map<std::pair<int, int>, double> g;
    sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x"), y(*m, "y"), d(*m, "intensity");
    double res = 0.1;
    for (; x != x.end(); ++x, ++y, ++d) g[{(int)std::lround(*x / res), (int)std::lround(*y / res)}] = *d;
    esdf_ = std::move(g);
    esdf_res_ = res;
  }
  // distance at (x,y); NAN if unobserved
  double esdf_at(const Vector2d& p) const {
    auto it = esdf_.find({(int)std::lround(p.x() / esdf_res_), (int)std::lround(p.y() / esdf_res_)});
    return it == esdf_.end() ? NAN : it->second;
  }
  bool path_clear(const Vector2d& a, const Vector2d& b) const {
    const double L = (b - a).norm();
    for (double s = 0; s <= L; s += 0.5 * esdf_res_) {
      const double d = esdf_at(a + (b - a) * (s / std::max(L, 1e-9)));
      if (!(d >= r_path_ + margin_time_ * speed_)) return false;  // unobserved or too close (speed-scaled)
    }
    return true;
  }

  // ---------------- frontier clusters -> candidates ----------------
  std::vector<Candidate> candidates(const Vector2d& drone) {
    std::vector<Candidate> out;
    if (!frontiers_) return out;
    std::map<std::pair<int, int>, std::vector<Vector3d>> cells;
    sensor_msgs::PointCloud2ConstIterator<float> x(*frontiers_, "x"), y(*frontiers_, "y"), z(*frontiers_, "z");
    for (; x != x.end(); ++x, ++y, ++z) cells[{(int)std::floor(*x / cell_), (int)std::floor(*y / cell_)}].push_back(Vector3d(*x, *y, *z));
    // Fixed-size tiles (cluster_cell, default 1.5 m): connected components merged the
    // whole rim of the observed region into one cluster with a meaningless centroid.
    for (auto& kv : cells) {
      if (attempts_[kv.first] >= max_attempts_) continue;  // targeted enough: unresolvable from here
      Candidate cd;
      cd.tile = kv.first;
      for (auto& p : kv.second) { cd.frontier_pts.push_back(p); cd.centroid += p; }
      cd.cluster_size = (int)cd.frontier_pts.size();
      if (cd.cluster_size < min_cluster_) continue;
      cd.centroid /= cd.cluster_size;
      // viewpoint: pull back toward the drone until safe, observed, reachable;
      // frontier closer than the standoff -> turn in place to face it
      Vector2d c2 = cd.centroid.head<2>(), dir = (c2 - drone);
      const double L = dir.norm();
      if (L < 1e-3) continue;
      dir /= L;
      bool ok = false;
      if (L <= standoff_ + 0.3) {
        cd.xy = drone; ok = true;
      } else {
        for (double back = standoff_; back <= L; back += 0.2) {
          const Vector2d v = c2 - dir * back;
          const double d = esdf_at(v);
          if (d >= r_safe_ + margin_time_ * speed_ && path_clear(drone, v)) { cd.xy = v; ok = true; break; }
        }
      }
      if (!ok) continue;
      const double facing = std::atan2(c2.y() - cd.xy.y(), c2.x() - cd.xy.x());
      cd.path_len = (cd.xy - drone).norm();
      const int ny = std::max(1, yaw_samples_);
      for (int k = 0; k < ny; ++k) {
        Candidate cy = cd;
        cy.yaw = std::remainder(facing + 2.0 * M_PI * k / ny, 2 * M_PI);
        const double dyaw = std::fabs(std::remainder(cy.yaw - drone_yaw_, 2 * M_PI));
        if (cy.path_len < 0.3 && dyaw < 20.0 * M_PI / 180.0) continue;  // already looking there
        cy.cost = cy.path_len + 0.5 * dyaw;  // m; 0.5 m per rad of turning
        out.push_back(cy);
      }
    }
    return out;
  }

  // ---------------- IG score: propagation + p(used) + virtual frontier landmarks ----------------
  void score_ig(std::vector<Candidate>& cands) {
    if (!jc_) return;
    const auto P = asi::problem_from_msg(*jc_);
    if (!P.ok) return;
    const Vector3d g(0, 0, 9.81);
    for (auto& cd : cands) {
      // OpenVINS propagation over the travel time with a synthetic constant-velocity IMU
      ov_msckf::StateOptions so;
      so.num_cameras = 2;
      auto st = std::make_shared<ov_msckf::State>(so);
      Eigen::Matrix<double, 16, 1> x = Eigen::Matrix<double, 16, 1>::Zero();
      for (int i = 0; i < 4; i++) x(i) = jc_->imu_pose.q_gtoi[i];
      for (int i = 0; i < 3; i++) x(4 + i) = jc_->imu_pose.p_iing[i];
      st->_imu->set_value(x);
      st->_imu->set_fej(x);
      const double T = cd.path_len / std::max(speed_, 0.05);
      const double dt = 0.02;
      MatrixXd S = P.Sigma;
      const int ic = P.imu_col;
      ov_core::ImuData a, b;
      a.wm.setZero(); b.wm.setZero();
      a.am = P.current.R_GtoI * g; b.am = a.am;  // level, unaccelerated: specific force = gravity
      for (double t = 0; t < T; t += dt) {
        a.timestamp = t; b.timestamp = std::min(t + dt, T);
        Eigen::MatrixXd F, Qd;
        prop_->predict_and_compute(st, a, b, F, Qd);
        const int d = (int)F.rows();
        MatrixXd rows = F * S.middleRows(ic, d); S.middleRows(ic, d) = rows;
        MatrixXd cols = S.middleCols(ic, d) * F.transpose(); S.middleCols(ic, d) = cols;
        S.block(ic, ic, d, d) += Qd;
      }
      const double ld_prop = active_slam_sim::logdet_spd(S.block(ic, ic, 6, 6));  // pose prior at the candidate, no measurements
      // clone at the candidate
      const int n = (int)S.rows();
      MatrixXd S2 = MatrixXd::Zero(n + 6, n + 6);
      S2.topLeftCorner(n, n) = S;
      S2.block(n, 0, 6, n) = S.middleRows(ic, 6);
      S2.block(0, n, n, 6) = S.middleCols(ic, 6);
      S2.block(n, n, 6, 6) = S.block(ic, ic, 6, 6);
      S = 0.5 * (S2 + S2.transpose());
      const int clone_col = n;
      // virtual frontier landmarks of THIS cluster (subsampled)
      std::vector<asi::LandmarkLinearization> virt;
      const int step = std::max(1, cd.cluster_size / max_virtual_);
      for (int k = 0; k < cd.cluster_size && (int)virt.size() < max_virtual_; k += step) {
        asi::LandmarkLinearization lm;
        lm.is_virtual = true;
        lm.p_FinG = cd.frontier_pts[k];
        virt.push_back(lm);
      }
      const int nv = (int)virt.size();
      const MatrixXd Sa = asi::augment_with_virtual_landmarks(S, nv, sigma_l_);
      for (int k = 0; k < nv; ++k) virt[k].rep = {{(int)S.rows() + 3 * k, Eigen::Matrix3d::Identity()}};
      MatrixXd T0 = P.T_metric();
      MatrixXd Tp = MatrixXd::Zero(T0.rows(), S.rows());
      Tp.leftCols(T0.cols()) = T0;
      const MatrixXd Ta = asi::augment_scoring_map(Tp, (int)S.rows(), nv);
      // candidate pose: IMU at (xy, height) with the candidate yaw, level
      asi::Pose cp;
      const Eigen::Matrix3d R_ItoG = Eigen::AngleAxisd(cd.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      cp.R_GtoI = R_ItoG.transpose();
      cp.p_IinG = Vector3d(cd.xy.x(), cd.xy.y(), height_);
      const asi::ConstantNoiseModel noise(sigma_px_ / std::sqrt(p_used_));  // expected info = p(used) x info
      std::vector<asi::PredictedMeasurement> meas, m_real, m_virt;
      auto add = [&](const asi::LandmarkLinearization& lm, std::vector<asi::PredictedMeasurement>& part) {
        for (const auto& cam : P.cameras) {
          asi::PredictedMeasurement pm;
          if (asi::predict_measurement(cp, cp, clone_col, cam, lm, noise, nullptr, &pm)) { meas.push_back(pm); part.push_back(pm); }
        }
      };
      for (const auto& lm : P.landmarks) add(lm, m_real);
      for (const auto& lm : virt) add(lm, m_virt);
      const asi::InformationPrior prior(Sa, Ta);
      if (!prior.ok()) continue;
      const auto gn = asi::evaluate(prior, meas);
      if (!gn.ok) continue;
      cd.score = gn.posterior_logdet;
      cd.delta = gn.delta;
      // Objective decomposition (overnight Stage 1a): gain from real SLAM-landmark
      // measurements alone and from virtual frontier landmarks alone, same prior.
      // Not additive (log-det); share = real / (real + virt).
      const auto gr = asi::evaluate(prior, m_real), gv = asi::evaluate(prior, m_virt);
      cd.delta_real = gr.ok ? gr.delta : NAN;
      cd.delta_virt = gv.ok ? gv.delta : NAN;
      cd.n_meas_real = (int)m_real.size();
      cd.n_meas_virt = (int)m_virt.size();
      // pose_cov terms (day 2, MATH_TO_CODE.md "pose-marginal objective"):
      // dI_pose = log det Sigma_pose(now) - log det Sigma_pose+(candidate), T selects the
      // 6-dof IMU pose (landmarks, real and virtual, marginalized out); prior is the CURRENT
      // pose covariance, so the propagation cost of getting there counts against the gain.
      {
        MatrixXd Tpose = MatrixXd::Zero(6, Sa.rows());
        Tpose.block(0, ic, 6, 6).setIdentity();
        const asi::InformationPrior pprior(Sa, Tpose);
        if (pprior.ok()) {
          const auto gp = asi::evaluate(pprior, meas);
          Eigen::LLT<MatrixXd> l0(P.Sigma.block(ic, ic, 6, 6));
          if (gp.ok && l0.info() == Eigen::Success) {
            const double ld_now = 2.0 * l0.matrixL().toDenseMatrix().diagonal().array().log().sum();
            cd.dI_pose = ld_now - gp.posterior_logdet;
            cd.dI_prop = ld_now - ld_prop;               // (b) day 3
            cd.dI_meas = ld_prop - gp.posterior_logdet;  // (a)
            const auto gpr = asi::evaluate(pprior, m_real);
            if (gpr.ok) cd.dI_meas_real = ld_prop - gpr.posterior_logdet;
          }
        }
        vis_diag(cd, P, cp);
        const auto& cam = P.cameras.front();
        const Eigen::Matrix3d R_GC = R_ItoG * cam.R_ItoC.transpose();
        const Vector3d p_GC = cp.p_IinG + R_ItoG * (-cam.R_ItoC.transpose() * cam.p_IinC);
        cd.coverage_gain = active_slam_sim::unknown_volume_in_view(coarse_, R_GC, p_GC, frustum_);
        cd.score_pc = cd.dI_pose + lambda_ * cd.coverage_gain;
      }
    }
  }

  // Step 1 diagnosis (day 3): classify every (real landmark, camera) pair at the endpoint.
  void vis_diag(Candidate& cd, const asi::JointCovProblem& P, const asi::Pose& cp) const {
    for (const auto& lm : P.landmarks) {
      bool any = false, any_occ = false;
      for (const auto& cam : P.cameras) {
        int nu = 0;
        const auto v = active_slam_sim::classify_visibility(cp, cam, lm.p_FinG, &coarse_, &nu);
        cd.vis[(int)v]++;
        if (v == active_slam_sim::Vis::VISIBLE && nu > 0) cd.unk_vis++;
        if (v == active_slam_sim::Vis::VISIBLE) any = true;
        if (v == active_slam_sim::Vis::OCCLUDED) any_occ = true;
      }
      cd.lm_vis += any || any_occ;  // visible as scored (no occlusion test)
      cd.lm_vis_occ += any;         // visible after an occupied-cell occlusion test
    }
  }

  // Day 3 path-integrated pose_cov score (candidate_scoring.hpp, MATH_TO_CODE.md day 3).
  void score_path(std::vector<Candidate>& cands) {
    if (!jc_) return;
    const auto P = asi::problem_from_msg(*jc_);
    if (!P.ok) return;
    const int ic = P.imu_col;
    bool okn = false;
    const double ld_now = active_slam_sim::logdet_spd(P.Sigma.block(ic, ic, 6, 6), &okn);
    if (!okn) return;
    // One-step OpenVINS propagation (synthetic level, unaccelerated IMU): the state does not
    // move (v = 0, w = 0, biases 0), so F and Qd are the same every step -- tabulate powers.
    const double dt = 0.02;
    Eigen::MatrixXd F1, Q1;
    {
      ov_msckf::StateOptions so;
      so.num_cameras = 2;
      auto st = std::make_shared<ov_msckf::State>(so);
      Eigen::Matrix<double, 16, 1> x = Eigen::Matrix<double, 16, 1>::Zero();
      for (int i = 0; i < 4; i++) x(i) = jc_->imu_pose.q_gtoi[i];
      for (int i = 0; i < 3; i++) x(4 + i) = jc_->imu_pose.p_iing[i];
      st->_imu->set_value(x);
      st->_imu->set_fej(x);
      ov_core::ImuData a, b;
      a.wm.setZero(); b.wm.setZero();
      a.am = P.current.R_GtoI * Vector3d(0, 0, 9.81); b.am = a.am;
      a.timestamp = 0; b.timestamp = dt;
      prop_->predict_and_compute(st, a, b, F1, Q1);
    }
    double Tmax = 0;
    std::vector<std::vector<active_slam_sim::Waypoint>> wps(cands.size());
    for (size_t i = 0; i < cands.size(); ++i) {
      wps[i] = active_slam_sim::executor_profile(Vector2d(P.current.p_IinG.x(), P.current.p_IinG.y()), drone_yaw_, cands[i].xy, cands[i].yaw,
                                                 height_, std::max(speed_, 0.05), yaw_rate_, path_spacing_);
      Tmax = std::max(Tmax, wps[i].back().t);
    }
    const int nsteps = (int)std::ceil(Tmax / dt) + 1;
    std::vector<MatrixXd> Phi(nsteps + 1), Qk(nsteps + 1);
    Phi[0] = MatrixXd::Identity(15, 15); Qk[0] = MatrixXd::Zero(15, 15);
    for (int k = 1; k <= nsteps; ++k) { Phi[k] = F1 * Phi[k - 1]; Qk[k] = F1 * Qk[k - 1] * F1.transpose() + Q1; }
    const active_slam_sim::PropagateFn prop = [&](MatrixXd& S, int c, double t0, double t1) {
      const int k = std::min(nsteps, (int)std::lround(t1 / dt)) - std::min(nsteps, (int)std::lround(t0 / dt));
      if (k <= 0) return;
      MatrixXd rows = Phi[k] * S.middleRows(c, 15); S.middleRows(c, 15) = rows;
      MatrixXd cols = S.middleCols(c, 15) * Phi[k].transpose(); S.middleCols(c, 15) = cols;
      S.block(c, c, 15, 15) += Qk[k];
    };
    const asi::ConstantNoiseModel noise(sigma_px_ / std::sqrt(p_used_));
    const int N0 = (int)P.Sigma.rows();
    const int PLACE = 1 << 28;  // placeholder column for clone k: PLACE + 6k
    auto one = [&](size_t i) {
      Candidate& cd = cands[i];
      const auto& W = wps[i];
      cd.n_wp = (int)W.size();
      // virtual frontier landmarks of this cluster (as the endpoint mode)
      std::vector<asi::LandmarkLinearization> virt;
      const int step = std::max(1, cd.cluster_size / max_virtual_);
      for (int k = 0; k < cd.cluster_size && (int)virt.size() < max_virtual_; k += step) {
        asi::LandmarkLinearization lm;
        lm.is_virtual = true;
        lm.p_FinG = cd.frontier_pts[k];
        virt.push_back(lm);
      }
      for (int k = 0; k < (int)virt.size(); ++k) virt[k].rep = {{PLACE / 2 + 3 * k, Eigen::Matrix3d::Identity()}};
      // geometry pass, original columns; the R_delta map uses the propagated attitude (= now)
      std::vector<asi::PredictedMeasurement> m_real, m_virt;
      std::vector<std::array<int, 3>> key_real;  // (landmark index, camera index, waypoint) per m_real entry
      for (size_t k = 0; k < W.size(); ++k)
        for (size_t ci = 0; ci < P.cameras.size(); ++ci) {
          const auto& cam = P.cameras[ci];
          asi::PredictedMeasurement pm;
          for (size_t li = 0; li < P.landmarks.size(); ++li) {
            const auto& lm = P.landmarks[li];
            if (asi::predict_measurement(P.current, W[k].pose, PLACE + 6 * (int)k, cam, lm, noise, nullptr, &pm)) {
              m_real.push_back(pm); key_real.push_back({(int)li, (int)ci, (int)k});
            }
          }
          for (const auto& lm : virt)
            if (asi::predict_measurement(P.current, W[k].pose, PLACE + 6 * (int)k, cam, lm, noise, nullptr, &pm)) m_virt.push_back(pm);
        }
      cd.n_meas_real = (int)m_real.size();
      cd.n_meas_virt = (int)m_virt.size();
      // reduce to IMU (15) + the real-landmark columns the measurements touch
      std::vector<char> mark(N0, 0);
      for (int c = 0; c < 15; ++c) mark[ic + c] = 1;
      for (const auto& pm : m_real)
        for (const auto& b : pm.blocks)
          if (b.col < N0) for (int c = 0; c < b.J.cols(); ++c) mark[b.col + c] = 1;
      std::vector<int> idx, map;
      for (int c = 0; c < N0; ++c) if (mark[c]) idx.push_back(c);
      MatrixXd Sr = active_slam_sim::marginal(P.Sigma, idx, &map);
      const int nr = (int)Sr.rows(), nv = (int)virt.size();
      Sr = asi::augment_with_virtual_landmarks(Sr, nv, sigma_l_);
      std::vector<double> times;
      for (const auto& w : W) times.push_back(w.t);
      const auto PP = active_slam_sim::build_path_prior(Sr, map[ic], times, prop);
      auto remap = [&](std::vector<asi::PredictedMeasurement>& ms) {
        for (auto& pm : ms)
          for (auto& b : pm.blocks) {
            if (b.col >= PLACE) b.col = PP.clone_cols[(b.col - PLACE) / 6];
            else if (b.col >= PLACE / 2) b.col = nr + (b.col - PLACE / 2);
            else b.col = map[b.col];
          }
      };
      remap(m_real); remap(m_virt);
      const int n = (int)PP.S.rows();
      MatrixXd Tp = MatrixXd::Zero(6, n);
      Tp.block(0, PP.clone_cols.back(), 6, 6).setIdentity();
      MatrixXd Jr;
      if (meas_model_ == "random_walk") {
        // group into tracks: same landmark and camera at consecutive waypoints
        std::map<std::pair<int, int>, std::vector<size_t>> by;
        for (size_t i = 0; i < m_real.size(); ++i) by[{key_real[i][0], key_real[i][1]}].push_back(i);
        Jr = MatrixXd::Zero(n, n);
        for (auto& kv : by) {
          std::vector<asi::PredictedMeasurement> tr; std::vector<double> fr;
          int prev_k = -2;
          auto flush = [&]() { if (!tr.empty()) Jr += active_slam_sim::information_random_walk(tr, n, rw_sigma0_, rw_q2_, fr); tr.clear(); fr.clear(); };
          for (size_t i : kv.second) {
            const int k = key_real[i][2];
            if (k != prev_k + 1) flush();
            fr.push_back(tr.empty() ? 1.0 : (W[k].t - W[prev_k].t) * rw_frame_rate_);
            tr.push_back(m_real[i]); prev_k = k;
          }
          flush();
        }
      } else {
        Jr = active_slam_sim::information(m_real, n);
      }
      const MatrixXd J = Jr + active_slam_sim::information(m_virt, n);
      const auto g = active_slam_sim::pose_gain_split(ld_now, PP.S, Tp, J);
      if (g.ok) {
        cd.dI_pose = g.total(); cd.dI_prop = g.propagation(); cd.dI_meas = g.measurement();
        const auto gr = active_slam_sim::pose_gain_split(ld_now, PP.S, Tp, Jr);
        if (gr.ok) cd.dI_meas_real = gr.measurement();
      }
      asi::Pose cp = W.back().pose;
      vis_diag(cd, P, cp);
      const auto& cam = P.cameras.front();
      const Eigen::Matrix3d R_ItoG = cp.R_GtoI.transpose();
      const Eigen::Matrix3d R_GC = R_ItoG * cam.R_ItoC.transpose();
      const Vector3d p_GC = cp.p_IinG + R_ItoG * (-cam.R_ItoC.transpose() * cam.p_IinC);
      cd.coverage_gain = active_slam_sim::unknown_volume_in_view(coarse_, R_GC, p_GC, frustum_);
      if (std::isfinite(cd.dI_pose)) cd.score_pc = cd.dI_pose + lambda_ * cd.coverage_gain;
    };
    const int nt = std::max(1, std::min<int>(threads_, (int)cands.size()));
    std::vector<std::thread> pool;
    std::atomic<size_t> next{0};
    for (int t = 0; t < nt; ++t)
      pool.emplace_back([&] { for (size_t i; (i = next++) < cands.size();) one(i); });
    for (auto& th : pool) th.join();
  }

  void tick() {
    if (!odom_) return;
    const auto& p = odom_->pose.pose.position;
    const Vector2d drone(p.x, p.y);
    const double now = get_clock()->now().seconds();
    std_msgs::msg::String st;
    st.data = done_ ? "done" : "exploring";
    pub_status_->publish(st);
    if (done_) return;
    // Plan only once airborne at flight height: on the ground the slice is unobserved.
    if (p.z < height_ - 0.3) { airborne_since_ = -1; return; }
    if (airborne_since_ < 0) airborne_since_ = now;
    const bool reached = have_goal_ && (goal_xy_ - drone).norm() < reach_tol_ &&
                         std::fabs(std::remainder(goal_yaw_ - drone_yaw_, 2 * M_PI)) < 15.0 * M_PI / 180.0;
    const bool timed_out = have_goal_ && now - goal_time_ > goal_timeout_;
    if (have_goal_ && !reached && !timed_out) { publish_goal(); return; }
    auto cands = candidates(drone);
    const auto t0 = std::chrono::steady_clock::now();
    if (type_ == "pose_cov" && path_spacing_ > 0) score_path(cands);
    else if (type_ == "ig" || type_ == "pose_cov") score_ig(cands);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const int n_lm = jc_ ? (int)jc_->landmarks.size() : -1;
    std::ostringstream js;
    js << "{\"t\":" << now << ",\"type\":\"" << type_ << "\",\"sigma_virtual\":" << sigma_l_ << ",\"lambda\":" << lambda_ << ",\"yaw_samples\":" << yaw_samples_ << ",\"path_spacing\":" << path_spacing_
       << ",\"ms\":" << ms << ",\"n_lm\":" << n_lm << ",\"drone_yaw\":" << drone_yaw_ << ",\"drone\":[" << drone.x() << "," << drone.y() << "],\"candidates\":[";
    int best = -1;
    for (size_t i = 0; i < cands.size(); ++i) {
      const auto& c = cands[i];
      js << (i ? "," : "") << "{\"xy\":[" << c.xy.x() << "," << c.xy.y() << "],\"size\":" << c.cluster_size << ",\"len\":" << c.path_len
         << ",\"score\":" << (std::isfinite(c.score) ? std::to_string(c.score) : "null")
         << ",\"delta\":" << (std::isfinite(c.delta) ? std::to_string(c.delta) : "null")
         << ",\"delta_real\":" << (std::isfinite(c.delta_real) ? std::to_string(c.delta_real) : "null")
         << ",\"delta_virt\":" << (std::isfinite(c.delta_virt) ? std::to_string(c.delta_virt) : "null")
         << ",\"n_real\":" << c.n_meas_real << ",\"n_virt\":" << c.n_meas_virt
         << ",\"dI_pose\":" << (std::isfinite(c.dI_pose) ? std::to_string(c.dI_pose) : "null")
         << ",\"coverage_gain\":" << (std::isfinite(c.coverage_gain) ? std::to_string(c.coverage_gain) : "null")
         << ",\"score_pc\":" << (std::isfinite(c.score_pc) ? std::to_string(c.score_pc) : "null")
         << ",\"yaw\":" << c.yaw << ",\"cost\":" << c.cost
         << ",\"dI_meas\":" << (std::isfinite(c.dI_meas) ? std::to_string(c.dI_meas) : "null")
         << ",\"dI_prop\":" << (std::isfinite(c.dI_prop) ? std::to_string(c.dI_prop) : "null")
         << ",\"dI_meas_real\":" << (std::isfinite(c.dI_meas_real) ? std::to_string(c.dI_meas_real) : "null")
         << ",\"vis\":[" << c.vis[0] << "," << c.vis[1] << "," << c.vis[2] << "," << c.vis[3] << "],\"unk_vis\":" << c.unk_vis
         << ",\"lm_vis\":" << c.lm_vis << ",\"lm_vis_occ\":" << c.lm_vis_occ << ",\"n_wp\":" << c.n_wp << "}";
      if (type_ == "frontier") {
        if (best < 0 || c.cost < cands[best].cost) best = (int)i;  // nearest (travel + turning)
      } else if (type_ == "pose_cov") {
        if (std::isfinite(c.score_pc) && (best < 0 || c.score_pc > cands[best].score_pc)) best = (int)i;  // max dI_pose + lambda*cov
      } else if (std::isfinite(c.score) && (best < 0 || c.score < cands[best].score)) {
        best = (int)i;  // lowest posterior log-det
      }
    }
    std::string pick = "argmax";
    if (best >= 0 && random_pick_ > 0 && std::uniform_real_distribution<double>(0, 1)(rng_) < random_pick_) {
      std::vector<int> ok;
      for (size_t i = 0; i < cands.size(); ++i)
        if (type_ == "frontier" || std::isfinite(type_ == "pose_cov" ? cands[i].score_pc : cands[i].score)) ok.push_back((int)i);
      if (!ok.empty()) { best = ok[std::uniform_int_distribution<size_t>(0, ok.size() - 1)(rng_)]; pick = "random"; }
    }
    js << "],\"chosen\":" << best << ",\"pick\":\"" << pick << "\"}";
    std_msgs::msg::String dec;
    dec.data = js.str();
    pub_dec_->publish(dec);
    if (best < 0) {
      if (now - airborne_since_ > grace_ && ++empty_replans_ >= done_after_) { done_ = true; RCLCPP_INFO(get_logger(), "no reachable frontier: exploration done"); }
      have_goal_ = false;
      return;
    }
    empty_replans_ = 0;
    attempts_[cands[best].tile]++;
    goal_xy_ = cands[best].xy;
    goal_yaw_ = cands[best].yaw;
    goal_time_ = now;
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f) yaw %.0f deg, %zu candidates", goal_xy_.x(), goal_xy_.y(), goal_yaw_ * 180 / M_PI,
                cands.size());
    publish_goal();
  }

  void publish_goal() {
    geometry_msgs::msg::PoseStamped g;
    g.header.stamp = get_clock()->now();
    g.header.frame_id = "global";
    g.pose.position.x = goal_xy_.x();
    g.pose.position.y = goal_xy_.y();
    g.pose.position.z = height_;
    g.pose.orientation.z = std::sin(0.5 * goal_yaw_);
    g.pose.orientation.w = std::cos(0.5 * goal_yaw_);
    pub_goal_->publish(g);
  }

  std::string type_;
  double height_, standoff_, r_safe_, r_path_, cell_, speed_, reach_tol_, goal_timeout_, p_used_, sigma_px_, sigma_l_;
  int min_cluster_, max_virtual_, done_after_;
  std::map<std::pair<int, int>, double> esdf_;
  double esdf_res_ = 0.1;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::PointCloud2::SharedPtr frontiers_;
  active_slam_msgs::msg::JointCovariance::SharedPtr jc_;
  std::unique_ptr<PropagatorAccess> prop_;
  bool have_goal_ = false, done_ = false;
  int empty_replans_ = 0, max_attempts_ = 2;
  double lambda_ = 0.1, margin_time_ = 1.0, path_spacing_ = 0.0, yaw_rate_ = 0.6;
  std::string meas_model_ = "white";
  double rw_sigma0_ = 0.3, rw_q2_ = 0.02, rw_frame_rate_ = 15.0;
  int yaw_samples_ = 1, threads_ = 8;
  double random_pick_ = 0.0;
  std::mt19937 rng_;
  active_slam_sim::CoarseMap coarse_{0.25};
  active_slam_sim::FrustumParams frustum_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_coarse_;
  std::map<std::pair<int, int>, int> attempts_;
  double grace_ = 10.0, airborne_since_ = -1;
  Vector2d goal_xy_{0, 0};
  double goal_yaw_ = 0, goal_time_ = 0, drone_yaw_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_esdf_, sub_front_;
  rclcpp::Subscription<active_slam_msgs::msg::JointCovariance>::SharedPtr sub_jc_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_, pub_dec_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationPlanner>());
  rclcpp::shutdown();
  return 0;
}
