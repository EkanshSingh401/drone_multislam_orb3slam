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

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "active_slam_information/information_gain.hpp"
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
  double score = NAN, delta = NAN, cost = 0;
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
    max_attempts_ = declare_parameter("max_tile_attempts", 2);  // airborne this long before "done" may be declared
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
    if (type_ == "ig")
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
    RCLCPP_INFO(get_logger(), "exploration_planner up: type=%s height=%.2f standoff=%.2f r_safe=%.2f", type_.c_str(), height_, standoff_,
                r_safe_);
  }

 private:
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
      if (!(d >= r_path_)) return false;  // unobserved or too close
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
          if (d >= r_safe_ && path_clear(drone, v)) { cd.xy = v; ok = true; break; }
        }
      }
      if (!ok) continue;
      cd.yaw = std::atan2(c2.y() - cd.xy.y(), c2.x() - cd.xy.x());
      cd.path_len = (cd.xy - drone).norm();
      const double dyaw = std::fabs(std::remainder(cd.yaw - drone_yaw_, 2 * M_PI));
      if (cd.path_len < 0.3 && dyaw < 20.0 * M_PI / 180.0) continue;  // already looking there
      cd.cost = cd.path_len + 0.5 * dyaw;  // m; 0.5 m per rad of turning
      out.push_back(cd);
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
      std::vector<asi::PredictedMeasurement> meas;
      auto add = [&](const asi::LandmarkLinearization& lm) {
        for (const auto& cam : P.cameras) {
          asi::PredictedMeasurement pm;
          if (asi::predict_measurement(cp, cp, clone_col, cam, lm, noise, nullptr, &pm)) meas.push_back(pm);
        }
      };
      for (const auto& lm : P.landmarks) add(lm);
      for (const auto& lm : virt) add(lm);
      const asi::InformationPrior prior(Sa, Ta);
      if (!prior.ok()) continue;
      const auto gn = asi::evaluate(prior, meas);
      if (!gn.ok) continue;
      cd.score = gn.posterior_logdet;
      cd.delta = gn.delta;
    }
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
    if (type_ == "ig") score_ig(cands);
    std::ostringstream js;
    js << "{\"t\":" << now << ",\"type\":\"" << type_ << "\",\"drone\":[" << drone.x() << "," << drone.y() << "],\"candidates\":[";
    int best = -1;
    for (size_t i = 0; i < cands.size(); ++i) {
      const auto& c = cands[i];
      js << (i ? "," : "") << "{\"xy\":[" << c.xy.x() << "," << c.xy.y() << "],\"size\":" << c.cluster_size << ",\"len\":" << c.path_len
         << ",\"score\":" << (std::isfinite(c.score) ? std::to_string(c.score) : "null") << "}";
      if (type_ == "frontier") {
        if (best < 0 || c.cost < cands[best].cost) best = (int)i;  // nearest (travel + turning)
      } else if (std::isfinite(c.score) && (best < 0 || c.score < cands[best].score)) {
        best = (int)i;  // lowest posterior log-det
      }
    }
    js << "],\"chosen\":" << best << "}";
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
