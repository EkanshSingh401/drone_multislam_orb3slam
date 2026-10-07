// octomap_mapper (PATCHES s64) -- CPU stand-in for nvblox in sim (nvblox needs
// Ampere; the 1080 Ti is Pascal). Same interface the planners use on the Jetson:
//   out  <esdf_topic> (default /nvblox_node/static_esdf_pointcloud)
//        sensor_msgs/PointCloud2, fields x y z intensity: a horizontal ESDF slice
//        at the flight altitude, intensity = distance [m] to the nearest occupied
//        voxel (as nvblox's static_esdf_pointcloud). Only observed cells.
//   out  ~/frontiers  sensor_msgs/PointCloud2 x y z: free voxels in the flight
//        altitude band with at least one unknown 6-neighbour.
//   out  ~/coverage   std_msgs/Float64MultiArray [t, known_m3, free_m3, occupied_m3]
// in   depth image (32FC1, metres, cam0 optical frame) + OpenVINS /odomimu (IMU
//      pose in OpenVINS's global frame) + T_cam_imu of cam0 (the depth camera sits
//      on cam0). Everything is mapped in OpenVINS's global frame, the frame the
//      planners and setpoints use.
// Map: OctoMap OcTree (liboctomap), resolution `resolution`; depth subsampled by
// `pixel_step`, rays beyond `max_range` inserted as free space up to max_range.

#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <tuple>
#include <mutex>

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class OctomapMapper : public rclcpp::Node {
 public:
  OctomapMapper() : Node("octomap_mapper") {
    res_ = declare_parameter("resolution", 0.10);
    max_range_ = declare_parameter("max_range", 5.0);
    step_ = declare_parameter("pixel_step", 6);
    fx_ = declare_parameter("fx", 446.802773);
    fy_ = declare_parameter("fy", 446.802773);
    cx_ = declare_parameter("cx", 424.0);
    cy_ = declare_parameter("cy", 240.0);
    // T_cam_imu of cam0 (kalibr_imucam_chain.yaml): p_C = R_CI p_I + t_CI
    auto Tci = declare_parameter("T_cam_imu", std::vector<double>{0, -1, 0, 0.0424, 0, 0, -1, 0.01174, 1, 0, 0, -0.00552, 0, 0, 0, 1});
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) R_CI_(r, c) = Tci[4 * r + c];
      t_CI_(r) = Tci[4 * r + 3];
    }
    slice_z_ = declare_parameter("slice_height", 1.5);   // OpenVINS-frame z of the slice
    band_ = declare_parameter("frontier_band", 0.5);     // +- around slice_height
    slice_half_ = declare_parameter("slice_half_extent", 8.0);
    rate_ = declare_parameter("map_rate", 5.0);
    body_free_radius_ = declare_parameter("body_free_radius", 0.5);
    band_below_ = declare_parameter("esdf_band_below", 1.0);
    band_above_ = declare_parameter("esdf_band_above", 0.6);
    const auto esdf_topic = declare_parameter("esdf_topic", std::string("/nvblox_node/static_esdf_pointcloud"));
    tree_ = std::make_unique<octomap::OcTree>(res_);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter("odom_topic", std::string("/ov_msckf/odomimu")), 50,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mu_);
          odom_.push_back(m);
          while (odom_.size() > 400) odom_.pop_front();
        });
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
        declare_parameter("depth_topic", std::string("/camera/depth/image_rect_raw")), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::SharedPtr m) { on_depth(m); });
    pub_esdf_ = create_publisher<sensor_msgs::msg::PointCloud2>(esdf_topic, 2);
    pub_front_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/frontiers", 2);
    pub_cov_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/coverage", 10);
    // Coarse known-space cloud for coverage-gain ray casting in the planner (day 2):
    // x y z occ (0 free, 1 occupied) at coarse_resolution within coarse_radius of the drone.
    coarse_res_ = declare_parameter("coarse_resolution", 0.25);
    coarse_radius_ = declare_parameter("coarse_radius", 10.0);
    pub_coarse_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/occupancy_coarse", 2);
    timer_coarse_ = create_wall_timer(std::chrono::milliseconds(1000), [this] { publish_coarse(); });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_products(); });
    RCLCPP_INFO(get_logger(), "octomap_mapper up: res %.2f m, range %.1f m, ESDF slice z=%.2f -> %s", res_, max_range_, slice_z_,
                esdf_topic.c_str());
  }

 private:
  bool pose_at(const rclcpp::Time& t, Matrix3d* R_GI, Vector3d* p_GI) {
    std::lock_guard<std::mutex> lk(mu_);
    if (odom_.empty()) return false;
    nav_msgs::msg::Odometry::SharedPtr best;
    double bd = 1e9;
    for (auto& o : odom_) {
      const double d = std::fabs((rclcpp::Time(o->header.stamp) - t).seconds());
      if (d < bd) { bd = d; best = o; }
    }
    if (bd > 0.05) return false;  // no pose within 50 ms
    const auto& q = best->pose.pose.orientation;
    *R_GI = Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();  // /odomimu: Hamilton q_ItoG
    const auto& p = best->pose.pose.position;
    *p_GI = Vector3d(p.x, p.y, p.z);
    return true;
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr m) {
    const rclcpp::Time t(m->header.stamp);
    if (last_insert_.nanoseconds() > 0 && (t - last_insert_).seconds() < 1.0 / rate_) return;
    if (m->encoding != "32FC1") { RCLCPP_WARN_ONCE(get_logger(), "depth encoding %s not 32FC1", m->encoding.c_str()); return; }
    Matrix3d R_GI;
    Vector3d p_GI;
    if (!pose_at(t, &R_GI, &p_GI)) return;
    last_insert_ = t;
    // camera pose in G: p_G = R_GI R_CI^T (p_C - t_CI) + p_GI
    const Matrix3d R_GC = R_GI * R_CI_.transpose();
    const Vector3d p_GC = p_GI - R_GC * t_CI_;
    octomap::Pointcloud cloud;
    const float* d = reinterpret_cast<const float*>(m->data.data());
    for (uint32_t v = 0; v < m->height; v += step_) {
      for (uint32_t u = 0; u < m->width; u += step_) {
        float z = d[v * m->width + u];
        if (!std::isfinite(z) || z <= 0.05f) z = (float)max_range_ + 1.0f;  // no return: free ray
        const double zz = std::min<double>(z, max_range_ + 1.0);
        const Vector3d pc((u - cx_) * zz / fx_, (v - cy_) * zz / fy_, zz);
        const Vector3d pg = R_GC * pc + p_GC;
        cloud.push_back((float)pg.x(), (float)pg.y(), (float)pg.z());
      }
    }
    tree_->insertPointCloud(cloud, octomap::point3d((float)p_GC.x(), (float)p_GC.y(), (float)p_GC.z()), max_range_, false, true);
    // The vehicle's own volume is free (it is flying there) but the forward camera
    // never observes it; without this the ESDF is unknown at the drone itself.
    const double r = body_free_radius_;
    for (double dx = -r; dx <= r; dx += res_)
      for (double dy = -r; dy <= r; dy += res_)
        for (double dz = -r; dz <= r; dz += res_)
          if (dx * dx + dy * dy + dz * dz <= r * r)
            tree_->updateNode(octomap::point3d((float)(p_GI.x() + dx), (float)(p_GI.y() + dy), (float)(p_GI.z() + dz)), false, true);
    drone_xy_ = p_GI.head<2>();
    have_pose_ = true;
  }

  // coarse known space (free/occupied) around the drone
  void publish_coarse() {
    if (!have_pose_) return;
    const double r = coarse_radius_;
    octomap::point3d lo((float)(drone_xy_.x() - r), (float)(drone_xy_.y() - r), (float)(slice_z_ - 3.0));
    octomap::point3d hi((float)(drone_xy_.x() + r), (float)(drone_xy_.y() + r), (float)(slice_z_ + 3.0));
    std::map<std::tuple<int, int, int>, float> cells;
    for (auto it = tree_->begin_leafs_bbx(lo, hi), end = tree_->end_leafs_bbx(); it != end; ++it) {
      const octomap::point3d c = it.getCoordinate();
      const double s = it.getSize();
      const bool o = tree_->isNodeOccupied(*it);
      // a large pruned leaf covers several coarse cells: mark each
      const int n = std::max(1, (int)std::round(s / coarse_res_));
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
          for (int k = 0; k < n; ++k) {
            const double x = c.x() - s / 2 + (i + 0.5) * s / n, y = c.y() - s / 2 + (j + 0.5) * s / n, z = c.z() - s / 2 + (k + 0.5) * s / n;
            auto key = std::make_tuple((int)std::floor(x / coarse_res_), (int)std::floor(y / coarse_res_), (int)std::floor(z / coarse_res_));
            auto f = cells.find(key);
            if (f == cells.end()) cells[key] = o ? 1.f : 0.f; else if (o) f->second = 1.f;
          }
    }
    sensor_msgs::msg::PointCloud2 pc;
    pc.header.stamp = get_clock()->now();
    pc.header.frame_id = "global";
    sensor_msgs::PointCloud2Modifier mod(pc);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                             "z", 1, sensor_msgs::msg::PointField::FLOAT32, "occ", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> ix(pc, "x"), iy(pc, "y"), iz(pc, "z"), io(pc, "occ");
    for (auto& kv : cells) {
      *ix = (std::get<0>(kv.first) + 0.5f) * (float)coarse_res_; *iy = (std::get<1>(kv.first) + 0.5f) * (float)coarse_res_;
      *iz = (std::get<2>(kv.first) + 0.5f) * (float)coarse_res_; *io = kv.second;
      ++ix; ++iy; ++iz; ++io;
    }
    pub_coarse_->publish(pc);
  }

  // nvblox-style ESDF slice + frontiers + coverage
  void publish_products() {
    if (!have_pose_) return;
    const rclcpp::Time now = get_clock()->now();
    // occupied voxels near the slice height (for the 2D distance at that height)
    std::vector<Eigen::Vector2d> occ;
    double v_free = 0, v_occ = 0;
    const double vox = res_ * res_ * res_;
    std::vector<Eigen::Vector3d> fronts;
    for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) {
      const double s = it.getSize();
      const double vol = s * s * s;
      const bool o = tree_->isNodeOccupied(*it);
      (o ? v_occ : v_free) += vol;
      const octomap::point3d c = it.getCoordinate();
      // Obstacles that count for the 2D ESDF at the flight height: the vehicle's whole
      // vertical extent plus margin, i.e. [slice - band_below, slice + band_above]. With
      // +-0.6 m a 1.2 m box under a 1.45 m flight was nearly invisible and the landing
      // gear reached it (day 2, s2_ig_2 / s3_sig0.3_3).
      if (o && c.z() >= slice_z_ - band_below_ && c.z() <= slice_z_ + band_above_) occ.emplace_back(c.x(), c.y());
      if (!o && s <= res_ + 1e-6 && std::fabs(c.z() - slice_z_) <= band_) {
        // frontier: a free leaf with an unknown 6-neighbour
        static const int nb[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        for (auto& n : nb) {
          octomap::point3d q(c.x() + n[0] * res_, c.y() + n[1] * res_, c.z() + n[2] * res_);
          if (tree_->search(q) == nullptr) { fronts.emplace_back(c.x(), c.y(), c.z()); break; }
        }
      }
    }
    (void)vox;
    // ESDF slice on a grid around the drone (nvblox static_esdf_pointcloud equivalent)
    sensor_msgs::msg::PointCloud2 pc;
    pc.header.stamp = now;
    pc.header.frame_id = "global";
    sensor_msgs::PointCloud2Modifier mod(pc);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                             "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    std::vector<std::array<float, 4>> pts;
    // Exact 2D Euclidean distance transform (Felzenszwalb & Huttenlocher) on a local
    // grid centred on the drone; occupied voxels near the slice height are the sites.
    const int n = (int)std::round(slice_half_ / res_);
    const int N = 2 * n + 1;
    const double x0 = drone_xy_.x() - n * res_, y0 = drone_xy_.y() - n * res_;
    const double INF = 1e20;
    std::vector<double> f(N * N, INF);
    for (const auto& o : occ) {
      const int i = (int)std::lround((o.x() - x0) / res_), j = (int)std::lround((o.y() - y0) / res_);
      if (i >= 0 && i < N && j >= 0 && j < N) f[i * N + j] = 0.0;
    }
    // 1D squared EDT, lower envelope of parabolas; empty sites (>= INF) are skipped.
    auto edt1d = [&](std::vector<double>& g, int len) {
      std::vector<int> v(len);
      std::vector<double> z(len + 1), d(len, INF);
      int k = -1;
      for (int q = 0; q < len; q++) {
        if (g[q] >= INF) continue;
        double sv = -INF;
        while (k >= 0) {
          sv = ((g[q] + double(q) * q) - (g[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
          if (sv <= z[k]) k--; else break;
        }
        k++;
        v[k] = q;
        z[k] = (k == 0) ? -INF : sv;
        z[k + 1] = INF;
      }
      if (k >= 0) {
        int j = 0;
        for (int q = 0; q < len; q++) {
          while (z[j + 1] < q) j++;
          d[q] = double(q - v[j]) * (q - v[j]) + g[v[j]];
        }
      }
      g = d;
    };
    std::vector<double> row(N);
    for (int i = 0; i < N; i++) { for (int j = 0; j < N; j++) row[j] = f[i * N + j]; edt1d(row, N); for (int j = 0; j < N; j++) f[i * N + j] = row[j]; }
    for (int j = 0; j < N; j++) { for (int i = 0; i < N; i++) row[i] = f[i * N + j]; edt1d(row, N); for (int i = 0; i < N; i++) f[i * N + j] = row[i]; }
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < N; ++j) {
        const double x = x0 + i * res_, y = y0 + j * res_;
        if (tree_->search(x, y, slice_z_) == nullptr) continue;  // unobserved: not part of the ESDF (as nvblox)
        const double dist = f[i * N + j] >= INF / 2 ? slice_half_ * 2 : std::sqrt(f[i * N + j]) * res_;
        pts.push_back({(float)x, (float)y, (float)slice_z_, (float)dist});
      }
    }
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(pc, "x"), iy(pc, "y"), iz(pc, "z"), ii(pc, "intensity");
    for (auto& p : pts) { *ix = p[0]; *iy = p[1]; *iz = p[2]; *ii = p[3]; ++ix; ++iy; ++iz; ++ii; }
    pub_esdf_->publish(pc);

    sensor_msgs::msg::PointCloud2 fc;
    fc.header = pc.header;
    sensor_msgs::PointCloud2Modifier fm(fc);
    fm.setPointCloud2FieldsByString(1, "xyz");
    fm.resize(fronts.size());
    sensor_msgs::PointCloud2Iterator<float> fx(fc, "x"), fy(fc, "y"), fz(fc, "z");
    for (auto& f : fronts) { *fx = (float)f.x(); *fy = (float)f.y(); *fz = (float)f.z(); ++fx; ++fy; ++fz; }
    pub_front_->publish(fc);

    std_msgs::msg::Float64MultiArray cov;
    cov.data = {now.seconds(), v_free + v_occ, v_free, v_occ};
    pub_cov_->publish(cov);
  }

  double band_below_ = 1.0, band_above_ = 0.6;
  double body_free_radius_ = 0.5, coarse_res_ = 0.25, coarse_radius_ = 10.0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_coarse_;
  rclcpp::TimerBase::SharedPtr timer_coarse_;
  double res_, max_range_, fx_, fy_, cx_, cy_, slice_z_, band_, slice_half_, rate_;
  int step_;
  Matrix3d R_CI_;
  Vector3d t_CI_;
  std::unique_ptr<octomap::OcTree> tree_;
  std::mutex mu_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_;
  rclcpp::Time last_insert_{0, 0, RCL_ROS_TIME};
  Eigen::Vector2d drone_xy_{0, 0};
  bool have_pose_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_esdf_, pub_front_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cov_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapMapper>());
  rclcpp::shutdown();
  return 0;
}
