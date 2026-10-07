#pragma once
// coverage_gain.hpp (day 2) -- expected new mapped volume from a candidate view.
//
// Unknown volume inside the camera frustum, by ray casting through a coarse
// known-space map (free / occupied cells; anything absent is unknown):
//   rays sampled uniformly on the normalized image plane (x, y) in [-tan a, tan a] x
//   [-tan b, tan b]; ray (x, y, 1) subtends dOmega = dx dy / (1 + x^2 + y^2)^(3/2);
//   along each ray, cells are visited in steps of step_frac * resolution from the
//   camera to max_range; the ray stops at the first OCCUPIED cell; each UNKNOWN step
//   [r1, r2] contributes dOmega * (r2^3 - r1^3) / 3 (exact volume of that shell piece).
// With an empty map (all unknown, no obstacles) the sum converges to the frustum
// volume Omega * R^3 / 3, Omega = 4 asin(sin a sin b) -- checked in the unit test.
// Free cells are already mapped and contribute nothing; occupied cells occlude.

#include <cmath>
#include <tuple>
#include <unordered_map>

#include <Eigen/Dense>

namespace active_slam_sim {

struct KeyHash {
  size_t operator()(const std::tuple<int, int, int>& k) const {
    return (size_t)(std::get<0>(k) * 73856093) ^ (size_t)(std::get<1>(k) * 19349663) ^ (size_t)(std::get<2>(k) * 83492791);
  }
};

class CoarseMap {
 public:
  explicit CoarseMap(double res = 0.25) : res_(res) {}
  void set(double x, double y, double z, bool occupied) { cells_[key(x, y, z)] = occupied; }
  void clear() { cells_.clear(); }
  // -1 unknown, 0 free, 1 occupied
  int state(const Eigen::Vector3d& p) const {
    auto it = cells_.find(key(p.x(), p.y(), p.z()));
    return it == cells_.end() ? -1 : (it->second ? 1 : 0);
  }
  double res() const { return res_; }
  size_t size() const { return cells_.size(); }

 private:
  std::tuple<int, int, int> key(double x, double y, double z) const {
    return {(int)std::floor(x / res_), (int)std::floor(y / res_), (int)std::floor(z / res_)};
  }
  double res_;
  std::unordered_map<std::tuple<int, int, int>, bool, KeyHash> cells_;
};

struct FrustumParams {
  double half_fov_x = 0.5 * 87.0 * M_PI / 180.0;  // D455 IR horizontal FOV
  double half_fov_y = 0.5 * 58.0 * M_PI / 180.0;  // vertical
  double max_range = 8.0;
  int nx = 32, ny = 20;
  double step_frac = 0.5;
};

inline double frustum_volume(const FrustumParams& f) {
  return 4.0 * std::asin(std::sin(f.half_fov_x) * std::sin(f.half_fov_y)) * std::pow(f.max_range, 3) / 3.0;
}

// R_GC: camera (optical: x right, y down, z forward) -> global; p_GC camera centre.
inline double unknown_volume_in_view(const CoarseMap& map, const Eigen::Matrix3d& R_GC, const Eigen::Vector3d& p_GC,
                                     const FrustumParams& f) {
  const double tx = std::tan(f.half_fov_x), ty = std::tan(f.half_fov_y);
  const double dx = 2 * tx / f.nx, dy = 2 * ty / f.ny;
  const double step = f.step_frac * map.res();
  double vol = 0.0;
  for (int i = 0; i < f.nx; ++i) {
    for (int j = 0; j < f.ny; ++j) {
      const double x = -tx + (i + 0.5) * dx, y = -ty + (j + 0.5) * dy;
      const double n2 = 1.0 + x * x + y * y;
      const double dOmega = dx * dy / (n2 * std::sqrt(n2));
      const Eigen::Vector3d dir = R_GC * Eigen::Vector3d(x, y, 1.0).normalized();
      for (double r = 0.0; r < f.max_range; r += step) {
        const double r2 = std::min(r + step, f.max_range);
        const int st = map.state(p_GC + dir * (0.5 * (r + r2)));
        if (st == 1) break;  // occluded
        if (st == -1) vol += dOmega * (r2 * r2 * r2 - r * r * r) / 3.0;
      }
    }
  }
  return vol;
}

}  // namespace active_slam_sim
