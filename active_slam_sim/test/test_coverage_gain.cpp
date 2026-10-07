// test_coverage_gain (day 2): unknown_volume_in_view against analytic volumes.
#include <cstdio>
#include "active_slam_sim/coverage_gain.hpp"
using namespace active_slam_sim;
static int fails = 0, passes = 0;
#define CHECK(c, ...) do { if (c) ++passes; else { ++fails; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)
int main() {
  FrustumParams f; f.nx = 64; f.ny = 40; f.step_frac = 0.25;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // camera looking along +z
  const Eigen::Vector3d p(0, 0, 0);
  CoarseMap empty(0.25);
  const double v = unknown_volume_in_view(empty, R, p, f), ref = frustum_volume(f);
  std::printf("empty map: %.2f m3 vs analytic %.2f m3 (rel %.4f)\n", v, ref, std::fabs(v - ref) / ref);
  CHECK(std::fabs(v - ref) / ref < 0.01, "empty-map volume off: %.2f vs %.2f", v, ref);
  // an occupied wall filling the view, plane z = 3 (cells z in [3, 3.25)): the visible
  // unknown volume is the PYRAMID to that plane, (2*3*tan a)(2*3*tan b)*3/3 (off-axis rays
  // travel farther than 3 m before hitting it -- not a sphere cap of radius 3).
  CoarseMap wall(0.25);
  for (double x = -12; x < 12; x += 0.25) for (double y = -12; y < 12; y += 0.25) wall.set(x + 0.1, y + 0.1, 3.1, true);
  const double vw = unknown_volume_in_view(wall, R, p, f);
  const double refw = 4.0 * 9.0 * std::tan(f.half_fov_x) * std::tan(f.half_fov_y);
  std::printf("wall at 3 m: %.2f m3 vs %.2f m3 (rel %.4f)\n", vw, refw, std::fabs(vw - refw) / refw);
  CHECK(std::fabs(vw - refw) / refw < 0.03, "occluded volume off: %.2f vs %.2f", vw, refw);
  // everything known-free -> nothing new to map
  CoarseMap freem(0.5);
  for (double x = -9; x < 9; x += 0.5) for (double y = -6; y < 6; y += 0.5) for (double z = -0.5; z < 9; z += 0.5) freem.set(x + 0.2, y + 0.2, z + 0.2, false);
  const double vf = unknown_volume_in_view(freem, R, p, f);
  std::printf("all known free: %.4f m3\n", vf);
  CHECK(vf < 1e-9, "known-free space counted as new: %.4f", vf);
  // segment_occluded (day 3): occupied cells occlude, UNKNOWN cells do not, the target's own cell does not
  {
    CoarseMap m(0.25);
    const Eigen::Vector3d a(0.1, 0.1, 0.1), b(4.1, 0.1, 0.1);
    int nu = 0;
    CHECK(!segment_occluded(m, a, b, &nu) && nu > 10, "all-unknown segment counted as occluded (nu=%d)", nu);
    for (double x = 0; x < 4.5; x += 0.25) m.set(x + 0.1, 0.1, 0.1, false);
    m.set(b.x(), b.y(), b.z(), true);  // landmark on its own occupied cell
    CHECK(!segment_occluded(m, a, b, &nu) && nu == 0, "landmark hidden by its own cell");
    m.set(2.1, 0.1, 0.1, true);
    CHECK(segment_occluded(m, a, b), "occupied cell in between not detected");
    CoarseMap u(0.25);  // occupied wall behind a stretch of unknown: still occluded
    u.set(3.0, 0.1, 0.1, true);
    CHECK(segment_occluded(u, a, b), "occupied cell beyond unknown not detected");
  }
  std::printf("%d passed, %d failed\n", passes, fails);
  return fails ? 1 : 0;
}
