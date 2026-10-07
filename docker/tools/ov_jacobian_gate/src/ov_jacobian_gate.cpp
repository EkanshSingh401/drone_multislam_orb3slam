// ov_jacobian_gate (PATCHES s60) -- convention cross-check, a GATE for the planner.
//
//   ov_jacobian_gate <bag_with_/openvins/joint_covariance> [max_messages]
//
// For each recorded JointCovariance message (OpenVINS's linearization point,
// published by the fork in the same callback as the covariance snapshot):
//   1. rebuild a minimal ov_msckf::State from the message: clones (value + FEJ),
//      camera extrinsics/intrinsics, FEJ option;
//   2. for every SLAM landmark and every clone OTHER than its anchor from which
//      it projects in front of the camera and inside the image (cam0 and cam1),
//      compute OpenVINS's own UpdaterHelper::get_feature_jacobian_full for that
//      single (feature, clone, camera) measurement;
//   3. compute active_slam_information::predict_measurement with
//      current = candidate = that clone's linearization pose, the landmark's
//      published p_FinG_fej and representation Jacobians (H_f, H_x);
//   4. compare per block, after converting OpenVINS's pixel Jacobian to
//      normalized coordinates (rows / fx, fy; zero distortion is asserted):
//        clone dtheta (2x3), clone dp (2x3)              [observing pose]
//        landmark representation (2x3)                    [H_f]
//        anchor clone dtheta (2x3), anchor clone dp (2x3) [through H_x]
//      reporting max relative error, and flagging sign flips (A ~ -B) and
//      transposed rotations (A ~ B with R -> R^T in the rotation sub-block).
// Measurement uvs are synthesized from the projection: H does not depend on the
// measured value, only on the linearization point (the residual is not compared).

#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "active_slam_msgs/msg/joint_covariance.hpp"
#include "cam/CamRadtan.h"
#include "state/State.h"
#include "state/StateOptions.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "update/UpdaterHelper.h"
#include "utils/quat_ops.h"

#include "active_slam_information/view_geometry.hpp"

using namespace ov_msckf;
namespace asi = active_slam_information;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

struct BlockStats {
  double max_rel = 0, max_abs = 0, max_ref = 0;
  int n = 0, sign_flips = 0, transposed = 0;
  void add(const MatrixXd &planner, const MatrixXd &ov, const MatrixXd &planner_if_transposed = MatrixXd()) {
    const double ref = ov.norm();
    const double err = (planner - ov).norm();
    const double rel = err / std::max(ref, 1e-12);
    max_rel = std::max(max_rel, rel);
    max_abs = std::max(max_abs, err);
    max_ref = std::max(max_ref, ref);
    n++;
    if (ref > 1e-9 && rel > 1e-6) {
      if ((planner + ov).norm() / ref < 1e-6) sign_flips++;
      else if (planner_if_transposed.size() && (planner_if_transposed - ov).norm() / ref < 1e-6) transposed++;
    }
  }
};

static Vector4d v4(const std::array<double, 4> &a) { return Vector4d(a[0], a[1], a[2], a[3]); }
static Vector3d v3(const std::array<double, 3> &a) { return Vector3d(a[0], a[1], a[2]); }

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <bag> [max_messages]\n", argv[0]);
    return 2;
  }
  const int max_msgs = argc > 2 ? std::atoi(argv[2]) : 1000000;
  // NEGATIVE CONTROL: "hamilton" feeds the planner R = quat_2_Rot(q)^T, i.e. reads
  // OpenVINS's JPL q_GtoI as a Hamilton quaternion -- the mismatch this gate
  // exists to catch. The gate must FAIL in this mode.
  const bool neg_hamilton = argc > 3 && std::string(argv[3]) == "hamilton";
  if (neg_hamilton) std::printf("NEGATIVE CONTROL: planner rotations read as Hamilton (transposed)\n");
  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);
  rclcpp::Serialization<active_slam_msgs::msg::JointCovariance> ser;

  std::map<std::string, BlockStats> st;
  int n_msgs = 0, n_meas = 0, n_skipped_anchor_missing = 0;
  while (reader.has_next() && n_msgs < max_msgs) {
    auto bm = reader.read_next();
    if (bm->topic_name != "/openvins/joint_covariance") continue;
    active_slam_msgs::msg::JointCovariance msg;
    rclcpp::SerializedMessage sm(*bm->serialized_data);
    ser.deserialize_message(&sm, &msg);
    if (msg.cameras.empty() || msg.clone_poses.empty() || msg.landmarks.empty()) continue;
    n_msgs++;

    // ---- 1. rebuild the OpenVINS state at the published linearization point ----
    StateOptions opt;
    opt.num_cameras = (int)msg.cameras.size();
    opt.do_fej = msg.state_uses_fej;
    opt.do_calib_camera_pose = false;
    opt.do_calib_camera_intrinsics = false;
    opt.do_calib_camera_timeoffset = false;
    auto state = std::make_shared<State>(opt);
    std::map<int, double> width_of, height_of;
    for (const auto &c : msg.cameras) {
      Eigen::Matrix<double, 7, 1> x;
      x << v4(c.q_itoc), v3(c.p_iinc);
      state->_calib_IMUtoCAM.at(c.camera_id)->set_value(x);
      state->_calib_IMUtoCAM.at(c.camera_id)->set_fej(x);
      Eigen::Matrix<double, 8, 1> k;
      for (int i = 0; i < 8; i++) k(i) = c.intrinsics[i];
      if (k.tail<4>().norm() != 0.0) {
        std::fprintf(stderr, "gate: camera %d has distortion; the fx,fy pixel conversion assumes none\n", c.camera_id);
        return 3;
      }
      state->_cam_intrinsics.at(c.camera_id)->set_value(k);
      state->_cam_intrinsics.at(c.camera_id)->set_fej(k);
      auto cam = std::make_shared<ov_core::CamRadtan>(848, 480);
      cam->set_value(k);
      state->_cam_intrinsics_cameras[c.camera_id] = cam;
    }
    for (const auto &cp : msg.clone_poses) {
      auto pose = std::make_shared<ov_type::PoseJPL>();
      Eigen::Matrix<double, 7, 1> x, xf;
      x << v4(cp.q_gtoi), v3(cp.p_iing);
      xf << v4(cp.q_gtoi_fej), v3(cp.p_iing_fej);
      pose->set_value(x);
      pose->set_fej(xf);
      pose->set_local_id(cp.state_id);
      state->_clones_IMU[cp.timestamp] = pose;
    }

    // ---- 2.-4. per landmark, per non-anchor clone, per camera ----
    for (const auto &L : msg.landmarks) {
      const bool relative = L.anchor_camera_id >= 0;
      if (relative && state->_clones_IMU.find(L.anchor_clone_timestamp) == state->_clones_IMU.end()) {
        n_skipped_anchor_missing++;
        continue;
      }
      // Representation Jacobians as published (OpenVINS's own function, FEJ as OpenVINS does)
      Eigen::Matrix3d Hf;
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) Hf(r, c) = L.h_f[3 * r + c];
      int hx_cols = 0;
      for (int s : L.hx_sizes) hx_cols += s;
      MatrixXd Hx(3, hx_cols);
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < hx_cols; c++) Hx(r, c) = L.h_x[(size_t)r * hx_cols + c];

      // OpenVINS feature at the same point. p_FinA from the representation value.
      ov_type::Landmark lm(3);
      lm._feat_representation = ov_type::LandmarkRepresentation::from_string(L.representation);
      lm.set_value(v3(L.rep_value));
      lm.set_fej(v3(L.rep_value));
      UpdaterHelper::UpdaterHelperFeature f;
      f.featid = (size_t)L.feature_id;
      f.feat_representation = lm._feat_representation;
      if (relative) {
        f.anchor_cam_id = L.anchor_camera_id;
        f.anchor_clone_timestamp = L.anchor_clone_timestamp;
        f.p_FinA = lm.get_xyz(false);
        f.p_FinA_fej = lm.get_xyz(true);
      }
      f.p_FinG = v3(L.p_fing);
      f.p_FinG_fej = v3(L.p_fing_fej);

      for (const auto &cp : msg.clone_poses) {
        if (relative && cp.timestamp == L.anchor_clone_timestamp) continue;  // keep blocks separable
        for (const auto &C : msg.cameras) {
          // planner camera / pose at OpenVINS's linearization point (FEJ if enabled)
          asi::CameraModel cam;
          cam.R_ItoC = ov_core::quat_2_Rot(v4(C.q_itoc));
          cam.p_IinC = v3(C.p_iinc);
          cam.fx = C.intrinsics[0]; cam.fy = C.intrinsics[1]; cam.cx = C.intrinsics[2]; cam.cy = C.intrinsics[3];
          cam.min_depth = 0.05; cam.max_depth = 1e9; cam.border_px = 0.0;
          asi::Pose P;
          const Vector4d q = msg.state_uses_fej ? v4(cp.q_gtoi_fej) : v4(cp.q_gtoi);
          P.R_GtoI = neg_hamilton ? Eigen::Matrix3d(ov_core::quat_2_Rot(q).transpose()) : ov_core::quat_2_Rot(q);
          P.p_IinG = msg.state_uses_fej ? v3(cp.p_iing_fej) : v3(cp.p_iing);
          asi::LandmarkLinearization pl;
          pl.p_FinG = msg.state_uses_fej ? v3(L.p_fing_fej) : v3(L.p_fing);
          // columns: observing pose 0..5, representation 6..8, then H_x states
          pl.rep.push_back({6, Hf});
          int col = 9, c0 = 0;
          std::map<int, int> col_of_state;
          for (size_t k = 0; k < L.hx_state_ids.size(); k++) {
            pl.rep.push_back({col, Hx.block(0, c0, 3, L.hx_sizes[k])});
            col_of_state[L.hx_state_ids[k]] = col;
            col += L.hx_sizes[k];
            c0 += L.hx_sizes[k];
          }
          asi::ConstantNoiseModel noise(1.0);
          asi::PredictedMeasurement pm;
          if (!asi::predict_measurement(P, P, 0, cam, pl, noise, nullptr, &pm)) continue;  // not visible

          // OpenVINS, one measurement from this clone and camera
          UpdaterHelper::UpdaterHelperFeature g = f;
          const Vector3d p_C = cam.R_ItoC * (P.R_GtoI * (pl.p_FinG - P.p_IinG)) + cam.p_IinC;
          const Eigen::Vector2d zn(p_C.x() / p_C.z(), p_C.y() / p_C.z());
          g.timestamps[C.camera_id] = {cp.timestamp};
          g.uvs[C.camera_id] = {Eigen::VectorXf(Eigen::Vector2f((float)(cam.fx * zn.x() + cam.cx), (float)(cam.fy * zn.y() + cam.cy)))};
          g.uvs_norm[C.camera_id] = {Eigen::VectorXf(Eigen::Vector2f((float)zn.x(), (float)zn.y()))};
          MatrixXd H_f, H_x;
          Eigen::VectorXd res;
          std::vector<std::shared_ptr<ov_type::Type>> order;
          UpdaterHelper::get_feature_jacobian_full(state, g, H_f, H_x, res, order);
          // pixel -> normalized: rows / fx, fy (dz_dzn = diag(fx, fy) with zero distortion)
          Eigen::Matrix2d Kinv = Eigen::Vector2d(1.0 / cam.fx, 1.0 / cam.fy).asDiagonal();
          H_f = Kinv * H_f;
          H_x = Kinv * H_x;
          std::map<int, MatrixXd> ov_block;
          int oc = 0;
          for (auto &t : order) {
            ov_block[t->id()] = H_x.block(0, oc, 2, t->size());
            oc += t->size();
          }

          // planner blocks
          MatrixXd H_obs, H_rep, H_anchor;
          for (auto &b : pm.blocks) {
            if (b.col == 0) H_obs = b.J;
            else if (b.col == 6) H_rep = b.J;
            else H_anchor = b.J;  // only anchor clone for ANCHORED_*
          }
          // transposed-rotation alternative for the observing rotation block
          const Eigen::Vector3d p_I = P.R_GtoI * (pl.p_FinG - P.p_IinG);
          Eigen::Matrix<double, 2, 3> dz;
          dz << 1 / p_C.z(), 0, -p_C.x() / (p_C.z() * p_C.z()), 0, 1 / p_C.z(), -p_C.y() / (p_C.z() * p_C.z());
          MatrixXd H_obs_th_T = dz * cam.R_ItoC.transpose() * asi::skew(p_I);

          const MatrixXd ov_obs = ov_block.at(cp.state_id);
          st["observing clone dtheta"].add(H_obs.leftCols(3), ov_obs.leftCols(3), H_obs_th_T);
          st["observing clone dp"].add(H_obs.rightCols(3), ov_obs.rightCols(3));
          st["landmark representation (H_f)"].add(H_rep, H_f);
          if (relative) {
            const int anchor_id = L.hx_state_ids.at(0);
            const MatrixXd ov_a = ov_block.at(anchor_id);
            st["anchor clone dtheta"].add(H_anchor.leftCols(3), ov_a.leftCols(3));
            st["anchor clone dp"].add(H_anchor.rightCols(3), ov_a.rightCols(3));
          }
          n_meas++;
        }
      }
    }
  }
  std::printf("gate: %d messages, %d (feature, clone, camera) measurements, %d landmarks skipped (anchor not in window)\n",
              n_msgs, n_meas, n_skipped_anchor_missing);
  bool pass = n_meas > 0;
  for (auto &kv : st) {
    const auto &b = kv.second;
    const bool ok = b.max_rel < 1e-6;
    pass &= ok;
    std::printf("  %-32s n=%6d  max rel err %.2e  max abs %.2e (|ref| up to %.2e)  sign flips %d  transposed-R %d  %s\n",
                kv.first.c_str(), b.n, b.max_rel, b.max_abs, b.max_ref, b.sign_flips, b.transposed, ok ? "PASS" : "FAIL");
  }
  std::printf("GATE: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
