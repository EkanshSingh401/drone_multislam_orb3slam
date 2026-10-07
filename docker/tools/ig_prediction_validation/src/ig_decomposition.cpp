// ig_decomposition (PATCHES s62) -- where predicted vs realized information gain
// comes from. Decomposition, not a fix.
//
//   ig_decomposition <jc.bag> <imu.txt> <ov_state_est.txt> <openvins.log> <gt.tum>
//                    <seg_len> <seg_every> <t_win0> <t_win1> <scene> <records.jsonl>
//
// Needs a serial run with joint_cov_rate 0, joint_cov_stages true and DEBUG
// verbosity ([SLAMEV] lines). Per segment [t0, t1] (post-update messages), same
// marginal as s61 (IMU + global XYZ of landmarks present at both ends):
//   realized               L0 - L1
//   realized by stage      summed over the segment's frames, from the stage
//                          snapshots: prop (propagate+clone; negative), msckf,
//                          slam, init (new SLAM landmarks), marg (clone marg.)
//   pred_all               s61 prediction (b): OpenVINS propagation + clone per
//                          frame + every predicted-visible (landmark, camera)
//   pred_used              same, but only (frame, landmark, camera) that OpenVINS
//                          actually used ([SLAMEV] outcome=used, camera had a meas.)
//   over  = pred_all - pred_used     gain over-predicted from visible-but-unused
//   under = msckf + init             gain the prediction never models
// Records: one JSON line per predicted-visible (frame, landmark, camera).

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <regex>
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
using Eigen::Vector3d;

struct PropagatorAccess : public ov_msckf::Propagator {
  using ov_msckf::Propagator::Propagator;
  using ov_msckf::Propagator::predict_and_compute;
};

static double logdet_spd(const MatrixXd& A, bool* ok) {
  Eigen::LLT<MatrixXd> llt(A);
  *ok = llt.info() == Eigen::Success;
  return *ok ? 2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum() : NAN;
}

// JSON number or null (NaN/inf are not valid JSON)
static std::string jn(double x, const char* fmt = "%.6f") {
  if (!std::isfinite(x)) return "null";
  char b[64];
  std::snprintf(b, sizeof b, fmt, x);
  return b;
}

struct SlamEv { int n0 = 0, n1 = 0; std::string outcome; double chi2 = NAN, thr = NAN; };
static long long tkey(double t) { return std::llround(t * 1e6); }

int main(int argc, char** argv) {
  if (argc < 12) {
    std::fprintf(stderr, "usage: %s jc.bag imu.txt ov_state_est.txt openvins.log gt.tum seg_len seg_every t0 t1 scene records.jsonl\n", argv[0]);
    return 2;
  }
  const double seg_len = std::atof(argv[6]), seg_every = std::atof(argv[7]), tw0 = std::atof(argv[8]), tw1 = std::atof(argv[9]);
  const std::string scene = argv[10];
  FILE* rec = std::fopen(argv[11], "w");

  // ---- messages by (time, stage) ----
  std::map<long long, std::map<std::string, active_slam_msgs::msg::JointCovariance>> jc;
  std::vector<double> post_times;
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
      const double t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec;
      const std::string st = m.stage.empty() ? "post" : m.stage;
      if (st == "post") post_times.push_back(t);
      jc[tkey(t)][st] = m;
    }
  }
  std::sort(post_times.begin(), post_times.end());
  std::vector<ov_core::ImuData> imu;
  {
    std::ifstream f(argv[2]);
    ov_core::ImuData d;
    while (f >> d.timestamp >> d.wm(0) >> d.wm(1) >> d.wm(2) >> d.am(0) >> d.am(1) >> d.am(2)) imu.push_back(d);
  }
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
  // ---- [SLAMEV] per (frame time, feature id) ----
  std::map<long long, std::map<long long, SlamEv>> ev;
  {
    std::ifstream f(argv[4]);
    std::string line;
    const std::regex re(R"(\[SLAMEV\] t=([0-9.]+) id=([0-9]+) n0=([0-9]+) n1=([0-9]+) outcome=([a-z0-9]+)(?: chi2=([-0-9.e+naif]+) thr=([-0-9.e+naif]+))?)");
    std::smatch m;
    while (std::getline(f, line)) {
      if (!std::regex_search(line, m, re)) continue;
      SlamEv e;
      e.n0 = std::stoi(m[3]); e.n1 = std::stoi(m[4]); e.outcome = m[5];
      if (m[6].matched) { e.chi2 = std::atof(m[6].str().c_str()); e.thr = std::atof(m[7].str().c_str()); }
      ev[tkey(std::atof(m[1].str().c_str()))][std::stoll(m[2])] = e;
    }
  }
  std::vector<std::array<double, 8>> gt;
  {
    std::ifstream f(argv[5]);
    std::array<double, 8> a;
    while (f >> a[0] >> a[1] >> a[2] >> a[3] >> a[4] >> a[5] >> a[6] >> a[7]) gt.push_back(a);
  }

  ov_msckf::NoiseManager nm;
  nm.sigma_w = 1.6e-4; nm.sigma_w_2 = nm.sigma_w * nm.sigma_w;
  nm.sigma_a = 2.0e-3; nm.sigma_a_2 = nm.sigma_a * nm.sigma_a;
  nm.sigma_wb = 2.0e-6; nm.sigma_wb_2 = nm.sigma_wb * nm.sigma_wb;
  nm.sigma_ab = 3.0e-4; nm.sigma_ab_2 = nm.sigma_ab * nm.sigma_ab;
  PropagatorAccess prop(nm, 9.81);
  const asi::ConstantNoiseModel noise(1.0);

  auto marginal_logdet = [](const active_slam_msgs::msg::JointCovariance& m, const std::set<long long>& keep, bool* ok) {
    const auto P = asi::problem_from_msg(m);
    if (!P.ok) { *ok = false; return (double)NAN; }
    for (long long id : keep) if (!P.landmark_index.count(id)) { *ok = false; return (double)NAN; }
    const MatrixXd T = P.T_metric(&keep);
    return logdet_spd(T * P.Sigma * T.transpose(), ok);
  };

  double next_t0 = tw0;
  for (size_t i0 = 0; i0 < post_times.size(); ++i0) {
    const double t0 = post_times[i0];
    if (t0 < next_t0 || t0 > tw1 - seg_len) continue;
    size_t i1 = i0 + 1;
    while (i1 < post_times.size() && post_times[i1] < t0 + seg_len - 1e-6) ++i1;
    if (i1 >= post_times.size()) break;
    const double t1 = post_times[i1];
    next_t0 = t0 + seg_every;
    if (t1 - t0 > 1.5 * seg_len) continue;
    const auto& M0 = jc[tkey(t0)]["post"];
    const auto& M1 = jc[tkey(t1)]["post"];
    const auto P0 = asi::problem_from_msg(M0), P1 = asi::problem_from_msg(M1);
    if (!P0.ok || !P1.ok) continue;
    std::set<long long> common;
    for (auto& kv : P0.landmark_index) if (P1.landmark_index.count(kv.first)) common.insert(kv.first);
    bool ok0, ok1;
    const double L0 = marginal_logdet(M0, common, &ok0), L1 = marginal_logdet(M1, common, &ok1);
    if (!ok0 || !ok1) continue;

    // ---- realized gain by stage over the frames in (t0, t1] ----
    std::map<std::string, double> real{{"prop", 0}, {"msckf", 0}, {"slam", 0}, {"init", 0}, {"marg", 0}};
    bool stages_ok = true;
    double Lprev = L0;
    std::vector<double> ftimes;
    for (size_t i = i0 + 1; i <= i1; ++i) ftimes.push_back(post_times[i]);
    for (double tf : ftimes) {
      auto& S = jc[tkey(tf)];
      const char* order[] = {"propagated", "msckf", "slam", "init", "post"};
      const char* name[] = {"prop", "msckf", "slam", "init", "marg"};
      for (int k = 0; k < 5; ++k) {
        if (!S.count(order[k])) { stages_ok = false; break; }
        bool ok;
        const double L = marginal_logdet(S.at(order[k]), common, &ok);
        if (!ok) { stages_ok = false; break; }
        real[name[k]] += Lprev - L;
        Lprev = L;
      }
      if (!stages_ok) break;
    }

    // ---- predictions (b): all visible vs actually used ----
    auto e0 = est.find(t0);
    if (e0 == est.end()) continue;
    ov_msckf::StateOptions so;
    so.num_cameras = 2;
    so.integration_method = ov_msckf::StateOptions::IntegrationMethod::RK4;
    auto st = std::make_shared<ov_msckf::State>(so);
    st->_imu->set_value(e0->second);
    st->_imu->set_fej(e0->second);
    MatrixXd S = P0.Sigma;
    const int ic = P0.imu_col;
    std::vector<int> clone_col;
    double tcur = t0;
    for (double tf : ftimes) {
      auto readings = ov_msckf::Propagator::select_imu_readings(imu, tcur, tf, false);
      for (size_t k = 0; k + 1 < readings.size(); ++k) {
        Eigen::MatrixXd F, Qd;
        prop.predict_and_compute(st, readings[k], readings[k + 1], F, Qd);
        const int d = (int)F.rows();
        MatrixXd rows = F * S.middleRows(ic, d);
        S.middleRows(ic, d) = rows;
        MatrixXd cols = S.middleCols(ic, d) * F.transpose();
        S.middleCols(ic, d) = cols;
        S.block(ic, ic, d, d) += Qd;
      }
      tcur = tf;
      const int n = (int)S.rows();
      MatrixXd S2 = MatrixXd::Zero(n + 6, n + 6);
      S2.topLeftCorner(n, n) = S;
      S2.block(n, 0, 6, n) = S.middleRows(ic, 6);
      S2.block(0, n, n, 6) = S.middleCols(ic, 6);
      S2.block(n, n, 6, 6) = S.block(ic, ic, 6, 6);
      S = 0.5 * (S2 + S2.transpose());
      clone_col.push_back(n);
    }
    const MatrixXd T0 = P0.T_metric(&common);
    MatrixXd Tb = MatrixXd::Zero(T0.rows(), S.cols());
    Tb.leftCols(T0.cols()) = T0;
    std::vector<asi::PredictedMeasurement> m_all, m_used;
    int n_vis = 0, n_used = 0, n_tracked = 0;
    for (size_t k = 0; k < ftimes.size(); ++k) {
      auto it = est.find(ftimes[k]);
      if (it == est.end()) continue;
      asi::Pose fp;
      fp.R_GtoI = asi::jpl_quat_to_rot({it->second(0), it->second(1), it->second(2), it->second(3)});
      fp.p_IinG = it->second.segment<3>(4);
      // frame motion from the state file (previous processed frame)
      double w_fr = NAN, v_fr = NAN;
      if (it != est.begin()) {
        auto ip = std::prev(it);
        const double dt = it->first - ip->first;
        const Eigen::Matrix3d Rp = asi::jpl_quat_to_rot({ip->second(0), ip->second(1), ip->second(2), ip->second(3)});
        w_fr = Eigen::AngleAxisd(fp.R_GtoI * Rp.transpose()).angle() / dt;
        v_fr = (fp.p_IinG - ip->second.segment<3>(4)).norm() / dt;
      }
      const auto& evk = ev[tkey(ftimes[k])];
      for (const auto& lm : P0.landmarks) {
        if (!common.count(lm.id)) continue;
        for (size_t c = 0; c < P0.cameras.size(); ++c) {
          const auto& cam = P0.cameras[c];
          asi::PredictedMeasurement pm;
          if (!asi::predict_measurement(fp, fp, clone_col[k], cam, lm, noise, nullptr, &pm)) continue;
          n_vis++;
          m_all.push_back(pm);
          auto e = evk.find(lm.id);
          const bool tracked = e != evk.end();
          const int ncam = tracked ? (c == 0 ? e->second.n0 : e->second.n1) : 0;
          const bool used = tracked && e->second.outcome == "used" && ncam > 0;
          n_tracked += tracked;
          if (used) { m_used.push_back(pm); n_used++; }
          // ---- training record ----
          const Vector3d p_I = fp.R_GtoI * (lm.p_FinG - fp.p_IinG);
          const Vector3d p_C = cam.R_ItoC * p_I + cam.p_IinC;
          const double u = cam.fx * p_C.x() / p_C.z() + cam.cx, v = cam.fy * p_C.y() / p_C.z() + cam.cy;
          const double border = std::min(std::min(u, cam.width - u), std::min(v, cam.height - v));
          // viewing angle: between the rays to the landmark from this camera and from the anchor camera
          double view_angle = NAN, age = NAN;
          const auto& L = M0.landmarks[P0.landmark_index.at(lm.id)];
          if (L.anchor_camera_id >= 0) {
            for (const auto& cp : M0.clone_poses) {
              if (cp.timestamp != L.anchor_clone_timestamp) continue;
              const Eigen::Matrix3d Ra = asi::jpl_quat_to_rot(cp.q_gtoi);
              const Vector3d pa(cp.p_iing[0], cp.p_iing[1], cp.p_iing[2]);
              const auto& ca = P0.cameras[L.anchor_camera_id];
              const Vector3d ca_G = asi::camera_center_in_global({Ra, pa}, ca);
              const Vector3d cc_G = asi::camera_center_in_global(fp, cam);
              const Vector3d r1 = (lm.p_FinG - cc_G).normalized(), r2 = (lm.p_FinG - ca_G).normalized();
              view_angle = std::acos(std::max(-1.0, std::min(1.0, r1.dot(r2)))) * 180.0 / M_PI;
              age = ftimes[k] - cp.timestamp;
            }
          }
          std::fprintf(rec,
                       "{\"scene\":\"%s\",\"t\":%.6f,\"seg_t0\":%.3f,\"feature_id\":%lld,\"cam\":%zu,\"depth\":%s,\"u\":%s,\"v\":%s,"
                       "\"border_px\":%s,\"view_angle_deg\":%s,\"age_s\":%s,\"frame_ang_speed\":%s,\"frame_speed\":%s,"
                       "\"tracked\":%d,\"n_cam_meas\":%d,\"outcome\":\"%s\",\"chi2\":%s,\"chi2_thr\":%s,\"used\":%d}\n",
                       scene.c_str(), ftimes[k], t0, (long long)lm.id, c, jn(p_C.z(), "%.4f").c_str(), jn(u, "%.2f").c_str(),
                       jn(v, "%.2f").c_str(), jn(border, "%.2f").c_str(), jn(view_angle, "%.3f").c_str(), jn(age, "%.3f").c_str(),
                       jn(w_fr, "%.5f").c_str(), jn(v_fr, "%.5f").c_str(), (int)tracked, ncam,
                       tracked ? e->second.outcome.c_str() : "not_tracked", jn(tracked ? e->second.chi2 : NAN, "%.4f").c_str(),
                       jn(tracked ? e->second.thr : NAN, "%.4f").c_str(), (int)used);
        }
      }
    }
    const asi::InformationPrior prior_b(S, Tb);
    const auto g_all = asi::evaluate(prior_b, m_all), g_used = asi::evaluate(prior_b, m_used);
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
    const double pa = g_all.ok ? L0 - g_all.posterior_logdet : NAN, pu = g_used.ok ? L0 - g_used.posterior_logdet : NAN;
    std::printf(
        "{\"scene\":\"%s\",\"t0\":%.3f,\"t1\":%.3f,\"frames\":%zu,\"common\":%zu,\"stages_ok\":%d,\"realized\":%.6f,"
        "\"real_prop\":%.6f,\"real_msckf\":%.6f,\"real_slam\":%.6f,\"real_init\":%.6f,\"real_marg\":%.6f,"
        "\"pred_all\":%s,\"pred_used\":%s,\"over\":%s,\"under\":%.6f,\"n_visible\":%d,\"n_tracked\":%d,\"n_used\":%d,"
        "\"ang_speed\":%.5f,\"speed\":%.5f}\n",
        scene.c_str(), t0, t1, ftimes.size(), common.size(), (int)stages_ok, L0 - L1, real["prop"], real["msckf"], real["slam"],
        real["init"], real["marg"], jn(pa).c_str(), jn(pu).c_str(), jn(pa - pu).c_str(), real["msckf"] + real["init"], n_vis, n_tracked, n_used, ang / (t1 - t0),
        dist / (t1 - t0));
  }
  std::fclose(rec);
  return 0;
}
