#!/usr/bin/env python3
"""Round-trip test: generated SDF IMU noise -> gz-sensors' bias process -> effective
random walk, which must equal the random walk the estimators are configured with.

    python3 docker/sim/tools/test_gen_d455_sim.py

PATCHES.md s54: gz-sensors8 (src/GaussianNoiseModel.cc) evolves the dynamic bias as

    bias = exp(-dt/tau) * bias + N(0, sqrt(sigma_b^2 * tau/2 * (1 - exp(-2 dt/tau))))

The test parses <dynamic_bias_stddev>/<dynamic_bias_correlation_time> out of the
GENERATED model.sdf files, runs exactly that recursion at the IMU rate, and compares
the bias spread after a flight-length T with sigma_rw * sqrt(T) from the GENERATED
kalibr_imu_chain.yaml (OpenVINS) and orbslam3_d455_stereo_inertial.yaml (ORB-SLAM3).
It also checks the white-noise <stddev> = density * sqrt(rate).
"""
from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SDFS = ["docker/sim/models/d455/model.sdf", "docker/sim/models/d455_nodepth/model.sdf"]
T_FLIGHT = 120.0      # s: covers sim start -> touchdown of the longest flights
N_REAL = 4000         # Monte Carlo realisations


def gz_bias_walk(sigma_b: float, tau: float, dt: float, n_steps: int, n_real: int,
                 rng: np.random.Generator) -> np.ndarray:
    """gz-sensors8 GaussianNoiseModel dynamic bias, verbatim, from bias = 0."""
    sigma_b_d = math.sqrt(-sigma_b * sigma_b * tau / 2 * math.expm1(-2 * dt / tau))
    phi_d = math.exp(-dt / tau)
    b = np.zeros(n_real)
    for _ in range(n_steps):
        b = phi_d * b + rng.normal(0.0, sigma_b_d, n_real)
    return b


def axis_blocks(sdf: str, kind: str) -> list[tuple[float, float, float]]:
    """(stddev, dynamic_bias_stddev, corr_time) per axis of <angular_velocity>/<linear_acceleration>."""
    block = re.search(rf"<{kind}>(.*?)</{kind}>", sdf, re.S).group(1)
    out = []
    for ax in ("x", "y", "z"):
        a = re.search(rf"<{ax}>(.*?)</{ax}>", block, re.S).group(1)
        g = lambda tag: float(re.search(rf"<{tag}>([0-9.eE+-]+)</{tag}>", a).group(1))
        out.append((g("stddev"), g("dynamic_bias_stddev"), g("dynamic_bias_correlation_time")))
    return out


def yaml_num(txt: str, key: str) -> float:
    return float(re.search(rf"{re.escape(key)}:\s*([0-9.eE+-]+)", txt).group(1))


class ImuStamp(unittest.TestCase):
    """PATCHES s56: Gazebo IMU goes to imu_gz; imu_restamp.py shifts it by IMU_STAMP_LAG_S."""
    def test_restamp_lag_matches_generator(self):
        sys.path.insert(0, str(ROOT / "docker/sim/tools"))
        import gen_d455_sim as g
        src = (ROOT / "docker/scripts/imu_restamp.py").read_text()
        lag = float(re.search(r"^LAG_S = ([0-9.eE+-]+)", src, re.M).group(1))
        self.assertAlmostEqual(lag, g.IMU_STAMP_LAG_S, delta=1e-12)

    def test_bridges_publish_raw_imu_on_imu_gz(self):
        for p in sorted((ROOT / "docker/sim/config_sim_only").glob("gz_bridge_d455*.yaml")):
            txt = p.read_text()
            self.assertIn('ros_topic_name: "/camera/imu_gz"', txt, p.name)
            self.assertNotIn('ros_topic_name: "/camera/imu"\n', txt, p.name)


class RoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        imu = (ROOT / "docker/sim/config_sim_only/kalibr_imu_chain.yaml").read_text()
        orb = (ROOT / "docker/sim/config_sim_only/orbslam3_d455_stereo_inertial.yaml").read_text()
        cls.rate = yaml_num(imu, "update_rate")
        cls.cfg = {
            "angular_velocity": (yaml_num(imu, "gyroscope_noise_density"), yaml_num(imu, "gyroscope_random_walk")),
            "linear_acceleration": (yaml_num(imu, "accelerometer_noise_density"), yaml_num(imu, "accelerometer_random_walk")),
        }
        cls.orb = {
            "angular_velocity": (yaml_num(orb, "IMU.NoiseGyro"), yaml_num(orb, "IMU.GyroWalk")),
            "linear_acceleration": (yaml_num(orb, "IMU.NoiseAcc"), yaml_num(orb, "IMU.AccWalk")),
        }
        cls.sdfs = {p: (ROOT / p).read_text() for p in SDFS}

    def test_estimators_agree(self):
        self.assertEqual(self.cfg, self.orb)

    def test_white_noise(self):
        for p, sdf in self.sdfs.items():
            for kind, (dens, _) in self.cfg.items():
                for sd, _, _ in axis_blocks(sdf, kind):
                    self.assertAlmostEqual(sd, dens * math.sqrt(self.rate), delta=1e-8 * sd, msg=f"{p} {kind}")  # SDF prints 9 sig. digits

    def test_bias_random_walk_round_trip(self):
        rng = np.random.default_rng(54)
        dt = 1.0 / self.rate
        n = int(T_FLIGHT * self.rate)
        for p, sdf in self.sdfs.items():
            for kind, (_, rw) in self.cfg.items():
                for _, sigma_b, tau in axis_blocks(sdf, kind)[:1]:  # axes are generated identically
                    # analytic: per-step increment variance / dt -> sigma_b^2 as dt/tau -> 0
                    eff_analytic = math.sqrt(-sigma_b ** 2 * tau / 2 * math.expm1(-2 * dt / tau) / dt)
                    self.assertAlmostEqual(eff_analytic / rw, 1.0, delta=1e-3, msg=f"{p} {kind} analytic")
                    # Monte Carlo of the exact recursion over a flight
                    b = gz_bias_walk(sigma_b, tau, dt, n, N_REAL, rng)
                    eff_mc = b.std() / math.sqrt(T_FLIGHT)
                    # 4000 realisations: std of the std estimate ~1.1%; allow 5%
                    self.assertAlmostEqual(eff_mc / rw, 1.0, delta=0.05,
                                           msg=f"{p} {kind}: gz walks at {eff_mc:.3g}, configured {rw:.3g}")

    def test_old_convention_would_fail(self):
        """Guard: the s54 bug (sigma_b = rw*sqrt(tau/2)) gives 42x and must not reappear."""
        for p, sdf in self.sdfs.items():
            for kind, (_, rw) in self.cfg.items():
                _, sigma_b, tau = axis_blocks(sdf, kind)[0]
                self.assertLess(sigma_b / rw, 2.0, msg=f"{p} {kind}: sigma_b/rw = {sigma_b / rw:.1f}")


if __name__ == "__main__":
    sys.exit(not unittest.main(exit=False, verbosity=2).result.wasSuccessful())
