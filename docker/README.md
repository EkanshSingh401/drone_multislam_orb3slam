# Dockerized single-UAV ORB-SLAM3 + COVINS reproduction

Two containers on one Docker network reproduce the single-UAV setup from this
repo's `feature/covins` branch, **including** the COVINS backend.

Built and tested on **Apple Silicon (arm64) macOS**. Both images are **native
arm64** — nothing is emulated. See [PATCHES.md](PATCHES.md) for every deviation
from the upstream repo and for the derivation of the version pins.

```
┌────────────────────────── slamnet (docker bridge) ───────────────────────────┐
│                                                                              │
│  sim  (ROS 2 Jazzy / Ubuntu 24.04)          covins-backend  (ROS Melodic)     │
│  ───────────────────────────────────        ──────────────────────────────    │
│  gz sim -s  (Gazebo Harmonic, headless)     roscore                           │
│  PX4 SITL   (x500_depth, forest world)      covins_backend_node               │
│  MicroXRCEAgent                             rviz  (COVINS)                    │
│  ros_gz_bridge ──> /uav_1/rgb, /uav_1/depth                                   │
│  ORB-SLAM3 RGB-D ══════ TCP 9033 ═════════> covins_comm listener              │
│  rviz2                                                                        │
│  noVNC :6901                                noVNC :6902                       │
└──────────────────────────────────────────────────────────────────────────────┘
```

**The agent→backend link is COVINS's own TCP socket, not ROS topics and not
`ros1_bridge`.** `ORB_SLAM3/src/System.cc:271` constructs
`Communicator(covins_params::sys::server_ip, covins_params::sys::port, ...)`, and
`CommunicatorBase::ConnectToServer()` resolves the address with `getaddrinfo()`
— so the Compose service name `covins-backend` works directly as `server_ip`.
`ros1_bridge` is only needed for the repo's separate goal of bringing COVINS
poses *back* into ROS 2, and is off by default.

## Requirements

- Docker Desktop with **≥ 16 GB** allocated (Settings → Resources → Memory).
  8 GB is not enough: Gazebo + ORB-SLAM3 + a backend holding a ~145 MB ORB
  vocabulary will thrash.
- ~25 GB free disk for both images.
- No GPU passthrough exists on macOS Docker, so Gazebo's sensor pipeline and
  RViz both render through Mesa **llvmpipe** (software). This is the dominant
  cost in the real-time factor — expect RTF well below 1.0.

## Build

```bash
# from the repository root
docker compose -f docker/compose.yaml build
```

**Build the two images one at a time** (which is what `docker compose build`
does by default — don't add `--parallel`). These builds are memory-bound: a
single `cc1plus` on the wrapper can exceed 2 GB, and running both images
concurrently OOM-kills them even with 16 GB. See [PATCHES.md](PATCHES.md) §19c.

The sim image builds PX4-Autopilot at a pinned commit, Pangolin, the Micro
XRCE-DDS agent, ORB-SLAM3 (+ DBoW2, g2o), `covins_comm` as a standalone shared
library, and the colcon workspace. The backend image compiles OpenCV 3.4.2,
Ceres 1.14.0 and the COVINS backend. Budget ~30–45 min total on an M-series Mac.

## Run

```bash
docker compose -f docker/compose.yaml up -d

# 1. COVINS backend (own terminal -- this streams the backend's stdout, which is
#    the only place keyframe receipt is visible)
docker compose -f docker/compose.yaml exec covins-backend /opt/scripts/run_backend.sh --rviz

# 2. Simulation stack: Gazebo + PX4 + bridge + static TF + ORB-SLAM3
docker compose -f docker/compose.yaml exec sim /opt/scripts/bringup_sim.sh

# 3. Verify before trusting anything
docker compose -f docker/compose.yaml exec sim /opt/scripts/check_rates.sh
```

Start the backend **first**: the agent connects once, at ORB-SLAM3 construction
time, and does not retry.

### Displays

| What | URL |
|---|---|
| RViz 2 (ROS 2) | <http://localhost:6901/vnc.html> |
| COVINS RViz (ROS 1) | <http://localhost:6902/vnc.html> |

Raw VNC is on 5901 / 5902 if you prefer a native client. Gazebo has no GUI at
all — `gz sim -s` is server-only, which is what makes it headless.

```bash
# RViz 2, onto the sim container's VNC display
docker compose -f docker/compose.yaml exec sim bash -lc 'rviz2 --ros-args -p use_sim_time:=true'
```

## Evaluate against ground truth

```bash
# scripted flight + bag + ATE/RPE for ORB-SLAM3
docker compose -f docker/compose.yaml exec sim /opt/scripts/record_and_eval.sh

# COVINS's optimised keyframe trajectory, then re-evaluate including it
docker compose -f docker/compose.yaml exec covins-backend /opt/scripts/export_covins_trajectory.sh
docker compose -f docker/compose.yaml exec sim /opt/scripts/record_and_eval.sh --eval-only /out/eval/<run>
```

Ground truth comes from Gazebo's `/world/forest/dynamic_pose/info`, bridged to
`/ground_truth/pose_info` as a `TFMessage` — deliberately **not** onto `/tf`, so
it can never compete with the SLAM TF tree. `evo` is run with `-a` (SE(3)
Umeyama alignment) because the SLAM map frame and the Gazebo world frame have
different origins; scale is **not** fitted, since RGB-D SLAM is metric and
letting evo solve for scale would hide real scale error.

Everything lands in `docker/out/` on the host.

## Scripts

| Script | Container | Purpose |
|---|---|---|
| `bringup_sim.sh` | sim | Start the whole sim stack, with a health gate at each step. `--stop` tears it down. |
| `check_rates.sh` | sim | Measure RTF, topic rates, tracking state, backend reachability. |
| `fly_path.py` | sim | Scripted PX4 offboard flight (climb + yawing rectangle). |
| `record_and_eval.sh` | sim | Record a flight and compute ATE/RPE with evo. |
| `bag_to_tum.py` | sim | Extract TUM trajectories from a rosbag2. |
| `run_backend.sh` | covins-backend | Start `covins_backend_node` (`--rviz` for COVINS RViz). |
| `export_covins_trajectory.sh` | covins-backend | Trigger GBA and export the optimised trajectory as TUM. |

## Gotchas worth knowing

- **`config_comm.yaml`'s path is baked in at compile time.** `config_comm.hpp`
  derives it from `__FILE__`, so the agent reads
  `/root/.local/config/config_comm.yaml` and the backend reads
  `/root/covins_ws/src/covins/covins_comm/config/config_comm.yaml`. A missing
  file fails **silently** — the parameters are `const std::string`s initialised
  at static-init time. Compose mounts the *same* host file into both so their
  comm parameters cannot diverge.
- **`comm.start_sending_after_kf` delays the first keyframe** (5 by default),
  and `comm.kf_buffer_withold` holds back the newest 5. The backend will look
  idle until the agent has built enough keyframes — move the drone.
- **`placerec.inter_map_matches_only: 1`** in this repo's `config_backend.yaml`.
  With one agent there is no second map, so expect **no loop closures**. That is
  the configuration, not a failure.
- **COVINS is a visual-*inertial* system** and this runs it RGB-D with no IMU.
  Keyframes flow, but treat the backend's inertial machinery as unexercised —
  use GBA `action: 4`/`5` (visual), never `0`/`1` (visual-inertial).
- **ORB-SLAM3 is linked against `libcovins_comm.so`**; the image build asserts
  this with `ldd`, because a silently-unlinked build would look fine and simply
  never contact the backend.

---

# Phase 2: the D455-mirror rig (`x500_d455`)

A second drone variant carrying a simulated **D455-mirror** sensor head, so a
VIO estimator developed against a real D455 on a Jetson can be run unchanged in
simulation.

| | |
|---|---|
| stereo IR pair | 848x480 @ 30 Hz, monochrome, **95 mm** baseline |
| depth | 848x480 @ 30 Hz, 32FC1 metres, aligned to the left imager |
| IMU | 200 Hz, explicit noise model |
| intrinsics | fx = fy = 446.802773, cx = 424, cy = 240, zero distortion |

**Topic names match the Jetson stack exactly**, so OpenVINS configs and launch
files transfer unchanged:

```
/camera/infra1/image_rect_raw      /camera/infra1/camera_info
/camera/infra2/image_rect_raw      /camera/infra2/camera_info
/camera/depth/image_rect_raw       /camera/depth/camera_info
/camera/imu
```

## Everything is generated from one spec

`docker/sim/tools/gen_d455_sim.py` is the single source of truth. It emits the
Gazebo SDF **and** the estimator configs together:

```bash
python3 docker/sim/tools/gen_d455_sim.py            # print the derivation
python3 docker/sim/tools/gen_d455_sim.py --write    # regenerate
python3 docker/sim/tools/gen_d455_sim.py --check    # fail if anything is stale
```

Generated:

| file | purpose |
|---|---|
| `docker/sim/models/d455/model.sdf` | sensor module |
| `docker/sim/models/x500_d455/model.sdf` | x500 carrying it |
| `config_sim_only/kalibr_imu_chain.yaml` | OpenVINS IMU noise (continuous densities) |
| `config_sim_only/kalibr_imucam_chain.yaml` | OpenVINS intrinsics + `T_cam_imu` |
| `config_sim_only/orbslam3_d455_stereo_inertial.yaml` | ORB-SLAM3 stereo-inertial |
| `config_sim_only/gz_bridge_d455.yaml` | Jetson-named bridge mapping |

The SDF and the estimator configs must agree **exactly** — otherwise a VIO run
silently measures calibration error instead of estimator error. One spec plus a
`--check` mode is how that is enforced.

## IMU noise units — read this before touching the numbers

Gazebo's `<noise><stddev>` is a **per-sample (discrete)** sigma. OpenVINS,
Kalibr and ORB-SLAM3 all want **continuous-time densities**:

```
sigma_discrete = sigma_density * sqrt(rate)      # sqrt(200) = 14.142
```

Mixing them up misstates the IMU noise by ~14x, and nothing errors — the filter
just mis-weights the IMU. Bias is worse: Gazebo models it as a first-order
Gauss-Markov (OU) process via `dynamic_bias_stddev` / `dynamic_bias_correlation_time`,
not as a random walk, and `sigma_rw = sigma_b * sqrt(2/tau)`. The spec declares
the densities and derives Gazebo's values, never the reverse.

Full derivation, geometry table and exact camera-IMU extrinsics:
[`config_sim_only/D455_SIM_CALIBRATION.md`](sim/config_sim_only/D455_SIM_CALIBRATION.md).

**`config_sim_only/` is simulation-only by construction.** Nothing in it may be
copied over a real sensor's Kalibr output.

## Bringing it up

`bringup_sim.sh` serves both rigs; the phase-1 defaults are unchanged.

```bash
# D455-mirror rig, no SLAM (there is no stereo-inertial node in the wrapper yet)
docker compose -f docker/compose.yaml exec sim \
  env MODEL=gz_x500_d455 START_SLAM=0 /opt/scripts/bringup_sim.sh

# measure what the three image streams and the IMU actually deliver
docker compose -f docker/compose.yaml exec sim \
  python3 -u /opt/scripts/rate_monitor.py --stereo --duration 180
```

`rate_monitor.py` exists because Gazebo's own `real_time_factor` field is
instantaneous and strongly bimodal — sampling it gave 0.032 on one run and 0.538
on another, while the true average over the same flight was **0.289**. The only
honest RTF is simulated time over wall time across the window, which is what it
reports.
