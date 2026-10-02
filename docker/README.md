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
