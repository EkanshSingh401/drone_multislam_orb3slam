# PATCHES.md — every deviation from the upstream repository

Reproduction target: `Cnoobmaster69/drone_multislam_orb3slam` @ `feature/covins`
(`02e4344`), single UAV **including** the COVINS backend, in Docker on an
Apple Silicon (arm64) Mac.

This file records **every** change made to get the stack to build and run, so
that any later numerical result can be attributed either to the original system
or to a local fix. Nothing here silently rewrites algorithm code; the one class
of change that would have (x86 SIMD intrinsics) turned out to be unnecessary —
see §0.

---

## 0. Architecture audit: what did NOT need patching

Verified before changing anything, because the obvious risk on arm64 is x86-only
SIMD:

| Site | Finding |
|---|---|
| `aslam_cv2` `aslam/common/hamming{,-inl}.h` | Already has a NEON path (`veorq_u8`/`vcntq_u8`) selected by the compiler-predefined `__ARM_NEON`. No patch. |
| `covins_backend/include/covins/brisk/hamming{,-inl}.h` | Vendored BRISK copy, same structure, same `__ARM_NEON` path. No patch. |
| `covins_backend/include/covins/brisk/macros.h` | `__m128i` appears only in a comment; the typedefs are `__may_alias__` only. Architecture-neutral. |
| `aslam_cv2/.../detect_simd.cmake` | Runs `${CMAKE_CXX_COMPILER} -march=native -dM -E`. Verified on arm64 GCC 7.5: exits 0 and reports `__ARM_ARCH 8`, so it takes the NEON branch. `IS_SSE_ENABLED`/`IS_NEON_ENABLED` are set but never consumed anywhere, so the result gates nothing. |
| `-march=native` in `ORB_SLAM3`, `Thirdparty/DBoW2`, `Thirdparty/g2o`, `covins_comm` | Left exactly as upstream wrote it. Verified `g++ -O3 -march=native` exits 0 on both arm64 GCC 7.5 (bionic) and arm64 GCC 13.3 (noble). |
| OpenCV 3.4.2 + contrib (`opencv3_catkin`) | Its CMake args are architecture-neutral (no SSE/AVX flags). Compiled clean on arm64/GCC 7.5 in ~1m38s: *"All 3 packages succeeded"*. |
| `rapidjson` (inside cereal) | Its SSE2 path is opt-in behind `RAPIDJSON_SSE2`, not enabled. |

**Conclusion: zero architecture patches were required.** The only reason the
COVINS container is not simply the upstream one is the base *image*, below.

---

## 1. Base image: `osrf/ros:melodic-desktop-bionic` → `ros:melodic-ros-base-bionic`

*File: `docker/covins/Dockerfile` (replaces `covins/docker/Dockerfile`)*

`docker manifest inspect osrf/ros:melodic-desktop-bionic` resolves to a single
platform, `{"architecture":"amd64","os":"linux"}` — it has no arm64 variant, so
on this host it could only run under emulation. The official `ros` image is
multi-arch (`linux/amd64`, `linux/arm64/v8`), and `packages.ros.org/ros/ubuntu
dists/bionic binary-arm64` still serves 3439 Melodic packages including
`ros-melodic-desktop-full`, so the Melodic side builds and runs **natively**.

Consequences:
- `ros-melodic-desktop-full` → the specific packages `covins_backend` actually
  needs (`rviz`, `pcl-ros`, `tf2-ros`, `tf2-geometry-msgs`, `cmake-modules`,
  `message-generation`, `serial`) plus OpenCV's build deps (`libtbb-dev`,
  `libgtk2.0-dev`, `libvtk6-dev`). Smaller image, same backend.
- Added a headless display stack (`xvfb`, `x11vnc`, `novnc`, `websockify`,
  `fluxbox`) because the osrf desktop image is not the base any more and
  COVINS RViz needs an X server.

**Rosetta note:** Rosetta emulation is enabled in Docker Desktop as the fallback
path. It was not needed — nothing in this build is emulated.

## 2. Dropped the two `apt-key adv --keyserver` calls

*File: `docker/covins/Dockerfile`*

Upstream `covins/docker/Dockerfile` lines 7–8 repair the 2019 ROS key rotation
via `apt-key adv --keyserver hkp://keyserver.ubuntu.com:80`. Omitted because:
`apt-key` is deprecated, keyserver fetches are unreliable from a build sandbox,
and they are unnecessary here — `ros:melodic-ros-base-bionic` points at
`snapshots.ros.org/melodic/final` with a valid baked-in key. Verified:
`apt-get update` exits 0 with no `EXPKEYSIG`/`NO_PUBKEY`, and
`ros-melodic-rviz` resolves to `1.13.30-1bionic.20230330.122946`.

Side benefit: `snapshots.ros.org/melodic/final` is a frozen snapshot, so the
Melodic package set is reproducible rather than drifting.

## 3. Pinned all 17 COVINS dependencies

*New file: `docker/covins/dependencies.pinned.rosinstall`*

Upstream `covins/dependencies.rosinstall` pins only `pangolin` (by SHA) and
`robopt_open` (by branch); the other 15 track their default branch, so
`wstool up` resolves to whatever HEAD is on the build date.

Every entry is now pinned to its default-branch HEAD **as of 2026-02-24**, the
date of the repo's `bbea776` "Successful Covins integration" commit — i.e. the
dependency state the author had when COVINS last worked. These repos are dormant
(newest commit 2023-11-13), so this resolves to current HEAD for all of them;
the pins exist for reproducibility, not to roll anything back.

This matters more than usual: **`covins/fix_eigen_deps.sh` patches
`eigen_catkin`, `ceres_catkin` and `opengv` by absolute line number**
(`sed -i '13d'`, `sed -i '162d'`, …). An upstream file that shifted by one line
would be silently corrupted rather than fail loudly. The pins make those
line-number patches deterministic.

Resulting versions: **Eigen 3.3.4**, **OpenCV 3.4.2** (+ contrib 3.4.2, pinned
by URL+MD5 inside `opencv3_catkin`), **Ceres 1.14.0**, system protobuf 3.0.0.

## 4. Build only `covins_backend`'s dependency closure

*File: `docker/covins/Dockerfile`*

Upstream `install_file.sh` also builds `covins_frontend` and `covins/orb_slam3`
(plus Pangolin and their DBoW2/g2o), which are the **ROS 1** agent frontends.
The agent in this setup is ORB-SLAM3 running in the ROS 2 container, so they are
dead weight. `catkin build covins_backend` builds only the closure, which
excludes them without editing the workspace. The upstream `install_file.sh`
steps that *are* required (`wstool`, `fix_eigen_deps.sh`, `eigen_catkin` +
`opencv3_catkin` first, the bundled `covins_backend/thirdparty/DBoW2`, unzipping
`ORBvoc.txt`) are all replicated in the same order.

Also skipped the upstream `vision_opencv`/`cv_bridge` rebuild stage: that exists
to link `cv_bridge` against `opencv3_catkin` for the ROS 1 agents.
`covins_backend` does not depend on `cv_bridge`.

## 5. `covins_comm` — restored the catkin build path

*File: `covins/covins_comm/CMakeLists.txt`*

The repo's standalone-library refactor commented out
```cmake
# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)
```
but left `cs_add_library()` / `cs_install()` / `cs_export()` in the
`if (NOT BUILD_LIBRARY)` branch. Those are catkin_simple macros, so with the
lookup removed they are unresolvable CMake commands and the ROS 1 build of
`covins_comm` — which `covins_backend` depends on — cannot configure.

Fix: re-enable the lookup for that branch only, leaving the `BUILD_LIBRARY`
(ROS 2 / standalone) path exactly as the repo had it:
```cmake
if(NOT BUILD_LIBRARY)
  find_package(catkin_simple REQUIRED)
  catkin_simple(ALL_DEPS_REQUIRED)
endif()
```

## 6. `covins_comm` — find Sophus

*File: `covins/covins_comm/CMakeLists.txt`*

The repo's local `utils_base.{hpp,cpp}` changes replaced the `cv::Mat` overloads
of `ToEigenMat44d()`/`ToEigenVec3d()` with `Sophus::SE3f`/`Eigen::Vector3f` ones
(ORB-SLAM3 v1.0 switched its pose type from `cv::Mat` to `Sophus::SE3f`), and
`utils_base.hpp` now does `#include <sophus/se3.hpp>`. Nothing in the build
provided that include path.

Added a `find_path` for `sophus/se3.hpp` that **prefers the Sophus vendored
inside the ORB-SLAM3 tree being linked** (`-DSOPHUS_INCLUDE_DIR=...`), falling
back to `covins/orb_slam3/Thirdparty/Sophus` then the system. The preference is
deliberate: these overloads are called across the `covins_comm` ↔ `ORB_SLAM3`
boundary, so two different Sophus versions would mean two different mangled
signatures.

## 7. `covins_comm` — install `config/` and `thirdparty/cereal`

*File: `covins/covins_comm/CMakeLists.txt`*

Two install rules were missing from the standalone path:

**`config/`** — `config_comm.hpp` resolves its runtime YAML path at *compile*
time from `__FILE__`:
```cpp
const std::string s0_comm (__FILE__);
const std::size_t p0_comm = s0_comm.find("include/covins");
const std::string conf_comm = s0_comm.substr(0,p0_comm) + "config/config_comm.yaml";
```
With headers installed at `<prefix>/include/covins/...`, that resolves to
`<prefix>/config/config_comm.yaml`. If that file is absent the failure is
**silent** — the `covins_params` values are `const std::string`s initialised at
static-init time from a missing file. Now installed, and the Dockerfile asserts
its presence.

**`thirdparty/` (cereal)** — cereal is a *public* dependency of the installed
headers: `typedefs_base.hpp`, which every covins header pulls in, does
`#include <cereal/cereal.hpp>`. Without this, ORB-SLAM3 could not compile a
single translation unit against the installed tree
(`fatal error: cereal/cereal.hpp: No such file or directory`). This was an
actual observed build failure, not a precaution.

## 8. Hardcoded absolute paths

Commit `5fa1a1e`. Upstream had 4 foreign home directories baked into source.

| File | Was | Now |
|---|---|---|
| `orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake` | `set(ORB_SLAM3_ROOT_DIR "~/ws_offboard_control/src/ORB_SLAM3")` | honours `-D`/`$ORB_SLAM3_ROOT_DIR` first; fallback uses `$ENV{HOME}`. **CMake does not expand `~` in `find_path`/`find_library` PATHS**, so the original could not have worked unless the variable was already set. |
| `orb_slam3_ros2_wrapper/launch/rgbd.launch.py` | `/home/carlos/...` vocabulary + settings | `$ORB_SLAM3_ROOT_DIR`/`$ORB_SLAM3_VOCABULARY`, settings from the installed `share/params` |
| `.../launch/stereo.launch.py` | `/home/carlos/...` | same scheme |
| `.../launch/rgbd_imu.launch.py`, `mono_imu.launch.py` | `/home/orb/...`, `/root/colcon_ws/...` | same scheme |
| `.../launch/unirobot.launch.py` | `/root/colcon_ws/.../monitor_cpu_ram.sh` | resolved via `get_package_prefix()`; also added the missing `install(PROGRAMS)` entries so the referenced scripts actually exist |
| `covins/covins_backend/config/config_backend.yaml` | `sys.map_path0: /home/manthan/ws2/...` | `/root/covins_ws/src/covins/...` |
| `README.md` | `-p config_file:=/home/carlos/...` | `$(ros2 pkg prefix --share multi_slam)/config/gz_bridge.yaml` |

(`covins/docs/`, `covins/orb_slam3/covins_examples/*.sh` and
`covins_backend/thirdparty/DBoW2/.../demoBRISK2.cpp` also contain
`/home/pschmuck` and `/home/fabiola` dataset paths. Left alone: they are
upstream COVINS EuRoC example scripts and docs, not on this build or run path.)

## 9. Bridge namespace mismatch — single-UAV bringup could not have worked

*File: `multi_slam/config/gz_bridge.yaml`*

`gz_bridge.yaml` published `uav_3/rgb/image_raw` and `uav_3/depth/image`, but
`rgbd.launch.py` with `ag_n:=0` (the documented single-agent invocation)
subscribes to `/uav_1/rgb/image_raw` and `/uav_1/depth/image`. Nothing connected
them. Changed `uav_3` → `uav_1`; the commented-out `uav_2` block is left as
upstream had it.

## 10. Added `/clock` and ground-truth bridges

*File: `multi_slam/config/gz_bridge.yaml`*

- `/clock` (`gz.msgs.Clock` → `rosgraph_msgs/msg/Clock`): upstream had it
  commented out, yet every node is supposed to run `use_sim_time:=true`. Without
  it those nodes block on a zero clock.
- `ground_truth/pose_info` (`gz.msgs.Pose_V` → `tf2_msgs/msg/TFMessage`) from
  `/world/forest/dynamic_pose/info`, for the ATE/RPE evaluation. Deliberately
  **not** bridged onto `/tf`, so ground truth can never be mistaken for, or
  compete with, the SLAM TF tree.

## 11. `ros1_bridge` made opt-in

*File: `multi_slam/launch/bringup.launch.py`*

`bringup.launch.py` unconditionally ran `ros2 run ros1_bridge parameter_bridge`,
which fails on any system without `ros1_bridge` built. It is now behind
`use_ros1_bridge` (default `false`).

This is correct rather than merely convenient: the ORB-SLAM3 → COVINS path does
**not** use ros1_bridge. `ORB_SLAM3/src/System.cc:271` constructs
`Communicator(covins_params::sys::server_ip, covins_params::sys::port, ...)`
and `CommunicatorBase::ConnectToServer()` opens a raw TCP socket. ros1_bridge is
only relevant to the repo's separate, still-in-progress goal of bringing COVINS
poses *back* into ROS 2.

## 12. `use_sim_time` propagation

*Files: `multi_slam/launch/bringup.launch.py`, `multi_slam/launch/static_frames.launch.py`*

`bringup.launch.py` included `rgbd.launch.py` with no `launch_arguments`, so the
SLAM node silently used its own defaults instead of the bringup's
`robot_namespace`/`use_sim_time`. `static_frames.launch.py` had no
`use_sim_time` argument at all. Both now accept and forward it, so every node in
the stack is on the Gazebo clock.

## 13. Default `sys.server_ip` → the Compose service name

*File: `covins/covins_comm/config/config_comm.yaml`*

Was `127.0.0.1`, which cannot reach another container. Now `covins-backend`.
This is safe because `ConnectToServer()` resolves the address with
`getaddrinfo()`, so a DNS name works as well as a literal IP — Docker's embedded
DNS resolves the service name on the shared network. Previous values are kept as
comments. The sim container's entrypoint rewrites this from `$COVINS_SERVER_IP`
at startup.

## 14. Purge committed x86-64 build artifacts

*Files: `docker/sim/Dockerfile`, `docker/covins/Dockerfile`, `.dockerignore`*

The repo commits prebuilt shared objects and CMake trees:
`ORB_SLAM3/lib/libORB_SLAM3.so`, `ORB_SLAM3/Thirdparty/{DBoW2,g2o}/lib/*.so`,
`covins/covins_comm/lib/libcovins_comm.so`, plus `ORB_SLAM3/build/` and
`covins/covins_comm/build/`. `file` reports all of them as
`ELF 64-bit LSB shared object, x86-64`, and the CMake caches are baked with
`/home/carlos/...` paths (`COVINS_COMM_LIB:FILEPATH=/home/carlos/.local/lib/libcovins_comm.so`).

They are unusable on arm64 and a stale `CMakeCache.txt` would poison configure,
so they are removed inside the image and excluded from the build context. **The
repository is not modified** — the files remain in git history and on the
branch; only the container ignores them.

## 15. Pangolin pinned to `v0.9.1` for the ROS 2 side

*File: `docker/sim/Dockerfile`*

The repo pins Pangolin `b8abe866` (2019) only for the ROS 1 COVINS workspace;
the ROS 2 side used whatever was on the author's host, so there was nothing to
reproduce. `b8abe866` does not compile with GCC 13, so the sim image pins
`v0.9.1`, which builds clean on arm64/GCC 13.

Low risk: ORB-SLAM3's use of Pangolin here is link-only — the viewer is off via
`visualization: false` (ROS params) and `orb.activate_visualization: 0`
(`config_comm.yaml`).

## 16. Micro XRCE-DDS Agent added

*File: `docker/sim/Dockerfile`* — pinned `v2.4.3`.

Not in the repo's container story at all (the README calls it "optional at
current stage"), but the scripted flight in step 6 needs PX4 offboard control
over uORB, which requires the agent.

## 17. `libprotobuf-dev` added — a regression caused by §1/§4

*File: `docker/covins/Dockerfile`*

`aslam_cv_common` (needed by `covins_backend`) depends on `protobuf_catkin`,
whose `USE_SYSTEM_PROTOBUF` is `AUTO`: it uses the system protobuf when
`find_package(Protobuf)` reports a version `> 2.6.1`, and otherwise compiles
**protobuf 2.6.1 from source** — a path whose `autogen.sh` fetches gmock from
the long-dead `googlecode.com`.

Upstream never reached that path: `osrf/ros:melodic-desktop-bionic` plus
`ros-melodic-desktop-full` pull `libprotobuf-dev` in transitively. The slimmer
base from §1/§4 does not, so `AUTO` found nothing and fell back to the source
build, which failed (`Failed << protobuf_catkin:make`, taking 10 downstream
packages with it).

Diagnosis, step by step — worth recording because the obvious guess was wrong:

1. First hypothesis was an arm64 gap in protobuf 2.6.1. **Checked and rejected**:
   its `platform_macros.h` does define `GOOGLE_PROTOBUF_ARCH_AARCH64`, and
   `atomicops.h` selects a real `atomicops_internals_arm64_gcc.h` for it. So
   protobuf 2.6.1 supports aarch64 and this is *not* an architecture problem.
2. Checked the base image: `dpkg -l | grep protobuf` → **0 packages**.
3. Installed `libprotobuf-dev` (bionic/arm64 ships `3.0.0-9.1ubuntu1.1`) and
   re-ran `protobuf_catkin`'s exact detection logic under CMake 3.10.2:
   `Protobuf_FOUND=TRUE`, `Protobuf_VERSION=3.0.0` →
   `AUTO would select USE_SYSTEM_PROTOBUF=ON`.

Fix: add `libprotobuf-dev` and `protobuf-compiler` (plus `libboost-all-dev`,
another thing `desktop-full` used to supply). This *restores* upstream
behaviour rather than diverging from it — the source build was never intended.

Note `USE_SYSTEM_PROTOBUF` is declared `CACHE INTERNAL`, which implies `FORCE`,
so `-DUSE_SYSTEM_PROTOBUF=ON` on the command line would be silently overwritten.
Installing the package is the only way to steer this. The Dockerfile now asserts
`! test -d build/protobuf_catkin/protobuf_src` so a regression here fails loudly.

## 18. Sophus include path for `orb_slam3_ros2_wrapper`

*File: `orb_slam3_ros2_wrapper/CMakeLists.txt`*

The wrapper calls `find_package(Sophus REQUIRED)` but never lists `Sophus` in
the `dependencies` passed to `ament_target_dependencies`, so its include
directory was never applied:
`type_conversion.hpp:12:10: fatal error: sophus/se3.hpp: No such file or directory`.

Added `${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus` to `include_directories()`.
Two reasons for that specific copy rather than `ros-jazzy-sophus`:

1. `ros-jazzy-sophus` installs a **flat** `/opt/ros/jazzy/include/sophus/se3.hpp`.
   `/opt/ros/jazzy/include` is not a default include directory under Jazzy's
   isolated layout, so `<sophus/se3.hpp>` would still not resolve without
   explicitly adding it.
2. `Sophus::SE3f` crosses the wrapper ↔ `libORB_SLAM3.so` ↔ `libcovins_comm.so`
   boundaries (see §6). All three must instantiate the same Sophus templates;
   two versions would be an ODR violation.

## 19b. Sophus and COVINS headers are *public* dependencies — consumers need them

Two further build failures, both the same shape as §6/§7 but one level out: the
repo's local modifications turned Sophus and COVINS into **public** dependencies
of headers that other packages include, and nothing propagated them.

**Sophus → `covins_backend`** (*file: `docker/covins/Dockerfile`*).
`covins_comm/include/.../utils_base.hpp` declares
`ToEigenMat44d(const Sophus::SE3f&)` and `#include <sophus/se3.hpp>`, so every
*consumer* of covins_comm needs Sophus too, not just covins_comm itself:
`covins_backend` failed with
`utils_base.hpp:32:10: fatal error: sophus/se3.hpp: No such file or directory`.
Fixed by installing the header-only Sophus to `/usr/local/include/sophus`,
which fixes covins_comm and covins_backend in one place without editing
`covins_backend/CMakeLists.txt`. The source is the copy the repo vendored at
`covins/orb_slam3/Thirdparty/Sophus`, which is **byte-identical** to
`ORB_SLAM3/Thirdparty/Sophus` used by the ROS 2 container — so the whole
project uses a single Sophus version, and §6's ODR concern is satisfied
project-wide rather than only inside one container.

**COVINS → `orb_slam3_ros2_wrapper`** (*file: `orb_slam3_ros2_wrapper/CMakeLists.txt`*).
Grafting the COVINS communicator into ORB-SLAM3 made COVINS a public dependency
of ORB-SLAM3's headers: `ImuTypes.h` does
`#include <covins/covins_base/typedefs_base.hpp>` (which itself includes
cereal). So the wrapper failed with
`ImuTypes.h:38:10: fatal error: covins/covins_base/typedefs_base.hpp: No such file or directory`.
Added a `find_path` for the COVINS include tree (mirroring the one already in
`ORB_SLAM3/CMakeLists.txt`), with a `FATAL_ERROR` that explains the dependency
rather than failing with a bare missing-header message.

## 19c. Build parallelism capped (memory, not correctness)

*Files: `docker/sim/Dockerfile`, `docker/covins/Dockerfile`*

With a 16 GB Docker VM, these builds are memory-bound rather than CPU-bound:

- `orb_slam3_ros2_wrapper`'s translation units pull in ORB-SLAM3 + Eigen + PCL +
  Sophus templates under C++20, and a single `cc1plus` can exceed 2 GB.
- `opengv` and OpenCV 3.4.2 have similarly heavy units.

Running the two images' builds **concurrently** OOM-killed both
(`c++: fatal error: Killed signal terminated program cc1plus`, BuildKit
`ResourceExhausted`). The sim image now builds its colcon workspace with
`--executor sequential --parallel-workers 1` and `MAKEFLAGS=-j1`, the backend
image defaults to `NR_JOBS=2`, and **the two images must be built one at a
time**. `docker compose build` does this sequentially by default.

This is a resource limit of this host, not a property of the code.

## 19. `-DBUILD_TESTING=OFF` for the wrapper

*File: `docker/sim/Dockerfile`*

The wrapper's `if(BUILD_TESTING)` block calls `find_package(gtest)` (lowercase,
which is not a real package) and `ament_find_gtest()`. colcon enables
`BUILD_TESTING` by default, producing
`Could not find a package configuration file provided by "gtest"`.
Tests are not part of this reproduction, so the image builds with testing off.
Upstream's test code is untouched.

---

## Pinned versions, for the record

| Component | Version / commit | How it was determined |
|---|---|---|
| ROS 2 | Jazzy (Ubuntu 24.04 noble) | `orb_slam3_ros2_wrapper/CMakeLists.txt:16` hardcodes `/opt/ros/jazzy/lib/python3.12/site-packages/` |
| Gazebo | Harmonic (`gz-harmonic`) | PX4's `Tools/setup/ubuntu.sh` at the pinned commit; also the `ros_gz` pairing for Jazzy |
| `px4_msgs` | upstream `main` `7c596a0b6d60b3107c9ef656a451296b56ccba34` | **exact content match**: all 238 `.msg` git blob hashes identical. Neighbouring commits differ by 2–6 blobs |
| PX4-Autopilot | `main` `6bc24c8cd1485edbf1b3b11565db4108f7c2fcaa` (2026-01-06) | that px4_msgs commit's message is literally `Update to PX4 6bc24c8...`. In the v1.17 dev cycle; `diverged` from `v1.17.0` (303 ahead / 27 behind), so **no release tag works** |
| PX4-gazebo-models | `fe3fe236e36a3ed5bce01a7501347d20a466c407` | submodule pin at that PX4 commit; contains `worlds/forest.sdf` and `models/x500_depth` |
| ROS 1 | Melodic, `snapshots.ros.org/melodic/final` | `covins/docker/Dockerfile:3` |
| Eigen | 3.3.4 | `fix_eigen_deps.sh` + `covins_backend`'s `find_package(Eigen3 3.3.4 EXACT REQUIRED)` |
| OpenCV (ROS 1) | 3.4.2 + contrib 3.4.2 | `opencv3_catkin` URL + MD5 |
| OpenCV (ROS 2) | 4.6.0 (noble `libopencv-dev`) | satisfies ORB-SLAM3's `find_package(OpenCV 4.2)` minimum |
| Ceres | 1.14.0 | `ceres_catkin` `GIT_TAG` |
| Pangolin | `v0.9.1` (ROS 2) / `b8abe866` (ROS 1, unused) | see §15 |
| Micro XRCE-DDS Agent | `v2.4.3` | see §16 |

## Notes that are not patches, but affect interpretation

- **`placerec.inter_map_matches_only: 1`** in the repo's `config_backend.yaml`
  (upstream: `0`). With a **single** agent there is no second map, so COVINS will
  attempt essentially no place recognition. Expect keyframes to be received and
  the map to grow, but **no loop closures** — that is the configuration, not a
  fault. The repo also loosens the thresholds substantially
  (`matches_thres` 25→18, `inliers_thres` 20→12, `start_after_kf` 7→5).
- **COVINS is a visual-*inertial* system**; this runs it with RGB-D and no IMU.
  The repo claims success, but this is the main substantive risk for keyframe
  acceptance.
- **Eigen alignment**: `covins_comm` forces
  `-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16` while
  `covins_backend` does not. On arm64 Eigen's default is already 16, so the two
  agree; on x86-64 they also agree at 16 because `covins_backend` does not use
  `-march=native`. Consistent by default on both, but by luck rather than
  design — worth knowing if either flag set ever changes.
- **Cross-architecture wire compatibility** is relied upon only if the two
  containers are ever built for different architectures. The format is
  `cereal::BinaryOutputArchive` (non-portable: no endian/width normalisation),
  which is safe between amd64 and arm64 since both are little-endian LP64, but
  would break against a big-endian or ILP32 peer. Here both containers are
  arm64, so the question is moot.
