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

# Findings that are upstream bugs, not porting issues

The three items below were found by running the stack, not by reading it. Each
one independently prevents the single-UAV RGB-D path from working, on any host.

## 20. Gazebo systems come from the server config, not the world SDF

*File: `docker/scripts/bringup_sim.sh`*

A bare `gz sim -s forest.sdf` brings up the world and the drone, and everything
*looks* healthy — but no camera topics ever appear. The reason:
`grep -c "<plugin" forest.sdf` is **0**. PX4's world SDFs declare no systems at
all; every system, including `gz-sim-sensors-system` (which is what renders the
cameras), comes from `GZ_SIM_SERVER_CONFIG_PATH`.

PX4 generates the correct environment at build time, so the bringup now sources
`build/px4_sitl_default/rootfs/gz_env.sh`, which sets
`GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH` (without which the model
logs `Failed to load system plugin [MotorFailurePlugin]`) and
`GZ_SIM_SERVER_CONFIG_PATH`. The repo's README does not mention any of this
because it assumes PX4 launches Gazebo itself.

## 21. The depth camera's topic is overridden — upstream bridged the wrong name

*File: `multi_slam/config/gz_bridge.yaml`*

Upstream bridges depth from
`/world/forest/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image`.
**Nothing ever publishes there.** The OakD-Lite SDF contains an explicit
`<topic>depth_camera</topic>` for that sensor, which overrides the scoped name.
Gazebo says so plainly at `-v 4`:

```
[DepthCameraSensor.cc:283] Depth images for [..::StereoOV7251] advertised on [depth_camera]
[DepthCameraSensor.cc:311] Points      for [..::StereoOV7251] advertised on [depth_camera/points]
[CameraSensor.cc:695]      Camera info for [..::StereoOV7251] advertised on [/camera_info]
```

This is a nasty failure mode: the scoped topic *does* show up in `gz topic -l`,
because the bridge subscribes to it — so it looks like it exists. Only
`gz topic -i` reveals `No publishers on topic [...]` alongside the bridge's two
subscribers. The RGB camera has no `<topic>` override, which is why RGB worked
and depth did not.

Corrected to `/depth_camera`, `/camera_info`, `/depth_camera/points`. Verified:
`gz topic -e -t /depth_camera` returns `width: 640, height: 480,
pixel_format_type: R_FLOAT32`, which matches `DepthMapFactor: 1.0` (metres) in
`gazebo_rgbd.yaml`.

**Carry-over for multi-UAV:** these names are unscoped, so every drone's depth
camera would publish on the *same* `/depth_camera`. That must be resolved before
adding a second agent.

## 22. RGB and depth cameras have incompatible geometry

*File: `docker/sim/Dockerfile` (patches the pinned PX4-gazebo-models OakD-Lite model)*

| Sensor | Type | Resolution | horizontal_fov |
|---|---|---|---|
| IMX214 | `camera` | **1920×1080** | 1.204 |
| StereoOV7251 | `depth_camera` | 640×480 | 1.274 |

Meanwhile `gazebo_rgbd.yaml` declares `Camera.width: 640`, `Camera.height: 480`,
`fx: 432.496`, `cx: 320`, `cy: 240`. Those are the **depth** camera's intrinsics
(640/2/tan(1.274/2) = 433.9), applied to the RGB stream. Verified by measurement:
`ros2 topic echo /uav_1/rgb/image_raw --field width` returns **1920**.

Nothing in the pipeline resizes — checked `rgbd-slam-node.cpp`,
`orb_slam3_interface.cpp` and `scripts/sync_repub.py` (the last is a stereo
stamp-synchroniser that republishes unchanged).

As committed this cannot work: ORB-SLAM3's RGB-D path indexes the depth image at
the grayscale image's pixel coordinates, so a 1920×1080 RGB against a 640×480
depth map reads out of bounds, and the declared intrinsics are wrong for the RGB
frame regardless of that.

Fixed by setting IMX214 to 640×480 with `horizontal_fov` 1.274, so the RGB
stream matches both the depth camera and the repo's own committed calibration.
Applied in the Dockerfile because the model belongs to the pinned
PX4-gazebo-models submodule rather than to this repo.

Measured side effect on this host (no GPU, Mesa llvmpipe):

| | before (1920×1080) | after (640×480) |
|---|---|---|
| Gazebo real-time factor | 0.028 | ~0.6 (oscillating 0.07–1.0) |
| `/uav_1/rgb/image_raw` | 1.53 Hz | 8.38 Hz |

So the correctness fix is also what makes this tractable without a GPU. The
resolution change is nonetheless a deviation from the model as pinned, and is
flagged as such.

## 23. `use_sim_time` was declared but never applied to the SLAM node

*File: `orb_slam3_ros2_wrapper/launch/rgbd.launch.py`*

`rgbd.launch.py` declares a `use_sim_time` launch argument, but the value
appears in **none** of the three places that would reach the node: not in
`param_substitutions`, not in `params/ros_params/gazebo-rgbd-ros-params.yaml`,
and not in the `Node(...)` `parameters` list. So the SLAM node ran on the wall
clock no matter what was passed. Measured before the fix:

```
/ros_gz_bridge                 use_sim_time=True
/uav_1/ORB_SLAM3_RGBD_ROS2     use_sim_time=False
```

This silently invalidates any evaluation. Ground truth is bridged from Gazebo
with `use_sim_time:=true`, i.e. on sim time, while the estimator stamped its
poses with wall-clock time. At the measured real-time factor the two clocks
diverge by ~30x, so `evo` would be aligning stamps that do not correspond.
Nothing crashes; the ATE just comes out meaningless.

Fixed by passing `{'use_sim_time': <bool>}` as its own parameter dict after
`configured_params`, so it applies regardless of whether `RewrittenYaml`
inserts keys that are absent from the source YAML. Verified after the fix:
`use_sim_time=True` on every node that has the parameter.

## 24. PX4 must run with `-d`; its shell spam starved its own scheduler

*File: `docker/scripts/bringup_sim.sh`*

Without `-d` (daemon mode) PX4 starts its interactive `pxh` shell and writes
the prompt plus an ANSI clear-line sequence to the redirected log on every
loop. A ~45 minute run produced a **72 MB** `px4.log` of nothing but prompts;
the same run with `-d` produces **~6 KB**.

This was not cosmetic. That write amplification onto a bind-mounted volume
starved PX4's own scheduler, and it is what produced persistent, arming-blocking
preflight failures:

```
Preflight Fail: No valid data from Gyro 0 / Accel 0 / Baro 0 / Compass 0
Preflight Fail: High Gyro Bias
Preflight Fail: Attitude failure (roll)
Preflight Fail: height estimate not stable
```

With `-d` these are gone. What remains is `No connection to the GCS` (expected,
no QGroundControl attached) and a transient `ekf2 missing data` at startup.
Confirmed healthy afterwards: `sensor_combined` at 21.7 Hz, `vehicle_attitude`
at 10.8 Hz, `arming_state: 1` (disarmed, i.e. responsive).

## 25. PX4 publishes VERSIONED uORB topic names

*File: `docker/scripts/fly_path.py`*

The pinned PX4 (`main` @ `6bc24c8c`) is in the message-versioning era — which is
also why the pinned `px4_msgs` contains `MessageFormatRequest`/
`MessageFormatResponse`. Its **output** topics carry a `_v1` suffix:

```
/px4_1/fmu/out/vehicle_status_v1          -> px4_msgs/msg/VehicleStatus
/px4_1/fmu/out/vehicle_local_position_v1  -> px4_msgs/msg/VehicleLocalPosition
```

The unversioned names simply do not exist, so anything subscribing to
`vehicle_status` waits forever and never learns the vehicle is ready. The
*input* topics (`offboard_control_mode`, `trajectory_setpoint`,
`vehicle_command`) are **not** versioned. Message types are unchanged.

`fly_path.py` now subscribes to both spellings so it works against either
firmware generation.

## 26. Two regressions I introduced, and how they presented

Recorded because both failed *silently* rather than loudly, which is the
dangerous kind.

**(a) A comment broke a shell line continuation.** Adding an explanatory
comment between the backslash-continued `PX4_*` env assignments and the `nohup`
line severed the continuation. bash then treated the assignments as a standalone
statement and launched PX4 with **none** of them. PX4 logged `No autostart ID
found`, fell back to the **SIH** simulator (`INFO [init] SIH simulator`), and ran
with no Gazebo model and no sensors at all — while still appearing to be "PX4
running". Now uses an explicit `env ...` prefix, which cannot break this way.

**(b) A bool parameter passed as a string killed the TF tree.** The
`use_sim_time` value in `static_frames.launch.py` came from
`LaunchConfiguration(...).perform(context)`, which returns a **string**. rclcpp
rejects that for a bool parameter:

```
rclcpp::exceptions::InvalidParameterTypeException
  what(): parameter {use_sim_time} is of type {bool}, setting it to {string}
  is not allowed.
```

All three `static_transform_publisher` processes aborted with SIGABRT (exit -6)
~1 s after launch, leaving `/tf_static` empty. Bringup still reported success
because that step only `sleep`-ed instead of verifying. Fixed by coercing to
bool, **and** `bringup_sim.sh` now gates on
`ros2 topic info /tf_static` reporting a non-zero publisher count.

---

## Measured performance on this host (no GPU, Mesa llvmpipe)

All numbers measured, not estimated. `forest` world, single `x500_depth`.

| Configuration | Gazebo RTF | `/uav_1/rgb/image_raw` |
|---|---|---|
| RGB 1920x1080, no SLAM | 0.028 | 1.53 Hz |
| RGB 640x480, no SLAM | ~0.6 (oscillating 0.07-1.0) | 8.38 Hz |
| RGB 640x480, full stack incl. ORB-SLAM3 | **0.032** | 4.93 Hz |

With the full stack running, depth measures 6.34 Hz and the ORB-SLAM3 pose
output `/uav_1/robot_pose_slam` measures **6.40 Hz** — i.e. the frontend tracks
and keeps up with the frames it is given.

The practical consequence: **1 s of simulated time costs roughly 30 s of wall
clock** with everything running. Scripted flights must be scaled accordingly,
and this is a property of software rasterisation, not of the repo.

## 27. Ground-truth plumbing: three separate defects

Found while producing the ATE/RPE numbers. Each one on its own silently
produced an unusable or misleading evaluation rather than an error.

**(a) `ros_gz_bridge` discards entity names in `Pose_V -> TFMessage`.**
Every `frame_id` and `child_frame_id` in the recorded ground-truth stream is the
empty string, so filtering by model name matches nothing. The names *are* present
in Gazebo: `gz topic -e -t /world/forest/dynamic_pose/info` shows
`x500_depth_1`, `base_link`, `rotor_0..3`, `camera_link`. The conversion drops
them.

Worked around by selecting the transform by **index**: Gazebo orders that
message `[model, link, link, ...]`, so index 0 is the model's world pose.
`bag_to_tum.py --variance` verifies this rather than assuming it — index 0 sweeps
`x -0.18..3.20, y -0.23..3.20, z -0.01..2.10` (the commanded 3 m square at 2 m)
while indices 1-6 are constant link offsets. Index 6 sits at
`(0.12, 0.03, 0.24)`, which matches `robot_x/robot_y/robot_z` in the wrapper's
ROS params — an independent confirmation that the indexing is right.

**(b) `dynamic_pose/info` is the wrong topic for a naive read.** Its entries
after index 0 are per-link poses *relative to the model*, not world poses. A
first attempt that filtered by name and fell through to "whatever is there"
would have produced rotor offsets (`±0.174`) as the drone's trajectory.

**(c) The bridge stamps ground truth with the SYSTEM clock,** ignoring
`use_sim_time:=true`. Measured: ground truth spanned
`1791001442.97 .. 1791002867.67` (Unix epoch) while the SLAM estimate spanned
`13.10 .. 451.18` (sim time). `evo` reported
`found no matching timestamps ... with max. time diff 0.05 (s)` — zero overlap.

Fixed with `bag_to_tum.py --time-from-clock /clock`, which interpolates each
sample's bag receive time through the 109,533 recorded `/clock` samples onto
simulation time. After the remap both trajectories span **438.1 s**, and evo
matches them.

## 28. COVINS writes its trajectory in map order, not time order

*File: `docker/scripts/record_and_eval.sh`*

`KF_<id>_ftum.csv` is emitted in keyframe-map order. Observed: first row
`t=64.72`, last row `t=10.40`. evo rejects non-monotonic stamps, so the CSV is
now sorted by timestamp and de-duplicated before evaluation.

## 29. The recorded window is not the flight window

Worth stating because it changes how the numbers should be read.
`ros2 bag record` did not stop on `SIGINT` and needed `SIGTERM`, so the bag kept
running for ~6.5 minutes of simulated time after the drone landed. The result:

| | window |
|---|---|
| bag / SLAM estimate / ground truth | 13.1 - 451.2 s sim |
| actual flight (and all COVINS keyframes) | 10.4 - 64.7 s sim |

So ~88% of the recorded samples are of a **stationary** vehicle. Evaluating over
the full bag gives ORB-SLAM3 an ATE RMSE of 0.192 m with a median of 0.037 m --
flattering numbers that mostly measure a parked drone. Restricting to the flight
window gives 0.421 m. Both are reported below; the windowed figure is the
meaningful one, and it is also the only one comparable to COVINS.

(The hard kill also left no `metadata.yaml`; `ros2 bag reindex -s mcap`
regenerates it.)

---

## Results: ATE / RPE against Gazebo ground truth

Single `x500_depth`, `forest` world, 3 m square at 2 m altitude with yaw slewed
per leg. Ground truth from Gazebo, `evo` with SE(3) Umeyama alignment (`-a`) and
**no scale fitting** — RGB-D SLAM is metric, so letting evo solve for scale would
hide real scale error.

**Like-for-like, flight window only (10.4 - 64.8 s sim):**

| | ATE RMSE | ATE mean | ATE median | ATE max | RPE(1 m) RMSE | RPE(1 m) mean |
|---|---|---|---|---|---|---|
| ORB-SLAM3 frontend | 0.421 m | 0.364 m | 0.456 m | 0.646 m | **0.172 m** | 0.150 m |
| COVINS optimised (217 KFs, after visual GBA) | **0.303 m** | 0.286 m | 0.285 m | 0.525 m | 1.185 m | 1.114 m |

**COVINS improves global accuracy**: ATE RMSE 0.421 -> 0.303 m, which is what a
global bundle adjustment is for.

**The RPE comparison is not like-for-like and should not be read as COVINS being
worse locally.** `--delta 1 --delta_unit m` with consecutive pairs is evaluated
over 217 sparse keyframes for COVINS versus 13,276 dense poses at ~10 Hz for the
frontend. Consecutive COVINS keyframes are far apart, so the 1 m delta is badly
conditioned on that trajectory. It measures sparsity, not drift.

For reference, over the full 438 s bag (with the stationary tail, see §29):
ORB-SLAM3 ATE RMSE 0.192 m / median 0.037 m, RPE(1 m) RMSE 0.172 m.

Caveat worth keeping in view: COVINS is a visual-**inertial** system and this
runs it RGB-D with no IMU, so its inertial machinery is unexercised and the GBA
is visual-only (service `action: 5`). With a single agent and the repo's
`placerec.inter_map_matches_only: 1` there is also no place recognition, hence no
loop closures — by configuration, not by failure.

## 30. ros1_bridge (step 7, optional): NOT built — requires amd64 emulation

**Attempted natively on arm64 and stopped, as instructed, rather than forced.**

`ros-jazzy-ros1-bridge-builder` is partly arm64-aware — it branches on
`uname -m` for `aarch64` to fix a pkgconfig path — so a native build looked
plausible. It fails at stage 5:

```
E: Unable to locate package ros-noetic-desktop
ERROR: process "... apt -y install ros-noetic-desktop ..." exit code: 100
```

Root cause, which the upstream Dockerfile states in its own comment:

```
# 5.) Install ROS1 Noetic desktop
# (Currently, ppa contains AMD64 builds only)
RUN add-apt-repository ppa:ros-for-jammy/noble
```

Noetic targets Ubuntu Focal, so getting it onto Noble depends on that PPA, and
the PPA is amd64-only. Verified against the PPA index rather than taken on
trust:

| `ppa:ros-for-jammy/noble` | `ros-noetic-*` packages |
|---|---|
| `binary-amd64` | **235** (incl. `ros-noetic-desktop`) |
| `binary-arm64` | **0** (no index published) |

So on this host `ros1_bridge` can only be built under amd64 emulation. Rosetta
is enabled in Docker Desktop, so `--platform linux/amd64` is available if it is
ever wanted; it was not attempted here because the instruction was to stop and
report, and because **the reproduction does not depend on it**:

- The ORB-SLAM3 -> COVINS path uses covins_comm's own TCP socket, not ROS
  topics. See the architecture note at the top of docker/README.md.
- `multi_slam/launch/bringup.launch.py` now has ros1_bridge behind
  `use_ros1_bridge` (default false) -- see §11.
- ros1_bridge is only relevant to the repo's separate, still-in-progress goal
  of bringing COVINS's ROS 1 pose/TF output back into ROS 2.

Everything in steps 1-6 was completed and measured without it.

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

---

# Phase 2 (D455-mirror rig, stereo-inertial)

## 31. Upstream ORB-SLAM3 bug: `System::GetTimeFromIMUInit()` segfaults before the first keyframe

**Not patched in vendored code.** Worked around in our own node. Reported here
because it is a genuine upstream defect, not an arm64 or packaging artefact.

`LocalMapping::mpCurrentKeyFrame` is a raw pointer:

```
ORB_SLAM3/include/LocalMapping.h:185:    KeyFrame* mpCurrentKeyFrame;
```

It appears in **neither** the constructor's initialiser list nor its body
(`ORB_SLAM3/src/LocalMapping.cc:33-47`); it is first assigned in
`LocalMapping::ProcessNewKeyFrame()`. Until the first keyframe reaches the local
mapper it therefore holds **indeterminate** memory.

`GetCurrKFTime()` guards with a null test, which an indeterminate non-null value
passes:

```
ORB_SLAM3/src/LocalMapping.cc:1592
double LocalMapping::GetCurrKFTime()
{
    if (mpCurrentKeyFrame)
        return mpCurrentKeyFrame->mTimeStamp;   // <-- dereferences garbage
    else
        return 0.0;
}
```

and `System::GetTimeFromIMUInit()` calls it **before** testing whether the IMU
is initialised, so the short-circuit that would have made it safe never gets a
chance:

```
ORB_SLAM3/src/System.cc:1454
double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;  // <-- unconditional
    if ((aux > 0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
        return 0.f;
}
```

### Symptom

Every stereo-inertial run died at the same point — immediately after ORB-SLAM3
printed `not IMU meas`, i.e. on the frames *before* stereo initialisation
succeeds — with `exit code -11` and no diagnostic:

```
[stereo_inertial-1] [INFO] ... first stereo pair at t=56.332000 with 12 IMU samples
[stereo_inertial-1] not IMU meas
[ERROR] [stereo_inertial-1]: process has died [pid 4061, exit code -11, ...]
```

`not IMU meas` is a red herring: it comes from
`Tracking::StereoInitialization()` (`ORB_SLAM3/src/Tracking.cc:2364`), which for
`IMU_STEREO` requires both `mCurrentFrame.mpImuPreintegrated` and
`mLastFrame.mpImuPreintegrated` and otherwise merely `return`s. On the first
frames nothing precedes them, so it is expected and benign. The crash is the
next statement in our callback. `gdb` inside the sim container gave the answer:

```
Thread 27 "stereo_inertial" received signal SIGSEGV, Segmentation fault.
#0  ORB_SLAM3::LocalMapping::GetCurrKFTime ()   from .../libORB_SLAM3.so
#1  ORB_SLAM3::System::GetTimeFromIMUInit ()    from .../libORB_SLAM3.so
#2  ORB_SLAM3_Wrapper::StereoInertialSlamNode::StereoCallback (...)
#3  message_filters ... ApproximateTime::publishCandidate ()
```

Two earlier hypotheses were checked and **rejected** before this:

- *Config shape.* Compared the generated
  `orbslam3_d455_stereo_inertial.yaml` against ORB-SLAM3's own
  `Examples/Stereo-Inertial/RealSense_D435i.yaml`: identical structure
  (`Camera.type: "Rectified"` + `Camera1.*` + `Stereo.b` + `Stereo.ThDepth` +
  `IMU.T_b_c1` + IMU noise + `IMU.Frequency`). Not the cause.
- *COVINS.* The crash also occurs on runs where the agent connects cleanly
  (`newfd_: 17`, `Set Client ID: 0`), so it is independent of the
  `ConnectToServer` → `return 2` → close-stderr path described in §25.

### Workaround

Test `Atlas::isImuInitialized()` — public and mutex-guarded
(`ORB_SLAM3/src/Atlas.cc:296`) — **before** calling `GetTimeFromIMUInit()`:

```cpp
if (!imuInitialised_ && interface()->slam()->GetAtlas()->isImuInitialized())
{
    const double tFromInit = interface()->slam()->GetTimeFromIMUInit();
    ...
}
```

That ordering is sufficient, not merely probable: `isImuInitialized()` can only
become true from `LocalMapping::InitializeIMU()`, which runs *after*
`ProcessNewKeyFrame()` has assigned `mpCurrentKeyFrame`. `System::GetAtlas()` is
already public (`ORB_SLAM3/include/System.h:191`), so no vendored header or
algorithm file is touched.

A real upstream fix would be a one-word reorder of the `&&` in
`System::GetTimeFromIMUInit()` plus `mpCurrentKeyFrame(nullptr)` in
`LocalMapping`'s initialiser list. Both are left alone here deliberately: the
standing rule for this reproduction is not to rewrite vendored algorithm code,
and the workaround is strictly in our own node.

### Confirmation

With the guard in place, 200 s against the live `x500_d455_nodepth` rig:

```
newfd_: 17 / --> Set Client ID: 2
first stereo pair at t=113.160000 with 7 IMU samples
IMU INITIALISED at frame t=115.600000 (2.440 s after first frame, frame #75, 504 IMU samples seen)
STEREO-INERTIAL node stopped. frames=1555 imu_samples=10276 imu_initialised=yes
```

No segfault. The 87 subsequent `Not enough motion for initializing. Reseting...`
messages are correct behaviour for that test: the drone was parked, so
`LocalMapping`'s `(mTinit < 10.f) && (dist < 0.02)` check keeps resetting the
active map. They disappear once the scripted flight actually moves the airframe.

## 32. RETRACTED: "ORB-SLAM3 consumes ~8 of the 30 published stereo Hz"

**This section originally claimed ORB-SLAM3 was processing only 7-8 of the 30.3
published Hz and that OpenVINS would have to be throttled to match. That claim
was wrong and is retracted.** It is kept rather than deleted because it was
acted on: it was reported as a phase-3 comparability constraint, and throttling
OpenVINS to ~8 Hz on the strength of it would have crippled the comparison.

The claim came from the node's own log line:

```
[INFO] ... Current ORB-SLAM3 tracking frequency: 8.00065 frames / sec
```

That figure is frames per **wall** second. At RTF 0.28 it corresponds to
8.0 / 0.28 ~= 28.6 frames per **simulated** second -- which is the rate the
cameras publish. The standalone 200 s test it was read from reported
`frames=1555`, and 1555 frames over the ~56 s of simulated time in that window
is 27.8 Hz, not 8 Hz.

The published trajectories settle it directly. Counting poses in
`est_orbslam3.tum` against the 30.33 Hz camera rate over the same span:

| run | est poses | span (s) | est Hz | expected | dropped |
|---|---|---|---|---|---|
| 1 | 1930 | 63.66 | 30.32 | 1931.7 | 0.09% |
| 2 | 1917 | 63.43 | 30.22 | 1924.8 | 0.40% |
| 3 | 1947 | 64.32 | 30.27 | 1951.7 | 0.24% |
| 4 | 1898 | 63.39 | 29.94 | 1923.7 | 1.33% |

One pose published per stereo pair, to within a frame or two. Run 4's 1.33% is
not dropped input either -- it had 24 `Tracking LOST` events, and the wrapper
publishes nothing while tracking is lost.

**Frames were never being dropped**, so no rate matching is needed for phase 3,
and replaying bags more slowly cannot improve the ATE by recovering frames that
were never lost. The real explanation for the error is s34.

## 33. Two defects in `record_and_eval.sh` found by the first stereo-inertial experiment

Both mine, both produced wrong-or-missing numbers rather than errors.

**(a) `evo --save_results` prompts on an existing archive.** `run_experiment.sh`
calls this script twice per run (the flight, then `--eval-only`), so the second
pass always found its own `.zip` from the first and asked to overwrite. On a
non-tty the prompt hits EOF:

```
[WARNING] /out/eval/.../ape_orbslam3.zip exists, overwrite?
EOFError: EOF when reading a line
[ERROR] evo module evo.main_ape crashed
```

`evo` prints the statistics **before** that point and `aggregate_results.py`
parses them out of the `.txt`, so no number was ever wrong — but evo exited
non-zero and the run was reported as "eval returned 1". The result archives are
now removed before each evo call.

**(b) An unbounded `wait` on a recorder that ignores every catchable signal.**
The stop ladder was SIGINT → SIGTERM → `echo WARNING` → `wait "${BAG_PID}"`.
`wait` on a live child has **no timeout**, so when both signals failed the script
blocked indefinitely. Run 5 of the first stereo-inertial experiment wedged for 36
minutes: the flight ended at 18:50 and the mcap was still growing at 19:26.

`/proc/<pid>/status` on the stuck recorder shows the handler situation is not
what the old comment assumed:

```
State:  S (sleeping)
SigIgn: 0000000001001006     # bit 2 set -> SIGINT IGNORED
SigCgt: 0000000100004000     # bit 15 set -> SIGTERM CAUGHT
```

So rosbag2 *does* install a SIGTERM handler; it simply did not stop the
recording. The ladder was missing its last rung. Now: SIGINT → SIGTERM →
**SIGKILL**, and if a process somehow survives SIGKILL the script marks the run
`INVALID_RECORDER_OVERRUN` and refuses to `wait` on it. SIGKILL costs only
`metadata.yaml`, which the existing `ros2 bag reindex` regenerates.

Why this matters for the numbers and not just for the clock: an SE(3)-aligned
ATE **rewards** stationary samples. Run 5's trajectory held 18,300 poses against
~1,900 for every other run, almost all of a parked vehicle, and it scored the
*best* ATE in the set (0.249 m against a 1.369 m four-run mean). Averaging it in
would have improved the headline figure by inventing accuracy. The run is
excluded from the aggregate below.

## Results: ORB-SLAM3 stereo-inertial on the D455-mirror rig

`RIG=d455 ./docker/scripts/run_experiment.sh 5`, depth OFF, IR 848x480,
IMU 200 Hz, scripted 3 m square at 2.0 m altitude.

| run | ATE rmse | RPE rmse | RTF | IMU init | path | `Tracking LOST` |
|---|---|---|---|---|---|---|
| 1 | 1.774 m | 0.946 m | 0.279 | +2.440 s | 18.74 m | 0 |
| 2 | 0.831 m | 0.342 m | 0.278 | +2.444 s | 18.86 m | 6 |
| 3 | 0.503 m | 0.239 m | 0.282 | +2.440 s | 18.83 m | 2 |
| 4 | 2.367 m | 1.406 m | 0.279 | +2.440 s | 18.95 m | 24 |
| 5 | *excluded* | | *0.258* | +2.444 s | 18.87 m | 0 |

Mean +/- sample stdev over the **four valid** runs:

```
ATE rmse (m)             1.369 +/- 0.856   [0.503..2.367]
RPE rmse (m) @1m delta   0.733 +/- 0.546   [0.239..1.406]
real-time factor         0.280 +/- 0.002   [0.278..0.282]
IMU init delay (s)       2.441 +/- 0.002   [2.440..2.444]
path length (m)         18.845 +/- 0.087   [18.740..18.950]
```

`SUMMARY.txt` in the experiment directory still prints `n=5`; the figures here
are the ones to quote.

### Interpretation

- **The per-run RTF decay is gone.** sigma = 0.002 across runs, and no decay
  *within* a run either (run 1: 0.246 -> 0.279 over 14 windows). Restarting
  `gz sim` per run plus `init: true` reaping orphans fixed it.
- **IMU initialisation is essentially deterministic** at +2.441 s (~frame 75,
  ~500 IMU samples) and never failed. But the scripted path only *barely*
  excites it: LocalMapping resets the active map **31-38 times** per run with
  `Not enough motion for initializing` while the airframe is parked, because its
  gate is `(mTinit < 10.f) && (dist < 0.02)`. Initialisation sticks only once
  the climb starts. Sufficient, with no margin -- a longer settle or slower
  climb would push it past the 10 s window.
- **Full-flight and post-IMU-init ATE are bit-identical, legitimately.** The
  published estimate starts at t ~= 16.4-16.7 s while IMU init happens at
  t ~= 10.1-10.5 s, so `--t_start` excludes nothing: the wrapper publishes only
  once `GetTrackingState()==2`, and the repeated resets mean the surviving map
  begins ~6 s after init. `imu_init.txt` now records `est_first_stamp` and
  `postinit_window_equals_full_flight` so this is visible rather than puzzling.
- **1.369 m over 18.8 m (~7% of path) is poor**, and worse than phase 1's RGB-D
  0.421 m. The estimator is nonetheless behaving: single map (id 0), VIBA 1 and
  2 both completing, no merges, no relocalizations, RPE *median* 0.032 m against
  an RPE max of 4.30 m -- accurate locally, with a few large excursions. The
  spread tracks tracking stability only loosely: run 4 had 24 losses and the
  worst ATE, but run 1 had **zero** losses and still scored 1.774 m, so drift
  alone reaches metre level here.
- **The 95 mm baseline is the likely physical cause.** With fx = 446.8,
  `Z = fx*b/d = 42.5/d` metres, so a feature at 10 m gives ~4 px disparity and
  anything past ~15 m is effectively monocular. In the forest world at 2 m
  altitude much of the scene is beyond useful stereo range. That is a faithful
  property of a real D455, not a simulation artefact -- but it makes this a hard
  baseline, and OpenVINS will face the same geometry.
- **Comparability constraint for phase 3**: see s32. ORB-SLAM3 consumed 7-8 of
  the 30.3 published Hz. OpenVINS must be compared at the same effective input
  rate or the comparison measures CPU budget, not estimator quality.

## 34. The phase-2 ATE is dominated by ORB-SLAM3's own VIBA corrections, not by drift or dropped frames

`docker/scripts/diagnose_run.py` was written to answer what an RMSE cannot: is
the error spread (drift) or concentrated (jumps), and where in the flight does
it live. Run over all four valid phase-2 runs it shows the same structure every
time.

**Coverage is complete.** The estimate spans 100.0-100.1% of the ground-truth
window, starts 0.04-0.10 s *before* the first ground-truth sample, associates
99.9-100% of its samples within 50 ms, and has no gap over 0.5 s. So nothing is
missing and nothing is being evaluated over a partial trajectory.

**The error is front-loaded and collapses mid-flight.** RMSE by decile, run 4
(the worst run):

```
  16.50.. 22.84s  rmse 4.242
  22.84.. 29.17s  rmse 4.713
  29.17.. 35.50s  rmse 1.428   <-- collapses here
  35.50.. 41.83s  rmse 1.115
  41.83.. 48.17s  rmse 0.482
  48.17.. 54.50s  rmse 1.197
```

Every run has its single largest error step at **t = 28.8-30.3 s**, and in every
case the error drops sharply immediately after it:

| run | largest step | at t | error after |
|---|---|---|---|
| 1 | +2.716 m | 29.80 s | 0.817 m |
| 2 | +1.065 m | 29.14 s | 0.650 m |
| 3 | +0.439 m | 30.30 s | 0.305 m |
| 4 | +3.418 m | 29.04 s | 1.515 m |

**That is VIBA 1.** Placing ORB-SLAM3's untimestamped `cout` markers on the
flight timeline via the surrounding ROS log stamps:

| run | IMU init | VIBA 1 | VIBA 2 |
|---|---|---|---|
| 1 | sim 10.13 s | sim ~30.43 s | sim ~58.7-60.4 s |
| 2 | sim 10.30 s | sim ~29.90 s | sim ~59.3-61.0 s |
| 3 | sim 10.26 s | sim ~31.03 s | sim ~59.3-61.3 s |
| 4 | sim 10.53 s | sim ~29.7-30.0 s | sim ~52.4-56.3 s |

VIBA 1 coincides with the large step in all four runs, and the *second* cluster
of steps (t ~= 53.9-61.5 s) coincides with VIBA 2.

### Why this makes the headline ATE misleading

ORB-SLAM3's visual-inertial bundle adjustment refines scale, gravity and biases
and rewrites the map **retroactively**. The wrapper, however, publishes each
pose once, online, as the frame is tracked. So `/robot_pose_slam` carries the
*pre-refinement* trajectory for the first ~13 s of a ~64 s window, and then a
discontinuity where the refinement lands. The ATE is mostly measuring that
transient: the converged segments run 0.05-0.5 m.

Consequences worth being explicit about:

- **"ATE after IMU init" does not isolate this.** IMU init completes at
  t ~= 10.1-10.5 s, VIBA 1 lands ~20 s later, and the published estimate does
  not even begin until t ~= 16.5 s. Cropping at IMU init removes none of it
  (s33), so the full-flight and post-init numbers being identical is not the
  whole story -- the interesting crop would be *post-VIBA-1*.
- **Replaying bags more slowly will not improve it.** That would fix dropped
  frames, and per s32 no frames were being dropped. The transient is inherent
  to streaming an online estimate from a system that corrects retroactively.
- **The honest comparison against a filter is a cropped window.** OpenVINS is a
  sliding-window filter with no retroactive global correction, so comparing its
  full-flight ATE against ORB-SLAM3's full-flight ATE compares a filter's
  steady-state against a smoother's start-up transient. Either evaluate both
  from after VIBA 1, or evaluate ORB-SLAM3's final optimised trajectory rather
  than its online stream, and say which.

The 95 mm-baseline hypothesis from the phase-2 write-up is **not** supported by
this: if weak stereo depth were the cause the error would grow with distance
flown, and instead it *shrinks* after VIBA 1 and stays low. Weak depth may still
limit the converged accuracy, but it is not what produced 1.369 m.

## 35. Stereo-only bisection: ORB-SLAM3 without the IMU is 100x more accurate on the same rig

The bisection asked a narrow question -- does the mid-flight discontinuity of
s34 disappear without VIBA? -- and returned a much larger answer.

Same rig, same scripted flight, same intrinsics, same extractor settings. The
only difference is `System::STEREO` instead of `System::IMU_STEREO` and an
ORB-SLAM3 settings file with the IMU block removed; both are emitted by
`gen_d455_sim.py` so they cannot drift apart (`diff` between them is the header
comment and the IMU block, nothing else).

| | stereo-inertial (4 runs) | **stereo-only (1 run)** |
|---|---|---|
| ATE rmse | 1.369 +/- 0.856 m | **0.0138 m** |
| ATE max | 4.941 m | **0.0386 m** |
| RPE rmse @1 m | 0.733 +/- 0.546 m | **0.0172 m** |
| largest single-sample error step | +3.418 m | **+0.020 m** |
| `Tracking LOST` | 0 - 24 | **0** |
| RTF | 0.280 | 0.286 |
| path | 18.845 m | 18.762 m |

**The ~30 s discontinuity is absent, as predicted.** The stereo-only log contains
zero occurrences of `VIBA`, `IMU INITIALISED`, `not IMU meas` and
`Not enough motion` -- the inertial machinery is genuinely not running -- and the
error is flat across the flight instead of front-loaded:

```
  52.14.. 58.45s  rmse 0.009
  58.45.. 64.75s  rmse 0.011
  64.75.. 71.05s  rmse 0.015
  71.05.. 77.36s  rmse 0.015
  77.36.. 83.66s  rmse 0.017
  83.66.. 89.96s  rmse 0.018
  89.96.. 96.26s  rmse 0.016
  96.26..102.57s  rmse 0.011
```

Coverage is 99.8% of the ground-truth window with 100% of samples associated
within 50 ms and a maximum gap of 36 ms, so this is not a short or sparse
trajectory flattering itself.

### This is bigger than the VIBA transient

s34 attributed the error to a start-up transient, and that is right as far as it
goes -- but stereo-inertial's *converged* deciles were 0.05-0.5 m, still 4-35x
worse than stereo-only's 0.009-0.018 m across the whole flight. Adding the IMU
to this configuration makes it worse everywhere, not just at the start.

### What is NOT the cause

- **Scene / baseline.** 1.4 cm over 18.8 m with the same 95 mm baseline in the
  same forest world retires the hypothesis from the phase-2 write-up outright.
  The scene has ample usable features and the baseline is sufficient.
- **Dropped frames.** 1911 of 1911 samples associated; see s32.
- **Camera-IMU extrinsics.** Checked numerically rather than assumed. ORB-SLAM3's
  `IMU.T_b_c1` is camera -> body(IMU) (`Settings.cc:422`, `Tbc_`). The emitted
  rotation maps camera optical (0,0,1) to body (1,0,0) -- 1 m in front of the
  camera is 1 m forward in body -- and the translation (0.00552, 0.0424, 0.01174)
  equals `P_CAM0 - P_IMU` exactly. Correct in both value and direction.

### Two things that ARE wrong, one of them ours

1. **Gravity magnitude mismatch.** The Gazebo worlds use `<gravity>0 0 -9.8`
   while ORB-SLAM3 hardcodes `const float GRAVITY_VALUE = 9.81`
   (`ImuTypes.h:46`) -- not configurable. The 0.01 m/s^2 difference is partly
   absorbed by the estimated accelerometer bias, but it couples straight into
   the gravity-direction estimate that VIBA solves for. OpenVINS takes
   `gravity_mag` from its config, so it would not have this problem, and
   comparing an estimator that is handed the right gravity against one that is
   not would be unfair in OpenVINS's favour. **Fix: the new validation world
   sets `-9.81` so the simulator matches ORB-SLAM3's constant, and the OpenVINS
   sim config sets `gravity_mag: 9.81` to match the world.** No vendored
   algorithm code is touched.

2. **The scripted path is close to degenerate for visual-inertial estimation.**
   Straight constant-velocity legs with little rotation leave accelerometer bias
   and inertial scale weakly observable. Stereo-only gets metric scale directly
   from a known baseline and never estimates it; stereo-inertial additionally
   solves for a scale that the motion barely constrains, and a poorly
   conditioned scale estimate corrupts a trajectory that was already metric.
   This is consistent with every observation above, including the 31-38
   `Not enough motion for initializing` resets of s33.

### Consequence for phase 3

Stereo-only ORB-SLAM3 at 0.0138 m is the right yardstick for "is the rig and
scene good enough", and it says yes emphatically. It is **not** the baseline
OpenVINS is being compared against -- that remains stereo-inertial, like for
like -- but it bounds what the sensor suite can deliver, and any VI result far
from ~1 cm on this rig is an estimator or excitation problem, not a sensing
limit.

## 36. Gravity alone accounts for most of the stereo-inertial error, and nearly all of its variance

One variable changed: PX4's worlds patched from `<gravity>0 0 -9.8` to `-9.81`,
matching ORB-SLAM3's hardcoded `ImuTypes.h:46 GRAVITY_VALUE = 9.81`. Same rig,
same path A, same image otherwise, same evaluation.

| | gravity 9.8 (4 runs) | **gravity 9.81 (5 runs)** |
|---|---|---|
| ATE rmse | 1.369 +/- 0.856 m | **0.437 +/- 0.129 m** |
| ATE range | 0.503 .. 2.367 | **0.306 .. 0.606** |
| RPE rmse @1 m | 0.733 +/- 0.546 m | **0.228 +/- 0.087 m** |
| real-time factor | 0.280 +/- 0.002 | 0.279 +/- 0.005 |
| IMU init delay | 2.441 +/- 0.002 s | 2.443 +/- 0.002 s |
| path length | 18.845 +/- 0.087 m | 18.881 +/- 0.098 m |

**Mean error down 68%; sample standard deviation down 85%.** The worst run
improved from 2.367 m to 0.606 m.

The variance collapse is the more telling half. A 0.01 m/s^2 constant offset
between the simulated world and the estimator's hardcoded constant is not a
fixed bias that shifts every run equally -- it feeds the gravity-direction and
accelerometer-bias states that VIBA solves for, and how badly it corrupts them
depends on where in the flight the solve lands and what the motion happened to
excite. Hence 9.8 produced 0.503 m on one run and 2.367 m on another, while 9.81
produced 0.306-0.606 m across five.

Per-run detail, with tracking losses (which still do not predict accuracy -- the
best run, 0.306 m, had 13 losses; the worst, 0.606 m, had 0):

| run | ATE | RPE | RTF | `Tracking LOST` |
|---|---|---|---|---|
| 1 | 0.499 | 0.177 | 0.280 | 0 |
| 2 | 0.606 | 0.366 | 0.271 | 0 |
| 3 | 0.467 | 0.258 | 0.280 | 0 |
| 4 | 0.306 | 0.197 | 0.281 | 13 |
| 5 | 0.309 | 0.144 | 0.283 | 7 |

Provenance is recorded per experiment now (`MANIFEST.txt`):
`world_gravity: <gravity>0 0 -9.81</gravity>`, `path_version: A`,
`git_commit: b81e854`, `git_dirty: no`.

**It does not close the gap to stereo-only.** 0.437 m against 0.0138 m is still
~32x, which leaves the near-degenerate path A geometry (s35) as the remaining
explanation and is what path B exists to test. Gravity was one of two identified
causes, not the whole story.

## 37. Editing a shell script while it is executing killed a completed experiment

Bash reads a script **incrementally**, not all at once. Editing
`run_experiment.sh` mid-run -- to add the `PATH_VERSION` passthrough,
`RECORD_SENSORS` and the `d455_stereo` rig -- shifted byte offsets under the live
interpreter, which then resumed reading at its old offset inside the new text:

```
./docker/scripts/run_experiment.sh: line 193: syntax error near unexpected token `done'
SI_EXIT=2
```

All five runs had already completed and their data was intact (it is written per
run), but the aggregation step was lost and had to be redone by hand.

These experiments run for an hour or more and editing the harness while one is
in flight is a normal thing to want to do, so the fix is in the script rather
than in discipline: `run_experiment.sh` now copies itself to a temp file and
`exec`s that, so a running experiment is immune to edits of the original.

The copy deletes *itself* via an EXIT trap set after the re-exec. Trapping EXIT
*before* the `exec` would not work -- `exec` replaces the process image, so that
shell never exits and its traps are discarded, leaking a temp file per run.
Verified: no `/tmp/run_experiment.*` remains after a run.

Separately, the staleness guard did exactly its job in the same experiment. The
three stereo-only runs refused to start because `record_and_eval.sh` had been
edited (`STALE record_and_eval.sh`), rather than running with host and container
scripts disagreeing. A non-zero exit there is a correct refusal, not a failure.

## 38. A 25x real-time-factor collapse, and why the run that hit it is unrecoverable

The first attempt at the three stereo-only runs produced a flight that took **26
minutes of wall time for ~80 s of simulated time**: `rates.log` recorded
**RTF 0.011** against the 0.279 +/- 0.005 measured across the five
stereo-inertial runs on the same image.

Observations while it was happening:

- `gz sim` pinned at **543-675% CPU** (~5.7 of the Docker VM's 18 CPUs) with 87
  threads, while producing almost no simulated time -- working hard and
  achieving nothing, not stalled.
- host load average 7.1-7.9 sustained.
- Gazebo's own `real_time_factor` field read **0.042 and then 0.957** on
  consecutive samples, which is the bimodality documented earlier and the reason
  the `/clock`-derived figure is the one to trust.

**It was not structural.** A clean stack on the same image measures
**RTF 0.2199** (3852 clock messages, 15.40 s sim over 70.1 s wall), so nothing
in the OpenVINS additions or the gravity change broke the simulator.

**No confident root cause.** The most likely explanation is contention of my own
making: the experiment was launched shortly after `colcon test`, the
`active_slam_information` gtest binary, two OpenVINS boot tests and a 45 s
stereo-sync measurement had run in the same container, and it was then probed
repeatedly with `docker stats`, `ps` and `gz topic` *while run 1 was flying*.
That is a hypothesis, not a finding. What is established is that the condition
did not persist.

Diagnosis was made worse by a cleanup command using `timeout`, which does not
exist on macOS: the command failed, `gz sim` kept running at full tilt for
several more minutes, and the measurements taken during that window were
contaminated.

### The defect worth fixing: the monitor is shorter than a slow flight

`rate_monitor.py`'s duration defaulted to 900 s via
`MONITOR_DURATION:-900` in `record_and_eval.sh`. At RTF 0.011 the flight ran 26
minutes, so the monitor hit its limit **mid-flight**, printed its RUN SUMMARY
and exited. The run then completed normally and its RTF was **unrecoverable** --
the single number that would have characterised what went wrong.

`run_experiment.sh` now passes `MONITOR_DURATION=2400` and records it in
`MANIFEST.txt`. It is set there rather than in `record_and_eval.sh` so it does
not invalidate the container's baked scripts and force an image rebuild.

### Standing rule this produced

Per-run RTF is now treated as an **acceptance criterion, not just a reported
number**. A run whose RTF falls outside the established band for its rig is
discarded rather than averaged in: at 0.011 the simulator is not delivering the
sensor timing the estimator is being evaluated on, and its ATE is not comparable
with runs that were. This is the same discipline applied to the recorder-overrun
run of s33, for the same reason -- a number produced under different conditions
is not a measurement of the same thing.

## 39. Gravity comparison, completed with a control -- and a correction to s35

The stereo-only runs exist as a **control**: stereo-only ORB-SLAM3 consumes no
IMU, so gravity cannot affect it. If the gravity change were doing something
other than what is claimed, the control would move too.

### Stereo-inertial (what gravity acts on)

| | gravity 9.8 (4 runs) | gravity 9.81 (5 runs) |
|---|---|---|
| ATE rmse | 1.369 +/- 0.856 m | **0.437 +/- 0.129 m** |
| RPE rmse @1 m | 0.733 +/- 0.546 m | **0.228 +/- 0.087 m** |

Mean error down 68%, sample standard deviation down 85%.

### Stereo-only (control)

| | gravity 9.8 | gravity 9.81 (3 runs) |
|---|---|---|
| ATE rmse | 0.0138 m **(n=1)** | 0.0302 +/- 0.0033 m |
| RPE rmse | 0.0172 m (n=1) | 0.0181 +/- 0.0008 m |

The control stayed in the same regime -- a few centimetres, flat error, no VIBA
markers, zero tracking losses in all three runs -- which is the point. It did
**not** reproduce 0.0138 m exactly, and it could not have been expected to.

### Correction to s35

**s35 reported the stereo-only/stereo-inertial gap as "100x". That figure came
from a single stereo-only run and is wrong.** With three samples the stereo-only
level on this rig is 0.0302 +/- 0.0033 m, and the 0.0138 m run was the best of
the distribution, not its centre. Recomputed against the three-run mean:

| | gap to stereo-only |
|---|---|
| stereo-inertial at 9.8 | **45x** (not 100x) |
| stereo-inertial at 9.81 | **14x** |

The qualitative conclusion of s35 survives intact -- stereo-only is more than an
order of magnitude better, the VIBA discontinuity is absent, the error is flat,
and the 95 mm baseline is not the limitation. Only the multiplier was
overstated, by quoting a ratio built on n=1. RPE is the more stable control
statistic and it barely moved (0.0172 -> 0.0181 +/- 0.0008), which is what gives
confidence the control itself is sound.

### The RTF acceptance criterion, applied

Run 1 came in at **RTF 0.113**, outside the 0.27-0.28 band (runs 2 and 3: 0.273,
0.265). Per s38 it is reported but flagged, not silently averaged:

```
  all three runs    0.0302 +/- 0.0033 m
  RTF-in-band only  0.0286 +/- 0.0024 m   (runs 2 and 3)
```

The depressed run cost about 17% of ATE -- measurable, and not nearly the
disqualifying effect a 2.4x RTF drop might suggest. Useful calibration on the
criterion itself: for a **visual-only** estimator, RTF mostly changes how long
the run takes rather than what it measures, because nothing in it integrates
over time. The criterion matters far more for the inertial configurations, where
IMU preintegration intervals and the timing of VIBA relative to the flight are
exactly what RTF perturbs. Both figures are given above rather than one.
---

# Host migration: Apple Silicon Mac -> amd64 Linux with NVIDIA GPU

## 40. Rebuilt for linux/amd64 with GPU rendering for Gazebo

Host: Ubuntu 22.04, Ryzen 9 5950X (16C/32T), 64 GB RAM, GTX 1080 Ti (Pascal,
compute 6.1). NVIDIA driver 580.178.04, the last driver branch that supports
Pascal, held with `apt-mark hold`. No CUDA toolkit on the host; any
CUDA in containers must be **12.x** (CUDA 13 dropped sm_61). Docker 29.4.3,
nvidia-container-toolkit 1.19.0; Docker's default runtime remains `runc` and
GPU access is requested per service.

Every earlier patch was re-audited for arm64-specific content:

| Patch | arm64-specific? | Action |
|---|---|---|
| §0 SIMD audit | No code was changed | None. `-march=native` now resolves to znver3 (this image is not portable to other CPUs; fine for a single server). |
| §1 `osrf/ros:melodic-desktop-bionic` -> `ros:melodic-ros-base-bionic` | Motivated by arm64, but the result is multi-arch and works on amd64 | **Kept** so the backend's package set doesn't change between hosts. Reverting would add a second variable to any Mac-vs-Linux comparison. |
| §14 purge committed x86-64 artefacts | Partly: the `.so` files *are* x86-64 now, but the CMake caches still carry `/home/carlos/...` paths | **Kept**; rebuilding from source is still the only reproducible path. |
| §15 Pangolin v0.9.1 | No (GCC 13) | Kept. |
| §19c build parallelism caps | Host-resource limit (16 GB VM) | **Raised**: compose `NR_JOBS` 2->8, `NJOBS` 6->16, sim colcon `MAKEFLAGS` -j1->-j4 (sequential executor kept). Build-only; no effect on binaries' behaviour. |
| §30 ros1_bridge needs amd64 | Now satisfiable | Not built; still optional and not on the measured path. |
| compose `platforms: ["linux/arm64"]`, image tags `*-arm64` | Yes | -> `linux/amd64`, `covins-backend:melodic-amd64`, `drone-sim:jazzy-amd64` (also in `scripts/run_experiment.sh`). |
| `LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe` in sim Dockerfile and compose | Yes (no GPU passthrough on macOS) | **Removed** from the sim service. Added `NVIDIA_DRIVER_CAPABILITIES=all` and a compose `deploy.resources.reservations.devices` nvidia entry (`gpu, graphics, compute, utility`). `bringup_sim.sh` already runs `gz sim --headless-rendering`, i.e. EGL, which now resolves to `libEGL_nvidia`. |
| `LIBGL_ALWAYS_SOFTWARE=1` in covins-backend | Only affects COVINS RViz on Xvfb | Kept; Xvfb cannot use the GPU anyway, and it is off the measured path. |

The edits are on local branch `amd64-gpu`, based on `phase3/openvins-sim`, and
are rebased onto each fresh pull from the Mac.

## 41. Eigen alignment ABI mismatch on amd64 — ORB-SLAM3 segfault after map init

*File: `docker/sim/Dockerfile` (ORB-SLAM3 Thirdparty build step)*

**Symptom.** The first amd64 stereo-only run died with SIGSEGV (exit -11)
immediately after `New Map created with 631 points`, so `/robot_pose_slam`
never appeared and nothing could be evaluated.

**Cause.** The "luck rather than design" note under *Notes that are not
patches* came true. `ORB_SLAM3/CMakeLists.txt` and `covins_comm` force
`EIGEN_MAX_ALIGN_BYTES=16`, but `Thirdparty/g2o` and `Thirdparty/DBoW2` compile
with `-march=native` and **no** override. Measured in the sim image:

    g++ -O3                -> EIGEN_MAX_ALIGN_BYTES 16
    g++ -O3 -march=native  -> EIGEN_MAX_ALIGN_BYTES 32   (Zen 3, AVX2)

On arm64 both are 16, so the Mac never saw it. On amd64, g2o's vertex/edge
classes use a different layout from the one `libORB_SLAM3.so` compiles against,
and the first bundle adjustment after initialisation corrupts memory.

**Fix.** g2o and DBoW2 are configured with
`-DCMAKE_CXX_FLAGS="-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16"`,
the same defines ORB-SLAM3 itself uses. `-march=native` is kept. No repo
source is modified, and every library now agrees on 16, which is what the Mac
build effectively ran with.

**Alignment alone was not enough.** With the defines fixed, the crash changed
from SIGSEGV to SIGABRT, `double free or corruption (out)`, at the same point.
gdb backtrace:

    free()
    g2o::HyperGraph::clear()                       libg2o.so
    g2o::OptimizableGraph::~OptimizableGraph()     libg2o.so
    ORB_SLAM3::Optimizer::PoseOptimization(Frame*) libORB_SLAM3.so
    ORB_SLAM3::Tracking::TrackReferenceKeyFrame()

The remaining difference was the language standard. `ORB_SLAM3/CMakeLists.txt`
sets `-std=c++14`; `Thirdparty/g2o` sets none, so GCC 13 compiles it as C++17.
g2o vertices and edges are allocated in `libORB_SLAM3` (C++14) and deleted
through virtual destructors in `libg2o` (C++17), and Eigen's
`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` / aligned-new handling differs between the
two standards, so allocation and deallocation stop pairing up.

**Verified before baking it in:** libg2o was rebuilt in the running container
with `-std=c++14` added and the stereo node rerun under gdb on the live rig. It
initialised the map and tracked at 17-18 fps for 10 minutes with no fault.

**Final fix:** g2o is configured with
`-DCMAKE_CXX_FLAGS="-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16 -std=c++14"`;
DBoW2 gets the alignment defines only (it does not exchange Eigen objects with
ORB-SLAM3). The Mac build had the same C++14/C++17 split, but there it was
apparently harmless at arm64's 16-byte Eigen default. That is unverified, and
it is a latent upstream build inconsistency, not an amd64 port bug.

## Results on the amd64 host: stereo-only ORB-SLAM3, path A, D455 rig

Code at `b81e854` plus §40-§41 (no upstream changes at any of the pre-run
pulls). World gravity is -9.81 (the Mac run used -9.8; irrelevant without an IMU).

| | Mac | amd64, 4 runs |
|---|---|---|
| ATE rmse | 0.0302 +/- 0.0033 m (3 runs; an earlier single run gave 0.0138) | 0.030, 0.038, 0.031, 0.037 m (mean ~0.034) |
| RPE rmse @1 m | 0.0172 m (single run) | 0.034, 0.017, 0.017, 0.019 m |
| RTF (post-hoc, from bag) | 0.286 | 0.579-0.584 |
| path | 18.762 m | 18.61-18.83 m |
| `Tracking LOST` | 0 | 0 |

**Resolved: the two machines agree.** The Mac reran stereo-only three times and
got **0.0302 +/- 0.0033 m**; its earlier 0.0138 m was a single favourable run.
The amd64 runs (0.030-0.038 m, mean ~0.034) overlap that distribution. RPE
agrees in 3 of 4 runs (~0.017-0.019 m vs 0.0172 m). The error is spread evenly
across the flight (per-window rmse 0.015-0.046 m), with complete association
(2051 poses, max gap 49 ms).

Note for anyone comparing hosts: sim time does not make ORB-SLAM3
deterministic, because Local Mapping runs asynchronously in wall-clock time, so
single runs should not be compared. The amd64 host runs at roughly twice the
Mac's RTF; that did not produce a measurable ATE difference at n=3-4, and the
sim is deliberately **not** throttled to match the Mac.
