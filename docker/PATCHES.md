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
