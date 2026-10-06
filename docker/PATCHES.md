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

## 40. CORRECTION to s36: the full-flight metric is not reproducible, and the gravity result does not survive it

The path A stereo-inertial runs were repeated to obtain clock beacons (s34's
VIBA timing needed them). That repeat is an unintentional but decisive
**repeatability check**: same gravity, same path, same image settings.

| sample | full-flight online ATE |
|---|---|
| path A, gravity 9.81, **set 1** (n=5) | 0.437 +/- 0.129 m  [0.306..0.606] |
| path A, gravity 9.81, **set 2** (n=5) | **2.159 +/- 1.742 m**  [0.453..4.893] |
| path A, gravity 9.80 (n=4) | 1.369 +/- 0.856 m  [0.503..2.367] |

**Two samples of five at identical gravity differ by 4.9x in the mean and do not
overlap in range. The gravity-9.80 sample sits between them.**

The pipeline was not behaving differently between sets -- every invariant
matches: IMU init 2.440-2.444 s (set 1: 2.443 +/- 0.002), map resets 31-37
(set 1: 31-38), RTF 0.278-0.279 (set 1: 0.279 +/- 0.005), zero or few tracking
losses in both. Set 2 is a different draw, not a different configuration.

### What this retracts

**s36 claimed gravity reduced the mean error by 68% and the variance by 85%.
That claim is withdrawn.** It rested entirely on comparing full-flight means,
and the metric cannot distinguish 9.80 from 9.81: a second sample at 9.81 came
out *worse* than the 9.80 sample. The effect of the gravity change is therefore
**unmeasured**, not disproven.

Two things are worth keeping separate here:

- **The gravity mismatch was real** and worth fixing on its own terms:
  ORB-SLAM3 hardcodes `GRAVITY_VALUE = 9.81` (`ImuTypes.h:46`) while the worlds
  shipped 9.8, so the estimator was handed a 0.01 m/s^2 error that OpenVINS
  would not have been handed. Removing a known error in an estimator's input
  needs no experimental justification, and the change stays.
- **The 68%/85% figures were an artefact** of a heavy-tailed metric sampled five
  times. They should never have been reported as a measurement.

Measuring the gravity effect properly needs converged-window data at BOTH
gravities, and there is none at 9.80 -- those runs predate the beacon, and
s39's evaluator refuses to estimate a window rather than guess one.

### Why the full-flight metric behaves this way

```
corr(worst online step, full-flight ATE) = +0.997
corr(VIBA 2 completion time, full ATE)   = +0.569
corr(VIBA 2 completion time, converged ATE) = -0.823
```

The full-flight figure is a rescaling of the size of the retroactive jump
(s39 measured +1.000 on path B; +0.997 here). Jump size depends on how much
scale error accumulated before VIBA 2 lands, which depends on *when* it lands --
46.5 s to 68.0 s across five runs of an identical ~64 s flight. A late
correction on a short flight is the worst case, and whether a given run gets one
is luck.

The converged ATE correlates NEGATIVELY with VIBA 2 timing (-0.823): a later
correction leaves a shorter, later window that has had more optimisation applied
to it. Opposite sign, which is another sign the two metrics measure different
things.

### The metric that is reproducible

| | converged ATE | coefficient of variation |
|---|---|---|
| path A, 9.81 (n=5) | 0.116 +/- 0.034 m | 0.29 |
| path B, 9.81 (n=5) | 0.088 +/- 0.050 m | 0.57 |
| full-flight, path A set 2 | 2.159 +/- 1.742 m | 0.81 |

Converged ATE is also consistent *across paths* (0.116 vs 0.088 m, overlapping),
while the full-flight figure differs by an order of magnitude between the same
two sets. **Report converged ATE plus the worst online step. Do not report
full-flight online ATE as an accuracy figure at all** -- it is a measure of the
start-up transient, and it is not reproducible at n=5.

### Path A vs path B, stated honestly

Converged ATE: 0.116 +/- 0.034 m (A) against 0.088 +/- 0.050 m (B). Path B is
nominally better, consistent with better excitation, but the intervals overlap
and n=5 each. **Path B's benefit is suggestive, not established.** What IS
established is that path B costs the visual-only control nothing: stereo-only
scored 0.0302 +/- 0.0033 m on A and 0.0301 +/- 0.0033 m on B -- a 0.1 mm
difference across 3.7x the path length.

Stereo-inertial converged (0.088-0.116 m) remains ~3-4x worse than stereo-only
(~0.030 m) on the same rig.
---

# Host migration: Apple Silicon Mac -> amd64 Linux with NVIDIA GPU

## 41. Rebuilt for linux/amd64 with GPU rendering for Gazebo

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

## 42. Eigen alignment ABI mismatch on amd64 — ORB-SLAM3 segfault after map init

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

Code at `b81e854` plus §41-§42 (no upstream changes at any of the pre-run
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

## 43. Containers run as the host user, not root

*Files: `docker/compose.yaml`, `docker/sim/Dockerfile`, `docker/covins/Dockerfile`*

Running as root made every file written through the `./out` bind mount
root-owned, so the host-side `run_experiment.sh` could not write into its own
output tree (`mkdir: Permission denied`, then
`path_version.txt: Permission denied` on every run).

- Both services now set `user: "${UID:-1000}:${GID:-1000}"`. The defaults matter:
  bash does not export `UID` and does not define `GID` at all, so a bare
  `${UID}:${GID}` would reach compose as `":"`.
- The images were built as root under `/root` (mode 700). Ownership is **not**
  changed: `/root` is made traversable (`755`, and everything below it was
  already `a+rX`), and only the paths written at **runtime** are made writable:
  `$COVINS_PREFIX/config` (the entrypoint rewrites `sys.server_ip`), PX4's
  `build/px4_sitl_default/rootfs` (per-instance params and logs) and
  `covins_backend/output`. Build outputs stay root-owned and read-only.
- `HOME=/home/sim` (mode `1777`, baked in, and set in compose), with
  `ROS_HOME=/home/sim/.ros`. A UID with no `/etc/passwd` entry otherwise gets
  `HOME=/`, and ROS 2 logging, gz's cache and evo's settings all write under
  `$HOME`.
- The `covins_output` named volume had to be recreated once: an existing volume
  keeps the root ownership it was created with.

# Phase 3 on the amd64 host: OpenVINS by deterministic replay

## 44. Evaluating OpenVINS fairly: frames, gravity override, evaluation prep

*Files: `docker/scripts/ov_prep.py` (new), `docker/scripts/replay_estimator.sh`*

**Frames.** ORB-SLAM3's wrapper publishes the **base_link** pose: it conjugates
the camera pose by `robotBase_to_cameraLink`, giving base motion relative to
the initial base (`orb_slam3_interface.cpp:380`). Gazebo ground truth index 0
is the model, i.e. base_link. OpenVINS publishes the **IMU** pose. On
`x500_d455` the IMU sits at (0.11448, 0.00510, 0.23026) m in base_link (d455_link
at (0.12, 0, 0.242), IMU at (-0.00552, 0.0051, -0.01174) inside it, with no
rotation in either `<pose>`), a 0.257 m lever arm. Under yaw that sweeps a
circle, so scoring the IMU position against base ground truth would charge
OpenVINS up to ~0.5 m of pure bookkeeping error. `ov_prep.py` therefore:

- converts OpenVINS's IMU poses to base_link (`T_w_b = T_w_i * T_i_b`) for ATE,
  so both estimators are scored base-to-base against the same ground truth;
- converts ground truth to the IMU (`T_w_i = T_w_b * T_b_i`) for NEES, because
  OpenVINS's covariance belongs to the IMU state, so NEES is IMU-to-IMU;
- crops both NEES inputs to the matched window (`--t-start`).

**Gravity override.** `replay_estimator.sh` copies the estimator config and the
two kalibr files it references by relative path into `<outdir>/ov_config/`, so
every replay keeps an exact record of its settings. `OV_GRAVITY_MAG=<value>`
edits only `gravity_mag` in that copy, and the edit is asserted. The generated
config in `config_sim_only/` stays the single source; no second config file
exists to drift from it.

## 45. Sensor bags: disk preflight, compressed-bag reader, output on the data drive

*Files: `docker/scripts/record_and_eval.sh`, `docker/scripts/run_experiment.sh`,
`docker/scripts/bag_to_tum.py`, `docker/scripts/analyze_bag.py`*

**What was lost.** The first five path A sensor recordings on this host went to
`docker/out` on a 50 GB root partition with 1.7 GB free. rosbag2 does not stop
with an error on a full disk: it leaves a truncated `flight.bag_0.mcap.zstd`
that fails only at evaluation time (`Could not open ... Error: file too small`),
after the flight is spent. Path B then died on `No space left on device`.

**Output location (host, not repo).** `docker/out` is now a symlink to
`/mnt/data/drone_sim_out` (866 GB ext4, mounted via fstab). Docker resolves the
symlink for the bind mount, and the host-side scripts follow it. `docker/out/`
is gitignored, so nothing in the repo changes. **A fresh clone needs the same
symlink or a large-enough disk.**

**Preflight.** `record_and_eval.sh` refuses to fly unless the output drive has
**3x the expected bag size** free: 1x for the uncompressed mcap, 1x for the zstd
copy written at close, 1x margin. Expected size = data rate x simulated length:

- rate is derived from the rig: 2 imagers x 848x480 mono8 x 30 Hz = 24.4 MB per
  simulated second with `RECORD_SENSORS=1`, or a 0.5 MB/s budget without;
- length is a generous per-path budget: 120 s for path A (measured 65-72 s),
  300 s for path B; `EXPECTED_SIM_S` overrides it.

On refusal it exits 5 and writes `PREFLIGHT_FAILED.txt`; `run_experiment.sh`
treats exit 5 as fatal and aborts the whole experiment rather than logging and
moving on to runs that would hit the same wall. Verified both ways: with
`EXPECTED_SIM_S=99999999` the experiment aborted at run 1 before flying; with
defaults it reported `need 8.8 GB free (3 x 2.9 GB expected), have 793.6 GB`.

**Compressed bags were unreadable.** Even a complete sensor bag could not be
evaluated: `bag_to_tum.py` and `analyze_bag.py` used `rosbag2_py.SequentialReader`,
which cannot open FILE-compressed bags (`invalid magic bytes in Header:
0x28B52FFD...`, the zstd frame magic). Both now read `compression_mode` from
`metadata.yaml` and use `SequentialCompressionReader` when it is `FILE`. Before
this, no `RECORD_SENSORS=1` bag had ever been evaluable. Side effect worth
knowing: that reader decompresses to an uncompressed `.mcap` beside the `.zstd`
and leaves it (1.79 GB next to 252 MB for one path A flight).

**Measured on the test flight:** 252 MB compressed / 1.79 GB raw for 72 s of
sim; 2181/2182 IR frames, 14396 IMU samples, 4363 ground-truth poses. ORB-SLAM3
stereo-inertial converged ATE 0.046 m (window from sim t = 97.42 s); the
full-flight figure, 11.9 m, is the start-up transient and is not an accuracy
metric (s40).

## 46. OpenVINS never initialised on a flight; replay through ROS topics is not deterministic

*Files: `docker/sim/tools/gen_d455_sim.py` -> `config_sim_only/openvins_estimator_config.yaml`,
`docker/scripts/ov_prep.py`, `docker/scripts/phase3_openvins.sh`*

### init_imu_thresh 1.5 -> 0.5 (measured)

On the first recorded path A bag OpenVINS tracked all 2191 frames and
**never initialised**: 1950 `failed static init: no accel jerk detected`, then
`... platform moving too much` once airborne. Its static test is the std of the
accel vector over each half of `init_window_time`: the newer half must exceed
`init_imu_thresh` while the older half must not. Measured from that bag's
`/camera/imu`:

| window | accel std |
|---|---|
| on the ground | ~0.05 m/s^2 |
| PX4 takeoff, peak | 1.07 m/s^2 |

At 1.5 the trigger is unreachable. 0.5 is 10x the stationary floor with ~0.6
m/s^2 below the takeoff peak, and the predicted trigger time (t ~ 56.55 s) matched
the observed initialisation (t = 56.576 s). Changed in the generator, the single
source of truth; `--check` passes and the regenerated config differs only in
this line. Phase 2 and the Mac never exercised this, because OpenVINS had only
been boot-tested, not run on a flight.

### Determinism: FAILS under replay through ROS topics

Same bag, replayed twice through `run_subscribe_msckf`:

- Pair 1: OpenVINS's own per-update state (`save_total_state`, 1895 rows) was
  byte-identical, but the published `/odomimu` stream (12485 rows) was not:
  517 rows differed, by up to 0.248 m.
- Pair 2: **the state files differed as well, from the first state at
  initialisation** (~1e-4 m), growing to 0.0276 m max later in the flight. Pair
  1's match was luck.

**Cause.** `run_subscribe_msckf` hard-codes `params.use_multi_threading_subs =
true` (overriding the YAML) and spins a `MultiThreadedExecutor`. A queued image
is processed on a detached thread launched from the first IMU callback stamped
after it (`ROS2Visualizer::callback_inertial`). Whether the image has arrived by
then, how many newer IMU samples are already buffered, and how the processing
thread interleaves with incoming callbacks all depend on DDS delivery and
scheduling during playback, not on the bag. (An earlier draft of this section
said subscriptions were single-threaded by default; the YAML default is, but
the live node overrides it.) The very first state already differing is
consistent with the static initialiser anchoring its window to the newest
buffered IMU sample. `/odomimu` is also propagated from whatever state exists
at each IMU callback, which is why it differs more.

**Consequence.** No replay rate makes this bit-exact; it is the subscription
path, not the data. The fix is OpenVINS's own serial mode: read the bag
directly and feed IMU and stereo pairs to `VioManager` in timestamp order on one
thread. The fork ships only the ROS 1 version (`ov_msckf/src/ros1_serial_msckf.cpp`);
ROS 2 builds get only `run_subscribe_msckf`. **Not yet done: it needs a change
to the open_vins fork and a pin bump.**

`ov_prep.py` already evaluates OpenVINS from the per-update state files rather
than `/odomimu` (diagonal covariance from `ov_state_std.txt`, with the
full-covariance `/odomimu` rows at update stamps kept as a NEES cross-check), so
it will work unchanged on serial-runner output.

## 47. Deterministic OpenVINS evaluation (ros2_serial_msckf) and path A results

*open_vins fork `5aa2c83` (pinned in `docker/sim/Dockerfile`), `docker/scripts/replay_estimator.sh`
(estimator `openvins_serial`), `docker/scripts/phase3_openvins.sh`, `docker/scripts/ov_prep.py`*

**Serial runner.** `ros2_serial_msckf` (new in the fork, beside the unchanged
live `run_subscribe_msckf`) reads the bag directly and drives the live node's
own `ROS2Visualizer` callbacks in recorded order on one thread, with
`use_multi_threading_subs/pubs` off and `num_opencv_threads 0`. It builds on
Jazzy (here) and Humble (verified in a `ros:humble-ros-base` container, with
`active_slam_msgs` and the joint-covariance publisher enabled).
**Determinism: PASS.** Two runs on one path A bag gave byte-identical
`ov_state_est.txt` / `ov_state_std.txt` (1896 rows), frame counts and summaries.
Each run takes ~23 s of wall time for a 72 s flight.

**Topic-replay spread (one-off, live node, 3 replays of the same bag):** all
three were bit-identical to each other and to the serial run. Together with the
§46 pair that diverged (up to 2.8 cm), the live node's run-to-run variation is
intermittent: usually zero, occasionally centimetres.

### Path A, OpenVINS vs ORB-SLAM3 stereo-inertial, window-matched (5 bags)

Window = each bag's live ORB-SLAM3 VIBA-2 completion to the end; same window,
SE(3) alignment, no scale, 0.05 s association for both estimators. OpenVINS is
scored base-to-base (§44).

| bag | ORB conv ATE | ORB max step (window / all) | OV ATE g9.81 | OV ATE g9.80 | OV RPE | OV max step | frames proc/pub |
|---|---|---|---|---|---|---|---|
| 053636 | 0.052 | 0.089 / 1.38 | 0.761 | 0.764 | 0.643 | 0.42 | 2187/2191 |
| 054056 | 1.146 | 26.6 / 26.6 | 26.52 | 26.48 | 1.060 | 0.21 | 2170/2175 |
| 054533 | 0.027 | 0.070 / 59.2 | 0.931 | 0.906 | 0.709 | 1.48 | 2170/2174 |
| 055010 | 37.92 | 2.91 / 2.91 | 1.840 | 1.822 | 0.641 | 0.18 | 2163/2168 |
| 055428 | 0.408 | 0.297 / 132.6 | 4.805 | 4.806 | 0.892 | 0.37 | 2174/2179 |

(metres). **OpenVINS is not working correctly on this rig yet, and its numbers
should not be read as an estimator comparison.** On bag 053636: tilt error 8.9
deg rms (max 46 deg), position NEES mean 19365 and orientation NEES mean 1163
(about 3 expected), path length 1.46x ground truth, 93% of updates with 0
MSCKF features (SLAM updates average 20). Ruled out: IMU data (it matches
ground-truth-derived specific force after equal smoothing, corr 0.98, rms
0.06-0.16 m/s^2), camera-IMU time offset (online estimate +2 ms), and
frame bookkeeping (§44). **Gravity 9.80 vs 9.81: differences of <= 0.03 m, an
order of magnitude below OpenVINS's own error here; no effect measurable
until OpenVINS works.** ORB-SLAM3's "converged" window is not converged on 2 of
5 bags (26.6 m and 2.9 m steps inside it).

### Joint covariance (serial run, 281 messages, full path A flight)

Analysed from raw recorded matrices with relative tolerances:

- **Rank deficit at `max_eig * 1e-12`: exactly 6 in 281/281.** 6th-smallest /
  max eigenvalue: median 2e-16 (max 9e-16). 7th-smallest / max: median 4.5e-11
  (min 8.7e-12), a gap of 4-5 orders. The extra "deficit" the checker reported
  (12-24) came from its looser 1e-9 relative threshold catching these ~1e-11
  directions.
- **Localization:** the 6 smallest eigenvectors span (IMU pose - newest clone)
  mid-flight; largest principal angle median 1.5e-4 deg, max 1.4e-3 deg.
- **When it is published:** `visualize()` -> `publish_joint_covariance()`
  runs after `feed_measurement_camera()` completes (propagate_and_clone, MSCKF
  update, SLAM update, delayed init, SLAM and oldest-clone marginalisation), the
  same point in both the live and serial nodes. The duplicate-row structure
  survives that: max|row(IMU pose) - row(newest clone)| / max|C| has median
  6.6e-17 and max 1.2e-13; ||C N|| / max_eig has median 4e-18. That is roundoff.
- **Unresolved:** `check_joint_cov.py`'s earlier sample at t = 83.956 reported
  dim 130 and a 2.3e-9 residual, while the recorded matrix at the same stamp is
  dim 132 with 3e-19. Two serial runs should publish identical matrices, so this
  is either a checker or receive-side issue, or joint-covariance content that is
  not deterministic even though the state files are. Not yet explained.

## 48. Fork sanity, MSCKF starvation, and the fast_threshold rule (pre-registered)

### Fork sanity: OpenVINS's own simulator is healthy

`run_simulation` on the fork's stock `config/rpng_sim` (`tum_corridor1`, 296 m
over 292 s), scored directly from its saved state, std and ground-truth files
(same world frame, first 10 s skipped): **position RMSE 0.046 m, orientation
RMSE 0.18 deg, NEES position 3.8 / orientation 0.7** (diagonal covariance, ~3
expected). The process segfaults during ROS publisher teardown *after* the
trajectory completes; the estimator itself is fine. **The fork and core
estimator are not the problem; the input from our sim is.** (EuRoC V1_01 could
not be fetched: the ETH Research Collection rejects scripted downloads with
HTTP 429.)

### Conventions ruled out (read-only checks)

- Stereo geometry: the config's cam0->cam1 is a 0.095 m baseline with cam1 at +x
  in cam0's optical frame, matching the SDF. Disparity measured on bag images is
  positive for 95-100% of matches, so infra1 is the left camera.
- Init: tilt error 1.07 deg at init, below 0.5 deg within 2 s, 0.5 deg rms
  overall, after aligning the constant world-yaw offset. (An earlier figure of
  "8.9 deg rms" compared gravity directions across world frames 180 deg apart
  in yaw; it was wrong.)
- IMU frame: stationary accel (-0.024, -0.016, 9.833) m/s^2 in the IMU frame
  implies 0.17 deg tilt; ground truth is 0 deg and IMU axes = base axes.
- `ov_eval error_singlerun posyaw` orientation RMSE (15-33 deg) and its NEES are
  distorted by a yaw alignment fitted to metre-scale position error; revisit
  once position is fixed.

### Why MSCKF is starved (instrumented diagnostic build, not committed)

Bag 053636, 1895 updates, current config:

| | |
|---|---|
| tracked features per image (trackhist) | ~30-45 of `num_pts: 200` |
| MSCKF candidates | 2019 total, ~1 per update; 70% of updates have none |
| ... from lost tracks / marginalised / max-track leftovers | 1743 / 132 / 145 |
| SLAM features in state | mean 20, max 48 (cap 50) |
| rejected in the MSCKF updater | 1672 (83%): triangulation 1345, <2 measurements 240, refine 47, **chi2 40** |

Not chi2 rejection, and not mainly SLAM promotion either: too few features are
tracked, and the short tracks of distant features (median stereo disparity
2.4-6 px, i.e. 7-18 m) fail triangulation. Sim(3) alignment gives scale
0.70-0.85 on the three non-diverged bags (0.03 / 0.11 on the diverged two),
with ATE still 0.65-1.2 m, so the long path is real metric error, not jitter.

### fast_threshold rule -- fixed BEFORE any ATE is computed with a new value

`fast_threshold` is a sensor-matching front-end setting: ORB-SLAM3 lowers its
FAST threshold automatically (20 -> 7) on low-contrast images, while OpenVINS
uses a fixed value; at 30 it tracks ~30-45 of 200 features here.

- **Rule:** on ONE tuning bag, run the serial runner at each threshold in the
  grid {30, 25, 20, 15, 12, 10, 8, 7, 6, 5}. Measure the **mean number of
  features tracked per frame in cam0** (left camera) over all frames. Choose
  the **highest threshold whose mean is >= 150**. If none reaches 150, choose 5
  and say so.
- **Tuning bag:** path A `20261004-053636`. It is excluded from the results.
- **Nothing else changes:** `num_pts`, grid, KLT, MSCKF/SLAM limits, noise,
  `init_imu_thresh` all stay as they are.
- **Results:** path A on the other four bags (054056, 054533, 055010, 055428).
- The same rule is to be applied on the real D455, so the counts at every
  threshold are logged below, not just the chosen value.

### fast_threshold sweep (tuning bag 053636, serial runner, all 2191 frames)

Features tracked per frame, from the tracker's last observations (diagnostic build):

```
fast_threshold  frames  cam0 mean  cam0 median  cam0 p10  cam1 mean
            30    2191       75.3           82        38       78.5
            25    2191       75.2           80        39       80.3
            20    2191       77.4           84        45       81.0
            15    2191       75.6           80        47       83.0
            12    2191       76.8           82        45       80.0
            10    2191       79.3           83        45       83.5
             8    2191       81.3           83        45       86.6
             7    2191       80.7           81        46       82.2
             6    2191       78.1           83        47       86.6
             5    2191       77.9           82        47       78.9
```

**No threshold reaches 150; by the rule the value is 5** (applied in
`gen_d455_sim.py`, nothing else changed). **The premise did not hold:** the
count is flat at 75-81 from 30 down to 5, so FAST threshold is not what limits
it. (The ~30-45 estimated by eye from trackhist was also low; the tracker's own
count is ~75.) A likely limiter, not yet verified, is the 5x5 extraction grid
(`num_pts` 200, so 8 per cell) with the upper half of every image blank sky.

**Path A with fast_threshold 5, the four non-tuning bags** (window-matched, g 9.81):

| bag | OV ATE thr 30 | **OV ATE thr 5** | OV RPE thr 5 | OV max step thr 5 | ORB conv ATE |
|---|---|---|---|---|---|
| 054056 | 26.52 | **27.21** | 1.068 | 0.22 | 1.146 |
| 054533 | 0.931 | **0.881** | 0.759 | 1.27 | 0.027 |
| 055010 | 1.840 | **1.381** | 0.564 | 0.39 | 37.92 |
| 055428 | 4.805 | **4.610** | 0.895 | 0.56 | 0.408 |

Essentially unchanged, as the flat feature counts predicted. OpenVINS remains at
metre-level error. Bag 054056 diverges at both thresholds (Sim(3) scale 0.03).

## 49. Validation scene, scene-degradation measurement, and the post-landing failure

*Files: `docker/sim/tools/gen_validation_world.py` (new) -> `docker/sim/worlds/validation.sdf`,
`docker/sim/models/validation_scene/`; `gen_d455_sim.py` (per-world bridge configs);
`bringup_sim.sh` / `run_experiment.sh` (`WORLD=`); `ov_scene_stats.py`, `ov_full_metrics.py`,
`phase3_table.py` (new); open_vins `5b14b93` (DEBUG-level `[FI] [MSCKF] [TRACK] [GRID]` logging,
estimator output verified byte-identical to `5aa2c83`; builds on Jazzy and Humble).*

### Which check rejects triangulation (forest bag 053636; defaults, none configured)

`fi_min_dist 0.10`, `fi_max_dist 60`, `fi_max_baseline 40`, `fi_max_cond_number 10000`.
Of 11006 triangulation rejects, **99% trip the condition-number check** (median
condA 1.1e5): cond only 10250, cond+near 620, near only 114, cond+far 22, NaN 0.
Median depth of rejected features 8.1 m; of accepted ones 3.75 m. Refine rejects
are almost all inherited (refine runs after a failed triangulation and gets NaN);
genuine depth/baseline > 40 rejections: 221.

Features per 5x5 grid cell (cam0 mean, cap 8 per cell): the top row holds 0.8-1.1
per cell (sky), the lower rows 1.3-6.7. **No cell reaches its cap, so the "sky
cells" explanation is only partial**: the top row is nearly empty, but grid
saturation is not what holds the total at ~78.

### Validation scene

Same rig and paths; textured walls and boxes 2-6 m from the path, interior left
clear for path B; flown at 1.5 m. Physics, gravity (-9.81), magnetic field, sun
and coordinates as forest. Lighting differs on purpose: with forest's settings
the enclosure left the first frame nearly black (mean 22/255, 7 FAST corners), so
shadows are off and textures are emissive (mean 112/255, 1796 FAST corners).
`WORLD=validation` selects it end to end (generated bridge with
`/world/validation/...` topics, experiment directories tagged `_validation`,
world and its gravity recorded in the manifest).

### Result: OpenVINS is accurate in flight; it fails after touchdown

**Airborne segment (init to touchdown), OpenVINS ATE, SE(3), no scale:**

| set | runs | ATE (m) | mean | Sim(3) scale | median depth (all attempts) | tri reject |
|---|---|---|---|---|---|---|
| validation A | 5 | 0.030 0.021 0.037 0.029 0.030 | **0.029** | 0.986-1.001 | 3.9-4.5 m | 57-69% |
| validation B | 5 | 0.071 0.079 0.047 0.058 0.051 | **0.061** | 0.998-1.003 | 4.2-4.4 m | 59-64% |
| forest A (excl. tuning bag) | 4 | 4.06 0.79 0.11 0.41 | 1.34 (median 0.60) | 0.23-1.00 | 6.0-8.7 m | 72-91% |
| forest B | 5 | 0.36 0.25 2.57 0.28 152.6 | median 0.36 | 0.002-0.97 | 7.2-8.0 m | 85-94% |

**The pipeline is validated end to end in flight.** Forest degradation is now
measured, not just observed: deeper features (median 7-9 m vs ~4 m) and more
triangulation rejections (72-94% vs 57-69%) go with 1-2 orders of magnitude worse
airborne ATE, and two forest flights diverge in the air.

**Post-landing failure (validation A runs 2, 3, 5 and B runs 1, 3, 4; full
post-init ATE 2-6 m).** Error stays at 0.01-0.14 m for the whole flight, then
OpenVINS's velocity climbs (0.8 -> 2.3 m/s) while ground truth is stationary.
On the ground, tracking is fine (86-87 features/frame), but **SLAM features
collapse from ~46 to ~0.5 per update and MSCKF to ~0.** With no motion there is
no parallax, so lost SLAM features cannot be re-initialised (triangulation
rejects jump to 3200-3500 per 600 frames), and the filter integrates the IMU
alone. In the bad run inspected, the stationary accelerometer read 9.635 m/s^2
(good run 9.807), and 0.17 m/s^2 integrated over ~15 s is the observed ~2.5 m/s.
In the good runs, SLAM features established in flight survive touchdown (49 per
update) and the estimate stays bounded. OpenVINS's remedy for stationary periods
is zero-velocity updates; `try_zupt` is **false** in this config. Not changed.

### Evaluation caveats found along the way

- The matched window starts at ORB-SLAM3's VIBA-2 completion and can land late:
  on the first validation bag it covered 1.9 m of a 17.6 m path, mostly hover
  and landing. On this rig it also routinely includes the post-landing phase,
  where OpenVINS fails for the reason above. Window coverage (`win path m`,
  `win frac`) is now reported next to every window figure.
- ORB-SLAM3's "converged" window still contains jumps: 102 m on the first
  validation bag, 14.6 m on validation A run 1, 26.6 m / 2.9 m on forest bags.
  "After VIBA 2" is not a sufficient definition of converged (noted, not acted on).
- `ov_eval error_singlerun posyaw` NEES is unreliable here: on a near-stationary
  window its position-fitted yaw was 4.7 deg off, inflating orientation RMSE to
  5.3 deg (true error < 1 deg). An independent computation reproduces ov_eval
  exactly, so it is the alignment, not the tool. NEES is now also computed on
  the full post-init trajectory, but those values still include the post-landing
  divergence and are not yet meaningful as a consistency measure.

## 50. ZUPT test: fires while airborne on all 10 validation bags — left disabled

**Semantics of `zupt_chi2_multipler: 0`** (`UpdaterZeroVelocity::try_update`):
a ZUPT is accepted if `disparity_passed || (chi2 <= mult*chi2_95 && |v| <= zupt_max_velocity)`.
With mult 0 the IMU branch can never pass, so ZUPT is **disparity-only**
(mean disparity < `zupt_max_disparity` with > 20 features), and the
velocity gate is bypassed on that path.

**Instrumentation.** Fork `7da42fa`: `[ZUPTEV] t=<image time> accepted=<0|1> v=<|v_est|>`
after every ZUPT decision (INFO, no behaviour change). Pin bumped in
`docker/sim/Dockerfile`. `docker/scripts/zupt_events.py <replay_dir> [--json]`
matches each decision to GT speed (+-0.1 s central difference) and height
above initial ground; airborne = height > 0.05 m; exit 1 on any airborne accept.

**Test.** `try_zupt: true`, `zupt_max_disparity 0.5`, mult 0, serial runner,
g 9.81, all 10 validation bags, outputs `replay/<run>_ovser_g9.81_zupt/`.

| run | accepted | **airborne** | after touchdown | max GT speed at accept (m/s) | first accept after touchdown (s) |
|---|---|---|---|---|---|
| A 193700 | 1307 | 483 | 595 | 0.224 | 0.09 |
| A 194115 | 1317 | 504 | 597 | 0.208 | 0.11 |
| A 194531 | 1232 | 417 | 599 | 0.186 | 0.10 |
| A 195005 | 1244 | 439 | 602 | 0.225 | 0.11 |
| A 195439 | 1330 | 512 | 593 | 0.175 | 0.24 |
| B 195913 | 2608 | 1803 | 594 | 0.226 | 0.13 |
| B 200912 | 2599 | 1789 | 595 | 0.226 | 0.10 |
| B 201905 | 2660 | 1838 | 596 | 0.216 | 0.11 |
| B 202940 | 2596 | 1780 | 601 | 0.255 | 0.13 |
| B 204137 | 2633 | 1817 | 601 | 0.222 | 0.09 |

**FAIL**: airborne ZUPTs on every bag (h ~1.3 m, GT speed 0.07–0.25 m/s; slow
climb and hover). Disparity does not separate the cases (run 193700):
airborne disparity min 0.039 / p1 0.075 / p5 0.131 px; after touchdown p50 0.000 /
p95 0.038 / p99 0.131 px. No threshold on mean disparity removes the
airborne accepts without losing ground accepts. Probable reason: noise-free
simulated cameras make a hovering view nearly pixel-static (OPEN_ISSUES §1).
The chi2 at these accepts (55–87) shows the IMU test would have rejected them.

`try_zupt` reverted to `false` in `gen_d455_sim.py`; ZUPT config is a pending
decision. ZUPT results above must not be used as estimator results.

## 51. Ground-truth airborne window (step 2): both estimators, all 20 bags

`docker/scripts/gt_window_eval.py`: window = [GT takeoff + 3 s, GT touchdown],
takeoff/touchdown at 0.05 m above initial ground (touchdown after the last
time above 0.5 m). Same window for every estimator; SE(3) ATE, no scale,
0.05 s association; Sim(3) scale on the same window. Inputs: live
`eval/<run>/est_orbslam3.tum` (ORB-SLAM3 stereo-inertial, **online** poses) and
serial-runner `replay/<run>_ovser_g9.81/est_openvins.tum` (base_link), ZUPT off.
Raw: `docker/out/replay/gt_window_s51.jsonl`. "coverage" ≈0.5 everywhere just
reflects estimate rate vs 50 Hz GT — no gaps in the window for either estimator.

ATE (m) / Sim(3) scale:

| run | ORB-SLAM3 | OpenVINS |
|---|---|---|
| valA 193700 | 6.02 / 0.14 | 0.029 / 0.995 |
| valA 194115 | 8.24 / 0.11 | 0.021 / 0.998 |
| valA 194531 | 1.42 / 0.72 | 0.037 / 0.986 |
| valA 195005 | 1.12 / 0.85 | 0.029 / 0.994 |
| valA 195439 | 31.8 / 0.03 | 0.029 / 1.001 |
| valB 195913 | 2.53 / 0.45 | 0.069 / 1.000 |
| valB 200912 | 2.83 / 0.41 | 0.077 / 1.000 |
| valB 201905 | 51.0 / 0.01 | 0.046 / 1.003 |
| valB 202940 | 1.47 / 0.71 | 0.056 / 0.998 |
| valB 204137 | 3.15 / 0.36 | 0.050 / 1.000 |
| forA 053636 (tuning, excluded) | 0.34 / 0.96 | 0.70 / 0.86 |
| forA 054056 | 11.6 / 0.07 | 4.19 / 0.23 |
| forA 054533 | 29.2 / 0.03 | 0.80 / 0.82 |
| forA 055010 | 6.73 / 0.08 | 0.110 / 1.000 |
| forA 055428 | 64.9 / 0.01 | 0.42 / 0.91 |
| forB 055847 | 0.30 / 0.98 | 0.36 / 0.97 |
| forB 060837 | 1.31 / 0.78 | 0.25 / 0.97 |
| forB 061845 | 0.65 / 0.94 | 2.57 / 0.47 |
| forB 062853 | 12.9 / 0.06 | 0.28 / 0.97 |
| forB 063858 | 0.51 / 0.94 | 154.1 / 0.002 |

OpenVINS validation values equal §49 R1 to 3 decimals (consistency check of
the new window). Time-to-usable-estimate (first output − takeoff): OpenVINS
+0.03 to +0.28 s on all runs. ORB-SLAM3 first output is either ~−10 s
(tracking from the ground before takeoff) or +0.0–0.3 s; its first output is
not a *usable* (metric) estimate — Sim(3) scales of 0.01–0.85 show the online
trajectory carries pre-IMU-init / map-reset segments. A usable-time definition
for ORB-SLAM3 (e.g. first time after which ATE on a trailing span stays below a
bound, or VIBA-2) is still open, as is replaying ORB-SLAM3 on the bags.

## 52. ORB-SLAM3 replayed on the validation bags: stereo-only cm-level, stereo-inertial broken by the IMU path

**Tooling.** `ORB_SLAM3/src/Tracking.cc` prints `[ORBEV]` lines stamped with
frame (sim) time on any change of active map / IMU init / VIBA 1 / VIBA 2 /
tracking state, on every active-map reset, and `frames=N` every 100 frames
(no behaviour change). `replay_estimator.sh`: `PLAY_RATE` (`ros2 bag play
--rate`); plays an explicit topic list because the input bags also carry the
LIVE `/robot_pose_slam`; copies the node's ROS log (`node.log`; its INFO lines
do not reach the launch stdout). `cam_stamps.py` (bag image stamps),
`orb_events.py` (report), `phase3_orbslam3.sh` (driver). Rate 0.5.
Raw: `docker/out/replay/orb_replay_s52.jsonl`, dirs `replay/<run>_orb_{st,si}_r0.5/`.

Definitions: usable time = IMU init completion inside the final map segment
(no reset or map change after it), relative to GT takeoff; stereo-only = start
of its final segment. ATE (SE(3)) on the §51 GT window, and on
[usable, touchdown]. Frames missing = bag images up to the last counter line −
frames reaching Track() − resets.

| run | flight s | ST ATE GT win | SI resets (rel. takeoff, s) | SI IMU init / VIBA1 / VIBA2 (final seg) | SI usable | SI ATE GT win / scale | SI ATE usable / scale |
|---|---|---|---|---|---|---|---|
| A 193700 | 43.5 | 0.012 | −5.1, −2.1 | 0.4 / 3.5 / 16.1 | 0.4 | 9.65 / 0.07 | 10.07 / 0.07 |
| A 194115 | 43.4 | 0.008 | −2.5, **10.7** | 13.1 / 17.9 / 31.9 | 13.1 | 5.30 / 0.19 | 5.67 / 0.22 |
| A 194531 | 43.3 | 0.013 | −3.1 | −0.6 / 5.4 / 19.9 | −0.6 | 16.89 / 0.04 | 17.14 / 0.04 |
| A 195005 | 43.3 | 0.012 | none | 2.4 / 5.1 / 19.9 | 2.4 | 7.44 / 0.10 | 7.46 / 0.10 |
| A 195439 | 43.6 | 0.012 | −4.4 | −1.9 / 5.3 / 25.4 | −1.9 | 0.68 / 0.88 | 0.68 / 0.87 |
| B 195913 | 170.0 | 0.010 | −2.4, **8.2** | 10.7 / 17.5 / 30.5 | 10.7 | 90.5 / 0.00 | 81.2 / 0.00 |
| B 200912 | 170.0 | 0.009 | **14** (−3.6 … 124.0) | 126.4 / 129.4 / 143.4 | 126.4 | 43.8 / 0.01 | 57.5 / 0.02 |
| B 201905 | 169.9 | 0.010 | none | 2.5 / 10.0 / 25.6 | 2.5 | 17.85 / 0.03 | 18.00 / 0.02 |
| B 202940 | 169.8 | 0.009 | none | 2.4 / 10.1 / 24.3 | 2.4 | 13.15 / 0.05 | 13.29 / 0.05 |
| B 204137 | 169.8 | 0.010 | −6.3, −0.3 | 2.2 / 10.4 / 23.6 | 2.2 | 20.88 / 0.02 | 21.19 / 0.02 |

- **Stereo-only: 0.7–1.3 cm on all 10 bags, scale 1.00**, frames missing 0–2.
  Scene and camera config are fine.
- Stereo-inertial: frames missing 0–8 (negligible). Resets on the ground
  ("Not enough motion for initializing") are normal for a stationary start.
  **The flights are long enough**: VIBA 2 completes on every bag (16–32 s after
  takeoff, except 200912 with 14 resets). Yet ATE over the usable segment
  (IMU-initialised, no further reset) is 0.7–81 m with scale 0.00–0.88: the
  inertial solution is wrong after a nominally successful init. Since vision
  alone is cm-level on the same frames, the fault is in the IMU path
  (data handling, IMU noise/extrinsics config, or units), not the scene.
- Not deterministic: 193700 SI gave different reset histories in three replays.
- Wrapper IMU path **ruled out**: the stereo-inertial node now reports on its
  clock beacon `imu_gaps` (consecutive drained samples > 10 ms apart),
  `imu_late` (sample stamped at/before the previous frame, i.e. preintegrated
  into the wrong interval) and `imu_maxgap`. Replay of 195005 at rate 0.5:
  **0 gaps, 0 late over 2197 frames**. The IMU data reach ORB-SLAM3 complete
  and in order, so the remaining suspects are ORB-SLAM3's inertial settings
  (`orbslam3_d455_stereo_inertial.yaml`: Tbc, IMU noise, frequency, units/axes)
  -- open, not investigated yet.

## 53. OpenVINS NEES on the GT window (step 3) and gravity 9.80 vs 9.81 (step 4)

**NEES tool.** `docker/scripts/nees_window.py <replay_dir> <gt.tum>`: §51 GT
window; GT interpolated to each estimate stamp (linear / slerp); diagonal
covariance from `ov_state_std.txt` (all that is saved deterministically).
Orientation error local (IMU frame), matching OpenVINS's JPL error. pos/ori
NEES after posyaw (4-DOF) alignment; **roll/pitch NEES with no alignment**:
error projected perpendicular to gravity in the IMU frame (a world-yaw offset
lies exactly along it). Cross-check: pos/ori means equal `ov_eval
error_singlerun posyaw` on the same window to within interpolation (e.g.
193700: 90.0/110.2 vs 85.9/104.0 interpolated). Raw: `replay/nees_s53.jsonl`.

Mean NEES (fraction inside 95% chi2 interval), g 9.81, validation, ZUPT off.
Expected: pos 3, ori 3, rp 2; in-95 0.95.

| run | pos | ori | roll/pitch | pos RMSE (m) | tilt RMSE / median σ (deg) |
|---|---|---|---|---|---|
| A 193700 | 86 (0.24) | 104 (0.15) | 117 (0.17) | 0.029 | 0.27 / 0.05 |
| A 194115 | 55 (0.39) | 89 (0.16) | 213 (0.07) | 0.022 | 0.42 / 0.05 |
| A 194531 | 30 (0.57) | 341 (0.35) | 460 (0.16) | 0.037 | 0.43 / 0.06 |
| A 195005 | 134 (0.29) | 96 (0.16) | 233 (0.06) | 0.029 | 0.42 / 0.06 |
| A 195439 | 47 (0.41) | 117 (0.19) | 289 (0.14) | 0.029 | 0.36 / 0.05 |
| B 195913 | 220 (0.27) | 318 (0.14) | 364 (0.15) | 0.069 | 0.36 / 0.03 |
| B 200912 | 257 (0.21) | 150 (0.26) | 163 (0.25) | 0.077 | 0.24 / 0.03 |
| B 201905 | 159 (0.34) | 203 (0.16) | 206 (0.18) | 0.046 | 0.29 / 0.03 |
| B 202940 | 257 (0.23) | 186 (0.18) | 208 (0.17) | 0.056 | 0.27 / 0.03 |
| B 204137 | 60 (0.46) | 101 (0.25) | 116 (0.23) | 0.051 | 0.20 / 0.03 |

**OpenVINS is overconfident by roughly 5–15x in sigma**, on every bag, while
accurate. Breakdown (193700, 195913): horizontal position σ ≈ 5 cm
(conservative), **vertical σ ≈ 2.4 mm (min 1.1 mm) vs ~2–3.5 cm z error**, tilt
σ 0.02–0.04°/axis vs ~0.2°/axis scatter. Artefacts ruled out: GT
association (interpolation changes NEES < 10%); GT–sensor time offset (scan
±60 ms: best offset ≤ 4 ms, no change); constant mounting tilt (tilt error
mean ≤ 0.12°, it is scatter, not bias); IMU noise mismatch (SDF white noise
0.00226 rad/s and 0.0283 m/s² per sample at 200 Hz = configured densities
1.6e-4 / 2e-3; OU bias σ_b·√(2/τ) = configured random walks). Cause open;
the diagonal-only covariance can change NEES but cannot explain a 10x
too-small z variance. Earlier NEES retractions (HANDOFF §7) still stand; these
are the first numbers on a well-conditioned airborne window.

**Gravity 9.80 vs 9.81** (world `<gravity>` is 9.81). All 10 validation bags
replayed with `OV_GRAVITY_MAG=9.80` (`replay/<run>_ovser_g9.80/`), same GT
window. Raw: `replay/gravity_s53.jsonl`, `gravity_nees_s53.jsonl`.

| run | ATE 9.81 | ATE 9.80 | diff (m) | Sim(3) scale 9.81 / 9.80 |
|---|---|---|---|---|
| A 193700 | 0.0293 | 0.0288 | −0.0005 | 0.9947 / 0.9952 |
| A 194115 | 0.0206 | 0.0203 | −0.0003 | 0.9977 / 0.9982 |
| A 194531 | 0.0366 | 0.0358 | −0.0009 | 0.9863 / 0.9868 |
| A 195005 | 0.0288 | 0.0283 | −0.0005 | 0.9945 / 0.9950 |
| A 195439 | 0.0290 | 0.0292 | +0.0003 | 1.0012 / 1.0017 |
| B 195913 | 0.0687 | 0.0688 | +0.0001 | 0.9996 / 1.0002 |
| B 200912 | 0.0768 | 0.0771 | +0.0003 | 0.9999 / 1.0005 |
| B 201905 | 0.0457 | 0.0462 | +0.0005 | 1.0029 / 1.0035 |
| B 202940 | 0.0556 | 0.0555 | −0.0000 | 0.9980 / 0.9985 |
| B 204137 | 0.0499 | 0.0500 | +0.0001 | 0.9999 / 1.0005 |

Mean ATE difference (9.80 − 9.81): set A −0.4 mm, 95% CI [−0.9, +0.1];
set B +0.2 mm, CI [−0.1, +0.4]. Opposite signs, both CIs include 0:
**no ATE effect** by the replication rule. Sim(3) scale rises by
+0.0005–0.0006 on all 10 bags (replicates in both sets): a real but
negligible scale effect, about half the 0.1% gravity change. NEES unchanged
(≤ 2%).

## 54. OpenVINS overconfidence: Gazebo's IMU bias walks 42x faster than configured

**Question.** NEES 30–460 on the GT airborne window (§53) while OpenVINS's
own simulator gives 3.8. Reprojection error is ~0.1 px against a modelled
1 px, which alone would make it *under*confident, so the filter trusts
something else too much. Diagnostics run cheapest first on validation bags
A 193700, A 194531, B 195913. Scripts: `docker/scripts/s54/` (run from a copy
in `/out/diag_s54/`), outputs in `docker/out/diag_s54/` and
`docker/out/replay/*_{calibR,synth_cfg,synth_gz}`. No config was changed.

**Ruled out**
1. *Config.* The key is `use_fej` (no `do_fej` exists): true, parsed as 1.
   All `calib_*` false, `timeshift_cam_imu` 0, rk4.
2. *Online calibration* (`OV_SET` intrinsics/extrinsics/timeoffset on; hook
   added to `replay_estimator.sh`). Settles near the SDF: f/c within ±0.8 px
   (cx +1.8 px on 193700, sign varies by bag), distortion ≤ 0.002, extrinsic
   rotation 0.03–0.3°, translation ≤ 6 mm; cam–IMU dt converges to
   **+2.08–2.16 ms on all three**. NEES pos/ori: 86/104 → 77/68,
   30/341 → 18/267, 220/318 → 242/297. Not the cause.
3. *Projection* (`geom_timing.py`): stereo-triangulated corners moved by GT
   and reprojected ~100 ms later: median 0.10 px on all bags; stereo row
   offset p95 ≤ 0.18 px. Cannot separate cx 424 from 423.5 (a shared
   principal-point shift cancels: 0.0999 vs 0.0996 px).
4. *Stamp intervals.* Every IMU and image stamp is on the 4 ms physics grid.
   IMU: 4,4,4,8 ms (mean 5.000 ms); cameras 32/36 ms. One 220 ms IMU gap in
   194531 at t = 58.3 s, before the window.
5. *Image stamp vs render pose.* Reprojection is minimised at 0 ms GT offset
   on all bags (+20% at ±4 ms).
6. *Frame.* GT is the model pose = base_link origin (PX4 `x500_base` gives
   base_link no `<pose>`); `ov_prep.py` already moves it to the IMU (0.257 m)
   before NEES, so position NEES is at the matching point.

**Ground-truth timing caveat.** GT stamps are bag receive time mapped through
`/clock` (`bag_to_tum.py --time-from-clock`), not sim stamps: median 0.15 ms
after the 4 ms grid, 1–2% more than 1 ms off. Snapped to the grid for the
IMU work; a few mis-snaps by one step remain and make second differences of
GT heavy-tailed, so robust (MAD) statistics are used below.

**IMU vs ground truth** (`imu_vs_gt.py`; no GT differentiation: IMU integrals
vs GT rotation increments and second divided differences of position, over
the airborne window)
- Gyro–GT time offset: best at IMU stamp **−2 ms** on all three bags (the
  IMU sample describes motion ~2 ms before its stamp; same size as the online
  cam–IMU dt).
- Gazebo reports acceleration at the sensor/d455_link point, not base_link
  (residual much larger with the base_link point): no lever-arm bug.
- White noise (MAD over ~32 ms spans; white-only expectation acc 0.0126,
  gyro 0.00118): acc 0.024–0.092, gyro 0.0014–0.0017 per axis — 1.2–1.5x
  for the gyro, 2–7x for the accelerometer, partly GT timing.
- **Accelerometer bias vs GT**, median per flight quarter: 0.03–0.24 m/s² per
  axis, different on every bag, and it **drifts within the flight** (194531 y
  0.003 → 0.162 m/s² over ~40 s; 195913 x −0.19 → −0.06 → −0.18). The
  configured random walk (3e-4 m/s²/√s) allows ~0.002 m/s² in 40 s. Gyro bias
  0.2–2.3e-3 rad/s.

**Cause, from the gz-sensors8 source** (`GaussianNoiseModel.cc:135`):

    sigma_b_d = sqrt(-sigma_b^2 * tau/2 * expm1(-2 dt/tau))  ~= sigma_b * sqrt(dt)
    bias = exp(-dt/tau) * bias + N(0, sigma_b_d)

`dynamic_bias_stddev` is the **driving density** (steady-state σ is
σ_b·√(τ/2)), not the steady-state σ that `gen_d455_sim.py` assumes (its
docstring and lines 134–135 set σ_b = σ_rw·√(τ/2)). Gazebo's actual random
walk is therefore acc **0.0127 m/s²/√s** (config 3e-4) and gyro
**8.5e-5 rad/s/√s** (config 2e-6): **42.4x** the configured values. With no
`bias_mean`/`bias_stddev`, the bias starts at 0 at sim start and has walked
to σ ≈ 0.0127·√60 ≈ 0.1 m/s² by takeoff — the magnitude measured above.
A 0.03 m/s² accel bias error is 0.18° of tilt, the observed tilt scatter
(§53: ~0.2° vs σ 0.03°). §53's statement that the SDF OU bias matches the
configured random walks is **wrong** (retracted).

**Decisive swap test** (`synth_imu.py`, `run_swap.sh`). The bag's
`/camera/imu` is replaced by IMU synthesised from GT (snapped stamps with
leave-one-out mis-snap repair, cubic position spline at the IMU point,
rotation spline; same stamps, real images untouched). `cfg`: white noise and
bias walk drawn exactly per the OpenVINS config. `gz`: same, but the bias
walks at Gazebo's effective density, started from N(0, σ_b²·t).
The spline signal itself differs from the real accelerometer by MAD
0.04–0.10 m/s² (white alone is 0.028), so the synthetic IMU carries some
unmodelled error of its own.

Mean NEES (in-95 fraction), expected pos 3, ori 3, rp 2:

| run | real Gazebo IMU (§53) | synth `cfg` | synth `gz` (control) |
|---|---|---|---|
| A 193700 pos / ori / rp | 86 / 104 / 117 | 18.8 / **3.2** / 8.3 | 7.5 / 74 / 82 |
| A 194531 | 30 / 341 / 460 | 2.5 / **2.7** / 5.6 | 13 / 76 / 74 |
| B 195913 | 220 / 318 / 364 | 70 / **3.9** / 9.0 | 155 / 249 / 298 |
| tilt RMSE (deg) | 0.27 / 0.43 / 0.36 | 0.11 / 0.08 / 0.07 | 0.20 / 0.18 / 0.30 |

**Conclusion.** The Gazebo IMU is the cause of the orientation/tilt
overconfidence: with config-consistent IMU noise orientation NEES is
2.7–3.9; putting back only the Gazebo-style bias walk restores 74–249.
Position NEES is only partly explained (2.5 / 19 / 70 with `cfg`);
the remainder is not attributed — candidates are the synthetic signal's own
spline error, the diagonal-only covariance and posyaw alignment. Roll/pitch
NEES (5.6–9.0) remains above 2 with `cfg`, same caveat.

**Not done (fix candidates, need a decision):** (a) regenerate the SDF with
σ_b = σ_rw (so Gazebo walks at the configured rate) and re-record; or
(b) keep the bags and set OpenVINS's random walks to Gazebo's effective
values (acc 0.0127, gyro 8.5e-5) for a replay. (a) keeps the simulated IMU
D455-like; (b) needs no new flights. The −2 ms IMU stamp lag is a separate,
smaller effect and is not addressed.

## 55. IMU bias fix, re-recorded bags, ov_prep position-σ bug, full-covariance NEES

**1. Confirmation replay (diagnostic, config not kept).** Old validation bags
replayed with OpenVINS random walks set to Gazebo's effective values
(`OV_IMU_RW="0.0127279221 8.48528137e-05"`, new asserted hook in
`replay_estimator.sh`). Ori / rp NEES: 193700 104/117 → 29/34, 194531 341/460
→ 10/20, 195913 318/364 → 25/27. Large drop, not ~3: the bias had walked
from 0 at sim start to ~0.1–0.2 m/s² by takeoff, beyond OpenVINS's initial
bias prior.

**2. Fix.** `gen_d455_sim.py`: `dynamic_bias_stddev = sigma_rw` (gz-sensors
uses it as the random-walk density; stationary σ is then σ_rw·√(τ/2)). The
s54 user note proposed σ_b = σ_rw·√(τ/2), which is the old buggy formula,
so it was not used. Only the SDFs change (accel 0.0127 → 3e-4,
gyro 8.5e-5 → 2e-6); estimator configs were already right.
`docker/sim/tools/test_gen_d455_sim.py` parses the generated SDFs and
configs, runs gz-sensors' exact recursion at 200 Hz for 120 s (4000
realisations) and requires the effective random walk = configured (±5%),
plus an analytic check (±0.1%) and a guard against σ_b/σ_rw > 2. Fails on
the old files (ratio 42.4), passes on the new.

*New IMU vs GT* (s54 tools, test flight 20261005-193151): accel bias ≤ 0.006
m/s² (old 0.03–0.24), drift ≈ 0.003 m/s² over the flight; gyro bias ≤
1.7e-4 rad/s (old ≤ 2.3e-3); accel MAD 0.0145 vs 0.0126 white-only (old
0.024–0.09: most of s54's "excess white noise" was the bias walk). The
gyro–GT lag of −2 ms is unchanged (not a noise-model effect).

*Re-recorded*, same parameters as the originals (all `20261005-`):

| condition | experiment dir |
|---|---|
| validation A | `experiment_d455_pathA_validation_20261005-154055` (194208 194656 195112 195546 200002) |
| validation B | `experiment_d455_pathB_validation_20261005-160305` (200418 201426 202424 203422 204400) |
| forest A | `experiment_d455_pathA_20261005-165226` (205349 205834 210306 210738 211216) |
| forest B | `experiment_d455_pathB_20261005-171524` (211637 212651 213658 214736 215853) |

Plus test flight `experiment_d455_pathA_validation_20261005-153028` (193151).

**3. `ov_prep.py` bug: position σ read from the wrong columns.**
`ov_state_std.txt` holds three orientation σ (error state), so its layout is
t, σθ(3), σp(3), σv(3)…; `ov_prep` read position from `sd[:, 5:8]` =
(p_y, p_z, v_x). Found because `/openvins/joint_covariance` diagonals
disagreed ~700x; after the fix they agree to ≤ 2% (std-file print
precision). **Every position NEES before this section is wrong**: s53's
position column and its "vertical σ ≈ 2.4 mm" breakdown, and s54's position
NEES (real, calib, swap). Orientation and roll/pitch used the right columns
and stand.

**4. OpenVINS on the new bags** (`run_fullcov.sh`: serial replay in ROS
domain 77 + `joint_cov_dump.py` + both NEES). GT-window ATE (s51 metric,
`docker/out/diag_s54/gtwin_s55.jsonl`):

| set | ATE per run (m) | Sim(3) scale | old bags (s51/R1) |
|---|---|---|---|
| validation A | 0.019 0.021 0.024 0.020 0.022 | 0.993–0.996 | 0.021–0.037 |
| validation B | 0.039 0.052 0.045 0.047 0.042 | 0.999–1.001 | 0.046–0.077 |
| forest A | 0.158 0.065 0.059 0.059 0.047 | 0.973–0.994 | 0.11–4.06 |
| forest B | 0.192 0.117 0.207 0.142 0.120 | 0.988–0.995 | 0.25–152.6 |

NEES, timeshift 0, mean (diag over the whole window / full 3x3 blocks):

| set | pos | ori diag → full | rp diag → full |
|---|---|---|---|
| validation A | 0.12–0.21 | 19–28 → 32–43 | 22–49 → 39–86 |
| validation B | 0.47–0.81 | 11–14 → 27–31 | 13–26 → 34–70 |
| forest A | 0.74–6.2 | 16–18 → 24–31 | 19–42 → 29–76 |
| forest B | 3.7–12.5 | 12–21 → 25–38 | 15–37 → 36–79 |

Full-covariance NEES uses `/openvins/joint_covariance`, which is published
at ~4.5 Hz (181–750 samples per run); diag NEES on that same subset equals
the whole-window value to within ~3%, so the subset is representative.
**Diagonal vs full:** position unchanged (≤ 10%); **orientation understated
1.4–2.4x, roll/pitch 1.5–2.7x by the diagonal approximation**
(`nees_compare_{validation,forest}.txt`). Position is conservative on
validation (NEES < 1) and mildly overconfident in the forest.

**Remaining orientation overconfidence: IMU stamp lag.** Timeshift scan on
193151 (`OV_TIMESHIFT`, new asserted hook; t_imu = t_cam + s), diag ori / rp:
−2 ms 66/106, 0 21/38, +1 ms 9.5/19.5, **+1.5 ms 6.9/15.9, +2 ms 6.9/16.8**,
+2.5 ms 8.6/20.6, +3 ms 12.8/29.1, +4 ms 29/61. Consistent with the −2 ms
gyro–GT lag (s54) and online dt +2.1 ms. Not applied to the config (needs a
decision); the residual ~2x in σ at the optimum is unattributed
(instantaneous 4/8 ms IMU sampling is the next candidate).

**5. ORB-SLAM3 stereo-inertial is still broken; the bias bug was not its
cause.** Live (during recording) and replayed (`phase3_orbslam3.sh si`,
rate 0.5), GT-window ATE:

| set | live (m) | replay (m) | replay Sim(3) scale |
|---|---|---|---|
| validation A | 20.4 18.7 1.8 35.6 2.1 | 56.8 76.0 11.1 14.2 2.0 | 0.012–0.48 |
| validation B | 18.3 44.0 2.1 3.7 21.0 | 4.9 21.8 33.4 7.9 6.3 | 0.014–0.24 |
| forest A | 17.5 1.6 159.5 1.0 0.9 | 42.9 1.1 0.5 20.1 1.5 | 0.020–0.90 |
| forest B | 3.5 1.6 8.4 57.5 69.5 | 14.8 16.4 7.3 26.9 0.8 | 0.017–0.91 |

Its IMU noise matches OpenVINS (verified by `--verify`), so the next suspect
stays s52's: `orbslam3_d455_stereo_inertial.yaml` (Tbc, axes, units).

**Operational: the first ORB-SLAM3 replay pass was contaminated.**
`run_experiment.sh` tears down the stack *before* each run, so after the last
run the forest Gazebo/PX4 stack kept publishing `/camera/*` and `/clock` on
the default ROS domain (2.7 h). The replay node received both streams (4700
frames counted for a 2229-frame bag; stamps ~520 s). Those outputs were
overwritten after `bringup_sim.sh --stop`. Run `bringup_sim.sh --stop` after
any recording before replaying on the default domain, or replay in another
`ROS_DOMAIN_ID` (OpenVINS serial runs here used domain 77).

## 56. Replay guard, IMU stamp lag fixed at source, ORB-SLAM3 config pass

**Replay guard.** `replay_estimator.sh` first runs `check_no_publishers.py`:
if anything already publishes `/camera/imu`, `/camera/infra{1,2}/image_rect_raw`
or `/clock` in the replay's ROS domain (3 s discovery wait), it lists the
publishers and exits 4. Tested: refuses with a stand-in `/clock` publisher,
passes on a clean graph. Covers the s55 leftover-stack failure.

**IMU stamp lag, characterised** (`s54/imu_lag.py`: gyro − GT body rate vs GT
angular acceleration, per sample class; GT snapped + mis-snap repaired):
gyro lag **1.89–1.96 ms, constant** across preceding interval (4 / 8 ms) and
all four phases of the 4,4,4,8 cycle, best shift −2.0 ms in every class, on
193151 and 200418. Half a physics step: Gazebo stamps the sample with the end
of the step, the gyro describes its middle. The accelerometer lags ~4.25–4.5
ms (robust scan, shallow minimum).

**Fix, offline first** (`fix_imu_timing.py`, test flight 193151, timeshift 0):
stamps −2.0 ms → ori/rp NEES 21/38 → **6.4/16.4**, online cam–IMU dt settles at
**+0.22 ms** (was +2.1). Re-timing the accelerometer by another 2.4 ms on
top: 6.8/17.3, no gain, so only the stamp is corrected.

**Fix at source.** Generated bridges now publish Gazebo's IMU on
`/camera/imu_gz`; `imu_restamp.py` (started by `bringup_sim.sh`, stopped by
`--stop`) republishes `/camera/imu` with stamp − `IMU_STAMP_LAG_S`
(= PHYSICS_STEP_S/2 in `gen_d455_sim.py`; `test_gen_d455_sim.py` checks the
node's constant against it and that every bridge routes the raw IMU to
`imu_gz`). The recorder keeps both topics. `timeshift_cam_imu` stays 0.
Verified on a new flight (`experiment_d455_pathA_validation_20261006-031253`,
run 071406): gyro–GT lag −0.01 ms (best shift 0.0 in all classes), raw and
corrected values identical on all 14531 shared samples; OpenVINS ori/rp NEES
3.7/8.8, online dt +0.15 ms.

**Existing bags.** All 20 §55 bags rewritten in place by `s54/fix_all.sh`:
original moved to `flight_gzstamp.bag`, corrected written as `flight.bag`
(uncompressed MCAP), marker `IMU_STAMP_FIXED.txt`. OpenVINS re-run
(`<run>_ovser_imufix_fc`), timeshift 0:

| set | GT-window ATE (m) | Sim(3) | ori NEES diag / full | rp diag / full | pos NEES |
|---|---|---|---|---|---|
| validation A | 0.008 0.010 0.012 0.013 0.009 | 1.000–1.002 | 2.5–4.5 / 3.7–7.0 | 2.9–7.2 / 4.3–14.0 | 0.02–0.07 |
| validation B | 0.020 0.029 0.025 0.020 0.025 | 1.002–1.003 | 3.5–4.7 / 6.6–7.8 | 4.3–10.1 / 8.1–18.6 | 0.13–0.27 |
| forest A | 0.064 0.054 0.045 0.050 0.032 | 0.988–1.004 | 7.1–10.8 / 10.9–12.7 | 11.6–19.7 / 15.1–29.9 | 0.36–1.19 |
| forest B | 0.106 0.069 0.136 0.088 0.068 | 0.994–1.000 | 6.2–15.7 / 9.9–26.9 | 11.6–33.8 / 23.0–60.8 | 1.34–5.06 |

Before the stamp fix (§55): validation 0.019–0.052 m, forest 0.047–0.21 m.
Online-dt check (one bag per set, `calib_cam_timeoffset` on): settles at
+0.15, +0.11, +0.26, +0.13 ms. Orientation is still above 3 (validation ~2x
in σ at most, forest more); not attributed.

**ORB-SLAM3 stereo-inertial, time-boxed config pass: nothing wrong found.**
`IMU.T_b_c1` is IMU←camera with R = R_opt_from_bodyᵀ and t = camera origin in
the IMU frame (0.00552, 0.0424, 0.01174), the exact inverse of OpenVINS's
T_cam_imu; noise/walks are continuous densities (ORB-SLAM3 scales by √freq
itself) and match OpenVINS; IMU.Frequency 200; axes FLU with +g on z at rest;
the startup log shows every value parsed as written (File.version 1.0 path,
reads `IMU.T_b_c1`). The ROS 2 wrapper builds `IMU::Point(acc, gyr, t)` in
the right order from header stamps. Stopped here as instructed; not re-run on
the stamp-fixed bags. VINS-Fusion is the proposed replacement baseline.

## 57. ORB-SLAM3 on the stamp-fixed bags, forest NEES vs perception, baseline options

**ORB-SLAM3 stereo-inertial on the 20 §56 bags** (replay, rate 0.5; replay
guard passed on all; frames 2200/2206 etc.): GT-window ATE validation A
1.1–25.9 m, B 5.4–28.3 m, forest A 0.52–6.7 m, forest B 0.41–21.3 m; Sim(3)
scale 0.02–0.98 (`docker/out/diag_s56/gtwin_orb_imufix.jsonl`). Still
metre-scale after the bias and stamp fixes and a clean config pass (§56):
replace the stereo-inertial baseline.

**Forest overconfidence vs perception quality** (`s57/segment_corr.py`,
`run_seg.sh`; 5 s segments over the GT window; 210 forest / 205 validation
segments). Frame k = k-th `[TRACK]` line of the serial run (count equals the
runner's stereo pairs exactly) ↔ k-th cam0∩cam1 stamp; `[FI]`/`[MSCKF]`
lines attributed to the preceding frame. Per-sample NEES from
`nees_window.py --dump`. Spearman correlation of segment-mean NEES, pooled
and within-flight (ranks per flight):

| forest | depth | tri reject | tracked | MSCKF used | GT ang. speed | GT speed |
|---|---|---|---|---|---|---|
| ori diag (within) | −0.12 | +0.08 | +0.19 | +0.07 | −0.22 | −0.13 |
| ori full (within) | −0.19 | −0.10 | +0.26 | +0.22 | +0.03 | +0.06 |
| rp diag (within) | −0.09 | +0.13 | +0.19 | +0.03 | −0.23 | −0.13 |

Validation: depth −0.03, tri reject +0.04, tracked −0.07, angular speed −0.38,
speed −0.32 (ori diag). Regression of log ori NEES with flight fixed effects
(per 1 SD), forest: depth −0.06±0.08, tri reject +0.12±0.10, tracked
+0.28±0.08, ang. speed −0.31±0.07 (R² 0.26).

**Within the forest, NEES does not rise with feature depth** (slightly
negative) or clearly with triangulation rejection; the segment-level data do
not support linearisation error from distant features. NEES is higher in
slow / low-rotation segments (both scenes) and, in the forest only, when more
features are tracked. The between-scene gap is real (median segment ori NEES
forest 6.7 vs validation 3.2; median attempt depth 8.5 vs 4.2 m) but is not
explained by depth variation inside the forest. Nothing changed.

## 58. Basalt stereo-inertial baseline; frame-rate test of correlated measurement error

**Basalt replaces ORB-SLAM3 stereo-inertial as the second VIO.** Basalt 0.1.7
upstream binary (`basalt-0.1.7-x86_64-unknown-linux-gnu`, built on Ubuntu
22.04 = this host; sha256 8ab56b2a…6d24 verified), run ROS-free on the host
from `docker/out/tools/basalt/release`. Pipeline (`basalt_export.py` in the sim
container, then `basalt_vio`, then `s58/basalt_score.sh`):
- **Data**: EuRoC/ASL export of `flight.bag` (the §56 stamp-corrected bags):
  stereo pairs = identical cam0/cam1 header stamps (as the OpenVINS serial
  runner pairs them), PNG, `imu0/data.csv` at the corrected stamps.
- **Calibration** read from the generated `/opt/config_sim_only` files, not
  re-typed: `T_imu_cam = inv(T_cam_imu)` (cam0 p = (0.00552, 0.0424, 0.01174),
  identical to ORB-SLAM3's `T_b_c1`), `pinhole` fx fy cx cy, zero distortion,
  `cam_time_offset_ns` = timeshift 0.
- **IMU noise**: Basalt's `*_noise_std` / `*_bias_std` are continuous-time
  (basalt-headers `calibration.hpp`: σ_d = σ_c·√rate), the Kalibr convention,
  so copied 1:1: accel 2e-3, gyro 1.6e-4, accel RW 3e-4, gyro RW 2e-6, 200 Hz.
  Logged per run in `docker/out/basalt/<run>/calib_conversion.txt`. (PyYAML
  reads `2e-06` as a string; the exporter casts explicitly.)
- **Config**: stock `data/euroc_config.json`, no tuning. GUI off, 8 threads.
- **Scoring**: Basalt's TUM trajectory is the IMU pose; moved to base_link
  with ov_prep's lever arm; `gt_window_eval.py` (§51 window, identical for
  every estimator).

Frames consumed (trajectory poses) == stereo pairs in the bag on **all 20**
runs (e.g. 2229/2229, 6046/6046). GT-window ATE (`docker/out/basalt/all.jsonl`):

| set | Basalt ATE (m) | Basalt Sim(3) | OpenVINS ATE (§56) |
|---|---|---|---|
| validation A | 0.035 0.034 0.035 0.032 0.031 | 1.011–1.013 | 0.008–0.013 |
| validation B | 0.068 0.063 0.064 0.068 0.068 | 1.012–1.014 | 0.020–0.029 |
| forest A | 0.042 0.047 0.055 0.056 0.054 | 1.004–1.016 | 0.032–0.064 |
| forest B | 0.151 0.158 0.160 0.175 0.188 | 1.014–1.022 | 0.068–0.136 |

Credible stereo-inertial baseline (cm-level everywhere, no failures, very low
run-to-run spread). Basalt's Sim(3) scale is > 1 on every bag (+0.4 to +2.2%):
a systematic, not investigated. Exported datasets kept (~50 GB in
`docker/out/basalt/`).

**Hypothesis: temporally correlated measurement error counted as independent.**
Diagnostic replays of all 20 bags with `OV_SET track_frequency=15` and `=10`
(not kept; baseline = §56 run, all 30 Hz frames). Segments classified per state
by GT angular speed (1 s median) against the scene median (0.053 / 0.051 rad/s).
Mean diag NEES across runs:

| scene | rate | frames/run | ori slow | ori fast | rp slow | rp fast | ATE mean (m) |
|---|---|---|---|---|---|---|---|
| validation | 30 Hz | all | 5.9 | 1.5 | 10.1 | 2.3 | 0.017 |
| validation | 15 Hz | ~1550 | 2.2 | 0.8 | 3.8 | 1.2 | 0.014 |
| validation | 10 Hz | ~1030 | 1.7 | 0.6 | 3.5 | 1.0 | 0.016 |
| forest | 30 Hz | all | 13.5 | 5.9 | 26.1 | 8.2 | 0.071 |
| forest | 15 Hz | ~1530 | 4.0 | 1.5 | 7.4 | 2.3 | 0.030 |
| forest | 10 Hz | ~1020 | 2.4 | 1.1 | 4.4 | 1.8 | 0.031 |

Reading. Lowering the frame rate removes most of the overconfidence: at 15 Hz
orientation NEES is near or below 3 everywhere except forest-slow (4.0), and
forest ATE improves 0.071 → 0.030 m. That fits correlated per-frame errors
being counted as independent information. **But the drop is not
concentrated in slow segments in relative terms**: slow/fast NEES ratio
validation 3.9 → 2.8 → 2.8, forest 2.3 → 2.7 → 2.2. Slow segments drop more
in absolute NEES only because they start higher. So the frame-rate effect
supports the correlated-error explanation for the overall level, but does not
by itself explain why slow motion is worse; the hypothesis as stated
(NEES drops MOST in slow segments) is only partly supported. Fast segments
become underconfident (NEES < 3) at 15–10 Hz.
