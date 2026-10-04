# syntax=docker/dockerfile:1.7
# =============================================================================
# ROS 2 simulation container: Gazebo Harmonic + PX4 SITL + ORB-SLAM3 (COVINS
# agent) + orb_slam3_ros2_wrapper + ros_gz_bridge + multi_slam.
#
# Originally built natively for linux/arm64 (Apple Silicon); now built for
# linux/amd64 with NVIDIA GPU rendering (PATCHES.md §40). Every package is
# published for both architectures, so nothing here is emulated. See docker/PATCHES.md for the deviations from the upstream repo.
# =============================================================================
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- Pinned versions ---------------------------------------------------------
# PX4_COMMIT is derived, not guessed: the vendored px4_msgs tree is an exact
# content match (all 238 .msg blob hashes) for upstream px4_msgs
# 7c596a0b6d60b3107c9ef656a451296b56ccba34, whose commit message is
# "Update to PX4 6bc24c8cd1485edbf1b3b11565db4108f7c2fcaa". That PX4 commit is
# on main in the v1.17 dev cycle and is NOT on any release branch, so a release
# tag would give mismatched message definition hashes over uXRCE-DDS.
ARG PX4_COMMIT=6bc24c8cd1485edbf1b3b11565db4108f7c2fcaa
ARG PANGOLIN_TAG=v0.9.1
ARG XRCE_AGENT_TAG=v2.4.3
# Cap build parallelism: ORB-SLAM3 / g2o translation units are Eigen-template
# heavy and will OOM the Docker VM at -j18.
ARG NJOBS=6

ENV PX4_DIR=/opt/PX4-Autopilot
ENV WS=/root/ws_offboard_control
# ORB-SLAM3's CMakeLists looks for COVINS under $HOME/.local; keep that layout
# so the upstream find_path()/find_library() calls work unmodified.
ENV COVINS_PREFIX=/root/.local

# -----------------------------------------------------------------------------
# 1. Base toolchain, Gazebo Harmonic, ROS 2 deps, headless display stack
# -----------------------------------------------------------------------------
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git wget curl unzip ca-certificates \
        gnupg lsb-release sudo pkg-config \
        python3-pip python3-dev python3-setuptools \
        libeigen3-dev libopencv-dev libboost-all-dev libssl-dev \
        libglew-dev libyaml-cpp-dev libopenmpi-dev \
        libgl1-mesa-dri libegl1-mesa-dev libgl1-mesa-dev mesa-utils \
        xvfb x11vnc novnc websockify fluxbox xterm x11-utils \
        net-tools iproute2 iputils-ping \
    && \
    # Gazebo Harmonic: PX4's Tools/setup/ubuntu.sh at the pinned commit installs
    # gz-harmonic from this repo, and it is the ros_gz pairing for Jazzy.
    wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-image \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-sophus \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-tf2-sensor-msgs \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-nav2-common \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-ament-cmake-auto \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# evo for ATE/RPE. Ubuntu 24.04 is PEP668-managed, hence --break-system-packages
# (the same mechanism PX4's own ubuntu.sh uses at this commit).
RUN python3 -m pip install --no-cache-dir --break-system-packages \
        "evo==1.31.1" "numpy<2" && \
    evo_ape --help > /dev/null && echo "evo OK"

# -----------------------------------------------------------------------------
# 2. Pangolin (ORB-SLAM3 link dependency)
# -----------------------------------------------------------------------------
# NOTE/DEVIATION: the repo pins Pangolin b8abe866 only for the ROS 1 COVINS
# workspace; the ROS 2 side used whatever was on the author's host. b8abe866
# (2019) does not compile with GCC 13, so this pins a modern tag instead.
# ORB-SLAM3's Pangolin use here is link-only: the viewer is disabled by
# `visualization: false` (ROS params) and `orb.activate_visualization: 0`
# (config_comm.yaml).
RUN git clone --depth 1 --branch ${PANGOLIN_TAG} \
        https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin && \
    cmake -S /tmp/Pangolin -B /tmp/Pangolin/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF && \
    cmake --build /tmp/Pangolin/build --parallel ${NJOBS} && \
    cmake --install /tmp/Pangolin/build && \
    ldconfig && rm -rf /tmp/Pangolin

# -----------------------------------------------------------------------------
# 3. Micro XRCE-DDS Agent (PX4 <-> ROS 2 uORB transport, needed for offboard)
# -----------------------------------------------------------------------------
RUN git clone --depth 1 --branch ${XRCE_AGENT_TAG} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce && \
    cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
        -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_USE_SYSTEM_FASTDDS=OFF && \
    cmake --build /tmp/xrce/build --parallel ${NJOBS} && \
    cmake --install /tmp/xrce/build && ldconfig && rm -rf /tmp/xrce && \
    MicroXRCEAgent --help > /dev/null 2>&1 || true

# -----------------------------------------------------------------------------
# 4. PX4-Autopilot SITL at the derived commit
# -----------------------------------------------------------------------------
RUN git clone https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR} && \
    cd ${PX4_DIR} && \
    git checkout --detach ${PX4_COMMIT} && \
    git submodule update --init --recursive --depth 1 -j ${NJOBS} && \
    echo "PX4 at: $(git rev-parse HEAD)" && \
    echo "PX4-gazebo-models at: $(git -C Tools/simulation/gz rev-parse HEAD)"

# PX4's own dependency script. --no-nuttx skips the embedded toolchain; the
# simulation tools (incl. gz-harmonic, already installed above) stay enabled.
RUN cd ${PX4_DIR} && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${PX4_DIR} && \
    make px4_sitl_default -j ${NJOBS} && \
    test -x ${PX4_DIR}/build/px4_sitl_default/bin/px4 && \
    echo "PX4 SITL binary built"

# -----------------------------------------------------------------------------
# 4b. Align the RGB camera with the depth camera (REQUIRED, not an optimisation)
# -----------------------------------------------------------------------------
# PX4-gazebo-models' OakD-Lite model gives the two sensors different geometry:
#
#   IMX214       (type camera)        1920x1080, horizontal_fov 1.204
#   StereoOV7251 (type depth_camera)   640x480,  horizontal_fov 1.274
#
# But the repo's ORB-SLAM3 settings (orb_slam3_ros2_wrapper/params/
# orb_slam3_params/gazebo_rgbd.yaml) declare 640x480 with fx=432.50, cx=320,
# cy=240 -- i.e. the DEPTH camera's intrinsics (640/2/tan(1.274/2) = 433.9),
# applied to the RGB stream.
#
# As committed, that combination cannot work: ORB-SLAM3's RGB-D path indexes the
# depth image at the grayscale image's pixel coordinates, so a 1920x1080 RGB
# against a 640x480 depth map reads out of bounds, and the declared intrinsics
# are wrong for the RGB frame regardless.
#
# Setting IMX214 to 640x480 with horizontal_fov 1.274 makes the RGB stream match
# both the depth camera and the repo's own committed calibration. It also
# happens to be what makes this tractable without a GPU: measured on this host,
# Gazebo's real-time factor went from ~0.028 to ~0.6-1.0 and the RGB rate from
# 1.5 Hz to 8.4 Hz, purely from dropping 1920x1080 -> 640x480 under llvmpipe.
#
# Done here (rather than in the repo) because the model belongs to the pinned
# PX4-gazebo-models submodule. Recorded in docker/PATCHES.md.
#
# The 1920x1080 / 1.204 values are unique to the IMX214 (RGB) block --
# StereoOV7251 is already 640x480 / 1.274 -- so plain substitutions are
# unambiguous. Each one is asserted afterwards so a future PX4-gazebo-models
# layout change fails the build loudly instead of silently reverting to
# 1920x1080 (which would look fine until ORB-SLAM3 read the depth map out of
# bounds).
RUN set -eux; \
    M="${PX4_DIR}/Tools/simulation/gz/models/OakD-Lite/model.sdf"; \
    cp "$M" "$M.orig"; \
    sed -i \
        -e 's|<horizontal_fov>1.204</horizontal_fov>|<horizontal_fov>1.274</horizontal_fov>|' \
        -e 's|<width>1920</width>|<width>640</width>|' \
        -e 's|<height>1080</height>|<height>480</height>|' \
        "$M"; \
    grep -q '1.204' "$M" && { echo "IMX214 hfov not patched"; exit 1; } || true; \
    grep -q '<width>1920</width>' "$M" && { echo "IMX214 width not patched"; exit 1; } || true; \
    test "$(grep -c '<width>640</width>' "$M")" = "2"; \
    test "$(grep -c '<height>480</height>' "$M")" = "2"; \
    test "$(grep -c '<horizontal_fov>1.274</horizontal_fov>' "$M")" = "2"; \
    echo "OakD-Lite: RGB and depth cameras now share 640x480 / hfov 1.274"

# -----------------------------------------------------------------------------
# 5. Repository sources
# -----------------------------------------------------------------------------
# Copy the ROS packages EXPLICITLY rather than `COPY . ${WS}/src/`.
#
# A blanket copy means any edit anywhere in the repo -- including to
# docker/scripts or docker/sim/models -- invalidates this layer and forces a full
# rebuild of DBoW2, g2o, covins_comm, ORB-SLAM3 and the colcon workspace
# (~15 min). Listing the packages keeps iteration on the Docker tooling cheap.
COPY ORB_SLAM3                ${WS}/src/ORB_SLAM3
COPY covins                   ${WS}/src/covins
COPY multi_slam               ${WS}/src/multi_slam
COPY orb_slam3_ros2_wrapper   ${WS}/src/orb_slam3_ros2_wrapper
COPY orb_slam3_map_generator  ${WS}/src/orb_slam3_map_generator
COPY px4_msgs                 ${WS}/src/px4_msgs
COPY px4_ros_com              ${WS}/src/px4_ros_com
COPY slam_msgs                ${WS}/src/slam_msgs

# The repo commits prebuilt x86-64 .so files and CMake trees baked with the
# original author's absolute paths. They cannot be reused on arm64 (verified:
# `file` reports "ELF 64-bit LSB shared object, x86-64"), and a stale
# CMakeCache.txt would poison the configure step. Remove them so everything is
# rebuilt from source for this architecture.
RUN cd ${WS}/src && \
    rm -rf ORB_SLAM3/build ORB_SLAM3/lib \
           ORB_SLAM3/Thirdparty/DBoW2/build ORB_SLAM3/Thirdparty/DBoW2/lib \
           ORB_SLAM3/Thirdparty/g2o/build   ORB_SLAM3/Thirdparty/g2o/lib \
           covins/covins_comm/build covins/covins_comm/lib \
           multi_slam/launch/__pycache__ && \
    echo "purged committed x86-64 build artifacts"

ENV ORB_SLAM3_ROOT_DIR=${WS}/src/ORB_SLAM3

# ORB vocabulary (shipped as a tarball, ~145 MB extracted)
RUN cd ${ORB_SLAM3_ROOT_DIR}/Vocabulary && \
    tar -xf ORBvoc.txt.tar.gz && test -s ORBvoc.txt && \
    echo "ORBvoc.txt extracted: $(wc -l < ORBvoc.txt) lines"

# -----------------------------------------------------------------------------
# 6. ORB-SLAM3 Thirdparty (DBoW2, g2o)
# -----------------------------------------------------------------------------
# -march=native is left as upstream wrote it: verified to work on arm64 GCC 13
# in this image (`g++ -O3 -march=native` exits 0, __ARM_ARCH 8 / __ARM_NEON set).
#
# amd64 (PATCHES.md §41): on an AVX CPU -march=native raises Eigen's
# EIGEN_MAX_ALIGN_BYTES from 16 to 32. ORB_SLAM3/CMakeLists.txt and covins_comm
# force 16, but g2o and DBoW2 do not, so the libraries disagree on the layout
# of every fixed-size Eigen member and ORB-SLAM3 segfaults in the first BA right
# after "New Map created". Pass the SAME defines ORB-SLAM3 uses. g2o also gets
# ORB-SLAM3's -std=c++14 (it sets none, so GCC 13 defaults to C++17); see §41.
ARG EIGEN_ALIGN_DEFS="-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16"
RUN cmake -S ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2 \
          -B ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/build \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS="${EIGEN_ALIGN_DEFS}" && \
    cmake --build ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/build --parallel ${NJOBS}

RUN cmake -S ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o \
          -B ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/build \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS="${EIGEN_ALIGN_DEFS} -std=c++14" && \
    cmake --build ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/build --parallel ${NJOBS}

# -----------------------------------------------------------------------------
# 7. covins_comm as a standalone shared library (the COVINS agent transport)
# -----------------------------------------------------------------------------
# This is what makes the frontend talk to the backend: ORB-SLAM3 links
# libcovins_comm.so and System.cc opens a TCP socket to sys.server_ip:sys.port.
# Installed to $COVINS_PREFIX so ORB-SLAM3's find_path/find_library
# ($ENV{HOME}/.local/{include,lib}) resolve without modification.
#
# Sophus is taken from ORB-SLAM3's vendored copy on purpose: the local
# utils_base changes expose Sophus::SE3f across the covins_comm <-> ORB_SLAM3
# boundary, so both must agree on the exact Sophus version.
RUN cmake -S ${WS}/src/covins/covins_comm \
          -B ${WS}/src/covins/covins_comm/build \
          -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DBUILD_LIBRARY=ON -DUSE_CATKIN=OFF \
          -DCMAKE_INSTALL_PREFIX=${COVINS_PREFIX} \
          -DSOPHUS_INCLUDE_DIR=${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus && \
    cmake --build ${WS}/src/covins/covins_comm/build --parallel ${NJOBS} && \
    cmake --install ${WS}/src/covins/covins_comm/build && \
    test -f ${COVINS_PREFIX}/lib/libcovins_comm.so && \
    test -f ${COVINS_PREFIX}/include/covins/covins_base/config_comm.hpp && \
    # config_comm.hpp derives its runtime YAML path from __FILE__ at compile
    # time: it strips from "include/covins" onwards and appends
    # "config/config_comm.yaml". For headers at $COVINS_PREFIX/include/covins/...
    # that resolves to $COVINS_PREFIX/config/config_comm.yaml. Assert it exists,
    # because a missing file here fails SILENTLY (the params are const
    # std::strings initialised at static-init time).
    test -f ${COVINS_PREFIX}/config/config_comm.yaml && \
    # cereal is a public dependency of the installed covins headers
    # (typedefs_base.hpp includes <cereal/cereal.hpp>), so it has to be in the
    # install tree too or the ORB-SLAM3 build below cannot compile a single
    # translation unit.
    test -f ${COVINS_PREFIX}/include/cereal/cereal.hpp && \
    echo "covins_comm installed; runtime config at ${COVINS_PREFIX}/config/config_comm.yaml"

ENV LD_LIBRARY_PATH=${COVINS_PREFIX}/lib:${ORB_SLAM3_ROOT_DIR}/lib:${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib:${LD_LIBRARY_PATH}

# -----------------------------------------------------------------------------
# 8. ORB-SLAM3 (COVINS-enabled: links libcovins_comm.so)
# -----------------------------------------------------------------------------
RUN cmake -S ${ORB_SLAM3_ROOT_DIR} -B ${ORB_SLAM3_ROOT_DIR}/build \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build ${ORB_SLAM3_ROOT_DIR}/build --parallel ${NJOBS} --target ORB_SLAM3 && \
    test -f ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so && \
    # Prove the COVINS transport is actually linked in, rather than silently
    # compiled out.
    ldd ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so | grep -q covins_comm && \
    echo "libORB_SLAM3.so links libcovins_comm.so"

# -----------------------------------------------------------------------------
# 9. colcon workspace
# -----------------------------------------------------------------------------
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WS} && \
    colcon build \
        --packages-select px4_msgs slam_msgs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --executor sequential --parallel-workers 1

# Built strictly sequentially, one compiler at a time. The wrapper's
# translation units pull in ORB-SLAM3 + Eigen + PCL + Sophus templates under
# C++20 and each cc1plus can exceed 2 GB; anything more parallel than this gets
# OOM-killed ("c++: fatal error: Killed signal terminated program cc1plus") on a
# 16 GB Docker VM. Raised to -j4 on the 64 GB amd64 host (PATCHES.md §40).
ENV MAKEFLAGS="-j4"
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WS}/install/setup.bash && \
    cd ${WS} && \
    colcon build \
        --packages-select px4_ros_com multi_slam orb_slam3_ros2_wrapper \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DORB_SLAM3_ROOT_DIR=${ORB_SLAM3_ROOT_DIR} \
                     -DBUILD_TESTING=OFF \
        --executor sequential --parallel-workers 1 && \
    test -x ${WS}/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/rgbd && \
    echo "orb_slam3_ros2_wrapper rgbd node built"

# -----------------------------------------------------------------------------
# 9b. OpenVINS + the active-SLAM packages (phase 3)
# -----------------------------------------------------------------------------
# Pinned by COMMIT, not by branch. "openvins-integration" is a moving target and
# a rebuild months from now must produce the same estimator, otherwise the ATE
# and NEES numbers in PATCHES.md stop meaning anything.
# 06242b2 adds __has_include shims for four ROS 2 headers whose .h spellings
# Jazzy removed, so one source tree builds on both Jazzy and Humble. The fix
# lives in the fork, NOT as a sed here, so the repo stays the single source of
# truth for what actually runs.
ARG OPEN_VINS_COMMIT=06242b2acaf027fac65d0b3d530fab50bbc6fa2c
ARG ACTIVE_SLAM_MSGS_COMMIT=a0790aafdb8dffa0eb33a0e7f471071d1edb9426
ARG ACTIVE_SLAM_PLANNER_COMMIT=f56e1146256f009bc71a3b4c644afc8c7f2dcd14
ARG ACTIVE_SLAM_INFORMATION_COMMIT=e2250686130f0d92adfb2bd126735638dddaf424

# Cloned over https: these repos are publicly readable, so the image needs no
# credentials and no SSH agent forwarding.
RUN set -eux; \
    cd ${WS}/src; \
    for spec in \
        "open_vins|${OPEN_VINS_COMMIT}" \
        "active_slam_msgs|${ACTIVE_SLAM_MSGS_COMMIT}" \
        "active_slam_planner|${ACTIVE_SLAM_PLANNER_COMMIT}" \
        "active_slam_information|${ACTIVE_SLAM_INFORMATION_COMMIT}" ; do \
        repo="${spec%%|*}"; sha="${spec##*|}"; \
        git clone --filter=blob:none --no-checkout \
            "https://github.com/EkanshSingh401/${repo}.git" "${repo}"; \
        git -C "${repo}" fetch --depth 1 origin "${sha}"; \
        git -C "${repo}" checkout --detach "${sha}"; \
        test "$(git -C "${repo}" rev-parse HEAD)" = "${sha}"; \
        echo "  ${repo} @ ${sha}"; \
    done

# active_slam_msgs MUST be built first and ON ITS OWN.
#
# ov_msckf/package.xml deliberately does NOT declare a dependency on it, so that
# a broken active_slam_msgs cannot take down the VIO build
# (ov_msckf/cmake/ROS2.cmake:19 uses find_package(active_slam_msgs QUIET)).
# The cost of that choice is that colcon has no ordering constraint between
# them: in a single build it may compile ov_msckf first, find nothing, and
# silently compile the joint-covariance publisher OUT. The only symptom is a
# runtime warning,
#   "joint covariance publisher NOT compiled in (active_slam_msgs was absent at
#    build time)"                        -- ov_msckf/src/ros/ROS2Visualizer.cpp:65
# and /openvins/joint_covariance simply never appears. Two separate colcon
# invocations make the ordering explicit rather than lucky.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WS} && \
    colcon build \
        --packages-select active_slam_msgs \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --executor sequential --parallel-workers 1 && \
    test -d ${WS}/install/active_slam_msgs/share/active_slam_msgs/cmake && \
    echo "active_slam_msgs built FIRST (ordering is explicit, not lucky)"

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WS}/install/setup.bash && \
    cd ${WS} && \
    colcon build \
        --packages-select ov_core ov_init ov_msckf ov_eval \
                          active_slam_information active_slam_planner \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
        --executor sequential --parallel-workers 1 \
        --event-handlers console_direct+ \
        > /tmp/ov_build.log 2>&1; \
    rc=$?; tail -40 /tmp/ov_build.log; \
    test $rc -eq 0 || { echo "FATAL: colcon build exited ${rc}"; exit $rc; }; \
    test -x ${WS}/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf \
        || { echo "FATAL: run_subscribe_msckf not built"; exit 1; }; \
    test -x ${WS}/install/ov_eval/lib/ov_eval/error_singlerun \
        || { echo "FATAL: ov_eval error_singlerun not built (needed for NEES)"; exit 1; }

# Assert the joint-covariance publisher was actually COMPILED IN.
#
# Checked on libov_msckf_lib.so, NOT on the run_subscribe_msckf executable, and
# with objdump rather than ldd. Both choices are corrections of earlier mistakes:
#
#   * ROS2Visualizer.cpp compiles into libov_msckf_lib.so, so active_slam_msgs
#     is a dependency of THAT library, not of the executable.
#   * `ldd` only enumerates dependencies it can RESOLVE. In an unsourced RUN
#     layer it reports "libov_msckf_lib.so => not found" and therefore never
#     looks inside it, so an ldd-on-the-executable test reports "not linked"
#     for a perfectly good build. That exact false negative failed a build whose
#     six packages had all compiled successfully.
#
# objdump -p reads DT_NEEDED straight out of the ELF header and needs no
# loader path, no sourced environment and no resolvable libraries.
#
# The `strings` check is independent of the link check and catches the case that
# actually matters: OV_JOINT_COV_AVAILABLE undefined, so the publisher is
# #ifdef'd out even though the library happens to link. The topic name is
# present as a literal only if that code was compiled.
#
# NOTE the `> /dev/null` instead of `grep -q`. This image's SHELL sets
# `-o pipefail`, and `grep -q` exits the instant it matches, closing the pipe;
# the upstream `strings` then dies of SIGPIPE, and pipefail propagates THAT
# failure even though the match succeeded. The check reported "topic literal
# absent" for a library that demonstrably contains it (verified afterwards with
# objdump -s -j .rodata). Without -q, grep reads to EOF and no SIGPIPE occurs.
# The objdump pipeline above is written the same way -- it only passed by luck,
# its output being small enough to be written before grep exited.
RUN set -eux; \
    LIB=$(find ${WS}/install/ov_msckf -name 'libov_msckf_lib.so*' -print -quit); \
    test -n "${LIB}" || { echo "FATAL: libov_msckf_lib.so not found"; exit 1; }; \
    echo "checking ${LIB}"; \
    objdump -p "${LIB}" | grep NEEDED | grep active_slam_msgs > /dev/null \
        || { echo "FATAL: libov_msckf_lib.so has no DT_NEEDED on active_slam_msgs."; \
             echo "  find_package(active_slam_msgs QUIET) failed at configure time, so"; \
             echo "  the joint-covariance publisher was compiled out and"; \
             echo "  /openvins/joint_covariance would never appear."; \
             echo "  Build active_slam_msgs BEFORE ov_msckf (see above)."; \
             objdump -p "${LIB}" | grep NEEDED; exit 1; }; \
    strings "${LIB}" | grep '/openvins/joint_covariance' > /dev/null \
        || { echo "FATAL: the topic name /openvins/joint_covariance is absent from"; \
             echo "  libov_msckf_lib.so, so the publisher is #ifdef'd out even though"; \
             echo "  the library links active_slam_msgs. Check OV_JOINT_COV_AVAILABLE."; \
             exit 1; }; \
    echo "joint covariance publisher IS compiled in (DT_NEEDED + topic literal)"

# -----------------------------------------------------------------------------
# 10. Runtime environment
# -----------------------------------------------------------------------------
# Gazebo must find the PX4 models (x500_depth) and worlds (forest).
# Simulation-only models (the D455-mirror rig) and the configs generated from
# its spec. Kept in a SIM-ONLY directory so they can never be mistaken for, or
# overwrite, calibration files for real hardware.
COPY docker/sim/models           /opt/sim_models
COPY docker/sim/config_sim_only  /opt/config_sim_only

# Our models come FIRST so they take precedence, then PX4's (x500_d455 includes
# PX4's x500 base, so both paths are needed).
ENV GZ_SIM_RESOURCE_PATH=/opt/sim_models:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds

# PX4 does NOT look on GZ_SIM_RESOURCE_PATH to spawn a model. Its gz_bridge
# builds the path "${PX4_GZ_MODELS}/<model>/model.sdf" and hands that to Gazebo,
# so a model that only exists elsewhere fails with
#   [Err] Unable to find uri[file:///opt/PX4-Autopilot/Tools/simulation/gz/
#         models/x500_d455/model.sdf]
# even though `gz` itself could resolve it. Symlink our models into PX4's tree:
# the repo stays the single source of truth, and the link makes the provenance
# obvious to anyone poking around in the container.
RUN set -eux;     for m in d455 x500_d455 d455_nodepth x500_d455_nodepth; do         ln -sfn "/opt/sim_models/$m" "${PX4_DIR}/Tools/simulation/gz/models/$m";         test -f "${PX4_DIR}/Tools/simulation/gz/models/$m/model.sdf";     done;     echo "linked sim-only models into PX4_GZ_MODELS"

# -----------------------------------------------------------------------------
# Gravity: align the simulated worlds with ORB-SLAM3's hardcoded constant.
# -----------------------------------------------------------------------------
# PX4's worlds ship <gravity>0 0 -9.8</gravity>. ORB-SLAM3 hardcodes
# ORB_SLAM3/include/ImuTypes.h:46  const float GRAVITY_VALUE = 9.81;
# with no way to configure it, while OpenVINS reads gravity_mag from its YAML.
# Left alone, a VIO comparison on this rig would hand OpenVINS the correct
# gravity and ORB-SLAM3 a 0.01 m/s^2 error -- a difference in the estimators'
# inputs, not in their estimation, and enough to bias the gravity-direction
# state that ORB-SLAM3's VIBA solves for.
#
# The world is edited rather than renamed on purpose. The generated bridge
# config embeds the world name in nine gz topic paths
# (/world/forest/model/.../image, /world/forest/dynamic_pose/info, ...), so a
# "forest_g981" world would change the topic names at the same time as the
# gravity and stop this being a one-variable experiment.
#
# Asserted both ways: the build fails if the expected value is absent (PX4
# changed its worlds) or if any -9.8 survives the edit.
RUN set -eux; \
    cd "${PX4_DIR}/Tools/simulation/gz/worlds"; \
    for w in *.sdf; do \
        if grep -q '<gravity>0 0 -9.8</gravity>' "$w"; then \
            sed -i 's|<gravity>0 0 -9.8</gravity>|<gravity>0 0 -9.81</gravity>|g' "$w"; \
            echo "  patched gravity in $w"; \
        fi; \
    done; \
    grep -q '<gravity>0 0 -9.81</gravity>' forest.sdf \
        || { echo "FATAL: forest.sdf has no -9.81 gravity after patch"; exit 1; }; \
    ! grep -rq '<gravity>0 0 -9.8</gravity>' . \
        || { echo "FATAL: a world still carries -9.8 gravity"; exit 1; }; \
    echo "gravity aligned to ORB-SLAM3 GRAVITY_VALUE=9.81 in all PX4 worlds"
# PATCHES.md §40: the Mac-era LIBGL_ALWAYS_SOFTWARE=1 / GALLIUM_DRIVER=llvmpipe
# are removed so the Gazebo sensor pipeline (ogre2, --headless-rendering = EGL)
# renders on the NVIDIA GPU exposed by nvidia-container-toolkit.
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DISPLAY=:99
ENV QT_X11_NO_MITSHM=1
ENV ROS_DOMAIN_ID=55
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY docker/sim/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/scripts/ /opt/scripts/
RUN chmod +x /usr/local/bin/entrypoint.sh /opt/scripts/*.sh /opt/scripts/*.py 2>/dev/null || true

# PATCHES.md §42: the container runs as the host user (compose `user:`), not
# root. Make the root-owned build tree usable without changing ownership:
# /root becomes traversable (everything under it is already a+rX), a shared
# world-writable HOME is provided, and the few paths written at RUNTIME are made
# writable. Build outputs stay root-owned and read-only.
RUN chmod 755 /root && \
    install -d -m 1777 /home/sim && \
    chmod -R a+rwX ${COVINS_PREFIX}/config ${PX4_DIR}/build/px4_sitl_default/rootfs
ENV HOME=/home/sim

WORKDIR ${WS}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
