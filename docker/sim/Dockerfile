# syntax=docker/dockerfile:1.7
# =============================================================================
# ROS 2 simulation container: Gazebo Harmonic + PX4 SITL + ORB-SLAM3 (COVINS
# agent) + orb_slam3_ros2_wrapper + ros_gz_bridge + multi_slam.
#
# Built NATIVELY for linux/arm64 (Apple Silicon). Every ROS 2 Jazzy and Gazebo
# Harmonic package this needs is published for noble/arm64, so nothing here is
# emulated. See docker/PATCHES.md for the deviations from the upstream repo.
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
# 5. Repository sources
# -----------------------------------------------------------------------------
COPY . ${WS}/src/

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
RUN cmake -S ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2 \
          -B ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/build \
          -DCMAKE_BUILD_TYPE=Release && \
    cmake --build ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/build --parallel ${NJOBS}

RUN cmake -S ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o \
          -B ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/build \
          -DCMAKE_BUILD_TYPE=Release && \
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

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WS}/install/setup.bash && \
    cd ${WS} && \
    colcon build \
        --packages-select px4_ros_com multi_slam orb_slam3_ros2_wrapper \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DORB_SLAM3_ROOT_DIR=${ORB_SLAM3_ROOT_DIR} \
                     -DBUILD_TESTING=OFF \
        --parallel-workers 2 && \
    test -x ${WS}/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/rgbd && \
    echo "orb_slam3_ros2_wrapper rgbd node built"

# -----------------------------------------------------------------------------
# 10. Runtime environment
# -----------------------------------------------------------------------------
# Gazebo must find the PX4 models (x500_depth) and worlds (forest).
ENV GZ_SIM_RESOURCE_PATH=${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds
# No GPU passthrough on macOS Docker: force Mesa software rasterisation for both
# the Gazebo sensor pipeline (ogre2) and RViz.
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV DISPLAY=:99
ENV QT_X11_NO_MITSHM=1
ENV ROS_DOMAIN_ID=55
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY docker/sim/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/scripts/ /opt/scripts/
RUN chmod +x /usr/local/bin/entrypoint.sh /opt/scripts/*.sh /opt/scripts/*.py 2>/dev/null || true

WORKDIR ${WS}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
