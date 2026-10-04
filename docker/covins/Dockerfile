# syntax=docker/dockerfile:1.7
# =============================================================================
# COVINS backend container (ROS Melodic), built from this repository's covins/
# tree INCLUDING its local modifications.
#
# Derived from covins/docker/Dockerfile. Two deliberate differences, both
# recorded in docker/PATCHES.md:
#
#  1. Base image: osrf/ros:melodic-desktop-bionic -> ros:melodic-ros-base-bionic.
#     The osrf image's manifest resolves to a single platform, linux/amd64, so
#     it can only run emulated on Apple Silicon. The official ros image is
#     multi-arch (amd64 + arm64/v8) and bionic/arm64 still serves the full
#     Melodic package set from the frozen snapshots.ros.org/melodic/final repo,
#     so this builds and runs natively.
#  2. Only covins_backend's dependency closure is built -- covins_frontend and
#     covins/orb_slam3 (the ROS 1 VINS/ORB-SLAM3 agents) are skipped, because
#     the agent in this setup is ORB-SLAM3 running in the ROS 2 container.
#
# Everything else -- the dependency set, the install_file.sh build order,
# fix_eigen_deps.sh, Eigen 3.3.4, OpenCV 3.4.2, Ceres 1.14.0 -- is unchanged.
# =============================================================================
FROM ros:melodic-ros-base-bionic

ENV CATKIN_WS=/root/covins_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm
ENV PYTHONIOENCODING=UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Cap parallelism: OpenCV 3.4.2, Ceres 1.14.0 and opengv have large translation
# units. At -j4 (and especially alongside a concurrent sim-image build) cc1plus
# gets OOM-killed on a 16 GB Docker VM. Build the two images one at a time.
ARG NR_JOBS=2

# -----------------------------------------------------------------------------
# 1. Packages
# -----------------------------------------------------------------------------
# NOTE: the upstream Dockerfile starts with two `apt-key adv --keyserver ...`
# calls to repair the 2019 ROS key rotation. They are omitted deliberately: this
# base image already points at snapshots.ros.org/melodic/final with a valid
# baked-in key (verified: apt-get update exits 0 with no EXPKEYSIG/NO_PUBKEY),
# and apt-key is deprecated and its keyserver fetch is unreliable today.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl unzip ca-certificates \
        vim sudo gdb net-tools iproute2 iputils-ping \
        autoconf doxygen pkg-config \
        python-dev python-numpy python3-dev python3-numpy \
        python3-catkin-tools python3-wstool python3-osrf-pycommon \
        ros-melodic-rviz \
        ros-melodic-pcl-ros \
        ros-melodic-tf2-ros \
        ros-melodic-tf2-geometry-msgs \
        ros-melodic-cmake-modules \
        ros-melodic-message-generation \
        ros-melodic-serial \
        libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
        libyaml-cpp-dev libvtk6-dev libv4l-dev libomp-dev libglew-dev \
        libtbb-dev libgtk2.0-dev libgl1-mesa-dev libglu1-mesa-dev \
        libboost-all-dev \
        # libprotobuf-dev is REQUIRED, and its absence is not obvious.
        # aslam_cv_common depends on protobuf_catkin, whose USE_SYSTEM_PROTOBUF
        # is "AUTO": it uses the system protobuf if find_package(Protobuf) reports
        # a version > 2.6.1, and otherwise compiles protobuf 2.6.1 from source --
        # whose autogen.sh fetches gmock from the long-dead googlecode.com.
        # Upstream never hit that path because osrf/ros:melodic-desktop-bionic +
        # ros-melodic-desktop-full pulled libprotobuf-dev in transitively. This
        # slimmer base does not, so it must be requested explicitly.
        # Verified: bionic/arm64 ships 3.0.0-9.1ubuntu1.1, and CMake 3.10.2 sets
        # Protobuf_VERSION=3.0.0, so AUTO correctly selects the system copy.
        libprotobuf-dev protobuf-compiler \
        mesa-utils libgl1-mesa-dri \
        xvfb x11vnc novnc websockify fluxbox xterm x11-utils \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2. COVINS sources (this repo's covins/, with its local modifications)
# -----------------------------------------------------------------------------
COPY covins ${CATKIN_WS}/src/covins
COPY docker/covins/dependencies.pinned.rosinstall ${CATKIN_WS}/src/covins/dependencies.pinned.rosinstall

# The repo commits a prebuilt x86-64 libcovins_comm.so and a CMake tree baked
# with the author's absolute paths; both must go before building on arm64.
RUN rm -rf ${CATKIN_WS}/src/covins/covins_comm/build \
           ${CATKIN_WS}/src/covins/covins_comm/lib && \
    echo "purged committed x86-64 covins_comm artifacts"

# Sophus, header-only, to the system include path.
#
# The repo's local covins_comm changes made Sophus a PUBLIC dependency of
# covins_comm's headers: utils_base.hpp now declares
# ToEigenMat44d(const Sophus::SE3f&) and does #include <sophus/se3.hpp>.
# So every consumer needs it too, not just covins_comm --
# `covins_backend` fails with "fatal error: sophus/se3.hpp: No such file or
# directory" otherwise. Installing it system-wide fixes covins_comm and
# covins_backend in one place without editing covins_backend's CMakeLists.
#
# Source is the copy the repo vendored at covins/orb_slam3/Thirdparty/Sophus,
# which is byte-identical to ORB_SLAM3/Thirdparty/Sophus used by the ROS 2
# container -- so the whole project uses one Sophus version.
RUN cp -r ${CATKIN_WS}/src/covins/orb_slam3/Thirdparty/Sophus/sophus \
          /usr/local/include/sophus && \
    test -f /usr/local/include/sophus/se3.hpp && \
    echo "Sophus headers installed to /usr/local/include/sophus"

WORKDIR ${CATKIN_WS}

# -----------------------------------------------------------------------------
# 3. Dependencies at pinned commits
# -----------------------------------------------------------------------------
# Upstream install_file.sh runs `wstool merge covins/dependencies.rosinstall`,
# which leaves 15 of 17 repos tracking their default branch. We merge the pinned
# manifest instead. This matters beyond general reproducibility:
# fix_eigen_deps.sh patches eigen_catkin, ceres_catkin and opengv by absolute
# LINE NUMBER, so a shifted upstream file would be silently corrupted rather
# than fail loudly.
RUN cd ${CATKIN_WS}/src && \
    wstool init && \
    wstool merge covins/dependencies.pinned.rosinstall && \
    wstool up -j ${NR_JOBS} && \
    echo "--- resolved dependency commits ---" && \
    for d in */; do \
        if [ -d "$d/.git" ]; then printf '%-32s %s\n' "${d%/}" "$(git -C "$d" rev-parse HEAD)"; fi; \
    done

# Pin Eigen to 3.3.4 and wire eigen_catkin into ceres_catkin / opengv.
RUN cd ${CATKIN_WS}/src && \
    chmod +x covins/fix_eigen_deps.sh && \
    ./covins/fix_eigen_deps.sh && \
    grep -q covins_patched eigen_catkin/CMakeLists.txt && \
    grep -q covins_patched ceres_catkin/CMakeLists.txt && \
    grep -q covins_patched opengv/CMakeLists.txt && \
    echo "fix_eigen_deps.sh applied to all three packages"

# -----------------------------------------------------------------------------
# 4. Build (mirrors install_file.sh, scoped to the backend)
# -----------------------------------------------------------------------------
RUN catkin init && \
    catkin config \
        --extend /opt/ros/melodic \
        --merge-devel \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Eigen 3.3.4 and OpenCV 3.4.2 first, exactly as install_file.sh does. This is
# the long step: OpenCV 3.4.2 + contrib are compiled from source.
RUN source /opt/ros/melodic/setup.bash && \
    cd ${CATKIN_WS} && \
    catkin build -j ${NR_JOBS} eigen_catkin opencv3_catkin && \
    ls ${CATKIN_WS}/devel/lib/libopencv_core.so* >/dev/null && \
    echo "opencv3_catkin built: $(basename "$(ls ${CATKIN_WS}/devel/lib/libopencv_core.so.* | head -1)")" && \
    echo "eigen_catkin Eigen: $(find ${CATKIN_WS}/devel/include -name Macros.h -path '*Eigen*' -exec grep -h 'define EIGEN_\(WORLD\|MAJOR\|MINOR\)_VERSION' {} + 2>/dev/null | tr -s ' ' | cut -d' ' -f3 | paste -sd. -)"

# DBoW2 bundled inside covins_backend (plain CMake, not a catkin package).
RUN source /opt/ros/melodic/setup.bash && \
    source ${CATKIN_WS}/devel/setup.bash && \
    cd ${CATKIN_WS}/src/covins/covins_backend/thirdparty/DBoW2 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j ${NR_JOBS}

# ORB vocabulary for the backend's place recognition.
RUN cd ${CATKIN_WS}/src/covins/covins_backend/config && \
    unzip -o ORBvoc.txt.zip && test -s ORBvoc.txt && \
    echo "ORBvoc.txt extracted: $(wc -l < ORBvoc.txt) lines"

# covins_comm + covins_backend. `catkin build covins_backend` builds only its
# dependency closure, which is how covins_frontend and covins/orb_slam3 are
# excluded without editing the workspace.
RUN source /opt/ros/melodic/setup.bash && \
    source ${CATKIN_WS}/devel/setup.bash && \
    cd ${CATKIN_WS} && \
    catkin build -j ${NR_JOBS} covins_backend && \
    test -x ${CATKIN_WS}/devel/lib/covins_backend/covins_backend_node && \
    # Assert protobuf_catkin took the system-protobuf path. If it fell back to
    # the source build it would have created a protobuf_src/ ExternalProject
    # directory; that path tries to download gmock from googlecode.com and fails.
    ! test -d ${CATKIN_WS}/build/protobuf_catkin/protobuf_src && \
    echo "protobuf_catkin used the system protobuf (no source build)" && \
    echo "covins_backend_node built"

# -----------------------------------------------------------------------------
# 5. Runtime
# -----------------------------------------------------------------------------
# Source the workspace from the ROS entrypoint, as upstream's Dockerfile does.
RUN sed -i '/exec "$@"/i source "${CATKIN_WS}/devel/setup.bash"' /ros_entrypoint.sh

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV DISPLAY=:98
# covins_comm's TCP listener. The ROS 2 frontend connects here.
EXPOSE 9033
EXPOSE 11311

COPY docker/covins/entrypoint.sh /usr/local/bin/covins-entrypoint.sh
COPY docker/scripts/ /opt/scripts/
RUN chmod +x /usr/local/bin/covins-entrypoint.sh && \
    chmod +x /opt/scripts/*.sh /opt/scripts/*.py 2>/dev/null || true

# PATCHES.md §43: the container runs as the host user (compose `user:`), not
# root. Make the root-owned build tree usable without changing ownership:
# /root becomes traversable (everything under it is already a+rX), a shared
# world-writable HOME is provided, and the few paths written at RUNTIME are made
# writable. Build outputs stay root-owned and read-only.
RUN chmod 755 /root && \
    install -d -m 1777 /home/sim && \
    chmod -R a+rwX ${CATKIN_WS}/src/covins/covins_backend/output
ENV HOME=/home/sim

WORKDIR ${CATKIN_WS}
ENTRYPOINT ["/usr/local/bin/covins-entrypoint.sh"]
CMD ["bash"]
