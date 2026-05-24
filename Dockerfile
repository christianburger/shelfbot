# ═══════════════════════════════════════════════════════════════════════════════
# Shelfbot — Docker image
# Base: ROS 2 Humble (Ubuntu 22.04)
#
# Build stages
#   1. base        — ROS 2 Humble + all apt packages
#   2. pangolin    — Pangolin (required by ORB-SLAM3, not in apt for jammy)
#   3. orbslam3    — ORB-SLAM3 shared library
#   4. final       — Everything combined; workspace mounted at /workspace
# ═══════════════════════════════════════════════════════════════════════════════

# ── Stage 1: base ─────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop AS base

ARG DEBIAN_FRONTEND=noninteractive

# Locale (required for ROS 2)
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# ── System utilities & build tools ────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    ninja-build \
    # Python
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    # Editors / shell helpers
    nano \
    vim \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Humble packages ─────────────────────────────────────────────────────
# All package names verified against Ubuntu 22.04 (jammy) ROS 2 Humble APT repo.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Core / rclcpp ──────────────────────────────────────────────────────
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    # ── Common message packages ────────────────────────────────────────────
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    # ── TF2 ───────────────────────────────────────────────────────────────
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    # ── ros2_control stack ────────────────────────────────────────────────
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-realtime-tools \
    ros-humble-pluginlib \
    # ── Robot description ─────────────────────────────────────────────────
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    # ── Visualisation ─────────────────────────────────────────────────────
    ros-humble-rviz2 \
    # ── Vision / Perception ───────────────────────────────────────────────
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-message-filters \
    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
    # ── Navigation 2 ─────────────────────────────────────────────────────
    ros-humble-nav2-bringup \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-common \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-smoother \
    # ── SLAM / Localisation ───────────────────────────────────────────────
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# ── System libraries needed for vision and ORB-SLAM3 ─────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # OpenCV (system)
    libopencv-dev \
    python3-opencv \
    # AprilTag C library (used directly, not via ROS wrapper)
    libapriltag-dev \
    # Eigen3
    libeigen3-dev \
    # OpenGL / GLEW (needed by Pangolin → ORB-SLAM3 viewer)
    libgl1-mesa-dev \
    libglew-dev \
    libglfw3-dev \
    # Misc maths / utility
    libssl-dev \
    libboost-all-dev \
    # X11 forwarding support (for GUI apps inside container)
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialise rosdep (already done in osrf image, but ensure it is up to date)
RUN rosdep update

# ── Stage 2: Build Pangolin ───────────────────────────────────────────────────
FROM base AS pangolin

# Pangolin is not in the Ubuntu 22.04 APT repos and must be built from source.
# ORB-SLAM3 requires Pangolin for its 3-D viewer.
WORKDIR /opt/deps
RUN git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git pangolin_src \
    && mkdir pangolin_src/build \
    && cmake \
        -S pangolin_src \
        -B pangolin_src/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/pangolin \
    && cmake --build pangolin_src/build --parallel $(nproc) \
    && cmake --install pangolin_src/build \
    && rm -rf pangolin_src

# ── Stage 3: Build ORB-SLAM3 ─────────────────────────────────────────────────
FROM pangolin AS orbslam3

WORKDIR /opt/deps

# ── DBoW2 (ORB-SLAM3 bundles it; build separately for cleaner layout) ─────────
RUN git clone --depth 1 https://github.com/dorian3d/DBoW2.git dbow2_src \
    && mkdir dbow2_src/build \
    && cmake \
        -S dbow2_src \
        -B dbow2_src/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/DBoW2 \
    && cmake --build dbow2_src/build --parallel $(nproc) \
    && cmake --install dbow2_src/build \
    && rm -rf dbow2_src

# ── ORB-SLAM3 ────────────────────────────────────────────────────────────────
ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
RUN git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git ${ORB_SLAM3_ROOT}

# Patch build script to use all CPU cores
RUN sed -i 's/make -j4/make -j$(nproc)/g' ${ORB_SLAM3_ROOT}/build.sh || true

RUN cd ${ORB_SLAM3_ROOT} \
    && chmod +x build.sh \
    && CMAKE_PREFIX_PATH=/opt/pangolin:${CMAKE_PREFIX_PATH:-} ./build.sh

# ── Stage 4: Final image ──────────────────────────────────────────────────────
FROM base AS final

# Copy Pangolin libraries
COPY --from=pangolin /opt/pangolin /opt/pangolin

# Copy ORB-SLAM3 (vocabulary + libs + headers)
ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
COPY --from=orbslam3 ${ORB_SLAM3_ROOT} ${ORB_SLAM3_ROOT}

# Make Pangolin and ORB-SLAM3 libraries discoverable
RUN echo "/opt/pangolin/lib"               > /etc/ld.so.conf.d/pangolin.conf \
    && echo "${ORB_SLAM3_ROOT}/lib"        > /etc/ld.so.conf.d/orbslam3.conf \
    && echo "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib" >> /etc/ld.so.conf.d/orbslam3.conf \
    && echo "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"   >> /etc/ld.so.conf.d/orbslam3.conf \
    && ldconfig

# ── Shell environment ─────────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /workspace/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc \
    && echo "export ORB_SLAM3_ROOT=${ORB_SLAM3_ROOT}" >> /etc/bash.bashrc \
    && echo "export CMAKE_PREFIX_PATH=/opt/pangolin:\${CMAKE_PREFIX_PATH}" >> /etc/bash.bashrc \
    && echo "export LD_LIBRARY_PATH=${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib:/opt/pangolin/lib:\${LD_LIBRARY_PATH}" >> /etc/bash.bashrc

ENV ORB_SLAM3_ROOT=${ORB_SLAM3_ROOT}
ENV CMAKE_PREFIX_PATH=/opt/pangolin
ENV LD_LIBRARY_PATH=${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib:/opt/pangolin/lib

# ── Workspace layout ──────────────────────────────────────────────────────────
# The host workspace is bind-mounted to /workspace at runtime.
# Nothing is copied here — the mount makes it a two-way share.
RUN mkdir -p /workspace/src

WORKDIR /workspace

# Source ROS 2 for any RUN commands that need it
SHELL ["/bin/bash", "-c"]

# Default interactive shell
CMD ["/bin/bash"]
