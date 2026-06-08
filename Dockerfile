# ─────────────────────────────────────────────────────────────────────────────
# Shelfbot Docker image — ROS 2 Humble
#
# User model
#   • Build stages (pangolin, orbslam3) run as root; artefacts land in /opt.
#   • The final stage creates user 'chris' (UID/GID 1000, matching the host).
#   • docker-entrypoint.sh runs as chris on every start and uses NOPASSWD sudo
#     to repair ownership of persistent volumes.
#
# Host repo:  /home/chris/shelfbot/shelfbot/
# ─────────────────────────────────────────────────────────────────────────────

# ── Stage 1: base ─────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000


# Locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip ninja-build \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-empy \
    python3-lark \
    python3-jsonschema \
    python3-yaml \
    nano vim bash-completion \
    joystick \
    jstest \
    evtest \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-interface \
    ros-humble-hardware-interface \
    ros-humble-realtime-tools \
    ros-humble-pluginlib \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-message-filters \
    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
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
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-cmake \
    ros-humble-rosidl-parser \
    ros-humble-rosidl-runtime-c \
    ros-humble-rosidl-typesupport-interface \
    ros-humble-builtin-interfaces \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# System libraries for vision
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv libapriltag-dev libeigen3-dev \
    libgl1-mesa-dev libglew-dev libglfw3-dev \
    libssl-dev libboost-all-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends python3-ament-package \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

# ── Create user 'chris' matching host UID/GID ─────────────────────────────────
RUN groupadd --gid ${USER_GID} chris \
    && useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        chris \
    && echo "chris ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/chris \
    && echo "Defaults !fqdn" > /etc/sudoers.d/no-fqdn \
    && chmod 0440 /etc/sudoers.d/chris

# ── Stage 2: Pangolin ─────────────────────────────────────────────────────────
FROM base AS pangolin

WORKDIR /opt/deps
RUN git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git pangolin_src \
    && mkdir pangolin_src/build \
    && cmake -S pangolin_src -B pangolin_src/build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/pangolin \
        -DBUILD_PYPANGOLIN=OFF -DCMAKE_CXX_FLAGS="-include limits" \
    && cmake --build pangolin_src/build --parallel $(nproc) \
    && cmake --install pangolin_src/build \
    && rm -rf pangolin_src


# ── Stage 3: ORB-SLAM3 ────────────────────────────────────────────────────────
FROM pangolin AS orbslam3

WORKDIR /opt/deps
ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
RUN git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git ${ORB_SLAM3_ROOT}
RUN sed -i 's/make -j4/make -j$(nproc)/g' ${ORB_SLAM3_ROOT}/build.sh || true
RUN cd ${ORB_SLAM3_ROOT} && chmod +x build.sh \
    && CMAKE_PREFIX_PATH=/opt/pangolin:${CMAKE_PREFIX_PATH:-} ./build.sh


# ── Stage 4: final ────────────────────────────────────────────────────────────
FROM base AS final

COPY --from=pangolin /opt/pangolin /opt/pangolin

ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
COPY --from=orbslam3 ${ORB_SLAM3_ROOT} ${ORB_SLAM3_ROOT}

RUN echo "/opt/pangolin/lib"                          >  /etc/ld.so.conf.d/pangolin.conf \
    && echo "${ORB_SLAM3_ROOT}/lib"                   >  /etc/ld.so.conf.d/orbslam3.conf \
    && echo "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib"  >> /etc/ld.so.conf.d/orbslam3.conf \
    && echo "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"    >> /etc/ld.so.conf.d/orbslam3.conf \
    && ldconfig

ENV ORB_SLAM3_ROOT=${ORB_SLAM3_ROOT}
ENV CMAKE_PREFIX_PATH=/opt/pangolin
ENV LD_LIBRARY_PATH=${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib:/opt/pangolin/lib

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /home/chris/shelfbot_ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

# ── Install entrypoint ─────────────────────────────────────────────────────
COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Pre-create workspace and IDE config parent directories with correct ownership
RUN mkdir -p /home/chris/shelfbot_ws/src \
    && chown -R chris:chris /home/chris/shelfbot_ws \
    && mkdir -p /home/chris/.cache /home/chris/.config /home/chris/.local \
    && chown -R chris:chris /home/chris/.cache /home/chris/.config /home/chris/.local

# ── Switch to chris ───────────────────────────────────────────────────────────
USER chris

RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default || true

RUN echo "" >> /home/chris/.bashrc \
    && echo "# ── ROS 2 / Shelfbot ──────────────────────────────────────────" >> /home/chris/.bashrc \
    && echo "source /opt/ros/humble/setup.bash"                                 >> /home/chris/.bashrc \
    && echo "source /home/chris/shelfbot_ws/install/setup.bash 2>/dev/null || true" >> /home/chris/.bashrc \
    && echo "export ORB_SLAM3_ROOT=${ORB_SLAM3_ROOT}"                           >> /home/chris/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=/opt/pangolin:\${CMAKE_PREFIX_PATH:-}"    >> /home/chris/.bashrc \
    && echo "export LD_LIBRARY_PATH=${ORB_SLAM3_ROOT}/lib:${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib:${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib:/opt/pangolin/lib:\${LD_LIBRARY_PATH:-}" >> /home/chris/.bashrc \
    && echo "export ROS_DOMAIN_ID=0"                                            >> /home/chris/.bashrc

WORKDIR /home/chris/shelfbot_ws

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]
