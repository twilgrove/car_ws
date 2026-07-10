ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# =========================
# APT 源 + 系统依赖
# =========================
ARG APT_MIRROR=""
RUN if [ -n "$APT_MIRROR" ]; then \
        sed -i \
          -e "s|ports.ubuntu.com|$APT_MIRROR|g" \
          -e "s|archive.ubuntu.com|$APT_MIRROR|g" \
          -e "s|security.ubuntu.com|$APT_MIRROR|g" \
          /etc/apt/sources.list; \
    fi && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential cmake git wget curl pkg-config nano tmux \
      python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
      libpcl-dev libeigen3-dev libsuitesparse-dev libceres-dev libtbb-dev libompl-dev \
      libxtensor-dev libxsimd-dev libsdl2-dev libzmq3-dev libcurl4-openssl-dev \
      libnanoflann-dev nlohmann-json3-dev libbenchmark-dev libncurses-dev \
      libomp-dev liblapack-dev libblas-dev libusb-1.0-0-dev libboost-all-dev \
      qtbase5-dev libbluetooth-dev libspnav-dev \
      python3-psutil python3-ntplib python3-yaml python3-zmq \
      python3-setuptools python3-pytest && \
    rm -rf /var/lib/apt/lists/*

# =========================
# ROS 源 + ROS 包
# =========================
ARG ROS_MIRROR=""
RUN if [ -n "$ROS_MIRROR" ]; then \
        for f in /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.sources; do \
            [ -f "$f" ] && \
            sed -i \
              -e "s|packages.ros.org/ros2/ubuntu|$ROS_MIRROR|g" \
              -e "s|^Types: deb deb-src|Types: deb|g" \
              "$f"; \
        done; \
    fi && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-desktop \
      ros-humble-libg2o \
      ros-humble-map-msgs \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-behaviortree-cpp-v3 \
      ros-humble-bondcpp \
      ros-humble-diagnostic-updater \
      ros-humble-diagnostic-aggregator \
      ros-humble-diagnostic-common-diagnostics \
      ros-humble-joy \
      ros-humble-joy-linux \
      ros-humble-teleop-twist-joy \
      ros-humble-pcl-conversions \
      ros-humble-pcl-ros \
      ros-humble-pcl-msgs \
      ros-humble-pointcloud-to-laserscan \
      ros-humble-octomap \
      ros-humble-octomap-msgs \
      ros-humble-xacro \
      ros-humble-angles && \
    rm -rf /var/lib/apt/lists/*

# =========================
# GitHub 源 + 源码依赖
# =========================
ARG GIT_MIRROR=""
RUN if [ -n "$GIT_MIRROR" ]; then \
        git config --global url."${GIT_MIRROR}/https://github.com/".insteadOf https://github.com/; \
    fi && \
    \
    # Livox SDK2
    git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    git -C /tmp/Livox-SDK2 checkout f5d9375f84efe2b15bc0a052d3e18482ed13adf4 && \
    cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build && \
    cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" && \
    cmake --install /tmp/Livox-SDK2/build && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2 && \
    \
    # Sophus
    git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cmake -S /tmp/Sophus -B /tmp/Sophus/build \
      -DSOPHUS_USE_BASIC_LOGGING=ON \
      -DBUILD_SOPHUS_TESTS=OFF \
      -DBUILD_SOPHUS_EXAMPLES=OFF && \
    cmake --build /tmp/Sophus/build -j"$(nproc)" && \
    cmake --install /tmp/Sophus/build && \
    ldconfig && \
    rm -rf /tmp/Sophus && \
    \
    # GTSAM
    git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git /tmp/gtsam && \
    cmake -S /tmp/gtsam -B /tmp/gtsam/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF && \
    cmake --build /tmp/gtsam/build -j"$(nproc)" && \
    cmake --install /tmp/gtsam/build && \
    ldconfig && \
    rm -rf /tmp/gtsam
    
# 默认环境
RUN echo "source /car_ws/scripts/env.sh" >> /etc/bash.bashrc && \
    echo "set -g mouse on" > /etc/tmux.conf

WORKDIR /car_ws

CMD ["bash"]
