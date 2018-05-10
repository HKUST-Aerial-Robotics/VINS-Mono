FROM ros:kinetic-perception

# Install Ceres dependencies
RUN apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev && \
    rm -rf /var/lib/apt/lists/*

# Build and install Ceres
ENV CERES_VERSION="1.14.0"
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) install && \
    rm -rf ../../ceres-solver

# Install ros dependencies
RUN apt-get update && apt-get install -y \
      python-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
    rm -rf /var/lib/apt/lists/*

# Setup catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Clone VINS-Mono
RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

# Build VINS-Mono
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build

# Source VINS-Mono from entrypoint
RUN sed -i '/exec "$@"/i \
source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
