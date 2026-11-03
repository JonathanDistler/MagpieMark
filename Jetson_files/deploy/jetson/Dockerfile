# CPU-only companion computer runtime; builds natively on Jetson ARM64.
FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=2
ARG AGENT_VERSION=v2.4.3
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git libasio-dev libtinyxml2-dev \
    python3-colcon-common-extensions python3-tk \
    ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-std-srvs \
    ros-jazzy-rosgraph-msgs ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch "${AGENT_VERSION}" \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build \
         -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/agent/build --parallel "${BUILD_JOBS}" \
    && cmake --install /tmp/agent/build && ldconfig \
    && rm -rf /tmp/agent
WORKDIR /opt/drone
COPY px4_msgs src/px4_msgs
COPY ros2/swarm_mavsdk_ros2 src/swarm_mavsdk_ros2
RUN source /opt/ros/jazzy/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}" colcon build \
         --executor sequential --cmake-args -DBUILD_TESTING=OFF
COPY config/bench.example.json /opt/drone/config/bench.example.json
COPY scripts/check_external_odometry.py /opt/drone/scripts/check_external_odometry.py
COPY deploy/jetson/entrypoint.sh /drone-entrypoint.sh
RUN chmod +x /drone-entrypoint.sh
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENTRYPOINT ["/drone-entrypoint.sh"]
CMD ["ros2", "--help"]
