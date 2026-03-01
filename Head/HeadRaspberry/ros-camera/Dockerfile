# === ros-camera
FROM ros:humble-perception

USER root
WORKDIR /home/ros

# Build + runtime deps for Raspberry Pi libcamera fork + camera_ros
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    git \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    python3-pip \
    python3-jinja2 \
    python3-yaml \
    python3-ply \
    python3-setuptools \
    python3-pybind11 \
    libyaml-dev \
    libgnutls28-dev \
    libssl-dev \
    libevent-dev \
    libdw-dev \
    libunwind-dev \
    libjpeg-dev \
    libtiff5-dev \
    libpng-dev \
    libopencv-dev \
    libdrm-dev \
    libegl1-mesa-dev \
    libgbm-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip
RUN python3 -m pip install --no-cache-dir "meson>=1.0.1"

# ---- Build Raspberry Pi libcamera fork
# camera_ros supports Raspberry Pi cameras via libcamera, but packaged libcamera can miss RPi camera module support.
# Building raspberrypi/libcamera is the reliable route for Pi camera modules. :contentReference[oaicite:1]{index=1}
WORKDIR /tmp
RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git
WORKDIR /tmp/libcamera
RUN meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true \
    -Dgstreamer=disabled
RUN ninja -C build
RUN ninja -C build install
RUN ldconfig

# ---- ROS2 workspace + camera_ros
WORKDIR /home/ros
RUN mkdir -p /home/ros/ros2_ws/src
WORKDIR /home/ros/ros2_ws/src
RUN git clone --depth 1 https://github.com/christianrauch/camera_ros.git

# Resolve ROS deps (skip libcamera rosdep key because we built our own)
WORKDIR /home/ros/ros2_ws
RUN apt-get update && rosdep update
RUN rosdep install -i --from-paths src --rosdistro humble -y --skip-keys=libcamera \
 && rm -rf /var/lib/apt/lists/*

# Entrypoint scripts (same pattern as ros-serial)
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh    /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]