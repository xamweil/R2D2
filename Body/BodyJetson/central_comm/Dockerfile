FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

ENV WS=/home/ros/ros2_ws
WORKDIR ${WS}

# System + ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    i2c-tools \
    libi2c-dev \
    bluez \
    bluez-tools \
    dbus \
    libbluetooth-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial

# Copy package manifests/source temporarily so rosdep can install dependencies.
# At runtime this directory is replaced by the bind-mounted host workspace.
COPY ros2_ws/src/ ${WS}/src/

RUN rosdep update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install \
        -i \
        --from-paths src \
        --rosdistro ${ROS_DISTRO} \
        -y

COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]