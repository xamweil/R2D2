FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WS=/opt/ros/ws

WORKDIR ${WS}

# System + ROS deps
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

COPY ros2_ws/ ${WS}/

RUN rosdep update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]