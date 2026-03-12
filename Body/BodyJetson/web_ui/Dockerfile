FROM ros:humble-ros-base

USER root
WORKDIR /home/ros

# Build tools + DDS + common msg pkgs for camera topics
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-pip \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-humble-vision-msgs \
 && rm -rf /var/lib/apt/lists/*

# FastAPI stack
RUN pip3 install --no-cache-dir \
    fastapi \
    "uvicorn[standard]" \
    pydantic

# Entrypoint scripts
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh    /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]