FROM ros:humble-ros-base
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update \
 && apt-get install -y ros-humble-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]