FROM microros/micro-ros-agent:humble

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*