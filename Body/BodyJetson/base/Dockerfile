# ===== Pull base image and install ROS and build tools =====
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release software-properties-common \
    build-essential python3-pip cmake git \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8 \
    LC_ALL en_US.UTF-8


# Enable Universe 
RUN apt-get update && apt-get install -y software-properties-common curl && add-apt-repository universe

# Install the ROS apt-source helper (adds key + repo for you)
RUN set -eux; \
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')"; \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"; \
    apt-get update && apt-get install -y /tmp/ros2-apt-source.deb && rm -f /tmp/ros2-apt-source.deb


# ROS 2 Humble + tools
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-rosdep python3-colcon-common-extensions \
    && rosdep init || true \
    && rosdep update

# Createing a non-root user 
RUN useradd -m ros && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER root
WORKDIR /home/ros

# Init ROS env in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc



