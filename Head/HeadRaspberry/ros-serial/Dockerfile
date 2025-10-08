# === ros-serial
FROM ros:humble-ros-base

USER root
WORKDIR /home/ros

# Add python libs
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-pip 
RUN pip3 install --no-cache-dir pyserial
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-example-interfaces \
 && rm -rf /var/lib/apt/lists/*

# Entrypoint scripts you created in central_comm/docker/
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh    /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]