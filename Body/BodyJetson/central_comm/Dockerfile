# === central_comm: build + runtime
# Build base image first
FROM bodyjetson-ros:humble AS builder

USER root
WORKDIR /home/ros


RUN apt-get update && apt-get install -y \
      python3-vcstool \
      python3-ament-package \
      python3-setuptools \
      ros-humble-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN bash -lc "source /opt/ros/humble/setup.bash && \
              rosdep install --from-paths ros2_ws/src --ignore-src -r -y \
                --rosdistro humble \
                --skip-keys ament_python"

# ===== Runtime image =====
FROM bodyjetson-ros:humble AS runtime
WORKDIR /home/ros

# Extra runtime tools for comms
RUN apt-get update && apt-get install -y \
      # bluetooth bluez bluez-tools rfkill dbus libbluetooth-dev # Whatever bluetooth stack is used
      i2c-tools libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# Add python libs
RUN pip3 install --no-cache-dir pyserial

# Entrypoint scripts you created in central_comm/docker/
COPY --chown=ros:ros docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY --chown=ros:ros docker/auto_launch.sh    /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]
