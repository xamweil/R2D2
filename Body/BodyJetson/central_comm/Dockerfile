# === central_comm
# Build base image first
FROM bodyjetson-ros:humble

USER root
WORKDIR /home/ros

# Extra runtime tools for comms
RUN apt-get update && apt-get install -y \
      # bluetooth bluez bluez-tools rfkill dbus libbluetooth-dev # Whatever bluetooth stack is used
      i2c-tools libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# Add python libs
RUN pip3 install --no-cache-dir pyserial
RUN pip3 install dalybms

# Entrypoint scripts you created in central_comm/docker/
COPY docker/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/auto_launch.sh    /usr/local/bin/auto_launch.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/auto_launch.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["/usr/local/bin/auto_launch.sh"]
