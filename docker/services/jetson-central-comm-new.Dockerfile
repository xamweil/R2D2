FROM docker.io/library/ros:humble
ENV ROOT="../.."

RUN apt update && apt install -y \
    curl \
    bash-completion

RUN echo "source /etc/profile.d/bash_completion.sh" >> /root/.bashrc

RUN mkdir -p /tmp/just && \
    curl -L https://github.com/casey/just/releases/download/1.43.0/just-1.43.0-aarch64-unknown-linux-musl.tar.gz | tar -xz -C /tmp/just && \
    mv /tmp/just/just /usr/local/bin/just && \
    mkdir -p /etc/bash_completion.d && mv /tmp/just/completions/just.bash /etc/bash_completion.d/just && \
    rm -rf /tmp/just

#### ros setup #################################################################

ENV OVERLAY_WS=/opt/ros/overlay_ws
WORKDIR $OVERLAY_WS

RUN apt update && apt install -y \
    ros-humble-joy


COPY $ROOT/Body/BodyJetson/central_comm/ros2_ws $OVERLAY_WS/

RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install && rm -rf src

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

#### pico platformio setup ########################################################

# ENV PICO_DIR=/opt/pico
# WORKDIR $PICO_DIR
#
# RUN apt install -y python3-venv
# RUN curl -fsSL -o /tmp/get-platformio.py "https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py"
# RUN curl -fsSL -o /tmp/99-platformio-udev.rules https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules
# RUN mkdir -p /etc/udev/rules.d /lib/udev/rules.d && \
#     cp /tmp/99-platformio-udev.rules /etc/udev/rules.d/ && \
#     cp /tmp/99-platformio-udev.rules /lib/udev/rules.d/ && \
#     rm /tmp/99-platformio-udev.rules
# RUN python3 /tmp/get-platformio.py && rm /tmp/get-platformio.py
#
# COPY $ROOT/Body/BodyPico_MotorControl/ $PICO_DIR/
#
# RUN mkdir -p /usr/local/bin && \
#     ln -s ~/.platformio/penv/bin/platformio /usr/local/bin/platformio && \ 
#     ln -s ~/.platformio/penv/bin/pio /usr/local/bin/pio && \
#     ln -s ~/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb && \
#     pio run -e release && rm -rf ./*

#### entrypoint ################################################################

WORKDIR $OVERLAY_WS

CMD ["bash", "-c", "trap 'exit 0' TERM; tail -f /dev/null & wait"]
