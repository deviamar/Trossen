# =============================================================================
# Meta Quest teleop source.
#
# The smallest image in the rig, on purpose. It talks to a headset and publishes
# ROS topics -- no vendor SDK, no CUDA, no device nodes, no privileges. That is
# what makes it the one you can rebuild in a minute while everything else keeps
# running, which is the whole reason teleop lives in its own container.
#
# ros-core, not ros-base or desktop: this needs rclpy, std_msgs, geometry_msgs
# and sensor_msgs and nothing else. Adding RViz here would triple the image to
# provide a viewer that the arm containers already have.
# =============================================================================
FROM ros:humble-ros-core

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# `ros:humble-ros-core` ALREADY provides rclpy and the message packages this
# needs -- ros-core depends on ros-humble-rclpy and ros-humble-common-interfaces,
# and common-interfaces carries std_msgs, geometry_msgs and sensor_msgs. So only
# pip is added here.
#
# An earlier version installed python3-rclpy, which DOES NOT EXIST in the ROS
# repo (the package is ros-humble-rclpy) and was already installed anyway. The
# build failed with "Unable to locate package python3-rclpy". Noted because the
# Debian-style name is the intuitive guess and the wrong one.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Empty by default. aiortc goes here if the WebRTC backend gets implemented --
# see workspace/backends/webrtc.py. Kept as a file rather than an inline pip
# install so adding a dependency does not mean editing the Dockerfile.
COPY requirements.txt /tmp/requirements.txt
RUN if [ -s /tmp/requirements.txt ]; then \
      pip3 install --no-cache-dir -r /tmp/requirements.txt; \
    fi

RUN existing_user="$(getent passwd ${USER_UID} | cut -d: -f1)" \
    && if [ -n "${existing_user}" ] && [ "${existing_user}" != "${USERNAME}" ]; then \
         userdel -r "${existing_user}"; \
       fi \
    && existing_group="$(getent group ${USER_GID} | cut -d: -f1)" \
    && if [ -n "${existing_group}" ] && [ "${existing_group}" != "${USERNAME}" ]; then \
         groupdel "${existing_group}"; \
       fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers 2>/dev/null || true

ENV ROS_WS=/home/${USERNAME}/workspace

COPY --chmod=644 ros-env.sh /usr/local/bin/ros-env.sh
COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh

# Same three-way sourcing as ../slate-base: entrypoint for the container's own
# process, .bashrc for `exec ... bash`, BASH_ENV for `exec ... bash -c`.
# ---- Make the ROS environment work WITHOUT a shell --------------------------
# `docker compose exec <svc> ./script.py` runs the script directly: no
# entrypoint, no bash, so neither ~/.bashrc nor $BASH_ENV gets a chance and the
# script dies on `import rclpy`. Baking the variables into the image config
# fixes it for every invocation, shell or not -- `exec`, `run`, `command:`, all
# of them.
#
# Values lifted from a container that had actually sourced
# /opt/ros/humble/setup.bash, not written from memory. ros-env.sh still exists
# and is still sourced; it stays authoritative for anything computed at runtime
# (a colcon overlay, the workspace path), and setting the same values twice is
# harmless.
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/home/${USERNAME}/workspace:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

ENV BASH_ENV=/usr/local/bin/ros-env.sh

USER ${USERNAME}
RUN mkdir -p ${ROS_WS} \
    && echo "source /usr/local/bin/ros-env.sh" >> /home/${USERNAME}/.bashrc
WORKDIR ${ROS_WS}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
