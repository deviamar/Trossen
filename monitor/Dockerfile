# =============================================================================
# Rig monitor: watch every topic, send simple test commands.
#
# No hardware, no SDK, no devices, no privileges -- it is a pure ROS client, and
# that is the point. When something is wrong, this is the container you trust,
# because there is almost nothing in it that could be the thing that is wrong.
#
# ros-core plus the message packages: enough to introspect and print any topic
# in the rig. ros2topic/ros2node give the CLI tools this leans on.
# =============================================================================
FROM ros:humble-ros-core

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# `ros:humble-ros-core` ALREADY HAS everything this needs, which is why the apt
# list below is so short. Verified against the ROS apt index rather than assumed:
#   ros-humble-ros-core        -> ros-humble-rclpy, ros-humble-common-interfaces
#   common-interfaces          -> std_msgs, geometry_msgs, sensor_msgs, nav_msgs,
#                                 std_srvs, and the rest
#   ros2cli-common-extensions  -> ros2topic, ros2node, ros2service, ros2param, ...
#
# An earlier version of this file installed python3-rclpy, which DOES NOT EXIST
# in the ROS repo -- the package is ros-humble-rclpy, and it was already present.
# The build failed with "Unable to locate package python3-rclpy". Left as a note
# because the Debian-style name is the intuitive guess and the wrong one.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Empty by default -- everything here is standard library plus rclpy.
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
