# =============================================================================
# Rig visualiser. Draws the real robot from its own topics, in a browser.
#
# Like `monitor`, this is a PURE ROS CLIENT: no devices, no privileges, no
# vendor SDK. It only ever subscribes. Nothing here can move the robot, which is
# what makes it safe to leave running while you drive.
#
# Not RViz, deliberately. RViz needs a display, and this rig is driven over ssh
# as often as it is driven at the bench -- `xhost +local:docker` plus a
# forwarded X socket is a lot of moving parts for a picture. Viser serves a web
# page instead, so the view works from any machine that can reach this one.
# =============================================================================
FROM ros:humble-ros-core

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# ros-core already provides rclpy and the common message packages; see
# monitor/Dockerfile for the full accounting of what that image ships.
# libgl1 is for trimesh's mesh loading path, which imports it even headless.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

RUN existing_user="$(getent passwd ${USER_UID} | cut -d: -f1)" \
    && if [ -n "${existing_user}" ] && [ "${existing_user}" != "${USERNAME}" ]; then \
         userdel -r "${existing_user}"; \
       fi \
    && existing_group="$(getent group ${USER_GID} | cut -d: -f1)" \
    && if [ -n "${existing_group}" ] && [ "${existing_group}" != "${USERNAME}" ]; then \
         groupdel "${existing_group}"; \
       fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

ENV ROS_WS=/home/${USERNAME}/workspace
USER ${USERNAME}
RUN mkdir -p ${ROS_WS}
WORKDIR ${ROS_WS}
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["./rig_sim.py"]
