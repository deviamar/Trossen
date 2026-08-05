FROM ubuntu:22.04

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# ---- ROS 2 Humble + RViz2, installed from the official apt repo ----
# This is arch-agnostic by construction (works identically on amd64 and arm64),
# rather than depending on a specific prebuilt image publishing an arm64 manifest.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# rosdep ships with no default sources list; without this, the first
# `rosdep install` in the container fails on a missing 20-default.list.
# Must run as root. (`rosdep update` runs later as the user, so its cache
# lands in that user's ~/.ros instead of /root/.ros.)
RUN rosdep init

# ---- System deps ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    git \
    cmake \
    build-essential \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ---- Non-root user matching host UID/GID (avoids bind-mount permission issues) ----
# Clears any pre-existing owner of the requested UID/GID first: ubuntu:22.04 has
# none, but ubuntu:24.04 ships an `ubuntu` user at 1000:1000, which would make a
# bare groupadd/useradd fail the build outright.
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
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && usermod -aG dialout ${USERNAME}

# ---- C++ driver: clone + build + install to /usr/local ----
RUN git clone https://github.com/TrossenRobotics/trossen_arm.git /opt/trossen_arm \
    && cd /opt/trossen_arm \
    && mkdir build && cd build \
    && cmake .. \
    && make -j"$(nproc)" install \
    && ldconfig

# ---- Python driver ----
# Installed into the system Python that ships with ROS 2 Humble (Python 3.10)
# so ROS 2 nodes can `import trossen_arm` directly without venv activation.
RUN pip3 install --no-cache-dir trossen-arm

# Single source of truth for the workspace path, so nothing downstream has to
# hardcode the username.
ENV ROS_WS=/home/${USERNAME}/workspace

# Installed to a fixed, username-independent path: exec-form ENTRYPOINT performs
# no variable substitution, so it cannot reference ${USERNAME}.
COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh

USER ${USERNAME}
RUN mkdir -p ${ROS_WS}
WORKDIR ${ROS_WS}

# Source ROS 2 on shell start, and seed the rosdep cache as this user.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && rosdep update

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
