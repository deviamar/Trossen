# =============================================================================
# SLATE mobile base (Trossen Robotics AGV) -- ROS 2 Humble driver container.
#
# The third container in this rig, and the simplest of the three: one vendor
# package, one node, no URDF, no per-arm calibration. What it does NOT share
# with the other two is worth stating up front, because it drives every choice
# below:
#
#   * The transport is USB serial, like ../middle-arm/, but the chip is a CH340
#     (VID:PID 1a86:7523), not the FTDI in the U2D2. Different udev rule, and
#     the two never collide on a name.
#   * The port is NOT configurable. base_driver::chassisInit() takes an OUTPUT
#     string: the driver enumerates USB serial devices with libudev, matches
#     idVendor/idProduct itself, and tells you afterwards which node it opened
#     ("Initalized base at port: '...'", upstream's typo). There is no port
#     parameter, no env var, and no equivalent of DXL_PORT to set. A udev
#     SYMLINK is therefore diagnostic only -- see host-setup/.
#   * Part of the driver is a PREBUILT BLOB. trossen_slate ships
#     lib/x86_64/libchassis_driver.so and lib/aarch64/libchassis_driver.so and
#     nothing else; its CMakeLists raises FATAL_ERROR on any other
#     CMAKE_SYSTEM_PROCESSOR. So this image builds on amd64 and arm64 and is
#     simply not buildable elsewhere -- unlike ../manip-arm/, which is
#     arch-agnostic by construction.
#
# Vendor stack lands in /opt/interbotix_ws, never in ~/workspace: that path is a
# bind mount at runtime and would shadow anything built into it at image-build
# time. Same rule as the other two images.
# =============================================================================
FROM ubuntu:22.04

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=humble

# ros-humble-desktop matches the other two images, so all three resolve to the
# same ROS build and interoperate over DDS without surprises. A base does not
# strictly need RViz -- but odom and the odom->base_link TF are exactly the kind
# of thing you want to look at rather than read as numbers. Set to `ros-base`
# for a ~2 GB smaller image if you never visualize anything.
ARG ROS_PKG=desktop

# Pin to a tag or SHA once this setup is proven. `humble` is a moving branch:
# two machines built weeks apart can get different upstream commits, which is
# the same reproducibility caveat called out in the other two READMEs.
ARG INTERBOTIX_REF=humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---- Base tooling + locale --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg2 lsb-release software-properties-common \
      build-essential cmake git sudo locales usbutils udev \
      python3 python3-dev python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 Humble ------------------------------------------------------------
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-${ROS_PKG} \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep ships with no default sources list; the first `rosdep install` fails on
# a missing 20-default.list without this. Must run as root.
RUN rosdep init

# ---- Non-root user matching host UID/GID ------------------------------------
# `dialout` for the CH340 serial port. In practice the host udev rule also sets
# the node 0666, so group membership is belt-and-braces -- but it is the thing
# that keeps working if someone installs a stricter rule later.
#
# Same defensive UID/GID reclaim as the other two images: ubuntu:22.04 has no
# user at 1000, but 24.04 ships one, and a bare useradd would fail the build.
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
    && usermod -aG dialout,plugdev ${USERNAME}

# ---- Interbotix SLATE stack -> /opt/interbotix_ws -----------------------------
# Three load-bearing steps, each of which fails quietly if skipped:
#
#   1. rm interbotix_ros_slate/COLCON_IGNORE
#        The repo ships the whole SLATE subtree ignored. Without this, colcon
#        builds a workspace containing no base driver at all and reports
#        success -- the failure only shows up later as
#        "Package 'interbotix_slate_driver' not found".
#   2. git submodule update --init interbotix_ros_slate/trossen_slate
#        trossen_slate is a separate repo (and holds the prebuilt
#        libchassis_driver.so). A non-recursive clone leaves the directory
#        EMPTY, and the build then fails on a missing `trossen_slate` CMake
#        package. This is why upstream's instructions say `git clone
#        --recursive`; only this one submodule is needed here.
#   3. Leaving interbotix_ros_xseries/COLCON_IGNORE and
#      interbotix_ros_common_drivers/COLCON_IGNORE in place
#        Those are the DYNAMIXEL stack, which belongs to ../middle-arm/ and has
#        no business in this image.
ENV INTERBOTIX_WS=/opt/interbotix_ws
RUN mkdir -p ${INTERBOTIX_WS}/src && cd ${INTERBOTIX_WS}/src \
    && git clone --depth 1 -b ${INTERBOTIX_REF} \
       https://github.com/Interbotix/interbotix_ros_core.git \
    && cd interbotix_ros_core \
    && rm -f interbotix_ros_slate/COLCON_IGNORE \
    && git submodule update --init --depth 1 interbotix_ros_slate/trossen_slate \
    && test -f interbotix_ros_slate/trossen_slate/CMakeLists.txt \
       || { echo "ERROR: trossen_slate submodule is empty" >&2; exit 1; }

# libudev-dev is the dependency that matters and the one rosdep would silently
# skip if the `|| true` fallback used elsewhere in this repo were applied here:
# the prebuilt chassis driver links against libudev to find its own serial port,
# so without it the build fails at link time rather than at runtime. No `|| true`
# on this rosdep install for exactly that reason -- a missing dep here is not
# something to discover later.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && cd ${INTERBOTIX_WS} \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && colcon build \
         --parallel-workers "$(nproc)" \
         --packages-up-to interbotix_slate_driver \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && test -x ${INTERBOTIX_WS}/install/interbotix_slate_driver/lib/interbotix_slate_driver/slate_base_node \
    && rm -rf /var/lib/apt/lists/* ${INTERBOTIX_WS}/build ${INTERBOTIX_WS}/log

ENV ROS_WS=/home/${USERNAME}/workspace

# Fixed, username-independent paths: an exec-form ENTRYPOINT performs no
# variable substitution, so it cannot reference ${USERNAME}.
COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh
COPY --chmod=644 ros-env.sh /usr/local/bin/ros-env.sh

USER ${USERNAME}
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Three entry paths into this container, all of which need the overlays sourced:
#   1. container's own process / `compose run`  -> ENTRYPOINT sources ros-env.sh
#   2. `compose exec ... bash` (interactive)    -> ~/.bashrc, appended below
#   3. `compose exec ... <non-interactive cmd>` -> $BASH_ENV, set here
#
# (3) is the easy one to miss. `docker exec` bypasses ENTRYPOINT entirely, and
# Ubuntu's stock ~/.bashrc returns early for non-interactive shells -- so
# without BASH_ENV, `docker compose exec slate-base ros2 topic list` fails with
# "ros2: command not found" while the identical command typed inside an
# interactive bash works. ros-env.sh guards against repeat sourcing, which
# BASH_ENV would otherwise cause on every nested shell.
ENV BASH_ENV=/usr/local/bin/ros-env.sh

RUN echo "source /usr/local/bin/ros-env.sh" >> /home/${USERNAME}/.bashrc \
    && rosdep update --rosdistro ${ROS_DISTRO}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
