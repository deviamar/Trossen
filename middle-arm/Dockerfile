# =============================================================================
# Middle arm (active vision): Interbotix WidowX-250 6DOF (wx250s) + ZED camera(s)
#
# Deliberately NOT built on the same base as ../manip-arm/Dockerfile. That image targets
# the WidowX AI arm (wxai_v0), which talks Ethernet via the `trossen_arm` SDK.
# The wx250s is an X-Series arm: DYNAMIXEL servos on a U2D2 USB-serial adapter,
# driven by the Interbotix `interbotix_xs_sdk` stack. Different driver, different
# transport, nothing shared but ROS 2 itself.
#
# The image has two variants, selected by BASE_IMAGE + WITH_ZED:
#
#   arm-only (default)  ubuntu:22.04, no ZED, no CUDA, no GPU needed.
#                       Fast to build, runs anywhere. Use this to bring the arm
#                       up first and prove the DYNAMIXEL chain works.
#   with-ZED            stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu22.04, which
#                       ships CUDA and the ZED SDK preinstalled. Requires an
#                       NVIDIA GPU and the container toolkit on every host.
#                       Enable via docker-compose.zed.yml.
#
# Constraints behind those tags:
#   - 'gl-devel' (not plain 'devel') carries the OpenGL stack that RViz2 and the
#     ZED tools need. The plain variant renders nothing.
#   - ubuntu22.04 either way: Interbotix supports ROS 2 Humble only, and Humble
#     only has apt binaries for jammy. 24.04 would mean building ROS from source.
#   - gl-devel currently ships only CUDA 12.8 (see the ZED wrapper docker README
#     configuration table), so the CUDA version is not freely choosable.
# =============================================================================
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=humble
ARG WITH_ZED=false

# MoveIt roughly doubles the build and adds a source build of moveit_visual_tools.
# An active-vision arm is normally driven by direct joint commands, so it is off
# by default. Flip to "true" if you want interbotix_xsarm_moveit.
ARG WITH_MOVEIT=false

# Pin these to a tag/SHA once the setup is working, so a rebuild on another
# machine reproduces this one instead of tracking upstream HEAD.
ARG ZED_WRAPPER_REF=master

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# The StereoLabs base asks the NVIDIA runtime for compute,video,utility only.
# 'graphics' and 'display' are additionally required or RViz2 and the ZED viewer
# get a software GL context (or no context at all) despite the GPU being present.
ENV NVIDIA_DRIVER_CAPABILITIES=compute,video,utility,graphics,display

# ---- Locale + build tooling -------------------------------------------------
# `|| true` on update mirrors upstream's Dockerfile: the CUDA apt repo in the
# base image intermittently fails to refresh, and the install below still
# succeeds from the cached lists when it does.
RUN apt-get update || true && apt-get install -y --no-install-recommends \
      curl wget gnupg2 lsb-release software-properties-common \
      build-essential cmake git sudo locales usbutils udev \
      python3 python3-dev python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 Humble ------------------------------------------------------------
# Same apt-repo approach as the wxai_v0 container, so both images resolve to the
# same ROS build and their DDS graphs interoperate without surprises.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-desktop \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep ships with no default sources list; the first `rosdep install` fails on
# a missing 20-default.list without this. Must run as root.
RUN rosdep init

# ---- Non-root user matching host UID/GID ------------------------------------
# `dialout` for the U2D2 serial port, `video`/`plugdev` for the ZED's USB video
# interface. Same defensive UID/GID reclaim as the wxai_v0 image.
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
    && usermod -aG dialout,video,plugdev ${USERNAME}

# ---- ZED ROS 2 wrapper -> /opt/zed_ws ---------------------------------------
# Skipped entirely when WITH_ZED=false (the default). ros-env.sh sources this
# workspace only if it exists, so the arm-only image needs no other changes.
#
# Vendor stacks build into /opt, never into the user's ~/workspace: that path is
# a bind mount at runtime, so anything built there would be shadowed by the host
# directory the moment the container starts.
#
# Two non-obvious flags, both lifted from StereoLabs' own build script:
#   -DCMAKE_LIBRARY_PATH=.../cuda/lib64/stubs  - no GPU is visible during
#     `docker build`, so there is no real libcuda.so to link against; the stub
#     satisfies the linker and the real driver library binds at runtime.
#   --skip-keys libopencv-dev  - the ZED base image ships an OpenCV matched to
#     the SDK build; letting rosdep pull Ubuntu's would swap it out underneath.
ENV ZED_WS=/opt/zed_ws
RUN if [ "${WITH_ZED}" != "true" ]; then \
      echo ">>> WITH_ZED=false, skipping ZED ROS 2 wrapper" && exit 0 ; \
    fi ; \
    set -e ; \
    mkdir -p ${ZED_WS}/src \
    && git clone --depth 1 -b ${ZED_WRAPPER_REF} \
       https://github.com/stereolabs/zed-ros2-wrapper.git ${ZED_WS}/src/zed-ros2-wrapper \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && { rosdep install --from-paths ${ZED_WS}/src --ignore-src -r -y \
         --rosdistro ${ROS_DISTRO} --skip-keys "libopencv-dev" || true ; } \
    && cd ${ZED_WS} \
    && colcon build \
         --parallel-workers "$(nproc)" \
         --base-paths src \
         --packages-up-to zed_ros2 \
         --cmake-args \
           -DCMAKE_BUILD_TYPE=Release \
           -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
           --no-warn-unused-cli \
    && rm -rf /var/lib/apt/lists/* ${ZED_WS}/build ${ZED_WS}/log

# ---- Interbotix X-Series stack -> /opt/interbotix_ws -------------------------
# This is the manual equivalent of xsarm_amd64_install.sh. That script is not
# usable in a Dockerfile: it installs ROS itself, prompts interactively, and
# writes udev rules that have no effect inside a container (see host-setup/).
#
# The COLCON_IGNORE removal and the two submodule inits are load-bearing --
# interbotix_ros_xseries (which holds interbotix_xs_sdk, the actual DYNAMIXEL
# driver) is ignored by colcon until that file is deleted, so a build without
# this step silently produces a workspace with no arm driver in it.
ENV INTERBOTIX_WS=/opt/interbotix_ws
RUN mkdir -p ${INTERBOTIX_WS}/src && cd ${INTERBOTIX_WS}/src \
    && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/Interbotix/interbotix_ros_core.git \
    && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/Interbotix/interbotix_ros_manipulators.git \
    && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/Interbotix/interbotix_ros_toolboxes.git \
    && rm -f interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE \
    && cd interbotix_ros_core \
    && git submodule update --init --depth 1 interbotix_ros_xseries/dynamixel_workbench_toolbox \
    && git submodule update --init --depth 1 interbotix_ros_xseries/interbotix_xs_driver \
    && cd ${INTERBOTIX_WS}/src \
    && if [ "${WITH_MOVEIT}" = "true" ]; then \
         rm -f interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE \
               interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE ; \
         git clone --depth 1 -b ros2 https://github.com/ros-planning/moveit_visual_tools.git ; \
       fi

# `rosdep update` must run here, not only in the ZED block above: that block is
# skipped entirely when WITH_ZED=false, which would leave rosdep initialized but
# with no cache. `rosdep install` then fails, `|| true` swallows it, and colcon
# builds against missing deps -- ros-humble-desktop does NOT carry
# controller_manager / joint_trajectory_controller / hardware_interface, so the
# arm-only variant fails deep in the build with confusing CMake errors.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && cd ${INTERBOTIX_WS} \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "libopencv-dev" || true \
    && if [ "${WITH_MOVEIT}" = "true" ]; then \
         colcon build --parallel-workers "$(nproc)" --cmake-args -DCMAKE_BUILD_TYPE=Release ; \
       else \
         colcon build --parallel-workers "$(nproc)" \
           --packages-up-to interbotix_xsarm_control \
                            interbotix_xsarm_descriptions \
                            interbotix_xs_modules \
           --cmake-args -DCMAKE_BUILD_TYPE=Release ; \
       fi \
    && rm -rf /var/lib/apt/lists/* ${INTERBOTIX_WS}/build ${INTERBOTIX_WS}/log

# ---- Python deps not covered by rosdep ---------------------------------------
# modern_robotics is imported by interbotix_xs_modules.xs_robot.arm (the module
# that provides InterbotixManipulatorXS, the high-level Python arm API). It is a
# plain pip package with no rosdep key, so the colcon build succeeds without it
# and the breakage only shows up at import time:
#   >>> from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
#   ModuleNotFoundError: No module named 'modern_robotics'
# Nothing in the launch path touches it, so the arm comes up fine without it --
# which is exactly why it is easy to miss until you write your first script.
RUN pip3 install --no-cache-dir modern_robotics

# The ZED SDK optimizes its neural depth models on first use and caches per-camera
# factory calibration by serial. Both directories are mounted as named volumes at
# runtime so that work survives `docker compose down` -- but they must be writable
# by the non-root user first, or the optimization fails at startup.
RUN mkdir -p /usr/local/zed/resources /usr/local/zed/settings \
    && chown -R ${USER_UID}:${USER_GID} /usr/local/zed/resources /usr/local/zed/settings

ENV ROS_WS=/home/${USERNAME}/workspace

# Fixed, username-independent paths: an exec-form ENTRYPOINT performs no variable
# substitution, so it cannot reference ${USERNAME}.
COPY --chmod=755 entrypoint.sh /usr/local/bin/entrypoint.sh
COPY --chmod=644 ros-env.sh /usr/local/bin/ros-env.sh

USER ${USERNAME}
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Three entry paths into this container, and each needs the overlays sourced:
#   1. container's own process / `compose run`  -> ENTRYPOINT sources ros-env.sh
#   2. `compose exec ... bash` (interactive)    -> ~/.bashrc, appended below
#   3. `compose exec ... <non-interactive cmd>` -> $BASH_ENV, set here
#
# (3) is the one that is easy to miss. `docker exec` bypasses ENTRYPOINT entirely,
# and Ubuntu's stock ~/.bashrc opens with a `case $- in *i*) ;; *) return;; esac`
# guard -- so a non-interactive shell returns before ever reaching the appended
# source line. Without BASH_ENV, `docker compose exec middle-arm ros2 topic list`
# fails with "ros2: command not found" while the identical command typed inside
# an interactive `bash` works fine. ros-env.sh guards itself against repeat
# sourcing, which BASH_ENV would otherwise cause on every nested shell.
ENV BASH_ENV=/usr/local/bin/ros-env.sh

RUN echo "source /usr/local/bin/ros-env.sh" >> /home/${USERNAME}/.bashrc \
    && rosdep update --rosdistro ${ROS_DISTRO}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
