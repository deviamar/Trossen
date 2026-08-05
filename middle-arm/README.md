# Middle Arm — WidowX-250 6DOF (wx250s) + ZED, Dockerized

The active-vision head of the mobile ALOHA rig: one Interbotix `wx250s` arm plus the
ZED camera(s) mounted on it, in a single container. Separate image from the `wxai_v0`
manipulator container in `../manip-arm/` — see [Why a separate container](#why-a-separate-container).

```
middle-arm/
├── Dockerfile              two variants via BASE_IMAGE + WITH_ZED (see below)
├── docker-compose.yml      arm-only: host networking, /dev + X11 passthrough, no GPU
├── docker-compose.zed.yml  override adding the ZED SDK base image + GPU reservation
├── entrypoint.sh           sources ros-env.sh, then execs the command
├── ros-env.sh              the four-overlay source order, shared by entrypoint and .bashrc
├── host-setup/
│   ├── setup-host.sh       udev rules + NVIDIA toolkit + xhost (run once per machine)
│   └── 99-interbotix-udev.rules
└── workspace/              bind-mounted to ~/workspace in the container — your code lives here
```

## Why a separate container

Beyond your collaborator's call, the two arms genuinely do not share a software stack:

| | `../manip-arm/` (manipulators) | this one (middle arm) |
|---|---|---|
| Arm | WidowX AI `wxai_v0` | WidowX-250 6DOF `wx250s` |
| Driver | `trossen_arm` SDK | Interbotix `interbotix_xs_sdk` |
| Transport | Ethernet / IP | DYNAMIXEL over U2D2 USB serial |
| Host needs | static IP on the NIC | udev rule → `/dev/ttyDXL` |
| GPU | not required | **required** (ZED SDK is CUDA-only) |
| Base image | `ubuntu:22.04` | `stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu22.04` |

They share exactly one thing — ROS 2 Humble — and that is the layer they talk over.

## Where the ZED fits

The ZED SDK is CUDA-only, which is the single biggest constraint on this container.
That drives three consequences worth knowing up front:

1. **The base image is chosen by the camera, not the arm.** StereoLabs publishes images
   with CUDA and the ZED SDK preinstalled; building that by hand is a bad trade. The
   `gl-devel` variant is required (not plain `devel`) — it carries the OpenGL stack that
   RViz2 and the ZED tools need.
2. **Ubuntu 22.04 is forced.** Interbotix supports ROS 2 Humble only, and Humble only has
   apt binaries for jammy. That pins the base tag to `...-ubuntu22.04`.
3. **This container is not GPU-portable the way `../manip-arm/` is.** Any machine it moves to needs
   an NVIDIA GPU and the container toolkit. The manipulator container runs anywhere.

Inside, everything is one ROS 2 graph — the arm's joint states and the camera's image
topics are siblings, published by nodes in the same container.

## Host setup (once per machine)

```bash
./host-setup/setup-host.sh
```

Installs the Interbotix udev rules (→ `/dev/ttyDXL`), installs and verifies the NVIDIA
Container Toolkit, and runs `xhost +local:docker`. This is the irreducible per-machine
remainder — udev runs in the host kernel and the GPU driver lives on the host, so neither
can be baked into an image.

> **On this machine specifically:** the NVIDIA Container Toolkit is **not installed** —
> `docker info` shows only the `runc` runtime. The compose GPU reservation will fail with
> `could not select device driver "nvidia" with capabilities: [[gpu]]` until you run the
> script above. The GPU driver itself (580.173.02) is fine.

`xhost` is per login session; re-run it after a logout, or add it to `~/.bashrc`.

## Build and run

Two variants. **Build the arm-only one first** — it needs no GPU and no toolkit, so it
isolates "does the DYNAMIXEL chain work" from "does the ZED stack work". Debugging both
at once is what makes this setup miserable.

`export UID GID` matches the container user to yours for the `./workspace` bind mount.
On this machine you are already uid/gid 1000, which is the compose default, so it is a
no-op here — it matters when moving to a host where you are not 1000.

**Arm-only** (`ubuntu:22.04`, ~20-30 min, ~5 GB → `middle-arm:latest`):

```bash
export UID GID
docker compose build
docker compose up -d
docker compose exec middle-arm bash
```

**With ZED** (CUDA base, ~30-45 min, ~15 GB pull → `middle-arm:zed`). Requires
`host-setup/setup-host.sh` to have installed the NVIDIA toolkit first:

```bash
export UID GID
docker compose -f docker-compose.yml -f docker-compose.zed.yml build
docker compose -f docker-compose.yml -f docker-compose.zed.yml up -d
docker compose -f docker-compose.yml -f docker-compose.zed.yml exec middle-arm bash
```

The two variants build to **different image tags** on purpose, so an arm-only rebuild
cannot silently clobber the ZED image. They share one `container_name`, though — so
`down` the one you are not using before `up`-ing the other.

> **Running one-off commands from the host:** wrap them in a shell —
> `docker compose exec middle-arm bash -c 'ros2 topic list'`, not
> `docker compose exec middle-arm ros2 topic list`. The bare form fails with
> `executable file not found in $PATH`: `docker exec` bypasses `ENTRYPOINT` and gets
> only the image's static `ENV`, whereas the ROS overlays exist only once something
> sources `ros-env.sh`. `BASH_ENV` makes any bash — interactive or not — do that,
> but a bare `ros2` never starts a shell in the first place.

### Smoke tests, inside the container

Both variants:

```bash
ros2 pkg list | grep interbotix    # arm stack
ls -l /dev/ttyDXL                  # arm serial port visible (needs U2D2 plugged in)
```

ZED variant only:

```bash
ros2 pkg list | grep zed           # camera stack
nvidia-smi                         # GPU visible (fails => toolkit/reservation problem)
ZED_Explorer                       # enumerate attached ZED cameras
```

### Bring up the arm

**This arm has no gripper.** DYNAMIXEL ID 9 was physically removed and a camera
mounted in its place, so the stock `wx250s` configs do not describe it — they list
a 9th motor that never answers, and `xs_sdk` aborts at startup. Use the wrapper,
which passes the corrected configs:

```bash
./launch-arm.sh              # real hardware, no RViz
./launch-arm.sh --rviz       # real hardware + RViz
./launch-arm.sh --sim        # DYNAMIXEL simulator, no hardware needed
./launch-arm.sh --limp       # real hardware, TORQUE OFF -- read-only bring-up
```

Work up in that order the first time. `--sim` proves the configs parse and the
stack launches with no hardware attached at all; `--limp` proves the real
DYNAMIXEL chain enumerates and streams live encoder values without energizing
anything; only then does the plain form add torque. Each step isolates one
failure mode instead of presenting all of them at once.

It lives in the bind-mounted workspace, so it is editable from the host at
`workspace/launch-arm.sh` and survives image rebuilds. What it expands to:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:=wx250s \
  robot_name:=middle \
  motor_configs:=/home/robot/workspace/config/wx250s_nogripper.yaml \
  mode_configs:=/home/robot/workspace/config/modes_nogripper.yaml \
  use_gripper:=false show_gripper_bar:=false show_gripper_fingers:=false \
  use_rviz:=false
```

Both halves matter, and they fail differently: without `motor_configs` the driver
aborts on the missing 9th motor; without the three gripper flags the driver runs
fine but `robot_description` still carries gripper links, so RViz and TF render a
gripper that is not on the robot.

After the first successful run against real hardware, `export LOAD_CONFIGS=false`
— the register values live in each motor's EEPROM and survive power cycles, so
rewriting them every launch only burns write cycles and startup time.

**The arm holds station on launch.** `position` mode with `torque_enable: true`
seeds each goal from the present encoder reading, so `xs_sdk` locks the arm
wherever it physically is and commands no motion. Nothing homes or sleeps it —
`sleep_positions` is only used when you call `go_to_sleep_pose()` yourself. To
let it be back-driven by hand:

```bash
ros2 service call /middle/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
  "{cmd_type: 'group', name: 'arm', enable: false}"
```

### Reading and commanding

Three helper scripts in `workspace/`, all usable from inside the container:

| Script | What it does |
|---|---|
| `read_joints.py` | read-only; `--watch` for a live table, `--raw` for piping |
| `move_joint.py` | single command; **dry-run by default**, `--execute` to send, refuses out-of-range targets |
| `teleop_keyboard.py` | arrow-key teleop; needs an interactive tty |
| `pose.py` | named poses — `save` captures the live pose, `go` recalls it |
| `arm_config.py` / `robot_control.py` | ROS 2 port of the stationary-ALOHA arm layer (see below) |

The last two are a ROS 2 Humble port of the ROS 1 `arm_config.py`/`robot_control.py`
from the stationary-ALOHA rig. Translation notes live in their docstrings; the
load-bearing ones are `bot.dxl` → `bot.core`, `rospy.init_node` →
`robot_startup()`, and `interbotix_xs_modules.arm` →
`interbotix_xs_modules.xs_robot.arm`. Poses are validated against this arm's
limits before anything is sent — the upstream `REST` pose is unreachable on a
wx250s and parks the arm against two mechanical stops.

Teleop must run on a real terminal, so attach interactively rather than passing a
command through `exec -T`:

```bash
docker compose exec middle-arm bash
./teleop_keyboard.py
```

Arrows drive the two spatial joints (up/down = `shoulder`, left/right = `waist`);
`w/s` `a/d` `r/f` `t/g` cover elbow, wrist_angle, wrist_rotate and forearm_roll;
`[`/`]` change step size; space resyncs the target to where the arm actually is;
ESC quits. It drops the motion profile to 300 ms for responsiveness and restores
the configured 2000 ms on exit, and it leaves torque **on** when it exits —
dropping torque would let the arm fall.

`r/f` and `t/g` drive the two roll axes that carry the camera. Check cable slack
before using them.

### The gripperless configs

`workspace/config/` holds copies of the two vendor configs, kept there rather
than edited in place under `/opt` so an image rebuild cannot silently revert them:

| File | Differs from stock how |
|---|---|
| `wx250s_nogripper.yaml` | `gripper` dropped from `joint_order`; `sleep_positions` trimmed to 6; `grippers:` block removed; motor ID 9 removed |
| `modes_nogripper.yaml` | `singles: gripper:` block removed |

Motors 1-8 and the shoulder/elbow shadow-calibration setup are untouched.

> **Not modeled:** the camera that replaced the gripper is absent from the URDF,
> so the wrist's mass and inertia are wrong and there is no TF frame for the
> optical center. Irrelevant for joint control and teleop; it matters as soon as
> you want to transform camera observations into the arm's frame, which for an
> active-vision head you eventually will. Add a fixed joint off `wrist_link` with
> the measured offset when you get there.

### Bring up the camera

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm \
  camera_name:=middle_cam
```

First camera launch is slow — the SDK optimizes its neural depth models for your specific
GPU. That work is cached in the `zed-resources` volume and does not repeat.

## How this fits the whole rig

Every container uses `network_mode: host` and the same `ROS_DOMAIN_ID`, so they form one
DDS graph. `ros2 topic list` in any container sees all the others — that is the integration
surface, and it is the reason separate containers cost you nothing architecturally.

```
        ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
        │  trossen-arm-1   │  │   middle-arm     │  │  slate-base      │
        │  wxai_v0 ×2      │  │  wx250s + ZED    │  │  (not built yet) │
        │  Ethernet        │  │  USB + GPU       │  │  USB serial      │
        └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
                 └─────────────────────┼─────────────────────┘
                       host network · ROS_DOMAIN_ID=0 · DDS
```

**One thing to change in `../manip-arm/docker-compose.yml`:** add
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to its `environment` block. It is currently unset
there and set here. Today both default to the same middleware so it happens to work, but
an RMW mismatch produces a silent no-discovery failure that presents as a network problem
and is genuinely unpleasant to debug. Pinning it in both costs one line.

The SLATE base is the remaining piece. Its ROS 2 packages ship inside
`interbotix_ros_core` (which this image already clones) under
`interbotix_ros_slate/trossen_slate`, currently `COLCON_IGNORE`d. It can be its own
container on the same pattern, or folded into this one.

## Notes and caveats

- **VRAM.** This machine's GPU is an RTX 500 Ada with **4 GB**. That is tight for the ZED
  SDK: one camera in `NEURAL` depth mode fits, but a second concurrent camera in that mode
  likely will not. If you are running two ZEDs, plan on `NEURAL_LIGHT` or `PERFORMANCE`
  depth mode, and check `nvidia-smi` under real load before committing to a config.
- **Which ZED model?** `ZED_CAMERA_MODEL` defaults to `zedm` (the ZED Mini, as in
  AV-ALOHA). If these are **ZED X** cameras, they will not work on this machine at all —
  ZED X is GMSL2, needs a capture card, and is Jetson-only. Worth confirming before the
  first build.
- **`privileged: true`** is StereoLabs' documented requirement, not laziness: the SDK
  enumerates the camera over raw USB, and the device re-enumerates at a new bus address
  on every camera reset, so a static `devices:` list goes stale the first time the camera
  hiccups. The tighter alternative is noted in `docker-compose.yml`.
- **Reproducibility.** `ZED_WRAPPER_REF` is `master` (set in `docker-compose.zed.yml`) and
  the Interbotix repos are cloned at branch `humble`. Once the setup works, pin both to tags
  or SHAs so a rebuild on another machine reproduces this one instead of tracking upstream
  HEAD.
- **`rosdep update` runs in the Interbotix build step, not only the ZED one.** It looks
  redundant with the ZED block, but that block is skipped entirely when `WITH_ZED=false`.
  Without it the arm-only variant would `rosdep init` with no cache, fail `rosdep install`
  into a `|| true`, and then colcon-build against absent deps — `ros-humble-desktop` does
  not carry `controller_manager` / `joint_trajectory_controller` / `hardware_interface`.
- **Vendor stacks build into `/opt`, not `~/workspace`.** `~/workspace` is a bind mount at
  runtime; anything built there during the image build would be shadowed by the host
  directory the moment the container starts. Your own packages go in `workspace/src/` and
  overlay the vendor stacks automatically.
- **MoveIt is off** (`WITH_MOVEIT=false`) — an active-vision arm is normally driven by
  direct joint commands, and MoveIt roughly doubles the build. Flip the build arg if you
  need `interbotix_xsarm_moveit`.

## References

- [Interbotix X-Series ROS 2 setup](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- [interbotix_ros_manipulators (humble)](https://github.com/Interbotix/interbotix_ros_manipulators/tree/humble)
- [zed-ros2-wrapper docker README](https://github.com/stereolabs/zed-ros2-wrapper/tree/master/docker)
- [stereolabs/zed image tags](https://hub.docker.com/r/stereolabs/zed/tags)
- [SLATE base ROS 2](https://docs.trossenrobotics.com/slate_docs/getting_started/ros_interface/ros2.html)
