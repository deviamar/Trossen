# Trossen rig — Dockerized ROS 2 setup

Containers for a mobile-ALOHA-style rig. Each arm is its own image because the
arms genuinely do not share a software stack; they meet at ROS 2 Humble and talk
over DDS.

```
Trossen/
├── setup.sh          per-machine bootstrap — run once after cloning
├── manip-arm/        WidowX AI (wxai_v0) ×2 — trossen_arm SDK, Ethernet
└── middle-arm/       WidowX-250 6DOF (wx250s) + camera — Interbotix, USB serial
```

| | `manip-arm/` | `middle-arm/` |
|---|---|---|
| Arm | WidowX AI `wxai_v0` | WidowX-250 6DOF `wx250s` |
| Driver | `trossen_arm` SDK | `interbotix_xs_sdk` |
| Transport | Ethernet / IP | DYNAMIXEL over U2D2 USB serial |
| Host needs | static IP on the NIC | udev rule → `/dev/ttyDXL` |
| GPU | not required | only for the ZED variant |

All containers run `network_mode: host` with the same `ROS_DOMAIN_ID`, so they
form one DDS graph — `ros2 topic list` in any container sees every arm.

## Setting up on a new computer

**Prerequisites:** Docker Engine with the Compose plugin, and git. Nothing else —
ROS 2, the drivers, and every vendor stack are built into the images.

```bash
git clone <your-repo-url> Trossen
cd Trossen
./setup.sh                              # writes .env with this machine's UID/GID
```

Then per container. The middle arm needs one privileged step first, because udev
runs in the host kernel and a container can only see device nodes the host has
already created:

```bash
./middle-arm/host-setup/setup-host.sh   # udev rules (+ NVIDIA toolkit for ZED)
cd middle-arm
docker compose build                    # ~20-30 min, ~5 GB (arm-only variant)
docker compose up -d
docker compose exec middle-arm bash
```

Inside, verify before trusting it:

```bash
ros2 pkg list | grep interbotix     # arm stack present
ls -l /dev/ttyDXL                   # serial port visible
./launch-arm.sh --sim               # full stack, no hardware needed
```

See [middle-arm/README.md](middle-arm/README.md) for the arm's own docs —
launching, the gripperless configs, and the helper scripts.

## What transfers, and what doesn't

The images are the portable part. Everything below is per-machine and is the
reason `setup.sh` and `host-setup/` exist:

| Thing | Why it can't be in the image |
|---|---|
| **UID/GID** (`.env`) | The container user is built to match the host user so bind-mounted files aren't root-owned. Wrong on any machine where you aren't 1000. |
| **udev rules** | udev runs in the host kernel; a container only sees nodes the host already made. |
| **NVIDIA toolkit** | Wires the host GPU driver into the container runtime. Host-side by definition. |
| **`xhost +local:docker`** | Per login session, not persistent. Needed for RViz. |
| **Static IP on the NIC** | A host-OS setting. `--network host` inherits it; Docker cannot assign it. |

Also machine-specific, and worth checking rather than assuming:

- **The U2D2's FTDI serial.** The shipped udev rule symlinks *any* U2D2 to
  `/dev/ttyDXL`, which races if a second one is ever attached. Keying on the
  serial is documented at the bottom of
  [99-interbotix-udev.rules](middle-arm/host-setup/99-interbotix-udev.rules).
- **Motor configs describe YOUR arm.** `middle-arm/workspace/config/` encodes a
  wx250s with the gripper (DYNAMIXEL ID 9) removed and a camera in its place. An
  arm with its gripper still fitted needs the stock vendor configs instead.
- **Build reproducibility.** The vendor repos are cloned at branch `humble` and
  the ZED wrapper at `master`. Two machines built weeks apart can get different
  upstream commits. Pin to tags or SHAs once a setup is proven.

## Faster than rebuilding: push the image

`docker compose build` on a new machine re-clones and re-compiles the vendor
stacks (~30 min). If you are setting up several machines, build once and push to
a registry instead:

```bash
# once, on the machine that already built it
docker tag middle-arm:latest ghcr.io/<user>/middle-arm:latest
docker push ghcr.io/<user>/middle-arm:latest

# on each new machine — minutes instead of half an hour
docker pull ghcr.io/<user>/middle-arm:latest
docker tag ghcr.io/<user>/middle-arm:latest middle-arm:latest
cd middle-arm && docker compose up -d       # compose finds the local tag, skips build
```

The catch: the image bakes in the UID/GID it was built with. If the target
machine's user is not the same UID, either rebuild there or accept root-owned
files in `workspace/`.

## Known issue

The U2D2 on this rig has re-enumerated to a new `/dev/ttyUSB*` several times
during a session (roughly every 20–35 minutes under load). The udev symlink
follows correctly and the container sees the new device immediately, but the
running `xs_sdk` does not — it either keeps a dead descriptor and publishes
garbage (−π on every joint, absurd velocities) or exits outright. Relaunching
recovers it. Suspect the USB cable, its routing along a moving arm, or supply
sag near a hard stop before suspecting software. `robot_control.joint_states_look_valid()`
detects the garbage-data case programmatically.
