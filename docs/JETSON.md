# Running the rig on a Jetson Orin

Written for whoever is doing the port — assumes you already know the rig from
the top-level [README](../README.md) and just need what differs on aarch64.

The short version: **the containers are architecture-clean and build on a Jetson
unchanged**, with one exception (the ZED camera image) that has its own compose
overlay. What follows is the checking that produced that claim, then the steps.

## Before anything: JetPack 6.x, not 7

Check the device:

```bash
cat /etc/nv_tegra_release
dpkg -l | grep nvidia-jetpack
```

JetPack 6 is Ubuntu 22.04 (jammy). That is not a preference — ROS 2 Humble
publishes apt binaries for jammy only, every image here installs Humble from
apt, and the two ROS images bake a `PYTHONPATH` naming `python3.10`. **JetPack 7
is Ubuntu 24.04 and will not work** without moving the whole rig to a newer ROS
distro, which is a much larger job than this port.

## What was verified for arm64

| Piece | Status |
| --- | --- |
| `ubuntu:22.04`, `ros:humble-ros-core` | multi-arch; both publish arm64 |
| `trossen-arm` (WXAI arms) | PyPI ships `manylinux_2_26_aarch64` wheels |
| `jaxlib` / `jax[cpu]`, pyroki, jaxls | linux-aarch64 wheels; pyroki and jaxls are pure Python |
| quest deps (`av`, `numba`, `opencv-python-headless`, `scipy`, `cryptography`) | all publish linux-aarch64 wheels |
| SLATE base driver | vendor ships `lib/aarch64/libchassis_driver.so` and selects on `CMAKE_SYSTEM_PROCESSOR` |
| Interbotix `xs_sdk`, trossen_arm C++ SDK | built from source, no arch assumption (this image does **not** run `xsarm_amd64_install.sh`) |
| ROS apt packages used here | `ros-humble-desktop` and friends all have arm64 builds for jammy |

Two things were actually wrong and are now fixed:

- `quest/Dockerfile` and `monitor/Dockerfile` hardcoded
  `LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu`. On arm64 the directory
  is `aarch64-linux-gnu`, and the wrong value is not a warning — rclpy fails to
  load its C extension and every node dies at `import rclpy`. Both triplets are
  now listed; a nonexistent entry is ignored.
- The ZED overlay's base image is desktop-CUDA x86-only. See below.

## Build

```bash
git clone --recursive <your-repo-url> Trossen   # --recursive: pyroki, or the middle arm dies at `import pyroki`
cd Trossen
./setup.sh                                      # .env with this machine's UID/GID
./middle-arm/host-setup/setup-host.sh           # udev -> /dev/ttyDXL
./slate-base/host-setup/setup-host.sh           # udev -> /dev/ttySLATE; offers to remove brltty
make build                                      # every container, arm-only middle-arm variant
```

**Expect this to take hours, not the half hour it takes on the tensorbook.** The
middle-arm and slate-base images compile vendor ROS workspaces from source, and
an Orin Nano has a fraction of the cores. Two ways to avoid it:

- Build on the x86 machine with `docker buildx build --platform linux/arm64`
  (needs qemu: `docker run --privileged --rm tonistiigi/binfmt --install arm64`)
  and push to a registry, then pull on the Jetson. Emulated builds are slow too,
  but they are slow on a machine you are not waiting at.
- Or build natively once and `docker save`/`docker load` the tarballs to any
  other Jetson, exactly as the README's "push the image" section describes.

Everything else — the udev rules, the static `192.168.1.1/24` NIC address for
the WXAI arms, `secrets/`, and the `av-aloha-unity` clone the gvlink backend
bind-mounts — is per-machine and transfers no differently here than between two
x86 boxes. See **What transfers, and what doesn't** in the main README.

## The ZED camera

Use [`middle-arm/docker-compose.zed-jetson.yml`](../middle-arm/docker-compose.zed-jetson.yml),
not `docker-compose.zed.yml`:

```bash
cd middle-arm
docker compose -f docker-compose.yml -f docker-compose.zed-jetson.yml build
docker compose -f docker-compose.yml -f docker-compose.zed-jetson.yml up -d
```

It differs from the desktop overlay in three ways, all forced:

- **Base image.** `stereolabs/zed:*-gl-devel-cuda*-ubuntu22.04` has no arm64
  manifest. Jetson uses the L4T images, which carry the ZED SDK built against
  the CUDA that JetPack provides. The default is `5.4-devel-jetson-jp6.2.2`;
  **the tag must match the host's JetPack** or the SDK reports no CUDA device at
  runtime. Override without editing the file: `ZED_JETSON_TAG=... docker compose ...`.
- **`runtime: nvidia`** instead of a `deploy.resources.reservations.devices`
  block. On Jetson the GPU is the SoC, not a discrete device the toolkit
  enumerates; the desktop-style reservation fails with *could not select device
  driver*.
- **Image tag `middle-arm:zed-jetson`**, so the three variants (plain, desktop
  ZED, Jetson ZED) cannot overwrite each other.

## Two hardware limits worth knowing before you commit

Neither is a software problem, and both bite this rig specifically.

- **The Orin Nano has no hardware video encoder.** NVENC was cut from the Orin
  Nano (the Orin NX has one). The gvlink headset path H.264-encodes two eyes at
  30 fps, which becomes a CPU software encode here. The Orin NX is pin-compatible
  with the same developer-kit carrier, so this is a module swap, not a redesign.
- **The developer kit exposes two MIPI CSI connectors.** Enough for a stereo
  pair, not for the six-camera setup. More cameras means either GMSL2
  aggregation with virtual channels — which needs a third-party carrier, not the
  dev kit — or USB3/GigE for the rest. Beware the cheap "multi-camera adapter"
  mux boards: they switch electrically, so only one camera streams at a time.
