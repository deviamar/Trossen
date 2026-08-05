# Trossen Arm — Dockerized Setup (manipulator arms)

Two services, `arm-1` and `arm-2`, one per WXAI manipulator. They share one image
and differ only in `container_name` and `ARM_IP`. For bringing these up alongside
the middle arm, see [Running all three arms](#running-all-three-arms) below.

## One-time host setup (outside Docker)

1. **Static IP on the Ethernet NIC connected to the switch.**
   This is a host-OS setting — Docker `--network host` inherits it, it cannot assign it.
   On this machine the wired NIC is `enp0s31f6`:

   ```bash
   sudo nmcli con add type ethernet ifname enp0s31f6 con-name trossen-arm \
     ipv4.method manual ipv4.addresses 192.168.1.1/24
   sudo nmcli con up trossen-arm
   ```

   Verify before going further — `ip -br addr show enp0s31f6` must print `UP`
   and `192.168.1.1/24`. A `DOWN` interface is the single most common cause of
   "the container starts fine but the driver times out".

   The Trendnet TEG-S*g is an unmanaged switch: no configuration, no VLANs, no
   web UI. Host NIC and both arms plug into any ports and land on one flat
   192.168.1.0/24 segment. If it has an "uplink"-labelled port, any port works
   anyway — auto-MDI/X handles the crossover.

2. **Allow the container to draw on your X11 display** (run once per login session):
   ```bash
   xhost +local:docker
   ```

3. **NVIDIA Container Toolkit** installed on host (needed for `--gpus`/GPU reservation to work):
   ```bash
   # if not already installed
   sudo apt install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```

## Build and run

```bash
cd manip-arm

# Once: pin the container user to your host user, so the ./workspace bind mount
# is writable from both sides. Compose reads .env automatically.
printf 'UID=%s\nGID=%s\n' "$(id -u)" "$(id -g)" > .env

docker compose build          # builds trossen-arm:latest once, shared by both services
docker compose up -d          # starts arm-1 AND arm-2
docker compose exec arm-1 bash
```

The `.env` file rather than `export UID GID`: bash marks `UID` read-only and
never defines `GID` at all, so both the export and the `UID=$(id -u) …` command
prefix fail. The compose defaults (`${UID:-1000}`) happen to be right on this
host, which is why it worked anyway — `.env` is what makes it correct on a host
where they are not 1000.

Once inside the container:
```bash
python3 -c "import trossen_arm; print(trossen_arm.__file__)"   # confirm python driver
find / -name "libtrossen_arm*" 2>/dev/null                     # confirm c++ driver installed
rviz2                                                            # should open on your host display
```

## Where the arm IP actually gets used

Docker Compose only gets `ARM_IP` *into* the container as an env var — it does not configure
any networking. Your Python/C++ code reads it and hands it to the Trossen SDK when opening
the connection, e.g.:

```python
import os
import trossen_arm

driver = trossen_arm.TrossenArmDriver()
driver.configure(
    model=trossen_arm.Model.wxai_v0,                              # match ARM_MODEL
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_base,    # required, no default
    serv_ip=os.environ["ARM_IP"],
    clear_error=True,
)
```

Because the same code reads `ARM_IP` in both containers and the two containers
set it differently, one unmodified script drives whichever arm it was started
in — that is the whole mechanism by which two identical containers control two
different arms.

## Running on macOS (Apple Silicon, e.g. M4 Pro)

The same Dockerfile builds natively for arm64 — no image changes needed. Three
runtime differences vs. the x86 Linux target, handled by `docker-compose.mac.yml`:

1. **No real host networking** on Docker Desktop for Mac (containers run in a VM).
   We fall back to default bridge networking, which is fine since the driver only
   makes outbound connections to the arm's IP — it doesn't need inbound/multicast.
2. **No NVIDIA GPU** on Apple Silicon — GPU reservation is removed; RViz2 falls
   back to software rendering (Mesa/llvmpipe). Works, just not GPU-accelerated.
3. **X11 via XQuartz**, not a shared Unix socket.

### One-time Mac host setup

```bash
brew install --cask xquartz
# Log out and back in after installing XQuartz (required once)
open -a XQuartz
```

In XQuartz → Settings → Security, enable **"Allow connections from network clients"**,
then restart XQuartz. Then, each session:

```bash
xhost + 127.0.0.1
```

### Build and run

```bash
cd manip-arm
docker compose -f docker-compose.yml -f docker-compose.mac.yml build
docker compose -f docker-compose.yml -f docker-compose.mac.yml up -d
docker compose -f docker-compose.yml -f docker-compose.mac.yml exec arm-1 bash
```

Inside the container, `rviz2` should pop up on your Mac's display via XQuartz.

### Moving to the x86 Linux box later

No changes needed to `docker-compose.yml` itself — just drop the `-f docker-compose.mac.yml`
override and run the plain command from the "Build and run" section above. That
restores `--network host` and NVIDIA GPU passthrough for the real deployment target.

## Two arms on one switch

**Every WXAI arm leaves the factory on 192.168.1.2.** Two of them on the same
switch is an address conflict, and it does not fail loudly: each container's
driver connects to whichever controller answers first, so both containers end
up commanding the same physical arm while the other never moves. Nothing in
Docker can fix this — the address lives in the arm controller's own config.

Do this once, with **only the arm being re-addressed plugged into the switch**;
with both connected there is no way to say which one you are talking to.

```bash
docker compose up -d arm-1
docker compose exec arm-1 bash

# inside the container:
./discover-arms.py                  # expect exactly one arm, at 192.168.1.2
./set-arm-ip.py 192.168.1.3         # writes manual IP to that controller
```

Power-cycle that arm — the controller reads the new address at boot. Then plug
the second arm in and confirm both are visible:

```bash
./discover-arms.py                  # expect 192.168.1.2 and 192.168.1.3
```

The setting is stored in the controller and survives power cycles, so this is a
one-time step per arm. `set-arm-ip.py` refuses to run when more than one arm
answers, which is the guard against re-addressing the wrong one.

Both compose services read their address from the environment if you want to
override without editing the file:

```bash
ARM_1_IP=192.168.1.2 ARM_2_IP=192.168.1.3 docker compose up -d
```

## Running all three arms

The two manipulators and the middle arm are separate Compose projects in
separate directories, so they are brought up with two commands, not one. They
coexist because all three use `network_mode: host`, the same `ROS_DOMAIN_ID=0`,
and the same `RMW_IMPLEMENTATION` — one DDS graph across all three containers.

The two transports are independent: the manipulators are reached over Ethernet
(`192.168.1.x` through the Trendnet switch), the middle arm over the U2D2's
micro-USB link at `/dev/ttyDXL`. Neither can interfere with the other, so the
order below is only about failing fast on the more fragile one.

```bash
# 0. Host prerequisites, once per boot / login session
ip -br addr show enp0s31f6      # must be UP with 192.168.1.1/24
ls -l /dev/ttyDXL               # must exist -> the U2D2 is connected
xhost +local:docker             # once per login session, for RViz

# 1. Middle arm (USB / DYNAMIXEL)
cd ~/Trossen/middle-arm
docker compose up -d
docker compose exec middle-arm ./launch-arm.sh --limp   # torque off, read-only first

# 2. Both manipulators (Ethernet)
cd ~/Trossen/manip-arm
docker compose up -d
docker compose exec arm-1 bash    # ARM_IP=192.168.1.2
# in another terminal:
docker compose exec arm-2 bash    # ARM_IP=192.168.1.3
```

Check all three are up:

```bash
docker ps --format '{{.Names}}\t{{.Status}}'
# middle-arm, trossen-arm-1, trossen-arm-2
```

And that they share one ROS 2 graph — from inside any of the three containers,
`ros2 node list` should show the middle arm's nodes:

```bash
docker compose exec arm-1 bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'
```

Seeing nothing there while `docker ps` shows all three running means a DDS
mismatch, not a wiring problem: check that `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION` match in both compose files.

### What host networking does and does not share

All three containers share the host's network stack. For DDS that is exactly
what you want — many participants on one host is normal. The cost is that there
is one port space: any node binding a fixed port can only run in one container
at a time, and two containers cannot both start, say, an RViz-side service on
the same port. Node *names* collide too, so if you ever launch the same ROS 2
launch file in both manip containers, give them distinct namespaces.

`./workspace` is bind-mounted into both manip containers at the same path, so
they share code. That is deliberate — one copy of the driver scripts, two arms.
Give each its own `volumes:` entry only if they need to diverge.

## Adding a third manipulator

Copy the `arm-2` service block in `docker-compose.yml`, changing `container_name`
and `ARM_IP` (e.g. `192.168.1.4`). It picks up everything else from the
`x-arm-common` and `x-arm-env` anchors. Re-address the new arm off the factory
`192.168.1.2` first, exactly as above.
