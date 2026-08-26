# Trossen rig — Dockerized ROS 2 setup

Containers for a mobile-ALOHA-style rig. Each component is its own image because
they genuinely do not share a software stack; they meet at ROS 2 Humble and talk
over DDS.

```
Trossen/
├── docker-compose.yml   the whole rig — `include:`s the five below
├── setup.sh             per-machine bootstrap — run once after cloning
├── docs/
│   ├── ROADMAP.md             what is built, what is next, in order
│   ├── frames.md              every frame defined, + what to measure
│   ├── topic-contract.md      the ONLY interface between containers
│   └── topics-by-container.md which container owns which topic
├── third_party/pyroki/  submodule — JAX kinematics, for the active-vision arm's IK
├── manip-arm/           WidowX AI ×2 — left-arm + right-arm, trossen_arm SDK, Ethernet
├── middle-arm/          WidowX-250 + camera — Interbotix, USB serial
├── slate-base/          SLATE AGV — drive (x, y) + lift (z), USB serial
├── quest/               Meta Quest teleop source — no hardware but a socket
└── monitor/             watch every topic, smoke-test the base
```

## Starting everything

```bash
docker compose build       # first time, or after any Dockerfile change
docker compose up -d       # every container, every node, every topic live
docker compose ps
docker compose down
```

**`up -d` is all of it.** Each container autostarts its own nodes (`start.sh`),
so when that command returns the drivers are running, the agents are listening
and every topic in
[docs/topics-by-container.md](docs/topics-by-container.md) is being published.
Check with:

```bash
docker compose exec monitor ./watch.py
```

Two things it does **not** do, both deliberate: the base's motors stay untorqued
(`./base_ctl.py torque on`), and the Quest starts on the `sim` backend rather
than reaching for your headset.

Set `AUTOSTART=false` on a service for a bare shell instead. You need that for
the manipulator CLIs — `arm_agent.py` holds the arm's only connection, so
`pose.py` and friends cannot run beside it.

Each project still works on its own — `cd slate-base && docker compose up -d`
means exactly what it always did. The top-level file composes them, it does not
replace them.

## Day to day

### Which command does what

| You changed | Do this |
|---|---|
| A script in a `workspace/` folder | `docker compose restart <service>` — the folder is bind-mounted, so there is nothing to rebuild; the container just has to restart the node that imported the old copy |
| A `Dockerfile` or `requirements.txt` | `docker compose up -d --build <service>` |
| A `docker-compose.yml` | `docker compose up -d <service>` — Compose recreates what the file changed |
| Nothing; you just want it running | `docker compose up -d` |

`--build` rebuilds and recreates in one step. There is no need to `down` first
and no need to delete anything — Compose replaces the container in place.

### "no such service: monitor"

You are in a subdirectory. `~/Trossen/slate-base` is the **slate-base project**
and it contains one service; `monitor`, `left-arm` and the rest are different
projects. Run multi-service commands from the repo root:

```bash
cd ~/Trossen
docker compose up -d slate-base monitor
```

### "container name is already in use"

```
Conflict. The container name "/slate-base" is already in use
```

This is not a broken container. Every service here has an explicit
`container_name:`, which is **global to the Docker daemon rather than scoped to
a Compose project** — that is what lets you type
`docker compose exec slate-base` instead of `trossen-slate-base-1`. The cost is
that the root project and a per-directory project cannot both have their
containers up at once.

You have hit it because something is already running from `slate-base/`. Bring
that down, then start from the root:

```bash
cd ~/Trossen/slate-base && docker compose down
cd ~/Trossen && docker compose up -d
```

Or remove the stray container directly, if you no longer know which project made it:

```bash
docker rm -f slate-base
```

**Pick one place to run `up` from and stay there.** The root is the better
default now that every container autostarts; the per-directory files exist so a
single subsystem can be worked on alone.

### `ros2: command not found` on the host

Inside the containers ROS is always sourced — entrypoint, `~/.bashrc` and
`$BASH_ENV` all do it, so `exec <svc> bash` and `exec <svc> bash -lc '...'` both
work. On the **host** nothing has sourced it:

```bash
source ~/Trossen/env.sh     # ROS 2 + ROS_DOMAIN_ID=42 + the pinned RMW
ros2 topic list
```

To have it in every shell:

```bash
echo 'source ~/Trossen/env.sh' >> ~/.bashrc
```

This works at all because every container uses `network_mode: host` — your shell
and the containers share one network stack, so host-side `ros2` tools see
container topics directly. The domain ID and RMW in `env.sh` must match the
compose files; if they do not you get an empty topic list and no error at all.

If the host has no ROS installed, use the monitor container instead — it exists
for exactly this:

```bash
docker compose exec monitor ./watch.py
```

### Do I still need `exec`?

For starting things, no — that is what autostart replaced. `docker compose up -d`
brings up every node and every topic; nothing has to be launched by hand.

`exec` is only for getting a shell or a terminal tool inside a container that is
already running:

```bash
docker compose logs -f slate-base                    # what the nodes are printing
docker compose exec monitor ./watch.py               # a live view of every topic
docker compose exec slate-base bash -lc './base_ctl.py torque on'
docker compose exec quest bash                       # interactive, for keyboard_teleop
```

The things that genuinely need it are the interactive ones — `watch.py`,
`keyboard_teleop.py`, anything reading single keypresses — because they need a
terminal attached, which a background service does not have.

## How the containers talk

**Only ROS topics, only standard message types.** No shared volume, no shared
Python module, no custom `.msg` package, no service call from one component into
another. The full interface is [docs/topic-contract.md](docs/topic-contract.md).

That constraint is the point. A custom message package would have to be built
into every image that touches it, so one interface change would force a
simultaneous rebuild of all six — precisely the coupling this layout exists to
avoid. Standard types cost some readability and buy the ability to rebuild any
one container, in any language, while the rest keep running.

Two consequences worth stating:

- **Components never know who is driving them.** `/slate/cmd_vel_teleop` is
  owned by the base; the Quest is one possible publisher. Swap it for a gamepad
  or a policy and no robot container changes.
- **Safety lives with the hardware, not the client.** The base's velocity clamp
  is in `slate-base/workspace/governor.py`, inside the container that owns the
  serial port, because an input device is now something you can rebuild or get
  wrong. The vendor driver does not clamp at all — see the topic contract.

## Teleoperating

```bash
# one shell each -- these are long-running nodes, not CLIs
docker compose exec left-arm   bash -lic './arm_agent.py'
docker compose exec right-arm  bash -lic './arm_agent.py'
docker compose exec middle-arm bash -lc  './launch-arm.sh'         # driver first
docker compose exec middle-arm bash -lc  './head_agent.py --urdf /tmp/wx250s.urdf'
docker compose exec slate-base bash -lc  './governor.py'           # base safety layer
docker compose exec quest ./launch-quest.sh --backend sim          # start here
```

Then hold **A** for the right arm, **X** for the left, either to move the camera
arm with your head, triggers for the grippers, thumbsticks to drive. Swap to
`--backend webrtc` once the chain looks right on `sim`. Full controls and
bring-up order in [quest/README.md](quest/README.md).

The camera arm needs a URDF for its solver, generated from the same xacro the
driver uses:

```bash
docker compose exec middle-arm bash -lc './launch-arm.sh --dump-urdf' > /tmp/wx250s.urdf
```

`arm_agent.py` holds an arm's single SDK connection, so `pose.py`,
`read_joints.py` and `teach.py` cannot run against that arm while it is up.
That is the controller's rule, not a design choice.


| | `manip-arm/` | `middle-arm/` | `slate-base/` |
|---|---|---|---|
| Hardware | WidowX AI `wxai_v0` ×2 | WidowX-250 6DOF `wx250s` | SLATE AGV |
| Driver | `trossen_arm` SDK | `interbotix_xs_sdk` | `interbotix_slate_driver` |
| Transport | Ethernet / IP | DYNAMIXEL over U2D2 (FTDI `0403:6014`) | CH340 `1a86:7523` |
| Host needs | static IP on the NIC | udev rule → `/dev/ttyDXL` | udev rule, no `brltty` |
| GPU | not required | only for the ZED variant | not required |
| Builds on | any architecture | amd64 / arm64 | amd64 / arm64 only |

All containers run `network_mode: host` with the same `ROS_DOMAIN_ID`, so they
form one DDS graph — `ros2 topic list` in any container sees every other.

The base is the one component that can drive itself into something, so its
container namespaces every topic under `/slate` rather than leaving `/cmd_vel`
at the graph root, and its scripts clamp velocity client-side. See
[slate-base/README.md](slate-base/README.md) for why the driver's own clamp does
not apply on the ROS path.

## Setting up on a new computer

**Prerequisites:** Docker Engine with the Compose plugin, and git. Nothing else —
ROS 2, the drivers, and every vendor stack are built into the images.

```bash
git clone <your-repo-url> Trossen
cd Trossen
git submodule update --init --recursive # pyroki, for the middle arm's IK
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

The mobile base needs its own privileged step, for the same udev reason plus one
more: Ubuntu's `brltty` claims the base's USB-serial chip as a braille display
and takes the port away seconds after it enumerates.

```bash
./slate-base/host-setup/setup-host.sh   # udev rules; offers to remove brltty
cd slate-base
docker compose build                    # ~10-15 min
docker compose up -d
docker compose exec slate-base bash
```

```bash
ros2 pkg list | grep slate          # driver present
ls -l /dev/ttySLATE                 # base connected and powered
./launch-base.sh                    # no --sim equivalent; needs real hardware
./base_ctl.py torque on             # the base ignores /cmd_vel until you do this
./read_base.py                      # position, battery, E-stop
```

**The base powers on with its motors released** and silently ignores every
velocity command until `torque on` — that catches everyone once.

Start with **[slate-base/README.md](slate-base/README.md)**, which opens with a
step-by-step quick start, a command reference and a troubleshooting table. The
rest of it covers the full ROS interface (which the vendor documentation does
not list at all), the 300 ms `/cmd_vel` deadline, how to read the E-stop, and
the duplicate-driver failure that is the one genuinely confusing thing about
this container.

## What transfers, and what doesn't

The images are the portable part. Everything below is per-machine and is the
reason `setup.sh` and `host-setup/` exist:

| Thing | Why it can't be in the image |
|---|---|
| **UID/GID** (`.env`) | The container user is built to match the host user so bind-mounted files aren't root-owned. Wrong on any machine where you aren't the same UID — this one is 1003, not 1000. |
| **udev rules** | udev runs in the host kernel; a container only sees nodes the host already made. |
| **NVIDIA toolkit** | Wires the host GPU driver into the container runtime. Host-side by definition. |
| **`xhost +local:docker`** | Per login session, not persistent. Needed for RViz. |
| **Static IP on the NIC** | A host-OS setting. `--network host` inherits it; Docker cannot assign it. |
| **Removing `brltty`** | A host package whose udev rule steals the SLATE base's CH340 port. Nothing inside a container can stop it. |

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
