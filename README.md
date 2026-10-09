# Trossen rig

A mobile-ALOHA-style rig: a SLATE AGV base, two WidowX AI manipulators, a
WidowX-250 camera arm, and a Meta Quest headset to drive it all. Every
component runs in its own Docker container; they talk over ROS 2 topics.

This file is the runbook. Design and interface docs live in [docs/](docs/):
[topic-contract.md](docs/topic-contract.md) is the interface between
containers, [frames.md](docs/frames.md) the frame tree, and
[JETSON.md](docs/JETSON.md) what differs on the Orin. Each container directory
has its own README with the details for that piece of hardware.

```
Trossen/
├── Makefile             every command below; `make help` lists them all
├── docker-compose.yml   the whole rig, includes the per-container files
├── setup.sh             per-machine bootstrap, run once after cloning
├── manip-arm/           left-arm + right-arm (WidowX AI, Ethernet)
├── middle-arm/          camera arm (WidowX-250, USB)
├── slate-base/          AGV base + scissor lift (USB)
├── quest/               headset teleop
├── monitor/             watch topics, keyboard control, arm pose commands
└── sim/                 the rig drawn live in a browser
```

## 1. Clone and set up a machine

Prerequisites: Docker Engine with the Compose plugin (v2.20 or newer), git,
and your user in the `docker` group.

```bash
git clone --recursive <repo-url> Trossen      # --recursive pulls pyroki for the camera arm's IK
cd Trossen
./setup.sh                                    # writes .env with this machine's UID/GID
./middle-arm/host-setup/setup-host.sh         # udev rule -> /dev/ttyDXL
./slate-base/host-setup/setup-host.sh         # udev rule -> /dev/ttySLATE, removes brltty
```

If `setup.sh` reports you are not in the docker group, run
`sudo usermod -aG docker $USER` and log out and back in before continuing.

**The two manipulators need a static IP on the wired NIC** connected to the
Ethernet switch. Docker inherits it from the host and cannot set it itself.
Find the interface name with `ip -br link`, then:

```bash
sudo nmcli con add type ethernet ifname <nic> con-name trossen-arm \
  ipv4.method manual ipv4.addresses 192.168.1.1/24
sudo nmcli con up trossen-arm
ip -br addr show <nic>        # must say UP and 192.168.1.1/24
```

The left arm is 192.168.1.2 and the right arm 192.168.1.3. Every WidowX AI
ships on .2, so a replacement arm must be re-addressed before both are plugged
into the switch. The procedure is in
[manip-arm/README.md](manip-arm/README.md#two-arms-on-one-switch).

**For headset control** the quest container bind-mounts the gvlink protocol
library from the Unity repo. Clone it next to this one:

```bash
git clone -b v2 https://github.com/Soltanilara/av-aloha-unity.git ~/av-aloha-unity
```

**Then build.** This takes about half an hour on a desktop and several hours on
a Jetson; [docs/JETSON.md](docs/JETSON.md#build) covers building elsewhere and
pulling the images instead.

```bash
make build
```

Check the machine-specific pieces came out right:

```bash
make check      # .env present in every project, container UIDs match, ipc=host
```

## 2. Headset control

One command brings up every container, waits for the arm agents to connect,
moves all arms to their saved `start` pose, and then hands them to the headset:

```bash
make quest
```

**Watch the arms during the move to `start`.** The command waits 15 s for that
move before handing over; use `make quest POSE_WAIT=25` if the arms are far
from it. Ctrl-C ends teleop and releases the arms.

On the headset, pick the robot named `trossen` from the app's robot list. The
headset and the rig must be on the same network; the rig finds nothing across
a router.

First time, or after changing the mapping, run the chain without a headset
before trusting it:

```bash
make quest QUEST_BACKEND=sim          # a fake circle drives the arms, no headset needed
```

Controls:

| Input | Does |
| --- | --- |
| **A** (right, hold) | right arm follows the right controller |
| **X** (left, hold) | left arm follows the left controller |
| either held | camera arm follows your head |
| **Y** (tap) | look-around on and off: camera arm follows your head, hands free |
| **B** (double-tap) | every arm to `rest`, then the session ends |
| Index trigger | that arm's gripper, analog closing force |
| Left stick | drive and turn the base |
| Right stick fwd/back | scissor lift up and down |

Driving and the lift are locked out while an arm is engaged. Let go, drive,
re-engage. The full control reference and the transport backends are in
[quest/README.md](quest/README.md).

**Do not run headset and keyboard control at the same time.** Both publish to
the same command topics and the newest message wins, which looks like the arms
stuttering rather than like a conflict.

## 3. Keyboard control (legacy)

For driving the rig without a headset, or for moving arms one step at a time:

```bash
make up           # every container, if not already running
make tmux         # one tmux session: live status on the left, keyboard control on the right
```

Nothing moves until you enable it. In the control pane:

| Key | Does |
| --- | --- |
| `1` `2` `3` | enable left arm, middle arm, right arm |
| `0` | torque the base |
| `SPACE` | everything on or off (the panic key) |
| `q w e` / `a s d` | left arm +x +y +z / −x −y −z |
| `r t y` / `f g h` | middle arm |
| `u i o` / `j k l` | right arm |
| `z` `x`, `n` `m` | left and right gripper open, close |
| arrows | drive the base (dead-man: stops when released) |
| `,` `.` | lift down, up |
| `[` `]` | step size |
| `ESC` | quit |

`Ctrl-b d` detaches and leaves everything running; `tmux attach -t rig` comes
back. When you are done:

```bash
make kill         # ends the tmux session and the control processes inside the containers
```

`make key` runs the keyboard tool alone, without tmux, and `make debug` opens a
second session for joint-level control of one arm.

## 4. A session on the Jetson

Power the rig, give it a minute, then from your laptop:

```bash
ssh <user>@<jetson>
cd ~/Trossen
make up                       # or `make quest` to go straight to headset teleop
make status                   # which containers are up, one line per subsystem
```

`make up` starts every driver and agent. Two things it deliberately does not do:
the base's motors stay off until `make torque` or an enable from teleop, and the
quest container sits idle until `make quest` or `make tmux` starts teleop.

While it runs:

```bash
make watch                    # live table of every topic and its rate
make logs SVC=left-arm        # follow one container's output
make arms                     # where every arm is, and the poses it knows
make home EXECUTE=1           # every arm to its saved `home` pose, joint-space
make arm-stop                 # release every arm
make shell SVC=monitor        # a shell inside a container
```

Done for the day:

```bash
make kill                     # if a tmux session was open
make down                     # stop every container
```

Two hardware rules:

- **Never restart or stop the middle-arm container while the arm is up.** That
  kills its driver, torque drops, and the arm falls. To reload the agent code,
  restart only the python process inside the container.
- **The manipulator containers are safe to restart.** The agent re-takes a
  position hold, but `arm_agent.py` holds the arm's only connection, so the
  per-arm CLIs in `manip-arm/workspace/` cannot run while it is up.

After editing a script in any `workspace/` folder, `make restart SVC=<service>`
is enough; the folder is bind-mounted. After a Dockerfile change, `make rebuild
SVC=<service>`.

## When something is wrong

| Symptom | Cause and fix |
| --- | --- |
| `container name "/slate-base" is already in use` | Something was started from a subdirectory. Run `cd slate-base && docker compose down`, then `make up` from the root. Always run from the root. |
| `ros2 topic list` shows everything, but no messages arrive | A container is on a different UID or lacks `ipc: host`. `make check` names it; `make env` rewrites `.env`, then `make rebuild`. |
| Arm container up, driver times out | The wired NIC is DOWN or lost its address. `ip -br addr show <nic>`, then `sudo nmcli con up trossen-arm`. |
| Both arm containers move the same arm | Both arms are on 192.168.1.2. Re-address one, see manip-arm/README.md. |
| `permission denied ... /var/run/docker.sock` | Not in the docker group. `sudo usermod -aG docker $USER`, log out and in. |
| quest: `ImportError` from the gvlink backend | `~/av-aloha-unity` is missing. Clone it (section 1) and `make up`. |
| quest: `error gathering device information` for `/dev/videoN` | The stereo camera index differs per host. `export QUEST_CAMERA=/dev/video0` (video4 on the laptop) before `make up`. |
| Base ignores velocity commands | Motors are untorqued. `make torque`. |
| `ros2: command not found` on the host | `source ~/Trossen/env.sh`, or use the monitor container (`make watch`). |
