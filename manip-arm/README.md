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

## Driving the arms: the script stack

Same shape as `middle-arm/workspace` — small single-purpose CLIs, dry-run by
default, limits checked before anything is sent — but built on the `trossen_arm`
SDK instead of ROS 2 topics. Run them from inside either container; each one
talks to whichever arm that container's `ARM_IP` points at.

| Script | What it does |
| --- | --- |
| `./read_joints.py` | Positions, velocities, external effort. `--watch`, `--temps`, `--raw`. Read-only. |
| `./move_joint.py <joint> <value>` | One joint, absolute or `--rel`, `--deg`, `--clamp`. `--group` for all six. |
| `./pose.py list\|show\|go\|save\|delete` | Named poses. `go` is a dry run until `--execute`. |
| `./gripper.py open\|close\|grasp\|set\|status\|release` | Force- or position-controlled gripper. |
| `./teach.py` | Floats the arm under gravity compensation so you can hand-guide it, and saves poses from where you put it. |
| `./recover.py` | Walks a joint parked outside its limits back into range. See below. |
| `./tracking.py <joint> --from A --to B` | Sweeps a joint and measures where it actually lands, to separate a calibration offset from gravity sag, friction and creep. |
| `./both.py read\|list\|save\|go` | Holds both arms in one process and moves them together. A paired pose is the same name saved under both arms in `poses.yaml`, so `pose.py` still drives either half alone. |
| `./reboot-controller.py` | Restarts the controller (clears a latched error). Does not move the arm. |
| `./launch-rviz.sh` | RViz with the wxai model, waiting for `/joint_states`. Opens no arm connection. |
| `./preview.py pose\|values\|live` | Draws a pose in that RViz before you send it. See [Previewing a pose](#previewing-a-pose-before-you-send-it). |
| `./discover-arms.py`, `./set-arm-ip.py` | One-time addressing, see above. |

```bash
docker compose exec arm-1 bash

./read_joints.py                  # where is it now
./pose.py list                    # sleep / upright / ready, plus anything saved
./pose.py go ready                # dry run: prints every joint's delta
./pose.py go ready --execute      # actually move
./gripper.py close --execute      # squeeze at 20 N, stops on the object
./teach.py                        # hand-guide, name poses as you go
```

Three things about this hardware that change how the scripts are written, all of
them different from the DYNAMIXEL arm:

1. **`idle` is braked, not limp.** Losing the connection puts every joint into
   idle, and on a WXAI that means it holds position. So a script can exit — or
   be Ctrl-C'd mid-move — and the arm just parks where it is. Nothing sags,
   nothing needs re-torquing.
2. **One connection at a time.** The controller accepts a single driver, so
   `read_joints.py --watch` will not run alongside `teach.py`. That is why
   `teach.py` saves poses itself instead of leaving it to `pose.py`.
3. **The gripper is linear and force-native.** Openings are metres (0 closed,
   0.04 open), and `external_effort` mode takes newtons directly — positive
   opens, negative closes.

That last point is what replaces the ROS 1 gripper recipe on the DYNAMIXEL arms:

| ROS 1 / Interbotix (middle arm) | Here |
| --- | --- |
| `set_register(..., "Current_Limit", 100)` | `./gripper.py grasp --force 40` — newtons, no register writes |
| `robot_set_operating_modes("single", "gripper", "current_based_position")` | `driver.set_gripper_mode(Mode.external_effort)` |
| `robot_torque_enable("single", "gripper", True/False)` | `Mode.position`/`external_effort` vs `Mode.idle` (which still holds) |
| `JointSingleCommand(name="gripper", cmd=-1.5)` | `driver.set_gripper_position(0.0)` — metres, not radians |
| `reboot_gripper()` after a stall | `./gripper.py status`, then reconnect with `--clear-error` |
| `print_gripper_registers()` | `./gripper.py status` (opening, load, velocity) |

Poses live in `workspace/config/poses.yaml`, keyed by `ARM_NAME` (`arm-1` /
`arm-2`, set in `docker-compose.yml`). That file is bind-mounted into **both**
containers, so the key matters: the same pose name is a different point in space
for each arm, and sharing one entry would drive one arm to the other's target.
The three built-in poses (`sleep`, `upright`, `ready`) come from
`trossen_arm_moveit`'s SRDF, so they mean the same thing here as in MoveIt.

### "Joint limit exceeded" the instant you command anything

```text
[Motor Interface] Joint 2 position limit exceeded: expected in range
[-0.200000, 2.556195], motor reported -1.550889. Setting to idle.
```

Read the reported value, not the target. The controller **clips commands** to
`[position_min, position_max]`, but **faults on feedback** outside
`[position_min - tolerance, position_max + tolerance]` — and it skips those
checks entirely in idle. So an arm parked outside its own limits sits there
quietly, reads out fine, and then faults the moment any joint leaves idle,
before your target is ever considered. A different target will not help.

`pose.py` and `move_joint.py` now catch this before commanding. To get out:

```bash
./recover.py                      # names the joint, shows the move, sends nothing
./recover.py --execute            # widens that one limit, walks it back in, restores
./recover.py --ratchet --execute  # short bursts, when one long move faults partway
```

Observed on this rig: a widened limit does not survive a full move. The joint
travels ~0.15 rad and then the Motor Interface faults against the **original**
range again, as if its copy of the limits is refreshed underneath the widened
ones. The progress is real and repeatable, which is what `--ratchet` exploits —
reconnect, widen, move ~0.2 rad, disconnect, repeat, stopping by itself if a
burst fails to move the joint rather than straining it against an obstruction.

There is nothing to shut down between attempts. Every script here is one-shot:
it connects, acts, and disconnects. The container running is not the same as the
arm being held — only a running script holds it. `./reboot-controller.py
--execute` restarts the controller (clearing a latched error, resetting
non-EEPROM config) but does **not** move the arm, so it is not a way out of this
state on its own.

Joint limits are driver-session configuration (`get_joint_limits` /
`set_joint_limits`), not calibration, and are not kept in EEPROM — they reset to
the defaults on the next `configure()`. That is what makes widening one a safe
recovery step rather than a permanent change.

**Look at the arm first.** Widening is right when the joint really is parked
past its limit (folded for shipping, hand-posed while powered down). It is wrong
when the *reading* is bogus: if the reported angles do not match the shape in
front of you, the position offsets are off and moving to a "safe" number would
drive the joint into a hard stop. Fix the calibration in that case instead.

The reference shape is **folded**, not standing up. Links 2 and 3 are 0.264 m
and 0.245 m and run in opposite directions, so at all-zeros they nearly cancel
and the tool sits ~0.25 m out, ~0.16 m up — compact and close to the base. That
is `sleep`. (Earlier text here said "at all-zeros this arm stands straight up",
which is wrong and inverts the test.) A joint reading a clean fraction of π away
from zero *in that shape* is a homing error, not a parking problem.

### A joint that is out of range on purpose

Widening is a recovery step. If a joint on this rig sits outside the stock range
as its normal working position, put it in `JOINT_LIMIT_OVERRIDES` in
[`arm_config.py`](workspace/arm_config.py) instead:

```python
JOINT_LIMIT_OVERRIDES = {
    2: (-1.75, 2.356194),
    GRIPPER_INDEX: (-0.005, 0.04),
}
```

`arm.connect()` re-applies that on every connection and reads it back, warning
if the controller refuses. Since limits are session state, re-asserting per
connect is the only way to make a non-stock range stick — there is nowhere to
write it once. `check-config.py` deliberately connects with `overrides=False`
so its audit and its backup still describe the controller, not this table.

## Previewing a pose before you send it

`pose.py go <name>` prints a table of radians. That catches a typo'd number but
not a wrong *posture* — six angles that each look reasonable can still put the
elbow through the camera mount. RViz answers that in one glance.

Two shells into the same container:

```bash
docker compose exec arm-1 ./launch-rviz.sh     # shell 1: RViz, no arm connection
docker compose exec arm-1 bash                 # shell 2:
./preview.py pose home_watch                   #   draw the pose
./preview.py pose home_watch --sweep           #   read the arm, animate current -> pose
./preview.py values --deg 0 60 30 0 0 0        #   an arbitrary vector, no pose file needed
./preview.py live                              #   mirror the real arm at 10 Hz
```

Once it looks right, the normal path resumes: `./pose.py go home_watch` for the
dry-run table, then `--execute`.

`launch-rviz.sh` sources ROS itself, so it can be `exec`'d directly. `preview.py`
cannot — `docker compose exec` bypasses the entrypoint, and a bare `exec arm-1
./preview.py` gets no ROS environment and fails on `import rclpy`. Run it from
inside a shell (as above), or `docker compose exec arm-1 bash -lic './preview.py ...'`.

The image carries **only `trossen_arm_description`** — the URDF, the meshes, and
`display.launch.py` — built into `/home/robot/ros2_ws` at image build time and
sourced by the entrypoint. Not the rest of `trossen_arm_ros`: `trossen_arm_bringup`
brings `ros2_control` and a hardware interface that opens *the same single
connection to the controller* as these scripts, so having it built here would
invite running both and having neither work. Description is inert, so
`launch-rviz.sh` is safe to leave up while the scripts drive the real arm — which
is exactly what makes `./preview.py live` possible.

**What this is not.** No physics, no contact, no collision checking — the model
will pass straight through your bench. `--sweep` interpolates linearly in joint
space while the real controller runs its own profile over `goal_time`, so the
endpoints match and the middle is an approximation. It is good for "is that the
posture I meant" and for spotting a swing that fouls the camera cable; it is not
a guarantee about the path. Collision-aware planning is what MoveIt is for, and
that means the full stack in the next section.

`launch-rviz.sh --sliders` swaps in `joint_state_publisher_gui` if you want to
drag joints by hand instead. Don't run `preview.py` at the same time — both
publish `/joint_states` and the model jitters between them.

Two arms, two RViz windows: both containers share the host network stack and
`ROS_DOMAIN_ID`, so a second `launch-rviz.sh` in `arm-2` collides on node names
and topics with the first. Preview one arm at a time, or give the second a
distinct `ROS_DOMAIN_ID`.

## Can the Interbotix ROS packages drive these arms?

No — `interbotix_xsarm_control`, `interbotix_xs_sdk`, and the rest of that stack
are for the **X-Series DYNAMIXEL** arms: the middle arm's world, where `xs_sdk`
opens a U2D2 serial port, writes DYNAMIXEL EEPROM registers from a motor-config
YAML, and publishes `/joint_states`. Look at that launch file's `robot_model`
choices — `px100 … wx250s … vx300s` — there is no `wxai` in the list, because
these manipulators are not DYNAMIXEL arms at all. They have their own controller
box that speaks its own protocol over Ethernet, and nothing in
`interbotix_ros_manipulators` knows how to open that connection. Cloning it
gets you a build, not a moving arm.

The equivalent stack for these arms exists, from Trossen, and it is a separate
repo:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b humble https://github.com/TrossenRobotics/trossen_arm_ros.git
cd ~/ros2_ws
vcs import src < src/trossen_arm_ros/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

It gives you `trossen_arm_description` (URDF, RViz), `trossen_arm_bringup` (a
`ros2_control` hardware interface wrapping this same SDK, with a
`JointTrajectoryController` for the arm and a `GripperActionController` for the
gripper), and `trossen_arm_moveit`. Launch it with the IP as an argument:

```bash
ros2 launch trossen_arm_bringup trossen_arm.launch.py \
    robot_model:=wxai arm_variant:=base ip_address:=192.168.1.2
```

Worth adding when you want motion planning, collision checking, or trajectory
following — MoveIt is the reason to take on `ros2_control`. It is not worth it
for "send the arm to a named pose and close the gripper", which is what the
scripts above already do in one process and no launch files. Note also that the
bringup opens the same single connection to the controller, so it and the
scripts here are mutually exclusive on a given arm.

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
