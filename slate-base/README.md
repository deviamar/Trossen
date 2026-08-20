# SLATE Mobile Base — Dockerized ROS 2 Setup

The drive base of the mobile ALOHA rig: a Trossen SLATE AGV, driven by the
Interbotix `interbotix_slate_driver` ROS 2 node. Separate container from the two
arm images for the same reason they are separate from each other — different
driver, different transport, nothing shared but ROS 2 Humble.

```
slate-base/
├── Dockerfile              ubuntu:22.04 + ROS 2 Humble + interbotix_slate_driver
├── docker-compose.yml      host networking, /dev + udev + X11 passthrough
├── entrypoint.sh           sources ros-env.sh, then execs the command
├── ros-env.sh              the three-overlay source order
├── host-setup/
│   ├── setup-host.sh       udev rules + brltty removal (run once per machine)
│   └── 99-trossen-slate.rules
└── workspace/              bind-mounted to ~/workspace — your code lives here
```

---

# Quick start

Everything below has been run against a real SLATE. Follow it in order the first
time — each step proves one thing, so when something breaks you know what.

## 1. Host setup — once per machine

```bash
cd ~/Trossen
./setup.sh                              # writes .env with this machine's UID/GID
./slate-base/host-setup/setup-host.sh   # udev rules; offers to remove brltty
```

Plug the base into USB and switch it on, then check the host can see it:

```bash
lsusb | grep 1a86:7523      # the base's CH340 USB-serial chip
ls -l /dev/ttySLATE         # the udev rule fired
```

Both must print something. If `lsusb` finds it but `/dev/ttySLATE` is missing,
the udev rule did not take — re-run `setup-host.sh`.

## 2. Build — once per machine

```bash
cd ~/Trossen/slate-base
docker compose build        # ~15 min
docker compose up -d
```

If Docker says `permission denied ... /var/run/docker.sock`, either put yourself
in the `docker` group (`sudo usermod -aG docker $USER`, then log out and back
in — note this is roughly equivalent to root) or prefix every `docker` command
with `sudo`.

## 3. Start the driver

**Always exactly one driver.** Run it detached with a log file, so it survives
the terminal closing and you keep the log either way:

```bash
cd ~/Trossen/slate-base
docker compose exec -d slate-base bash -lc './launch-base.sh > ~/workspace/driver.log 2>&1'
sleep 2 && cat workspace/driver.log
```

You want to see:

```
Initalized base at port: '/dev/ttyUSB0'.       <- upstream's typo, not yours
Base version: 'v2.1-3-g019c974'.
```

`FATAL Failed to initialize base port` means it never opened the port — and the
node **keeps running anyway**, so "the process is up" is not success.

## 4. Enable the motors

The base powers on with its drive motors released: it rolls freely and ignores
all velocity commands. Nothing moves until you do this:

```bash
docker compose exec slate-base bash        # you are now inside the container
./base_ctl.py torque on
```

## 5. Drive it

Everything from here runs **inside the container**, from `~/workspace`.

```bash
./read_base.py                        # where it is, how charged, E-stop state
./move_base.py forward 0.5            # prints the plan, sends nothing
./move_base.py forward 0.5 --execute  # actually move
./teleop_keyboard.py                  # arrow-key driving
```

**Keep a hand near the E-stop.** Every command that moves the base is a dry run
until you add `--execute`; `stop` is the exception and never needs it.

---

# Testing without the base

`sim_base.py` replaces the driver: no serial port, no hardware, odometry
integrated from whatever you command. Everything above it — governor, lift,
`move_base.py`, the monitor — is unchanged and cannot tell the difference.

```bash
# in slate-base/docker-compose.yml:  SLATE_SIM=true
docker compose up -d slate-base
docker compose exec monitor ./drive_test.py --execute
docker compose exec monitor ./watch.py
```

It reproduces the three things that actually catch people, which is the only
reason it beats a print statement:

- **Velocity expires.** A command older than 300 ms is dropped. Publish once and
  it moves for 300 ms, exactly like the real one.
- **Torque gates everything.** It starts released and silently discards
  `/cmd_vel` until `./base_ctl.py torque on` — no error, no warning, same as the
  hardware. (The autostart path passes `--torque-on`, since in sim that gate is
  just an obstacle once you have seen it work.)
- **It does not clamp.** Whatever reaches `/slate/cmd_vel` is acted on, because
  the real `cmd_vel_callback()` does not go through `set_cmd_vel()`. Bypass the
  governor and the sim will happily "drive" at 5 m/s. That is the point: it
  proves whether your clamp is actually in the path.

What it is **not**: no mass, no slip, no acceleration limit, no floor. Odometry
is the exact integral of the command, which the real base's never is. Use it to
prove plumbing, not to predict where the robot ends up.

# Everyday use

Once built, a normal session is three commands:

```bash
cd ~/Trossen/slate-base
docker compose up -d
docker compose exec -d slate-base bash -lc './launch-base.sh > ~/workspace/driver.log 2>&1'
docker compose exec slate-base bash
```

then inside: `./base_ctl.py torque on` and drive. To shut down:

```bash
docker compose down
```

## Command reference

All of these run inside the container. Add `--help` to any of them.

| Command | What it does |
| --- | --- |
| `./read_base.py` | One reading. `--watch` for a live block, `--raw` for piping. |
| `./move_base.py forward 0.5` | Drive 0.5 m, closing the loop on odometry. Negative for backwards. |
| `./move_base.py turn 90 --deg` | Turn in place. Positive is counter-clockwise. |
| `./move_base.py vel 0.2 0.1 --for 3` | Hold a raw velocity for 3 seconds. |
| `./move_base.py stop` | Stop now. The one command that needs no `--execute`. |
| `./teleop_keyboard.py` | Arrows drive, `[` `]` speed, space stops, ESC quits. |
| `./base_ctl.py torque on\|off` | Motors engaged, or released so it rolls freely. |
| `./base_ctl.py light green` | Light bar. `light list` shows every colour. |
| `./base_ctl.py text "hello"` | Write to the base's screen. |
| `./base_ctl.py charge on\|off` | Allow or block charging. |
| `./launch-base.sh` | Start the driver. `--tf` also broadcasts `odom → base_link`. |

Add `--execute` to any `move_base.py` command to actually move. Without it you
get the plan and nothing is sent.

## Moving the base by hand

```bash
./base_ctl.py torque off      # it now rolls freely -- level ground only
# ...push it where you want...
./base_ctl.py torque on
```

Odometry is meaningless after a hand-move. Restart the driver to re-zero it.

## When something is wrong

| Symptom | Most likely cause |
| --- | --- |
| Service calls fail at random, log shows only successes | **Two drivers running.** `pgrep -af '[s]late_base_node'` — expect one. See [Two drivers](#two-drivers-is-the-failure-mode-to-know-about). |
| Base does not move, no error | Motor torque is off. `./base_ctl.py torque on`. |
| `Failed to initialize base port` | Base unplugged or off, or a stale driver holds the port. |
| `/dev/ttySLATE` missing | Base off, or udev rule not installed, or `brltty` stole the port. |
| `No message on /slate/odom` | Driver not running. Check `workspace/driver.log`. |
| `ros2: command not found` via `exec` | Wrap it in a shell: `docker compose exec slate-base bash -c '...'`. |
| Topic list empty across containers | `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` mismatch, not wiring. |
| `move_base.py` reports `TIMED OUT` | Wheels blocked, E-stop engaged, or torque off. It stops safely. |

The driver's log is the first place to look, and it is on the host:

```bash
cat ~/Trossen/slate-base/workspace/driver.log
```

---

The rest of this document is why things are the way they are. Read it when
something surprises you.

## How this differs from the two arm containers

| | `../manip-arm/` | `../middle-arm/` | this one |
|---|---|---|---|
| Hardware | WidowX AI `wxai_v0` ×2 | WidowX-250 `wx250s` | SLATE base |
| Driver | `trossen_arm` SDK | `interbotix_xs_sdk` | `interbotix_slate_driver` |
| Transport | Ethernet / IP | FTDI U2D2 → `/dev/ttyDXL` | CH340 → auto-discovered |
| Interface | direct SDK, no ROS | ROS 2 topics | ROS 2 topics |
| Host needs | static IP on the NIC | udev rule | udev rule, no brltty |
| GPU | not required | ZED variant only | not required |
| Portable to | any arch | amd64 / arm64 | **amd64 / arm64 only** |

Three things about this component are genuinely unlike the arms, and each one
changed how the scripts here are written:

**1. The serial port is not yours to choose.** Every other device on this rig
gets a path — `ARM_IP`, `DXL_PORT`. Here there is nothing to set.
`base_driver::chassisInit()` takes an *output* string: the driver enumerates USB
serial devices with libudev, matches the CH340's `1a86:7523` itself, opens what
it finds, and then tells you which node it used. So `/dev/ttySLATE` (created by
the udev rule here) is a diagnostic convenience for humans, not something the
driver ever reads. `launch-base.sh` checks for it before starting the node, and
that is its whole job.

This is not academic: the base was observed at `/dev/ttyUSB0` on one run and
`/dev/ttyUSB1` on the next. Auto-discovery took it in its stride, where a
hardcoded path would have broken — and a *stale driver* holding the old node is
precisely how [two drivers](#two-drivers-is-the-failure-mode-to-know-about)
becomes so confusing.

**2. Part of the driver is a prebuilt blob.** `trossen_slate` ships
`lib/x86_64/libchassis_driver.so` and `lib/aarch64/libchassis_driver.so` and no
source for either, and its CMakeLists raises `FATAL_ERROR` on any other
`CMAKE_SYSTEM_PROCESSOR`. This image builds on amd64 and arm64 and is simply not
buildable elsewhere — unlike `../manip-arm/`, which is arch-agnostic by
construction and runs on Apple Silicon. Worth knowing before planning a port.

**3. It moves the whole robot.** The arms fail safe: a bad command hits a joint
limit and the controller refuses it. There is no equivalent here — there is no
position limit on a floor. That asymmetry is why `move_base.py` is dry-run by
default, closes its loop on odometry rather than a stopwatch, and clamps
velocities client-side (see [Velocity limits](#velocity-limits-are-this-repos-not-the-bases)).

## Host setup (once per machine)

```bash
./host-setup/setup-host.sh
```

Installs the udev rule (→ `/dev/ttySLATE`), offers to remove `brltty`, and adds
you to `dialout`. No NVIDIA step — the base has no camera and needs no GPU.

**`brltty` is the one to take seriously.** It ships enabled on Ubuntu desktop and
its udev rule claims `1a86:7523` as a braille display, taking the port away a
second or so after the base enumerates. The symptom is precise and misleading:
`/dev/ttyUSB0` appears, then vanishes, and the driver reports a port it cannot
open. Trossen's own instructions say to remove the package, and the setup script
does that rather than trying to out-rule a vendor file in `/usr/lib`.

> On this machine `brltty` is already in the `rc` state (removed, config files
> remaining), so its udev rule is gone and no action is needed. `libbrlapi0.8`
> and `python3-brlapi` are still installed; those are libraries, not the daemon,
> and they claim nothing.

Group membership only applies to new logins — log out and back in (or reboot, as
Trossen's docs say) before relying on `dialout` for host-side serial access.

## Build and run

```bash
cd slate-base

# Once, from the repo root: writes .env with this machine's UID/GID so the
# ./workspace bind mount is writable from both sides.
../setup.sh

docker compose build          # ~10-15 min
docker compose up -d
docker compose exec slate-base bash
```

Smoke tests, inside the container:

```bash
ros2 pkg list | grep slate          # interbotix_slate_driver, interbotix_slate_msgs, trossen_slate
ls -l /dev/ttySLATE                 # the base is connected and powered
```

> **Running one-off commands from the host:** wrap them in a shell —
> `docker compose exec slate-base bash -c 'ros2 topic list'`, not
> `docker compose exec slate-base ros2 topic list`. The bare form fails with
> `executable file not found in $PATH`: `docker exec` bypasses `ENTRYPOINT` and
> gets only the image's static `ENV`, whereas the ROS overlays exist once
> something sources `ros-env.sh`. `BASH_ENV` makes any bash — interactive or not
> — do that, but a bare `ros2` never starts a shell in the first place. Same
> mechanism, same fix, as `../middle-arm/`.

### Bring up the base

```bash
./launch-base.sh              # driver only
./launch-base.sh --tf         # also broadcast odom -> base_link
./launch-base.sh --rate 50    # faster control loop (default 20 Hz)
```

What it expands to:

```bash
ros2 run interbotix_slate_driver slate_base_node --ros-args \
  -r __ns:=/slate \
  -p update_frequency:=20 -p publish_tf:=false \
  -p odom_frame_name:=odom -p base_frame_name:=base_link
```

There is no launch file upstream — the package installs one executable and
nothing else — so this wrapper is the launch file.

**Run it detached, with a log file.** A driver started as `docker compose exec
slate-base ./launch-base.sh` dies when you close its terminal — and takes its
log with it, which is exactly how the first crash on this rig went undiagnosed.
Redirect into the bind mount instead, where the log is readable from the host
and survives everything:

```bash
docker compose exec -d slate-base bash -lc './launch-base.sh > ~/workspace/driver.log 2>&1'
sleep 2 && cat workspace/driver.log        # from the host
```

**Enable motor torque before expecting motion.** The base powers on with its
drive motors released — it rolls freely and ignores `/cmd_vel` entirely:

```bash
./base_ctl.py torque on
```

Vendor's own `advanced_demo.cpp` does this before commanding velocity;
`basic_demo.cpp` omits it and would not move a base in this state. The setting
sticks until the base is power-cycled, and **there is no way to read it back** —
the driver publishes no torque state, so the practical test is whether the base
moves.

**There is no simulation mode.** `../middle-arm/workspace/launch-arm.sh --sim`
can prove the whole stack launches with no hardware attached; nothing here can.
The driver calls `init_base()` in its constructor and that goes straight to a
real serial port. With no base connected the node starts, logs `FATAL Failed to
initialize base port`, and then **keeps running** with a timer that reads
nothing — it does not exit. A node that is up while `/slate/odom` is silent is
exactly that failure, not a slow start.

### Never mix select() with buffered stdin

Filed here because it cost a real debugging session and the symptom pointed
nowhere near the cause: `teleop_keyboard.py` appeared to exit the instant it
started, printing its help and then `stopped.` with no key pressed.

It was not exiting on startup. It was quitting on the **first arrow key**,
because it read the key as a bare ESC. The original read loop did
`sys.stdin.read(1)`, then `select()` on the file descriptor to see whether more
of the escape sequence had arrived. But `sys.stdin` is a buffered
`TextIOWrapper`: reading one character pulls the whole `ESC [ A` out of the OS
buffer into Python's own, so `select()` on the fd correctly reports nothing left
to read while two bytes sit in userspace. ESC with nothing after it means quit.

The fix is `os.read()` on the raw descriptor, so `select()` and the read see the
same buffer. **Any** code that selects on a stream must not also read it through
Python's buffered layer.

### Two drivers is the failure mode to know about

The single worst thing that can happen in this container, because it does not
look like a failure. It cost an hour during bring-up and every symptom pointed
somewhere else.

Both nodes are called `slate_base` and both offer the same services, so requests
**round-robin between them** — but only one holds the serial port. The other
ran `init_base()`, failed, logged `FATAL`, and **kept running anyway** (the node
does not exit on port failure), so it answers roughly half your service calls
and fails every one it answers.

What it looked like:

- `./base_ctl.py torque on` failed about four times in five, seemingly at random
- the driver log showed **57 successful torque writes and zero failures**
- `/slate/odom` published at a rock-steady 19.999 Hz throughout
- every reading — odometry, battery, E-stop — was correct

The natural reading of that is a flaky Modbus write, and this README said so for
a while. Nothing was flaky. `pgrep` told the real story:

```
pid  45  ->  /dev/ttyUSB0 (deleted)     # left over from a closed terminal
pid 124  ->  /dev/ttyUSB1               # the one actually working
```

The stale driver came from a `docker compose exec` whose terminal was closed —
that does not kill the process — and it was still holding a `/dev/ttyUSB0` that
had since re-enumerated away. With one driver: six calls, six successes, no
retries, zero `failed to send response` warnings.

`launch-base.sh` now refuses to start a second one. If service calls ever start
failing again, check this **before** anything else:

```bash
pgrep -af '[s]late_base_node'     # expect ONE driver (plus its `ros2 run` wrapper)
ros2 node list                    # expect ONE /slate/slate_base
```

`ros2 node list` does warn about duplicate node names, but only if you happen to
look. To clean up:

```bash
pkill -f '[s]late_base_node'      # the [s] stops pkill matching its own command line
```

Give DDS ~20 s afterwards: a `SIGKILL`ed participant lingers in `ros2 node list`
until its liveliness lease expires, so a stale entry right after a kill is not
evidence of a surviving process.

## Driving the base: the script stack

Same shape as the two arm workspaces — small single-purpose CLIs, dry-run by
default, limits checked before anything is sent.

| Script | What it does |
| --- | --- |
| `./read_base.py` | Odometry, measured velocity, battery, E-stop. `--watch`, `--raw`. Read-only. |
| `./move_base.py forward\|turn\|vel\|stop` | Measured moves, closed on odometry. Dry run until `--execute`. |
| `./teleop_keyboard.py` | Arrow-key teleop with a per-axis dead-man release. Needs a real tty. |
| `./base_ctl.py text\|light\|torque\|charge` | The four service calls. Moves nothing. |
| `./launch-base.sh` | Starts the driver. |
| `base_config.py` / `slate.py` | Topics, limits, state codes; shared rclpy helpers. |

```bash
docker compose exec slate-base bash

./launch-base.sh &                   # or a second shell
./read_base.py                       # where is it, how charged
./move_base.py forward 0.5           # dry run: prints the plan
./move_base.py forward 0.5 --execute # actually move
./base_ctl.py light green
./teleop_keyboard.py                 # drive it by hand
```

Every one of these is a **client of the driver node**, never a second driver.
That is a hardware constraint, not a style choice: the base speaks over one
serial port and `SerialDriver::init` opens it exclusively, so exactly one process
can talk to it. It is the same "one connection at a time" rule the WXAI arms
have, reached by a different route. `./launch-base.sh` must be running in another
shell before any script does anything.

`trossen_slate` does ship pybind11 bindings (`pytrossen_slate`), and they are not
installed here on purpose — they open the same exclusive port, so having them
available would only invite running two drivers and having neither work.

### The ROS interface, in full

The vendor documentation does not list any of this, so it is read out of
`slate_base.cpp` directly. Names below are shown with the `/slate` namespace
this rig applies.

| Topic | Type | Direction |
|---|---|---|
| `/slate/cmd_vel` | `geometry_msgs/Twist` | subscribed — `linear.x` and `angular.z` only |
| `/slate/odom` | `nav_msgs/Odometry` | published at `update_frequency` |
| `/slate/battery_state` | `sensor_msgs/BatteryState` | published every 10th update (~2 Hz) |

| Service | Type |
|---|---|
| `/slate/set_text` | `interbotix_slate_msgs/SetString` |
| `/slate/set_motor_torque_status` | `std_srvs/SetBool` |
| `/slate/enable_charging` | `std_srvs/SetBool` |
| `/slate/set_light_state` | `interbotix_slate_msgs/SetLightState` |

| Parameter | Default |
|---|---|
| `update_frequency` | `20` (Hz) |
| `publish_tf` | `false` |
| `odom_frame_name` | `odom` |
| `base_frame_name` | `base_link` |

`BatteryState` is mostly NaN by construction: the driver explicitly sets
`temperature`, `charge`, `capacity` and `design_capacity` to NaN and fills in
only `voltage`, `current` and `percentage`.

`percentage` is **0–100**, not the 0–1 that `sensor_msgs/BatteryState`
documents — the driver assigns it straight from a `uint32`. Measured on this rig
at `81.00` alongside `27.68 V`, which settles it.

`current` read `0.00 A` at every sample taken, including across a session where
the pack charged from 64% to 97% — so it is not a usable charging indicator, and
the sign convention remains unknown. `read_base.py` prints it unlabelled rather
than guessing.

`percentage` is also noisy under load: it moved 81 → 64 → 79 within one session
as the voltage sagged from 27.68 V to 26.49 V and recovered. Treat it as a
voltage-derived estimate, not a coulomb count.

### Why the namespace matters

The driver creates every topic and service relative to its node namespace and
takes no namespace argument of its own, so out of the box they land at `/odom`,
`/cmd_vel` and `/battery_state`. `launch-base.sh` remaps to `/slate` with
`--ros-args -r __ns:=`.

That is not tidiness. A bare `/cmd_vel` on a shared DDS graph means *anything on
the domain* that publishes the conventional topic name drives this base — a
stray `teleop_twist_keyboard`, a nav stack someone left running, or the third-party
teleop rig that `../middle-arm/docker-compose.yml` documents finding on the lab
wifi. The arms tolerate an unexpected publisher; a mobile base drives into a
wall. `ROS_DOMAIN_ID=42` is the first line of defence and the namespace is the
second.

### Velocity limits are this repo's, not the base's

`trossen_slate.hpp` defines `MAX_VEL_X` and `MAX_VEL_Z` as `1.0`, and
`TrossenSlate::set_cmd_vel()` clamps to them.

**The ROS node does not go through that function.** `SlateBase::cmd_vel_callback()`
assigns `msg->linear.x` and `msg->angular.z` straight into the chassis data
struct, and the next `update()` writes it to the holding registers. The clamp
exists in the C++ API and is simply not in the path a `/cmd_vel` message takes,
so whatever you publish is what the base is asked to do.

Every script here therefore clamps client-side, to `CLAMP_VEL_X` / `CLAMP_VEL_Z`
in [`base_config.py`](workspace/base_config.py) — 0.3 m/s and 0.8 rad/s, well
under the vendor maximum. 1.0 m/s is a fast walk, and a rig carrying three arms
and a camera mast has a high centre of mass. Raise them deliberately once you
know how this base behaves loaded, not to make a script stop complaining:

```bash
SLATE_MAX_VEL_X=0.5 ./move_base.py forward 2.0 --execute
```

Anything publishing to `/slate/cmd_vel` that is *not* one of these scripts gets
no clamp at all.

### Velocity commands expire after 300 ms

`CMD_TIME_OUT` is 300 ms. The driver zeroes the stored command when nothing has
arrived for that long, so `/cmd_vel` is a setpoint with a deadline, not a
move-to command:

- Publish once → the base moves for 300 ms and stops.
- Stop publishing mid-motion → it coasts to a halt within 300 ms.
- A script that dies takes the motion with it, which is the good case.

So every commanding script here runs a publish loop at 20 Hz for as long as it
wants motion — matched to the driver's own update rate, since publishing faster
than the driver reads gains nothing. It also means **stop the base before
stopping the driver**, not the other way round: `Ctrl-C` on `launch-base.sh`
while the base is moving leaves it coasting for up to 300 ms with nothing left
listening.

### Reading the E-stop

The driver keeps `SystemState` to itself — there is no topic carrying it. The one
bit that escapes is this, from `SlateBase::update()`:

```cpp
odom.pose.covariance[0] = (data_.system_state == SystemState::SYS_ESTOP) ? -1 : 1;
```

which is a covariance field being used as a flag. `read_base.py` and
`move_base.py` read it, because it is the only way a ROS client can see the
E-stop at all, and they treat anything other than exactly `-1` or `1` as
*unknown* rather than guessing. Do not build on the encoding: it is undocumented
and will break the moment upstream publishes the state properly or fills in a
real covariance.

**Confirmed on hardware**: the flag reads `E-STOP ENGAGED` with the button
pressed and `clear` with it released, in both directions. Note also that the
E-stop *brakes* the wheels — the base cannot be pushed by hand while it is
engaged, which is the opposite of what `./base_ctl.py torque off` does.

Bear in mind what the flag cannot tell you. `clear` means *not `SYS_ESTOP`* and
nothing more: `SYS_CHARGING`, `SYS_REMOTE`, `SYS_ERR_COLLISION` and every other
state in the enum are all indistinguishable from normal operation through ROS.
The base's own screen is the only place to see them.

It is a status readout sampled at 20 Hz over a serial link. **The E-stop button
is the safety device.** The full `SystemState` enum — including the collision,
low-voltage and over-temperature codes that never reach ROS at all — is
transcribed in [`base_config.py`](workspace/base_config.py) for when you need to
read it out of the C++ side.

### What odometry means here

`/slate/odom` is wheel odometry, and its origin is **wherever the base was when
the driver started** — the node subtracts the first sample it ever sees. Restart
`launch-base.sh` and x/y/yaw jump back to zero without the base moving. It is
not a room-fixed frame and not where the base was powered on.

There is also no service to reset it. `trossen_slate` exposes `reset_odometry()`
and its own demo uses it, but `interbotix_slate_driver` does not wrap it.
Restarting the driver is the way to re-zero.

`move_base.py` closes its loop on this, which is better than open-loop timing
(which is wrong by whatever the acceleration ramp costs you, in the direction of
overshoot) and still only as good as the wheels. Measured on this rig, a
`turn 30 --deg` landed at `+0.527 rad` against `+0.524` requested — 0.3% over,
in 1.6 s. On a clean floor that is the accuracy to expect; over a threshold or
with a wheel slipping it is not. It is a convenience for repeatable nudges, not
a positioning system.

Pushing the base by hand does update odometry (verified: a hand-push read
`0.161 m`), but the reading is only as good as the wheels turning without slip —
after a hand-move across a room, treat x/y/yaw as scrap and restart the driver
to re-zero.

## Running all four containers

Three Compose projects in three directories, so three commands. They coexist
because all of them use `network_mode: host`, `ROS_DOMAIN_ID=42`, and
`rmw_fastrtps_cpp` — one DDS graph across every container.

```bash
# 0. Host prerequisites, once per boot / login session
ip -br addr show enp0s31f6      # must be UP with 192.168.1.1/24  (manipulators)
ls -l /dev/ttyDXL               # the U2D2 is connected            (middle arm)
ls -l /dev/ttySLATE             # the base is connected            (this)
xhost +local:docker             # once per login session, for RViz

# 1. Middle arm (USB / DYNAMIXEL)
cd ~/Trossen/middle-arm && docker compose up -d

# 2. Both manipulators (Ethernet)
cd ~/Trossen/manip-arm && docker compose up -d

# 3. Mobile base (USB serial)
cd ~/Trossen/slate-base && docker compose up -d
docker compose exec slate-base ./launch-base.sh
```

```bash
docker ps --format '{{.Names}}\t{{.Status}}'
# middle-arm, trossen-arm-1, trossen-arm-2, slate-base
```

The three transports are independent — Ethernet through the switch, the U2D2's
FTDI at `/dev/ttyDXL`, and the base's CH340 — so none can interfere with the
others, and the order above is only about failing fast on the more fragile ones.
The two USB devices have different VID:PIDs (`0403:6014` vs `1a86:7523`) and
different udev rules, so both symlinks can coexist on one host.

## Notes and caveats

- **Verified against hardware** on 2026-08-13, base firmware `v2.1-3-g019c974`,
  driver `v1.0.0`. Port discovery, odometry, battery, the E-stop readout, all
  four services, closed-loop motion, teleop and the duplicate-driver guard have
  all been exercised on a real SLATE.
- **Teleop is tested through a pty**, not only by hand: `pty.fork()`, synthetic
  arrow-key events, and assertions on the MOVING/stopped states it prints. That
  is how the escape-sequence bug below was found and confirmed fixed, and it
  means the dead-man release is checked rather than assumed. Worth reaching for
  again — a keyboard UI is otherwise the one thing in this repo that can only be
  tested by a person being available.
- **Reproducibility.** `INTERBOTIX_REF` is `humble`, a moving branch, and
  `trossen_slate` is pinned only by whatever commit that branch's submodule
  pointer names today. Two machines built weeks apart can get different upstream
  code. Pin both to tags or SHAs once this setup is proven — the same caveat the
  other two READMEs carry.
- **An upstream sign error in the odometry TF.** In `SlateBase::update()`, the y
  translation reads `(data_.odom_y - -pose_[1])` where every neighbouring term
  subtracts the offset. It affects the TF and the `Odometry` pose whenever the
  driver starts at a non-zero `odom_y` — which, since the base's own odometry is
  never reset, is any time after the first run on a given power cycle. Reported
  as-is here rather than patched: this container builds vendor source unmodified,
  and a local patch is a thing to re-apply on every bump.
- **`privileged: true`** matches `../middle-arm/`, and the case is stronger here.
  The tighter form (`devices: [/dev/ttySLATE]` plus a `c 188:* rmw` cgroup rule)
  fails `up` whenever the base is unplugged, and more fundamentally the driver
  does not open a path you hand it — it runs a libudev scan, which needs the
  `/sys` and `/dev` views rather than one device node.
- **No URDF, no TF tree of its own.** Nothing in this image describes the base's
  geometry, so `publish_tf:=true` gives you `odom → base_link` and nothing else.
  Mounting the arms onto the base in TF means authoring that transform yourself —
  and it is the same gap `../middle-arm/README.md` notes for the camera that
  replaced the gripper.
- **The SLATE also carries the arms' power and the host computer.** Nothing in
  software knows that. `./base_ctl.py torque off` releases the drive motors so
  the base rolls freely, which is right for repositioning by hand and wrong on a
  ramp.

## References

- [SLATE getting started](https://docs.trossenrobotics.com/slate_docs/getting_started.html)
- [SLATE ROS 2 setup](https://docs.trossenrobotics.com/slate_docs/getting_started/ros_interface/ros2.html)
- [interbotix_ros_core (humble)](https://github.com/Interbotix/interbotix_ros_core/tree/humble/interbotix_ros_slate)
- [trossen_slate](https://github.com/Interbotix/trossen_slate) — the submodule, including the prebuilt `libchassis_driver.so`
