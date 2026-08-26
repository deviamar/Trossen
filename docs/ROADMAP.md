# Roadmap

Where this is going, in the order it should be built. Each stage is usable on
its own and is a prerequisite for the next — the point of the ordering is that
nothing later has to be debugged at the same time as something earlier.

Status as of the last update:

| Stage | State |
|---|---|
| 0. Containers, topics, keyboard control | **verified end to end** |
| 1. tmux control surface | **verified end to end** |
| 2. Combined URDF | **done** — Trossen ships `mobile_ai.urdf`, vendored into `sim/description/` |
| 3. Web visualisation | **working** — `sim` container, Viser on :8080, live from topics |
| 4. Headset / WebRTC | code ported, never run against a headset |
| 5. Full sim | not started — the visualiser is a MIRROR, no physics, no collision |

---

## Hardware right now

| Component | Connection | State |
|---|---|---|
| SLATE AGV | USB, `/dev/ttySLATE` | connected, driver up |
| Left manipulator | Ethernet switch, 192.168.1.2 | connected, agent up |
| Right manipulator | Ethernet switch, 192.168.1.3 | connected, agent up |
| Middle arm (wx250s) | — | not connected |
| ZED camera | — | not connected |
| Scissor lift | — | not connected |
| Quest headset | — | not connected |

The containers for the missing four still start; they detect their hardware is
absent, say so, and idle. That is deliberate: `make` should never fail because
something is unplugged.

---

## Stage 0 — containers and topics · WORKING

`make` from `~/Trossen` starts six containers, each publishing its own topics.
The full map is in [topics-by-container.md](topics-by-container.md); the
interface rules are in [topic-contract.md](topic-contract.md).

Three bugs found on real hardware here, all fixed, all worth remembering because
each produced a symptom that pointed somewhere else:

- **`ipc: host` missing.** Fast DDS discovers over UDP but moves data over
  shared memory. Without a shared `/dev/shm`, `ros2 topic list` was complete and
  not one message arrived.
- **UID mismatch.** `setup.sh` had a stale hardcoded project list, so two
  containers built with different UIDs and could not write into each other's DDS
  segments. Same symptom as above. `setup.sh` now discovers projects.
- **`set -u` in `start.sh`.** ROS's `setup.bash` reads unset variables, so the
  arm containers died on line 8 of it and exited. That read as "the arm is
  unreachable" rather than "the shell options are wrong".

`make check` tests for the first two.

## Stage 1 — tmux control surface · WORKING

`make tmux` builds one session with a pane per **live** device: state, arms,
base, and a free shell. Panes are skipped for hardware that is not up.

- **base** — `teleop_keyboard.py` inside `slate-base`. Arrows drive. x is
  forward, yaw is rotation. There is no lateral y: it is a differential-drive
  base and cannot strafe. The rig's real z is the scissor lift
  (`/slate/lift/*`), currently simulated.
- **arms** — `arm_key.py` in `monitor`. `1`/`2`/`3` select, `space` engages,
  arrows are ±z, `wsad` are ±x/±y, `g`/`h` the gripper.

### Verified

Full `make down` → `make` → `make status` → `make tmux` cycle, with every
command path exercised. Bugs found and fixed during that pass:

- **quest was autostarting**, publishing to `/left_arm/cmd_*`,
  `/right_arm/cmd_*` and `/slate/cmd_vel_teleop` at 72 Hz — including a
  dead-man zero on the `sim` backend. The newest message on a command topic
  wins, so it silently overrode every control tool and made them look dead.
  Now `AUTOSTART=false`: the container comes up, its teleop does not.
- **The base teleop status line flooded the pane.** A single `\r`-redrawn line
  only works while it fits; at 40 columns it wrapped, `\r` returned to the
  start of the wrapped fragment, and every tick left another partial line. Now
  truncated to the terminal width.
- **The dashboard was taller than a quarter pane**, so the BASE block — the
  part you actually watch — scrolled off the top. Now compact and both width-
  and height-aware.
- **`make status` printed 43 lines.** Now `ps` + the dashboard; the full table
  moved to `make topics`.
- **The tmux arms pane appeared with no arms connected**, because a topic keeps
  existing as long as anything subscribes to it. Now gated on an actual
  publisher count.

Verified working, on real hardware where it exists:

| Path | How |
|---|---|
| `make` from cold | 6 containers up, topics live in ~20 s |
| Base drive | preflight passes, dry run correct, torque on, odom 20 Hz |
| Base watchdog | recovered from a live re-enumeration during the test |
| `arm_ctl list/state/go/gripper/joints/stop` | all exercised against a stand-in agent |
| `arm_key` jog | `↑↑↓` → z +0.02/−0.01, `w` → x +0.01, `a`/`d` → y net 0, `g`/`h` gripper |
| tmux, 4 panes | dashboard, jog, base teleop, shell — all live |
| tmux, arms absent | correctly degrades to 3 panes |
| `make check` | .env, UIDs and ipc all correct |

**The one thing not verified against hardware is the arm SDK link**, because
the USB-Ethernet adapter is off the bus. It did work earlier in the same
session — both arms connected, firmware v1.11.1, real joint data — so what is
unverified is only that link, not the ROS path above it.

### Refinements worth making

- One combined pane that shows both arms' end-effector poses side by side, so
  bimanual work does not need two `state` reads.
- Named-pose buttons in the arm pane (`arm_ctl.py go ready` is a separate
  command today).
- A guard against two commanders: `drive_test.py` detects a competing publisher
  by name; `arm_key.py` does not yet.

## Stage 2 — combined URDF · NEXT

`assets/` holds 141 files: `mobile_AI`, `scissor_lift`, `slate_base`,
`widowX-250`, `widowX-AI` — 48 STL, 31 STEP, 15 SLDPRT, and one URDF.

**Do not model the arms from CAD.** Trossen ships `trossen_arm_description`
(already built into the manip-arm image) and Interbotix ships the wx250s
description. Both are correct, have inertias, and are maintained. What the CAD
is actually needed for is the part nobody ships: the frame, the mount plates,
and **the transforms between them**.

**Frames are defined in [frames.md](frames.md)** — read that before measuring
anything. It names every frame, says what its origin physically is, and lists
the 26 parameters (most zero by symmetry) that the model needs.

Shape of the work:

1. `rig.urdf.xacro` at the top, including the vendor arm xacros and the base.
2. `rig_params.yaml` holding every number from frames.md in one place, so the
   model is tuned by editing values rather than geometry.
3. Get the numbers from CAD assembly (exact) rather than a tape (cannot resolve
   3D pose or angle), and solve arm-to-arm by touch test rather than measuring
   it at all.
3. STL for visuals; primitives for collision. Triangle-soup collision meshes are
   the standard way to make a solver crawl.
4. The ZED is not in the wx250s URDF at all — the vendor xacro ends at the
   flange. A fixed joint with the measured offset is needed before
   `head_agent.py`'s `--ee-link` means the optical centre rather than the flange.

**This unlocks two things beyond visualisation**: the coupled three-arm IK
(currently each arm solves alone and no solver knows another arm exists), and
every option in stage 3.

The first measurement to take: where each arm base sits relative to the slate's
`base_link`.

## Stage 3 — web visualisation

Everything here is gated on stage 2.

World frame: the base starts at the origin and everything is drawn relative to
that starting point. Wheel odometry drifts and nothing corrects it, so treat the
drawn position as "where the robot thinks it is", not ground truth — worth
labelling on screen so it is never mistaken for a measurement.

Layout requested:

```
+-------------------+---------------------------+-------------------+
| arm states        |                           | headset +         |
| (top left)        |     3D system view        | controllers       |
+-------------------+     URDF + meshes         +-------------------+
| camera feed       |     live from TF          | slate AGV state   |
| (bottom left)     |                           | (bottom right)    |
+-------------------+---------------------------+-------------------+
```

**Recommendation: Viser.** It is already a pyroki dependency, `ViserUrdf` loads
URDF + STL directly, `add_gui_*` gives readouts and buttons, and giava has a
working `2arm_ik_viser.py` to crib from. A `viz` container subscribing to the
contract topics is a few hundred lines.

Alternative: **Foxglove** if the priority shifts to debugging — time-series
plots, image panels and a 3D URDF panel with no code. Weaker for custom buttons.

Either way it is a new container that subscribes only to the contract. No robot
container changes.

## Stage 4 — headset

The code is ported and unrun. `quest/workspace/giava/` holds four files vendored
from `giava@real-v2-spr26`: `webrtc_headset.py` (aiortc + Firestore signalling),
`headset_utils.py`, `transform_utils.py`, and `teleop_map.py` (the tuned
mapping). Unity app: <https://github.com/Soltanilara/av-aloha-unity>, eye-tracking
branch.

Order:

1. Drop the two secrets into `quest/secrets/` (see the README there).
2. `./launch-quest.sh --backend webrtc --driver-only` — raw `/quest/*` topics,
   no robot commands. Confirm the link before anything can move.
3. **Verify handedness.** Unity is left-handed Y-up, ROS is right-handed Z-up.
   Move a controller purely forward; `/quest/right/pose` must show +x with y and
   z near zero. Get this wrong and teleop feels almost right with one axis
   mirrored, which everyone diagnoses as bad tracking.
4. `--dry-run`, then live at `QUEST_ARM_SCALE=0.3`.

Two constants in `teleop_map.py` are giava's rig, not yours, and will be wrong
until measured: `R_arm_remap` (a 90° yaw describing how their arms sit relative
to the operator) and `position_scale` (1.35).

Known gap: `quest_teleop.py` publishes a dead-man zero to
`/slate/cmd_vel_teleop` at 72 Hz whenever it runs, including on the sim backend.
That beats a 20 Hz drive command. **Stop the quest container before using
`drive_test.py` or `arm_ctl.py`.**

## Stage 5 — full sim

Only meaningful once stages 2–4 are real: the combined URDF, the visualiser, and
the headset all feeding one graph, with the drivers swapped for simulated ones.

`slate-base/workspace/sim_base.py` already exists and is the pattern —
`SLATE_SIM=true` swaps the real driver for a simulator with the same topics,
same 300 ms deadline, same torque gate. The arms would need the equivalent.

Deliberately last. A simulator that diverges from the hardware is worse than no
simulator, and the only way to know it has not diverged is to have run the
hardware first.

---

## Standing rules

- **One commander at a time.** `quest_teleop`, `keyboard_teleop`, `arm_key` and
  `drive_test` all publish the same command topics; the newest message wins.
- **`arm_agent.py` holds the arm's only connection.** While it runs, `pose.py`,
  `read_joints.py` and `teach.py` cannot reach that arm. `AUTOSTART=false` to
  swap.
- **The base ignores velocity until torqued.** `make torque`. It discards
  commands silently, so "nothing moved" is more often this than a broken pipe.
- **Run `make` from `~/Trossen`, never from a subdirectory.** Container names
  are global to the Docker daemon; a per-directory project collides with the
  root one.
