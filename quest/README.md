# Quest teleop

The input device. Publishes ROS topics; owns no hardware but a socket.

```
headset ──transport──▶ quest_driver.py ──▶ /quest/*  ──▶ quest_teleop.py ──▶ /slate/cmd_vel_teleop
                        (raw device state)                (button meaning)     /left_arm/cmd_pose
                                                                               /right_arm/cmd_pose
```

Two nodes, deliberately. `quest_driver.py` knows the headset and no robots;
`quest_teleop.py` knows both and is the only thing in the rig that does. No
robot container knows this one exists — swap the Quest for a gamepad or a
policy and nothing downstream changes.

## Controls

| Input | Does |
| --- | --- |
| Left thumbstick | base: fwd/back drives, left/right turns |
| Right thumbstick fwd/back | scissor lift z velocity (drives the *simulated* lift until the hardware is wired; same topic either way) |
| **A** (right, hold) | right arm follows the right controller |
| **X** (left, hold) | left arm follows the left controller |
| either A or X | camera arm follows your head |
| Index trigger | that arm's gripper — released stages the fingers open, squeezed commands an analog **closing force** (`cmd_grip_force`), so grasping an object can never fault the arm on a position error |

Hold to engage, not toggle. Releasing stops the arm following you; a toggle
leaves an armed robot behind when you set the controller down.

Driving and the lift are locked out while any arm is engaged: the arms are
bolted to the lift on a mobile base, so moving either moves the arms' anchors
under a held target. Let go, drive, re-engage.

## No headset? Use the keyboard

`keyboard_teleop.py` publishes the same contract topics `quest_teleop.py` does,
so it exercises the whole ROS pipeline -- arm agents, head agent, governor --
with no headset, no WebRTC and no credentials.

```bash
docker compose exec quest bash        # must be interactive; it needs a tty
./keyboard_teleop.py
```

`1`/`2`/`3` select left arm / right arm / camera arm, `0` the base. `space`
engages (anchoring where the arm is, so nothing jumps). `wsadqe` nudge the
target 1 cm, `g`/`h` work the gripper, arrows drive the base.

This is not a nicer `move_joint.py`. That talks to the arm's SDK directly; this
talks to the ROS contract. When teleop misbehaves, the two together tell you
which side is at fault -- which is the one question a failed headset session
cannot answer on its own.

## Bring-up, in this order

```bash
docker compose exec quest bash

./launch-quest.sh --backend sim --dry-run   # 1. nothing can move
./launch-quest.sh --backend sim             # 2. robots move, no headset needed
./launch-quest.sh --backend udp             # 3. for real
```

Step 1 and 2 are worth doing every time you change the mapping. The alternative
is debugging a coordinate frame while standing in a room next to a robot that
might move.

`--driver-only` publishes `/quest/*` and no robot commands — that plus
`ros2 topic echo /quest/right/joy` is how you confirm which button is which.

## Transport backends

`backends/` — swap with `--backend`. The link to the headset is the least
settled part of this rig, so it is a seam rather than an assumption.

| Backend | State | Use |
| --- | --- | --- |
| `gvlink` | **Default.** The v2 Unity app | Direct UDP, no cloud and no secrets: the robot broadcasts a beacon, the headset connects, video goes out and poses come back. Needs the protocol library — see below. |
| `webrtc` | Legacy, for older app builds | The giava stack: aiortc + Firestore signalling + the Unity APK you already run. Needs `secrets/`. |
| `sim` | Works | No hardware. A slow circle and optional held buttons. |
| `udp` | Works | An app sending JSON datagrams. Simpler than WebRTC if you ever want it. |

### The gvlink backend (the current app)

`gvlink` is the wire protocol for the v2 Unity viewer, and it is **not vendored
here on purpose**: the Python sender and the C# viewer have to agree byte for
byte, so there is one copy of it and it lives beside the C#.

```bash
git clone -b v2 https://github.com/Soltanilara/av-aloha-unity.git ~/av-aloha-unity
docker compose up -d quest          # bind-mounts it at /opt/gvlink (GVLINK_PATH)
docker compose exec quest ./quest_driver.py --backend gvlink
```

A missing checkout surfaces as an ImportError from that backend and nothing
else. `GVLINK_SRC=/some/other/path docker compose up -d quest` overrides it.

| port | direction | carries |
| --- | --- | --- |
| 15550 | robot broadcasts | discovery beacon: robot name, cameras, ports |
| 15551 | headset → robot, TCP | control: session setup, camera geometry, stats, feedback |
| 15552 | robot → headset, UDP | video, fragmented H.264, one atlas per eye |
| 15553 | headset → robot, UDP | head/hand/controller poses + gaze, at display rate |

The headset finds the rig by LAN broadcast, so both must be on the same
network and `network_mode: host` (already set) is what makes the beacon
visible. `GIAVA_ROBOT_NAME` is the name in the headset's robot picker.

**The stereo view is the middle arm's ZED, and it arrives over ROS** rather
than being captured here — giava drives its OAK in-process, this rig does not.
The middle arm owns the camera and publishes rectified images on the contract;
this container subscribes and forwards them, which is why the camera can move
with the active-vision arm without anything here knowing.

The viewer is also told the **measured camera geometry** so it places each eye
from intrinsics instead of a field-of-view guess. That comes from the ZED's own
`camera_info`, and it is the one place a ZED is easier than an OAK:
`CameraInfo.P` is already the rectified projection matrix, so
`backends/gvlink.zed_camera_params()` reads fx/fy/cx/cy straight out of it and
recovers the baseline from `P2[3] = -fx·b` — no `stereoRectify`, and no way to
describe a different rectification than the one in the pixels. The intrinsics
are scaled when the images are resized for the headset, because intrinsics are
in pixels and a resize the geometry does not follow renders a correctly
rectified image at the wrong scale.

### The WebRTC backend

Wraps `giava/webrtc_headset.py`, vendored from `giava@real-v2-spr26` — the same
code and the same protocol your headset already speaks, so there is no new
protocol to debug. It needs two secrets in `quest/secrets/`; see the README
there.

Note it does **not** convert coordinates. giava's `on_message()` already calls
`convert_left_to_right_coordinates()` on every pose, so `HeadsetData` is
right-handed by the time this container sees it. `backends/udp.py` does convert,
because a raw Unity app has not. Converting twice looks exactly like a tracking
fault, so the split matters.

Eye-tracking data rides the same channel and is currently dropped — adding it
would be a new topic on the contract, not a change to the backend.

## The mapping is giava's

`giava/teleop_map.py` is lifted from the GIAVA rig, where its constants were
tuned over real sessions: **1.35× position scale**, **alpha 0.3** smoothing, and
a **2 cm per-step Cartesian clamp**. The clamp is the part worth understanding —
it clamps an over-large step rather than rejecting it, so a tracking glitch
becomes a slightly slower follow instead of a dropped frame. Rejecting reads to
the operator as the arm stuttering.

The per-rig parts are now explicit rather than baked in:

- **Session yaw is measured, not assumed.** The app world's yaw is wherever the
  headset faced at app start — arbitrary every session — so at engage time the
  mapping reads your horizontal gaze direction and takes it as "forward"
  (`teleop_map.session_yaw_remap`). A fixed remap matrix could never be right
  twice in a row.
- `QUEST_ARM_REMAP_YAW_DEG` (default 0) — how you *stand* relative to the rig:
  0 facing the same way as rig +x, 180 facing it head-on.
- `QUEST_POS_SCALE` (default 1.0) — how far the arm moves per metre of hand.
  giava ran 1.35 for their workspace; dial in by feel.

Orientation is composed in the world frame for every arm: turn your hand about
the room's vertical and the EE turns about the rig's vertical, wherever both
happen to point. (Upstream giava hand-shuffled rotation axes for the gripper
arms — a per-rig compensation this port replaces with the remap conjugation its
own camera arm already used.)

Everything else is about how a human hand moves and should transfer unchanged.

## The one thing to get right: handedness

Unity is **left-handed, Y up**. ROS is **right-handed, Z up**. Converting is a
handedness flip, not a relabelling of axes, and getting it wrong gives teleop
that feels nearly correct with exactly one axis mirrored — which reads as bad
tracking rather than bad maths.

The conversion is in the backend, next to the app that defines the frame. Set
`"unity": false` in the packet if your app has already converted.

Only *changes* in controller pose are used, so a constant offset between
`quest_origin` and the room cancels out. A handedness error does not.

## What stops the robots

Three independent layers, because the failure that matters here is silence, not
a crash:

1. Releasing the button publishes `enable=false`.
2. Losing the headset (`/quest/connected` false) releases everything and zeroes
   the base.
3. If this container dies outright, the arm agent drops its target after 300 ms
   and the base governor stops the base. Neither needs this node's cooperation.

None of that is a substitute for a hand near the power.

## Configuration

Environment, in `docker-compose.yml` — no code changes needed:

| Variable | Default | Meaning |
| --- | --- | --- |
| `QUEST_ARM_NS_LEFT` / `_RIGHT` | `/left_arm`, `/right_arm` | which arm each hand drives |
| `QUEST_BASE_NS` | `/slate` | which base the sticks drive |
| `QUEST_BASE_MAX_X` / `_Z` | 0.25, 0.6 | operator comfort limits |
| `QUEST_TURN_SIGN` | −1 | flip if stick-left turns the rig right |
| `QUEST_LIFT_MAX_VEL` | 0.05 | m/s, right stick full deflection |
| `QUEST_POS_SCALE` | 1.0 | hand→EE amplification (giava ran 1.35) |
| `QUEST_ARM_REMAP_YAW_DEG` / `QUEST_CAM_REMAP_YAW_DEG` | 0 | how the operator stands relative to the rig |
| `QUEST_GRASP_MIN_N` / `_MAX_N` | 5, 40 | trigger→closing-force range (arm agent hard-caps at 100) |

`QUEST_BASE_MAX_*` are comfort limits. The **safety** limit is `CLAMP_VEL_*` in
`slate-base`, enforced by `governor.py` inside the container that owns the
serial port — deliberately somewhere this file cannot raise it.
