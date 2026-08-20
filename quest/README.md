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
| Left thumbstick fwd/back | base drives forward/back |
| Right thumbstick fwd/back | base rotates clockwise/anticlockwise |
| **A** (right, hold) | right arm follows the right controller |
| **X** (left, hold) | left arm follows the left controller |
| Index trigger | that arm's gripper — analog, squeeze to close |

Hold to engage, not toggle. Releasing stops the arm following you; a toggle
leaves an armed robot behind when you set the controller down.

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
| `webrtc` | **Default for real use** | The giava stack: aiortc + Firestore signalling + the Unity APK you already run. Needs `secrets/`. |
| `sim` | Works | No hardware. A slow circle and optional held buttons. |
| `udp` | Works | An app sending JSON datagrams. Simpler than WebRTC if you ever want it. |

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

Two constants are per-rig and are almost certainly wrong here until measured:

- `R_arm_remap` — a 90° yaw describing how GIAVA's arms sit relative to the
  operator. Yours will differ.
- `position_scale` — how far the arm moves per centimetre of hand.

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
| `QUEST_TURN_AXIS` | `y` | `y` = right stick fwd/back turns; `x` = left/right |
| `QUEST_BASE_MAX_X` / `_Z` | 0.25, 0.6 | operator comfort limits |
| `QUEST_ARM_SCALE` | 1.0 | <1 damps arm motion; start at 0.3 |

`QUEST_BASE_MAX_*` are comfort limits. The **safety** limit is `CLAMP_VEL_*` in
`slate-base`, enforced by `governor.py` inside the container that owns the
serial port — deliberately somewhere this file cannot raise it.
