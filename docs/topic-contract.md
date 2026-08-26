# Topic contract

The interface between containers. Nothing else crosses a container boundary: no
shared Python module, no shared volume, no custom message package, no service
call into another component's process.

## The one rule that makes this modular

**Standard message types only.** Every topic below uses a type that ships with
ROS 2 Humble (`std_msgs`, `geometry_msgs`, `sensor_msgs`).

That is a deliberate constraint, not an accident. A custom `.msg` package would
have to be built into *every* image that touches it, which means one interface
change forces a rebuild of all six containers and they must all be rebuilt at
the same version. That is precisely the coupling this layout exists to avoid.
Standard types cost some expressiveness — a `Joy` array index means nothing on
its own — and buy the ability to rebuild any one container without touching the
others. The index tables below are what replaces a named struct field.

If you later decide the readability is worth it, the migration is: create the
interface package, add it to each image, change one publisher and one subscriber
at a time. The topic names do not have to change.

## Who owns what

A component **owns** its namespace. It defines which topics it subscribes to,
and anything may publish to them. It publishes its own state, and anything may
listen. No component knows another exists.

| Namespace | Container | Owns |
|---|---|---|
| `/left_arm` | `left-arm` | WXAI manipulator on 192.168.1.2 |
| `/right_arm` | `right-arm` | WXAI manipulator on 192.168.1.3 |
| `/middle` | `middle-arm` | WidowX-250 active-vision arm |
| `/slate` | `slate-base` | SLATE AGV |
| `/lift` | `scissor-lift` | Scissor lift (stub — no hardware yet) |
| `/quest` | `quest` | Meta Quest headset + controllers |

The Quest container is the only one that publishes into *other* components'
namespaces, and that direction is the point: input devices adapt to robots, not
the other way round. Swap the Quest for a gamepad, a scripted sequence or a
policy and no robot container changes. That is why the button mapping lives in
`quest/workspace/quest_teleop.py` and not in the arm or base containers.

---

## `/quest` — raw headset state

Published by `quest_driver.py`. Device state only; no robot semantics.

| Topic | Type | Meaning |
|---|---|---|
| `/quest/head/pose` | `geometry_msgs/PoseStamped` | Headset pose, ROS convention, frame `quest_origin` |
| `/quest/left/pose` | `geometry_msgs/PoseStamped` | Left controller pose |
| `/quest/right/pose` | `geometry_msgs/PoseStamped` | Right controller pose |
| `/quest/left/joy` | `sensor_msgs/Joy` | Left buttons and axes |
| `/quest/right/joy` | `sensor_msgs/Joy` | Right buttons and axes |
| `/quest/connected` | `std_msgs/Bool` | Transport is delivering frames |

Subscribed by `quest_driver.py`, for the return path. The headset is the only
device in the rig that is also a display, so alone among the components it
consumes as well as produces:

| Topic | Type | Meaning |
|---|---|---|
| `/quest/feedback` | `std_msgs/String` | JSON: arm poses in the operator's frame + out-of-sync flags, rendered by the Unity app |
| `/middle_cam/zed_node/left/image_rect_color` | `sensor_msgs/Image` | Left eye, pushed down the WebRTC video track |
| `/middle_cam/zed_node/right/image_rect_color` | `sensor_msgs/Image` | Right eye |

Feedback is JSON on a `String` rather than a typed message. The fields are
whatever that particular Unity app renders — a device-specific blob going to a
device-specific consumer — so a custom `.msg` would force all six containers to
rebuild for a change only two nodes care about. The rule at the top of this
document is doing its job here, not being bent.

### Joy layout

Same for both hands. `axes` are floats, `buttons` are 0/1 ints.

| `axes[i]` | Meaning | Range |
|---|---|---|
| 0 | Thumbstick X, right positive | −1 … 1 |
| 1 | Thumbstick Y, forward positive | −1 … 1 |
| 2 | Index trigger, analog | 0 … 1 |
| 3 | Grip trigger, analog | 0 … 1 |

| `buttons[i]` | Left controller | Right controller |
|---|---|---|
| 0 | X | A |
| 1 | Y | B |
| 2 | Thumbstick click | Thumbstick click |
| 3 | Menu | Oculus |

### Pose convention

REP-103: **x forward, y left, z up, right-handed**, metres, quaternion
normalised. `frame_id` is `quest_origin` — wherever the headset last recentred.

This matters more than it looks. Unity is **left-handed with Y up**; ROS is
**right-handed with Z up**. Converting between them is not a relabelling of
axes, it is a handedness flip, and getting it wrong produces teleop that looks
almost right and mirrors one axis. The conversion belongs in the transport
backend, closest to the app that defines the frame — see
`quest/workspace/backends/`. Everything downstream of `quest_driver.py` may
assume REP-103.

Only *changes* in controller pose are used while teleoperating, so a constant
offset between `quest_origin` and the room cancels out. A handedness error does
not cancel.

---

## `/left_arm`, `/right_arm` — WXAI manipulators

Subscribed by `arm_agent.py`, which holds the arm's single SDK connection.

| Topic | Type | Meaning |
|---|---|---|
| `<ns>/cmd_pose` | `geometry_msgs/PoseStamped` | **Absolute** target pose of the end effector, world axes. Position AND orientation — all six DOF are commanded |
| `<ns>/cmd_joints` | `sensor_msgs/JointState` | **Absolute** joint vector, radians. One discrete move, not a stream |
| `<ns>/cmd_pose_name` | `std_msgs/String` | A name from this arm's `config/poses.yaml`, e.g. `home` |
| `<ns>/cmd_gripper` | `std_msgs/Float32` | Opening in metres, 0.0 closed … 0.04 open |
| `<ns>/cmd_grip_force` | `std_msgs/Float32` | Squeeze in newtons. Bounds what the gripper can do to what it holds |
| `<ns>/enable` | `std_msgs/Bool` | `true` accepts motion; `false` holds position and ignores it |
| `<ns>/zero` | `std_msgs/Bool` | Re-capture the origin: the current EE becomes (0,0,0). Persisted |
| `<ns>/save_pose` | `std_msgs/String` | Record the current joints under this name |
| `<ns>/reset` | `std_msgs/Bool` | Exit so the container restarts and reconnects with `--clear-error` |

| Published | Type | Meaning |
|---|---|---|
| `<ns>/ee_pose` | `geometry_msgs/PoseStamped` | Measured end effector pose, world axes, origin applied |
| `<ns>/joint_states` | `sensor_msgs/JointState` | `joint_0`…`joint_5` + `left_carriage_joint` |
| `<ns>/pose_names` | `std_msgs/String` | Comma-separated names this arm can be sent to |
| `<ns>/active` | `std_msgs/Bool` | Agent holds the connection and is accepting commands |

**Joint space is not a convenience — it is the only escape from a singularity.**
Near one the controller refuses Cartesian IK outright and every `cmd_pose` is
rejected, so `cmd_joints` and `cmd_pose_name` are the only things that will move
the arm. This is why `home` is reachable by name rather than by coordinate.

**`save_pose` exists because the connection is exclusive.** While the agent is
up, `pose.py` cannot reach the arm at all, so without this topic the only way to
record a pose would be to stop the container — which drops the arm to idle and
loses the posture you wanted to keep.

**Absolute, not delta.** The publisher does the clutching and sends where the
arm should be. A dropped message then costs one frame of lag rather than
permanently shifting the arm's frame of reference, which is what accumulating
deltas at the receiver would do.

`cmd_pose` carries a quaternion; the SDK wants angle-axis. `arm_agent.py`
converts. Do not send angle-axis in a `Pose`.

**The connection is exclusive.** While `arm_agent.py` runs, `pose.py`,
`read_joints.py`, `teach.py` and the rest cannot connect to that arm. That is a
property of the controller, not of this design.

---

## `/slate` — the AGV

| Topic | Type | Meaning |
|---|---|---|
| `/slate/cmd_vel_teleop` | `geometry_msgs/Twist` | Human input. Highest priority. |
| `/slate/cmd_vel_nav` | `geometry_msgs/Twist` | Autonomy. Yields to teleop. |
| `/slate/cmd_vel` | `geometry_msgs/Twist` | **Governor output → driver. Do not publish here directly.** |

`governor.py` subscribes to the first two, clamps, applies priority and timeout,
and republishes to `/slate/cmd_vel` at 20 Hz.

**Why the governor exists.** The vendor driver does not clamp. `MAX_VEL_X` and
`MAX_VEL_Z` are `#define`d to 1.0 and `TrossenSlate::set_cmd_vel()` enforces
them, but `SlateBase::cmd_vel_callback()` does not go through that function — it
assigns `msg->linear.x` straight into the chassis struct. Whatever reaches
`/slate/cmd_vel` is what the base attempts.

Before, every client clamped itself, which was fine while every client was a
script in this repo. The moment an input device is a separate, independently
rebuildable container, that assumption is gone. So the limit moved into the
container that owns the serial port: a teleop container can be rewritten, or be
wrong, and the base still cannot exceed `CLAMP_VEL_X` / `CLAMP_VEL_Z`.

Velocity is a setpoint with a 300 ms deadline. The governor republishes at 20 Hz
so a publisher that stops sending stops the base rather than leaving it running.

---

## `/lift` — scissor lift

**No hardware is connected. `lift_agent.py` is a simulator** that integrates its
own height so the contract can be exercised end to end. It moves nothing.

| Topic | Type | Meaning |
|---|---|---|
| `/lift/cmd_velocity` | `std_msgs/Float32` | Up positive, m/s, clamped, 300 ms deadline |
| `/lift/cmd_height` | `std_msgs/Float32` | Absolute target height, m |
| `/lift/height` | `std_msgs/Float32` | Measured height, m |
| `/lift/moving` | `std_msgs/Bool` | Actuator in motion |

---

## `/middle` — active vision arm

`interbotix_xs_sdk` owns the namespace; `head_agent.py` adds the same command
contract the manipulators have, so the teleop node drives all three arms through
one code path.

| Topic | Type | Meaning |
|---|---|---|
| `/middle/cmd_pose` | `geometry_msgs/PoseStamped` | Absolute target pose of the camera link |
| `/middle/enable` | `std_msgs/Bool` | Follow, or hold position |
| `/middle/ee_pose` | `geometry_msgs/PoseStamped` | Measured camera pose, from FK |
| `/middle/active` | `std_msgs/Bool` | Agent is accepting commands |

**No `cmd_gripper`.** The ZED sits where the gripper would be.

**This arm is the one that needs a solver.** The WXAI controller does its own
Cartesian IK; `xs_sdk` speaks joint positions only, so `head_agent.py` runs
pyroki (`middle_ik.py`) and publishes an ordinary
`interbotix_xs_msgs/JointGroupCommand` on `/middle/commands/joint_group` — the
same topic `pose.py` and `move_joint.py` already use. It is one more publisher
to that topic, not a parallel control path, so the existing CLIs keep working
and the two must not run at once.

It is driven by your **head**, and engages whenever either hand does: you want
the view to follow you while your hands are busy, and to stop when you let go of
both. That is giava's behaviour, kept.

### What the split solver costs

giava solved all three arms in a single least-squares problem, which let it
trade one arm's posture against another's when they share a workspace. Here the
manipulators solve inside their own controllers and the camera arm solves alone;
**no solver in this rig knows that more than one arm exists.** Nothing prevents
the camera arm and a manipulator occupying the same space.

That was the price of using each arm's native controller, and it is worth
knowing before it is discovered. The way back is a single URDF containing all
three arms and a coupled solve — `middle_ik.py` follows giava's structure
closely enough that widening it is a contained change, but it would mean
commanding the WXAI arms in joint space and giving up their internal IK.
