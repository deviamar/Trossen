# Topics by container

Which container owns which topic. The full contract with message semantics is in
[topic-contract.md](topic-contract.md); this is the lookup table.

`ros2 topic list` shows the union of all of these. To see them live with rates
and values: `docker compose exec monitor ./watch.py`.

---

## `left-arm` / `right-arm` — WXAI manipulators

`<ns>` is `/left_arm` or `/right_arm`. One agent per container, one arm each.

| Direction | Topic | Type | Node |
|---|---|---|---|
| SUB | `<ns>/cmd_pose` | `geometry_msgs/PoseStamped` | streaming Cartesian, for teleop |
| SUB | `<ns>/cmd_joints` | `sensor_msgs/JointState` | one absolute joint vector |
| SUB | `<ns>/cmd_pose_name` | `std_msgs/String` | a saved pose, **by name** |
| SUB | `<ns>/cmd_gripper` | `std_msgs/Float32` | opening in metres — **position control, for staging the fingers** |
| SUB | `<ns>/cmd_grip_force` | `std_msgs/Float32` | squeeze in newtons, + opens − closes — **what you grasp with** |
| SUB | `<ns>/enable` | `std_msgs/Bool` | nothing moves while false |
| PUB | `<ns>/ee_pose` | `geometry_msgs/PoseStamped` | measured |
| PUB | `<ns>/joint_states` | `sensor_msgs/JointState` | measured |
| PUB | `<ns>/pose_names` | `std_msgs/String` | JSON list of saved poses, 1 Hz |
| PUB | `<ns>/active` | `std_msgs/Bool` | agent is accepting commands |

`cmd_pose_name` carries a **name, not values**, because the poses belong to the
arm: `config/poses.yaml` is keyed by `ARM_NAME`, and the same name is a different
point in space on the other arm. A caller that sent values would have to read
that file, making the pose library shared state between containers instead of
something one container owns. `pose_names` advertises what is available, re-read
each second so a pose saved by `teach.py` appears without a restart.

A named or joint move is **discrete**: sent once with a goal time from the
distance, and it cancels any streaming Cartesian target so the two cannot fight
over the arm mid-motion.
| PUB | `/joint_states` | `sensor_msgs/JointState` | `preview.py` (RViz only, unnamespaced) |

Everything else in that container — `pose.py`, `read_joints.py`, `gripper.py`,
`teach.py`, `move_joint.py` — publishes **no topics at all**. They talk to the
arm's SDK directly over Ethernet, which is why they cannot run while
`arm_agent.py` holds the connection.

## `middle-arm` — active vision (wx250s + ZED)

`<ns>` is `/middle`.

| Direction | Topic | Type | Node |
|---|---|---|---|
| SUB | `/middle/cmd_pose` | `geometry_msgs/PoseStamped` | `head_agent.py` |
| SUB | `/middle/enable` | `std_msgs/Bool` | `head_agent.py` |
| PUB | `/middle/ee_pose` | `geometry_msgs/PoseStamped` | `head_agent.py` |
| PUB | `/middle/active` | `std_msgs/Bool` | `head_agent.py` |
| PUB | `/middle/commands/joint_group` | `interbotix_xs_msgs/JointGroupCommand` | `head_agent.py`, `pose.py`, `move_joint.py` |
| PUB | `/middle/commands/joint_single` | `interbotix_xs_msgs/JointSingleCommand` | `move_joint.py`, `teleop_keyboard.py` |
| SUB | `/middle/joint_states` | `sensor_msgs/JointState` | all of the above |
| SUB | `/middle/robot_description` | `std_msgs/String` | `pose.py`, `move_joint.py`, `teleop_keyboard.py` |

`xs_sdk` publishes `/middle/joint_states` and `/middle/robot_description` and
subscribes to the two command topics. **`head_agent.py` is one more publisher to
`joint_group`, not a parallel path** — so it and the CLIs must not run together.

The ZED wrapper is launched separately and publishes under its own camera name:

| Direction | Topic | Type |
|---|---|---|
| PUB | `/middle_cam/zed_node/left/image_rect_color` | `sensor_msgs/Image` |
| PUB | `/middle_cam/zed_node/right/image_rect_color` | `sensor_msgs/Image` |

## `slate-base` — AGV: drive (x, y) + lift (z)

| Direction | Topic | Type | Node |
|---|---|---|---|
| PUB | `/slate/cmd_vel` | `geometry_msgs/Twist` | **`governor.py` only** |
| SUB | `/slate/cmd_vel_teleop` | `geometry_msgs/Twist` | `governor.py` |
| SUB | `/slate/cmd_vel_nav` | `geometry_msgs/Twist` | `governor.py` |
| PUB | `/slate/cmd_vel_teleop` | `geometry_msgs/Twist` | `move_base.py`, `teleop_keyboard.py` |
| SUB | `/slate/odom` | `nav_msgs/Odometry` | `read_base.py`, `move_base.py` |
| SUB | `/slate/battery_state` | `sensor_msgs/BatteryState` | `read_base.py` |
| SUB | `/slate/lift/cmd_velocity` | `std_msgs/Float32` | `lift_agent.py` |
| SUB | `/slate/lift/cmd_height` | `std_msgs/Float32` | `lift_agent.py` |
| PUB | `/slate/lift/height` | `std_msgs/Float32` | `lift_agent.py` |
| PUB | `/slate/lift/moving` | `std_msgs/Bool` | `lift_agent.py` |

The driver (`slate_base_node`) subscribes `/slate/cmd_vel` and publishes
`/slate/odom` and `/slate/battery_state`.

`sim_base.py` stands in for it with no hardware — same topics, same services,
same 300 ms deadline, same torque gate. Run one or the other, never both:
`SLATE_SIM=true` in `slate-base/docker-compose.yml`.

**Nothing but the governor publishes `/slate/cmd_vel`.** Everything else goes to
`cmd_vel_teleop` and is clamped on the way through. This matters more than it
looks: the governor republishes at 20 Hz to hold the driver's 300 ms deadline
open, so a second publisher on its output does not override it — the two
*interleave*, and the base alternates between setpoints 50 ms apart. That reads
as juddering, not as a conflict, which is why it would be hard to diagnose.

Services (unchanged, from the vendor driver): `/slate/set_text`,
`/slate/set_light_state`, `/slate/set_motor_torque_status`,
`/slate/enable_charging`.

## `quest` — teleop source

Publishes device state; publishes commands into *other* components' namespaces.
That direction is deliberate — input devices adapt to robots, not the reverse.

| Direction | Topic | Type | Node |
|---|---|---|---|
| PUB | `/quest/head/pose` | `geometry_msgs/PoseStamped` | `quest_driver.py` |
| PUB | `/quest/left/pose` | `geometry_msgs/PoseStamped` | `quest_driver.py` |
| PUB | `/quest/right/pose` | `geometry_msgs/PoseStamped` | `quest_driver.py` |
| PUB | `/quest/left/joy` | `sensor_msgs/Joy` | `quest_driver.py` |
| PUB | `/quest/right/joy` | `sensor_msgs/Joy` | `quest_driver.py` |
| PUB | `/quest/connected` | `std_msgs/Bool` | `quest_driver.py` |
| SUB | `/quest/feedback` | `std_msgs/String` | `quest_driver.py` (JSON → the Unity app) |
| SUB | `/middle_cam/zed_node/left/image_rect_color` | `sensor_msgs/Image` | `quest_driver.py` |
| SUB | `/middle_cam/zed_node/right/image_rect_color` | `sensor_msgs/Image` | `quest_driver.py` |

`quest_teleop.py` (and `keyboard_teleop.py`, which is interchangeable with it):

| Direction | Topic | Type |
|---|---|---|
| SUB | all `/quest/*` above | |
| SUB | `<arm>/ee_pose` for all three arms | `geometry_msgs/PoseStamped` |
| PUB | `<arm>/cmd_pose`, `<arm>/cmd_gripper`, `<arm>/enable` | |
| PUB | `/slate/cmd_vel_teleop` | `geometry_msgs/Twist` |
| PUB | `/quest/feedback` | `std_msgs/String` |

**Run only one of `quest_teleop.py` and `keyboard_teleop.py` at a time.** Two
sources publishing the same command topics is a fight neither wins.

## `monitor`

| Direction | Topic | Type | Node |
|---|---|---|---|
| SUB | *everything it discovers* | any | `watch.py` |
| PUB | `/slate/cmd_vel_teleop` | `geometry_msgs/Twist` | `drive_test.py` |
| PUB | `<arm>/cmd_pose_name`, `<arm>/cmd_joints`, `<arm>/cmd_gripper`, `<arm>/enable` | | `arm_ctl.py` |
| SUB | `<arm>/ee_pose`, `<arm>/joint_states`, `<arm>/pose_names`, `<arm>/active` | | `arm_ctl.py` |
| PUB | `<arm>/cmd_pose`, `<arm>/cmd_gripper`, `<arm>/enable` for **all three arms** | | `rig_key.py` |
| PUB | `/slate/cmd_vel_teleop`, `/slate/lift/cmd_velocity` | `Twist`, `Float32` | `rig_key.py` |
| SRV | `/slate/set_motor_torque_status` | `std_srvs/SetBool` | `rig_key.py` |
| SUB | `<arm>/ee_pose` for all three arms | `geometry_msgs/PoseStamped` | `rig_key.py` |

`rig_key.py` is the single keyboard controller: it is the only client that
touches every component at once, which is why it is the longest row here.

`watch.py` subscribes with best-effort depth-1 QoS so it can never slow a
publisher down or hold a queue on its behalf.

## `sim` — the rig drawn in a browser

Subscribes to everything below and **publishes nothing at all**. There is no
command topic in this container and no code path that could create one, which
is what makes it safe to leave open while driving.

| Subscribes | Type | Used for |
|---|---|---|
| `/left_arm/joint_states` | `sensor_msgs/JointState` | `follower_left/joint_0…5`, gripper |
| `/right_arm/joint_states` | `sensor_msgs/JointState` | `follower_right/joint_0…5`, gripper |
| `/left_arm/ee_pose`, `/right_arm/ee_pose` | `geometry_msgs/PoseStamped` | the readouts in the side panel |
| `/left_arm/active`, `/right_arm/active` | `std_msgs/Bool` | whether each agent holds its arm |
| `/slate/odom` | `nav_msgs/Odometry` | where the base is, and the E-stop flag |

Served at `http://<host>:8080` (`make sim` prints the URL).

**It is a mirror, not a simulator.** Every angle comes from a real encoder. No
physics, no contact, no prediction — if the model is somewhere the robot is not,
the model is wrong. Good for "are the arms about to reach the same point" and
"which way is the wrist actually facing"; useless for "would this trajectory
collide", which needs a planner.

The joint names are the one mapping this container does: the arms publish
`joint_0…joint_5`, and the combined model prefixes them `follower_left/` and
`follower_right/`. A name that does not resolve is logged once — silence there
is the failure this is most exposed to, because an unmapped link simply stays at
zero and the picture still looks plausible.
