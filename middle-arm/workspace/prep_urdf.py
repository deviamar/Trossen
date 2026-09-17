#!/usr/bin/env python3
"""Make the dumped wx250s URDF describe THIS arm: no gripper, a ZED on the flange.

    ./launch-arm.sh --dump-urdf | ./prep_urdf.py > /tmp/middle_arm.urdf
    ./prep_urdf.py /tmp/raw.urdf > /tmp/middle_arm.urdf

Two fixes, both required before head_agent.py's solver can be trusted:

1. THE PHANTOM GRIPPER JOINT COMES OUT. Even with use_gripper:=false and both
   show_gripper flags off, the vendor xacro still emits a `gripper` continuous
   joint driving gripper_prop_link. On this arm that servo (DYNAMIXEL ID 9) is
   physically removed -- the ZED sits in its place -- and the joint's presence
   is not cosmetic: pyroki counts SEVEN actuated joints where xs_sdk's arm
   group has SIX, so joint_states get mis-sliced and a solved configuration
   would be one value longer than the JointGroupCommand it is sent as.

2. THE ZED GETS A LINK. The vendor xacro ends at the flange; the solver needs
   the pose target to mean the CAMERA, not the flange, or every commanded pose
   is off by the mount offset. A fixed joint grafts `camera_link` onto the
   flange with the measured offset from the environment:

       MIDDLE_ZED_PARENT   default <ROBOT_NAME>/ee_arm_link
       MIDDLE_ZED_XYZ      default "0 0 0"  -- METRES; measure and set me
       MIDDLE_ZED_RPY      default "0 0 0"  -- radians

   The zero default is usable (targets mean the flange, exactly as before) but
   wrong by the mount geometry -- measure the ZED's optical centre relative to
   the flange from CAD and put the numbers in middle-arm/docker-compose.yml.

Pure stdlib, no ROS: it runs anywhere the dumped URDF can be piped, and
start.sh runs it in the pipeline that generates the URDF the agent loads.
"""
import os
import sys
import xml.etree.ElementTree as ET

ROBOT_NAME = os.environ.get("ROBOT_NAME", "middle")
PARENT = os.environ.get("MIDDLE_ZED_PARENT", f"{ROBOT_NAME}/ee_arm_link")
XYZ = os.environ.get("MIDDLE_ZED_XYZ", "0 0 0")
RPY = os.environ.get("MIDDLE_ZED_RPY", "0 0 0")
CAMERA_LINK = os.environ.get("MIDDLE_EE_LINK", "camera_link")

# The seventh joint: the camera's own yaw motor (DYNAMIXEL ID 9). Transform
# taken from giava's wx250s_7dof URDF, which describes this wrist; set
# MIDDLE_YAW_JOINT="" to build the old six-joint model.
YAW_JOINT = os.environ.get("MIDDLE_YAW_JOINT", "camera_yaw")
YAW_XYZ = os.environ.get("MIDDLE_YAW_XYZ", "0.04125 0.03725 0")
YAW_RPY = os.environ.get("MIDDLE_YAW_RPY", "-1.5707963 0 0")
YAW_AXIS = os.environ.get("MIDDLE_YAW_AXIS", "0 0 1")
YAW_LOWER = os.environ.get("MIDDLE_YAW_LOWER", "-3.10")
YAW_UPPER = os.environ.get("MIDDLE_YAW_UPPER", "3.07")


def main():
    if len(sys.argv) > 1 and sys.argv[1] not in ("-",):
        src = open(sys.argv[1]).read()
    else:
        src = sys.stdin.read()

    root = ET.fromstring(src)

    # -- 1. the phantom gripper joint -------------------------------------
    for joint in list(root.findall("joint")):
        if joint.get("name") != "gripper":
            continue
        child = joint.find("child").get("link")
        root.remove(joint)
        print(f"  prep_urdf: removed joint 'gripper' (servo not fitted)",
              file=sys.stderr)
        for link in list(root.findall("link")):
            if link.get("name") == child:
                root.remove(link)
                print(f"  prep_urdf: removed link {child!r}", file=sys.stderr)

    # -- 2. the camera YAW joint -------------------------------------------
    # The seventh motor. The vendor xacro stops at the arm's own last axis
    # (wrist_rotate / camera_roll), so a model built from it has six joints and
    # a solver using it cannot turn the camera about its own vertical -- head
    # yaw came out as a few degrees of wrist roll and nothing else.
    #
    # Grafted onto OUR chain rather than swapping wholesale to giava's
    # wx250s_7dof URDF: the first six joints here are the vendor's, they match
    # what xs_sdk reports for this arm, and giava's model uses different zero
    # conventions (its waist travels [-6.35, 0]). Mixing those would corrupt
    # kinematics that currently work. Only the joint that was missing is taken
    # from them -- its mount transform, measured on the same hardware.
    # NOT `PARENT` (ee_arm_link). The vendor chain runs
    #   wrist_rotate -> gripper_link  --ee_arm (fixed, xyz 0.043 0 0)-->  ee_arm_link
    # and giava's wx250s_7dof mounts the yaw motor on the wrist_rotate OUTPUT
    # link (their `gripper_motor`, our `gripper_link`), not 43 mm beyond it.
    # Grafting on ee_arm_link stacked that 43 mm on top of the yaw joint's own
    # 41.25 mm offset, putting the yaw AXIS -- not just the camera -- twice as
    # far out as it is, so head yaw swept the camera through the wrong arc.
    # Both frames share x (both 6th-axis definitions are `1 0 0`), so giava's
    # origin transplants directly onto gripper_link.
    yaw_parent = os.environ.get("MIDDLE_YAW_PARENT", f"{ROBOT_NAME}/gripper_link")
    if YAW_JOINT and yaw_parent in {l.get("name") for l in root.findall("link")}:
        ET.SubElement(root, "link", name="camera_yaw_link")
        j = ET.SubElement(root, "joint", name=YAW_JOINT, type="revolute")
        ET.SubElement(j, "origin", xyz=YAW_XYZ, rpy=YAW_RPY)
        ET.SubElement(j, "parent", link=yaw_parent)
        ET.SubElement(j, "child", link="camera_yaw_link")
        ET.SubElement(j, "axis", xyz=YAW_AXIS)
        ET.SubElement(j, "limit", lower=YAW_LOWER, upper=YAW_UPPER,
                      effort="10", velocity="3.14")
        print(f"  prep_urdf: added revolute {YAW_JOINT!r} on {yaw_parent!r} "
              f"(axis {YAW_AXIS}, limits {YAW_LOWER}..{YAW_UPPER})", file=sys.stderr)
        PARENT_FOR_CAMERA = "camera_yaw_link"
    else:
        PARENT_FOR_CAMERA = PARENT

    # -- 3. the camera ------------------------------------------------------
    links = {l.get("name") for l in root.findall("link")}
    cam_parent = PARENT_FOR_CAMERA
    if cam_parent not in links:
        print(f"prep_urdf: parent link {cam_parent!r} is not in the URDF.\n"
              f"  have: {', '.join(sorted(links))}\n"
              "  Set MIDDLE_ZED_cam_parent to one of those.", file=sys.stderr)
        return 2
    if CAMERA_LINK in links:
        print(f"  prep_urdf: {CAMERA_LINK!r} already present -- left alone",
              file=sys.stderr)
    else:
        ET.SubElement(root, "link", name=CAMERA_LINK)
        j = ET.SubElement(root, "joint", name="zed_mount", type="fixed")
        ET.SubElement(j, "origin", xyz=XYZ, rpy=RPY)
        ET.SubElement(j, "parent", link=cam_parent)
        ET.SubElement(j, "child", link=CAMERA_LINK)
        note = ("MEASURE ME -- targets currently mean the flange"
                if XYZ.split() == ["0", "0", "0"] else "measured offset")
        print(f"  prep_urdf: added {CAMERA_LINK!r} on {cam_parent!r} "
              f"at xyz=({XYZ}) rpy=({RPY})  [{note}]", file=sys.stderr)

    sys.stdout.write(ET.tostring(root, encoding="unicode"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
