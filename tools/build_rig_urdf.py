#!/usr/bin/env python3
"""Compose the rig's URDF from the vendor models plus this rig's own geometry.

    ./tools/build_rig_urdf.py            # -> sim/description/urdf/rig.urdf
    ./tools/build_rig_urdf.py --check    # report, write nothing

WHY THIS EXISTS RATHER THAN A HAND-EDITED URDF
----------------------------------------------
Three models have to become one, and two of them are vendor files that will be
re-vendored when the vendors update them:

  * mobile_ai.urdf              Trossen: SLATE base, wheels, casters, two wxai
  * wx250s.urdf                 Interbotix: the middle arm
  * scissor_lift.stl            ours: one rigid body on a prismatic joint

Hand-merging them produces a file nobody can re-derive, and the next vendor
update silently discards whatever was fixed by hand. So the merge is a script
and the rig-specific numbers live in sim/rig_params.yaml.

WHAT IT FIXES IN THE VENDOR MODEL
---------------------------------
Trossen's mobile_ai.urdf mounts both arms with rpy="0 0 0", which is right for a
stock Mobile AI and wrong for this rig -- the arms here are rotated, which is
visible immediately as the arm bases sitting at the wrong attitude. The correct
rotation is not a guess: it is the inverse of the ARM_WORLD_* command remap in
manip-arm/docker-compose.yml, which was tuned against the real arms until the
axis keys moved them the way the operator expected. THOSE TWO MUST STAY IN STEP.

NAMESPACING. The Interbotix xacro is generated with robot_name:=middle, which
prefixes its LINKS (middle/base_link, middle/shoulder_link, ...) and leaves its
JOINTS bare (waist, shoulder, elbow, ...). That asymmetry is theirs, not a
mistake here, and it is left alone on purpose: the bare names are exactly what
the middle arm's driver publishes on joint_states, so renaming them would buy a
tidier model at the cost of a mapping layer that could silently drift. Nothing
in Trossen's model uses those names, so there is no collision to avoid.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(ROOT, "sim", "description")
PARAMS = os.path.join(ROOT, "sim", "rig_params.yaml")
BASE_URDF = os.path.join(DESC, "trossen_arm_description", "urdf",
                         "generated", "mobile_ai.urdf")
MIDDLE_URDF = os.path.join(DESC, "urdf", "wx250s.urdf")
OUT_URDF = os.path.join(DESC, "urdf", "rig.urdf")



def fmt(vals):
    return " ".join(f"{float(v):.6g}" for v in vals)


def set_origin(joint, xyz, rpy):
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt(xyz))
    origin.set("rpy", fmt(rpy))


def build(params, check=False):
    notes = []
    tree = ET.parse(BASE_URDF)
    robot = tree.getroot()
    robot.set("name", "trossen_rig")

    joints = {j.get("name"): j for j in robot.findall("joint")}
    links = {l.get("name") for l in robot.findall("link")}

    # ---- 1. the two wxai mounts -----------------------------------------
    for side in ("left", "right"):
        name = f"follower_{side}/mount_joint"
        j = joints.get(name)
        if j is None:
            notes.append(f"MISSING {name} -- vendor URDF changed shape")
            continue
        cfg = params["arms"][side]
        before = j.find("origin")
        was = (before.get("xyz", "?"), before.get("rpy", "?")) if before is not None else ("?", "?")
        set_origin(j, cfg["xyz"], cfg["rpy"])
        notes.append(f"{name}: rpy {was[1]} -> {fmt(cfg['rpy'])}")

    # ---- 2. the scissor lift --------------------------------------------
    lift = params["lift"]
    lo, hi = lift["travel"]

    base = ET.SubElement(robot, "link")
    base.set("name", "lift_base")

    plat = ET.SubElement(robot, "link")
    plat.set("name", "lift_platform")
    vis = ET.SubElement(plat, "visual")
    geom = ET.SubElement(vis, "geometry")
    mesh = ET.SubElement(geom, "mesh")
    # The STL is exported in millimetres, like every other SolidWorks export in
    # assets/. URDF is metres, so it is scaled here rather than by re-exporting
    # -- the file is shared with the CAD workflow and should stay as CAD wrote it.
    mesh.set("filename", "package://rig/meshes_rig/scissor_lift.stl")
    mesh.set("scale", "0.001 0.001 0.001")

    jb = ET.SubElement(robot, "joint")
    jb.set("name", "lift_base_joint")
    jb.set("type", "fixed")
    set_origin(jb, lift["xyz"], lift["rpy"])
    ET.SubElement(jb, "parent").set("link", "base_link")
    ET.SubElement(jb, "child").set("link", "lift_base")

    jl = ET.SubElement(robot, "joint")
    jl.set("name", "lift_joint")
    jl.set("type", "prismatic")
    set_origin(jl, [0, 0, 0], [0, 0, 0])
    ET.SubElement(jl, "parent").set("link", "lift_base")
    ET.SubElement(jl, "child").set("link", "lift_platform")
    ET.SubElement(jl, "axis").set("xyz", "0 0 1")
    lim = ET.SubElement(jl, "limit")
    lim.set("lower", fmt([lo])); lim.set("upper", fmt([hi]))
    lim.set("effort", "100"); lim.set("velocity", "0.1")
    notes.append(f"lift: prismatic z, {lo}..{hi} m, at {fmt(lift['xyz'])}")

    # ---- 3. the middle arm, namespaced ----------------------------------
    if not os.path.exists(MIDDLE_URDF):
        notes.append(f"MISSING {MIDDLE_URDF} -- middle arm omitted")
    else:
        mid = ET.parse(MIDDLE_URDF).getroot()
        mid_links = {l.get("name") for l in mid.findall("link")}
        # Materials are a flat, global namespace in URDF, and both vendors ship
        # one for their own black plastic. Appending both leaves a duplicate
        # definition, which parsers are entitled to resolve either way -- so the
        # incoming one is dropped when the name is already taken.
        have_materials = {m.get("name") for m in robot.findall("material")}
        for el in list(mid):
            if el.tag == "material":
                if el.get("name") in have_materials:
                    continue
                have_materials.add(el.get("name"))
            if el.tag in ("link", "joint", "material"):
                robot.append(el)

        m = params["middle"]
        parent = m.get("parent", "lift_platform")
        # The middle arm rides ON the lift, so its parent is the moving platform
        # and not base_link. Attaching it to base_link would leave it hanging in
        # space while the lift travelled underneath it.
        jm = ET.SubElement(robot, "joint")
        jm.set("name", "middle_mount_joint")
        jm.set("type", "fixed")
        set_origin(jm, m["xyz"], m["rpy"])
        ET.SubElement(jm, "parent").set("link", parent)
        # The root is whichever link nothing else claims as a child -- asked of
        # the model rather than assumed from robot_name, so a vendor rename does
        # not silently attach the rig to the wrong link.
        children = {c.get("link") for j in mid.findall("joint")
                    for c in j.findall("child")}
        roots = sorted(mid_links - children)
        if len(roots) != 1:
            notes.append(f"MISSING single root in {MIDDLE_URDF}: found {roots}")
            roots = roots[:1] or [sorted(mid_links)[0]]
        ET.SubElement(jm, "child").set("link", roots[0])
        notes.append(f"middle arm: {len(mid_links)} links, root {roots[0]}, on {parent}")

    if check:
        return None, notes

    ET.indent(tree, space="  ")
    tree.write(OUT_URDF, encoding="unicode", xml_declaration=True)
    return OUT_URDF, notes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="report, write nothing")
    args = ap.parse_args()

    with open(PARAMS) as f:
        params = yaml.safe_load(f)

    out, notes = build(params, check=args.check)
    for n in notes:
        print(f"  {n}")
    if out:
        print(f"\n  wrote {os.path.relpath(out, ROOT)}")
        print("  restart the visualiser to pick it up:  docker compose restart sim")
    bad = [n for n in notes if n.startswith("MISSING")]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
