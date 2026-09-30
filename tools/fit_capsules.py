#!/usr/bin/env python3
"""Build rig/capsules.yaml: every arm's kinematic chain, mount, and one
circumscribed capsule per collision mesh -- the geometry the inter-arm
collision gate (rig/rig_collision.py) checks at 50 Hz.

    ./tools/fit_capsules.py            # -> rig/capsules.yaml
    ./tools/fit_capsules.py --check    # fit and report, write nothing

WHY CAPSULES, AND WHY CIRCUMSCRIBED
-----------------------------------
The gate answers one question per tick: "could this command bring two arms
within the margin?" A capsule is the cheapest shape with an exact, branch-free
distance function (segment-to-segment minus two radii), and a capsule that
CONTAINS its mesh makes the answer conservative in the safe direction:

    capsule_distance(q) <= true_mesh_distance(q)     for every q

so "capsule distance >= margin" PROVES the meshes are at least the margin
apart. The fit here is deliberately simple -- principal axis, then the smallest
radius that contains every vertex -- and its fatness (~10-30 mm over the true
surface on these links) is paid for in reach, never in safety. This is the
same argument giava's capsule_gate.py makes, with the mesh fit done offline so
the runtime needs numpy and nothing else.

WHERE THE NUMBERS COME FROM
---------------------------
  chains      the vendor URDFs (manip-arm/workspace/config/wxai_follower.urdf,
              sim/description/urdf/wx250s.urdf) plus the middle arm's camera
              yaw joint exactly as middle-arm/workspace/prep_urdf.py grafts it
  meshes      sim/description/{trossen_arm_description,interbotix_xsarm_descriptions}
  mounts      sim/rig_params.yaml -- THE SAME FILE THE 3D VIEW USES, so a
              corrected measurement fixes both at once. Its arm POSITIONS are
              marked as estimates there; the gate inherits that uncertainty,
              which is one reason its default margin is not small.

The middle arm ends in a ZED, not a gripper. Its camera is a hand-sized
capsule on the yaw link (MIDDLE_ZED_XYZ is still unmeasured), tagged 'camera'
so the gate gives it the largest margin.
"""
import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "rig", "capsules.yaml")
PARAMS = os.path.join(ROOT, "sim", "rig_params.yaml")
WXAI_URDF = os.path.join(ROOT, "manip-arm", "workspace", "config", "wxai_follower.urdf")
WXAI_MESHES = os.path.join(ROOT, "sim", "description", "trossen_arm_description",
                           "meshes", "wxai")
WX250S_URDF = os.path.join(ROOT, "sim", "description", "urdf", "wx250s.urdf")
WX250S_MESHES = os.path.join(ROOT, "sim", "description", "interbotix_xsarm_descriptions",
                             "meshes", "wx250s_meshes")

# The middle arm's seventh joint, verbatim from prep_urdf.py's defaults (which
# took them from giava's wx250s_7dof URDF). If those change, change these.
YAW_XYZ = [0.04125, 0.03725, 0.0]
YAW_RPY = [-1.5707963, 0.0, 0.0]
YAW_AXIS = [0.0, 0.0, 1.0]
YAW_LIMITS = [-3.10, 3.07]

# The ZED Mini body is 124.5 x 30.5 x 26.5 mm, long axis perpendicular to the
# yaw axis (yaw pans the view). Its height on the yaw motor is not measured
# (MIDDLE_ZED_XYZ=0 0 0), so the capsule must contain the body at ANY yaw.
# That sweep is a flat disc, and the tightest simple shape around a flat disc
# is a BALL at its centre with the body's half-diagonal as radius: a capsule
# along the yaw axis was tried and is fatter, because its end caps reach a
# whole radius beyond a 30 mm-thick body. ZED_H is the guess for the body
# centre above the yaw frame -- measure it (and which way the long axis
# points at camera_yaw = 0) and this can become a tight capsule along the body.
ZED_H = float(os.environ.get("MIDDLE_ZED_H", 0.03))
ZED_CENTER = [0.0, 0.0, ZED_H]
ZED_RADIUS = round(math.hypot(0.1245 / 2, 0.0305 / 2) + 0.001, 4)   # 64 mm


# ---------------------------------------------------------------------------
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def load_stl(path, scale=1.0):
    """Vertices of a binary STL (every vendor mesh here is binary)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        data = np.frombuffer(f.read(n * rec.itemsize), dtype=rec)
    return np.unique(data["v"].reshape(-1, 3), axis=0).astype(float) * scale


def fit_capsule(V, axis_hint=None):
    """Smallest simple capsule containing every vertex in V.

    Given an axis, the tightest containing capsule is closed-form: with r the
    largest radial offset, a vertex at axial position t and radial offset rho
    is inside iff it is within r of the segment, so the endpoints can come in
    no further than

        lo = min_v ( t_v + sqrt(r^2 - rho_v^2) )
        hi = max_v ( t_v - sqrt(r^2 - rho_v^2) )

    (a vertex out past an endpoint lives inside that end's hemisphere). Two
    candidate axes are tried -- the mesh's principal axis, and the line from
    this link's origin to its child joint, which is the physically meaningful
    long axis of an arm link -- and the smaller capsule wins. The first
    version iterated endpoints and radius against each other, which diverges
    (each pull-in raises the radius, which pulls in further) and collapsed
    every link into a fat sphere.
    """
    c = V.mean(axis=0)
    _, _, vt = np.linalg.svd(V - c, full_matrices=False)
    axes = [vt[0]]
    if axis_hint is not None and np.linalg.norm(axis_hint) > 0.03:
        axes.append(np.asarray(axis_hint, float) / np.linalg.norm(axis_hint))
    best = None
    for axis in axes:
        t = (V - c) @ axis
        rho = np.linalg.norm((V - c) - np.outer(t, axis), axis=1)
        r = float(rho.max()) + 1e-6
        reach = np.sqrt(np.maximum(r * r - rho * rho, 0.0))
        lo, hi = float((t + reach).min()), float((t - reach).max())
        if hi < lo:
            lo = hi = 0.5 * (lo + hi)
        p0, p1 = c + axis * lo, c + axis * hi
        # Exact containment check -- the guarantee the gate depends on.
        worst = float(seg_point_dist(p0, p1, V).max())
        if worst > r + 1e-6:
            r = worst
        vol = math.pi * r * r * (hi - lo) + 4.0 / 3.0 * math.pi * r ** 3
        if best is None or vol < best[3]:
            best = (p0, p1, r, vol)
    p0, p1, r, _ = best
    assert seg_point_dist(p0, p1, V).max() <= r + 1e-6
    return p0, p1, r


def seg_point_dist(p0, p1, P):
    d = p1 - p0
    L2 = float(d @ d)
    if L2 < 1e-12:
        return np.linalg.norm(P - p0, axis=1)
    s = np.clip(((P - p0) @ d) / L2, 0.0, 1.0)
    return np.linalg.norm(P - (p0 + np.outer(s, d)), axis=1)


def transform(V, xyz, rpy):
    return V @ rpy_matrix(*rpy).T + np.asarray(xyz, float)


def parse_chain(urdf_path, joint_names):
    """[{name, xyz, rpy, axis, lower, upper}] for the named actuated joints, in order."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    out = []
    for name in joint_names:
        j = joints[name]
        o = j.find("origin")
        lim = j.find("limit")
        out.append({
            "name": name,
            "xyz": [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()],
            "rpy": [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()],
            "axis": [float(v) for v in j.find("axis").get("xyz").split()],
            "lower": float(lim.get("lower")) if lim is not None else -math.pi,
            "upper": float(lim.get("upper")) if lim is not None else math.pi,
        })
    return out


def collision_origin(urdf_path, link_name):
    """(xyz, rpy, scale) of a link's <collision> mesh, or None."""
    root = ET.parse(urdf_path).getroot()
    for l in root.findall("link"):
        if l.get("name") != link_name:
            continue
        c = l.find("collision")
        if c is None:
            return None
        o = c.find("origin")
        m = c.find("geometry/mesh")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        scale = float((m.get("scale", "1 1 1") if m is not None else "1 1 1").split()[0])
        return xyz, rpy, scale
    return None


def capsule_entry(link, frame, V, klass, axis_hint=None):
    p0, p1, r = fit_capsule(V, axis_hint)
    return {"link": link, "frame": int(frame), "klass": klass,
            "p0": [round(float(v), 5) for v in p0],
            "p1": [round(float(v), 5) for v in p1],
            "r": round(r, 5), "n_vertices": int(len(V))}


# ---------------------------------------------------------------------------
def wxai_arm():
    """Chain + capsules for one WXAI, in its own base_link frame.

    frame k = the link after joint k-1 (link_k); frame 0 = base_link.
    Everything rigidly attached to link_6 -- the D405 mount, both carriages
    and both finger meshes -- is expressed in link_6's frame (frame 6). The
    carriages slide 0..0.044 m along their prismatic axes; their vertices are
    taken at BOTH ends of that travel so one capsule contains the whole sweep,
    whatever the gripper is doing.
    """
    joints = parse_chain(WXAI_URDF, [f"joint_{i}" for i in range(6)])
    caps = []
    caps.append(capsule_entry("base_link", 0, load_stl(f"{WXAI_MESHES}/base_link.stl"), "structure",
                              axis_hint=joints[0]["xyz"]))
    for k in range(1, 7):
        klass = "gripper" if k == 6 else "structure"
        # The link's long axis runs from its own origin to its child joint.
        hint = joints[k]["xyz"] if k < 6 else [0.0865, 0, 0]
        caps.append(capsule_entry(f"link_{k}", k, load_stl(f"{WXAI_MESHES}/link_{k}.stl"), klass,
                                  axis_hint=hint))

    # Fixed children of link_6, from the URDF.
    cam = load_stl(f"{WXAI_MESHES}/camera_mount_d405.stl")
    cam = transform(cam, [0.012, 0, 0], [0, 0, 0])
    # The D405 itself: camera_joint then camera_link_joint, then a 23x42x42 box.
    T1 = (np.array([0.02927207801, 0, 0.03824951197]), rpy_matrix(0, 0.3490658503988659, 0))
    box = np.array([[sx * 0.0115, sy * 0.021, sz * 0.021]
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + [-0.0078, -0.009, 0]
    box = box + [0.01085, 0.009, 0.021]                       # camera_link_joint
    box = box @ T1[1].T + T1[0] + [0.012, 0, 0]               # camera_joint, then mount
    caps.append(capsule_entry("camera_d405", 6, np.vstack([cam, box]), "camera"))

    for side, y0, ax in (("left", 0.023, 1.0), ("right", -0.023, -1.0)):
        carriage = load_stl(f"{WXAI_MESHES}/carriage_{side}.stl")
        finger = load_stl(f"{WXAI_MESHES}/gripper_{side}.stl")
        V = np.vstack([carriage, finger])
        closed = V + [0.0865, y0, 0.0]
        opened = V + [0.0865, y0 + ax * 0.044, 0.0]
        caps.append(capsule_entry(f"finger_{side}", 6, np.vstack([closed, opened]), "finger"))
    return joints, caps


def middle_arm():
    """Chain + capsules for the wx250s with the camera yaw joint and the ZED."""
    names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
    joints = parse_chain(WX250S_URDF, names)
    joints.append({"name": "camera_yaw", "xyz": YAW_XYZ, "rpy": YAW_RPY, "axis": YAW_AXIS,
                   "lower": YAW_LIMITS[0], "upper": YAW_LIMITS[1]})
    meshes = [  # (link name in the vendor URDF, mesh file, frame index, class)
        ("middle/base_link", "base", 0, "structure"),
        ("middle/shoulder_link", "shoulder", 1, "structure"),
        ("middle/upper_arm_link", "upper_arm", 2, "structure"),
        ("middle/upper_forearm_link", "upper_forearm", 3, "structure"),
        ("middle/lower_forearm_link", "lower_forearm", 4, "structure"),
        ("middle/wrist_link", "wrist", 5, "structure"),
        ("middle/gripper_link", "gripper", 6, "structure"),   # the wrist-rotate motor housing
    ]
    caps = []
    for link, mesh, frame, klass in meshes:
        xyz, rpy, scale = collision_origin(WX250S_URDF, link)
        V = transform(load_stl(f"{WX250S_MESHES}/{mesh}.stl", scale), xyz, rpy)
        hint = joints[frame]["xyz"] if frame < len(joints) else None
        caps.append(capsule_entry(link.split("/")[-1], frame, V, klass, axis_hint=hint))
    caps.append({"link": "zed_camera", "frame": 7, "klass": "camera",
                 "p0": ZED_CENTER, "p1": ZED_CENTER, "r": ZED_RADIUS, "n_vertices": 0,
                 "note": "ZED Mini any-yaw envelope (ball, body half-diagonal); height guessed (MIDDLE_ZED_H) -- measure"})
    return joints, caps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="fit and report, write nothing")
    args = ap.parse_args()

    with open(PARAMS) as f:
        params = yaml.safe_load(f)

    wj, wc = wxai_arm()
    mj, mc = middle_arm()

    def mount(entry):
        """An arm's mount. Anything parented to lift_platform gets the lift's
        own pose too, and the gate adds the live lift height on top."""
        m = {"parent": entry.get("parent", "base_link"),
             "xyz": entry["xyz"], "rpy": entry["rpy"]}
        if m["parent"] == "lift_platform":
            m["lift_xyz"] = params["lift"]["xyz"]
            m["lift_rpy"] = params["lift"]["rpy"]
        return m

    doc = {
        "generated_by": "tools/fit_capsules.py -- do not hand-edit; re-run after a mesh, URDF or rig_params change",
        "frame": "rig base_link (REP-103), the frame sim/rig_params.yaml is written in",
        "arms": {
            "left_arm": {
                "ns": "/left_arm",
                "joint_names": [f"joint_{i}" for i in range(6)],
                "mount": mount(params["arms"]["left"]),
                "joints": wj, "capsules": wc,
            },
            "right_arm": {
                "ns": "/right_arm",
                "joint_names": [f"joint_{i}" for i in range(6)],
                "mount": mount(params["arms"]["right"]),
                "joints": wj, "capsules": wc,
            },
            "middle": {
                "ns": "/middle",
                "joint_names": [j["name"] for j in mj],
                "mount": mount(params["middle"]),
                "joints": mj, "capsules": mc,
            },
        },
    }

    print(f"  {'arm':10s} {'link':16s} frame  class      length   radius   verts")
    for arm in ("left_arm", "middle"):
        for c in doc["arms"][arm]["capsules"]:
            L = float(np.linalg.norm(np.subtract(c["p1"], c["p0"])))
            print(f"  {arm:10s} {c['link']:16s} {c['frame']:5d}  {c['klass']:9s} "
                  f"{L * 1e3:6.1f}mm {c['r'] * 1e3:6.1f}mm  {c['n_vertices']:6d}")
    print("  (right_arm: same geometry as left_arm)")

    if args.check:
        return 0
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("# GENERATED by tools/fit_capsules.py -- see that file's docstring.\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"\n  wrote {os.path.relpath(OUT, ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
