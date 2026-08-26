#!/usr/bin/env python3
"""Merge a URDF's per-link meshes into ONE assembled STL, posed by forward kinematics.

    ./assemble_arm.py                                  # WXAI at all-zeros, mm
    ./assemble_arm.py --joints 0 0.08 -1.49 0.03 0.01 3.13
    ./assemble_arm.py --units m -o arm_metres.stl
    ./assemble_arm.py --urdf other.urdf --meshes /path/to/meshes

WHY THIS EXISTS. Trossen ships the WXAI as per-link meshes plus a URDF, and as
18 individual cosmetic STEP parts. There is no single-file model of the whole
arm anywhere in the assets. For CAD work you want one body you can drop into an
assembly and mate, not twelve you have to position by hand -- and positioning
them by hand means re-deriving the kinematics that the URDF already states
exactly.

So: read the joint origins from the URDF, run forward kinematics, transform each
link's mesh into base_link coordinates, and write the union as one STL.

THE ORIGIN IS base_link. Every vertex is expressed in the arm's own base frame,
whose origin sits exactly on the bottom mounting face (verified: base_link.stl
spans z 0 -> 60.2 mm). So when you mate this into an assembly, the STL origin IS
the URDF origin, and whatever transform your CAD reports is the number that goes
straight into the xacro. No offset arithmetic, no chance of being one face
thickness out.

UNITS. STL carries no units -- it is bare numbers. URDF meshes are metres, and
CAD tools default to millimetres on import, so a metres-valued file reads as a
60-micron arm. This writes MILLIMETRES by default, which is what SolidWorks,
Fusion and FreeCAD expect. `--units m` keeps metres if you are feeding something
that wants them.

WHAT THIS IS NOT. Not a STEP file, and not a solid. STL is a triangle shell:
you can see it, measure it, mate to it, and use it as a positioning reference,
but you cannot fillet it or take a cross-section of solid material. Converting
mesh -> BREP is possible in FreeCAD, but the output is a shell of thousands of
planar faces, which is slower than the STL and no more useful for mating. If you
need true solids, the individual STEP parts in assets/widowX-AI/step/ are real
geometry -- they are just cosmetic covers, not the structure.
"""
import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_URDF = os.path.join(REPO, "assets", "widowX-AI", "wxai_base.urdf")
DEFAULT_MESHES = os.path.join(REPO, "assets", "widowX-AI", "meshes")


# ---- small matrix helpers (no numpy: this has to run anywhere) --------------
def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def axis_angle_to_mat(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return [
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def apply(R, t, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2] + t[0],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2] + t[1],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2] + t[2])


def rotate(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def compose(Ra, ta, Rb, tb):
    """(Ra,ta) then (Rb,tb) -- i.e. parent frame times child transform."""
    return mat_mul(Ra, Rb), tuple(apply(Ra, ta, tb))


# ---- STL ------------------------------------------------------------------
def read_stl(path):
    """[(normal, v0, v1, v2)]. Handles binary and ASCII."""
    with open(path, "rb") as f:
        data = f.read()

    # ASCII detection has to be careful: a binary STL's 80-byte header often
    # begins with the word "solid" too, which is the classic way to misparse one.
    # The reliable test is whether the declared triangle count matches the size.
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + n * 50 == len(data):
            tris = []
            for i in range(n):
                o = 84 + i * 50
                nx, ny, nz = struct.unpack("<3f", data[o:o + 12])
                vs = [struct.unpack("<3f", data[o + 12 + k * 12:o + 24 + k * 12])
                      for k in range(3)]
                tris.append(((nx, ny, nz), vs[0], vs[1], vs[2]))
            return tris

    text = data.decode("ascii", "ignore")
    tris, normal, verts = [], (0.0, 0.0, 0.0), []
    for line in text.splitlines():
        w = line.split()
        if not w:
            continue
        if w[0] == "facet" and len(w) >= 5:
            normal = tuple(float(v) for v in w[2:5])
        elif w[0] == "vertex":
            verts.append(tuple(float(v) for v in w[1:4]))
            if len(verts) == 3:
                tris.append((normal, verts[0], verts[1], verts[2]))
                verts = []
    return tris


def write_stl(path, tris, name="assembled"):
    with open(path, "wb") as f:
        f.write(name.encode("ascii", "ignore")[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for nrm, a, b, c in tris:
            f.write(struct.pack("<3f", *nrm))
            for v in (a, b, c):
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")


# ---- URDF -----------------------------------------------------------------
def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        mim = j.find("mimic")
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())],
            "rpy": [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())],
            "axis": [float(v) for v in (a.get("xyz", "1 0 0").split() if a is not None else "1 0 0".split())],
            "mimic": mim.get("joint") if mim is not None else None,
        })

    visuals = {}
    for l in root.findall("link"):
        v = l.find("visual")
        if v is None:
            continue
        mesh = v.find("geometry/mesh")
        if mesh is None:
            continue
        o = v.find("origin")
        visuals[l.get("name")] = {
            "mesh": os.path.basename(mesh.get("filename")),
            "xyz": [float(x) for x in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())],
            "rpy": [float(x) for x in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())],
        }
    return joints, visuals


def forward_kinematics(joints, q):
    """link name -> (R, t) in base_link. q maps joint name -> value."""
    children = {}
    parents = set()
    for j in joints:
        children.setdefault(j["parent"], []).append(j)
        parents.add(j["child"])
    roots = [j["parent"] for j in joints if j["parent"] not in parents]
    root = roots[0] if roots else joints[0]["parent"]

    frames = {root: ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0.0, 0.0, 0.0))}
    stack = [root]
    while stack:
        link = stack.pop()
        Rp, tp = frames[link]
        for j in children.get(link, []):
            Rj = rpy_to_mat(*j["rpy"])
            tj = tuple(j["xyz"])
            # A mimic joint follows its source 1:1 here. Correct for the WXAI,
            # whose right carriage mimics the left with no multiplier or offset;
            # a URDF using those would need them applied.
            src = j["mimic"] or j["name"]
            val = q.get(src, 0.0)
            if j["type"] == "revolute" or j["type"] == "continuous":
                Rj = mat_mul(Rj, axis_angle_to_mat(j["axis"], val))
            elif j["type"] == "prismatic":
                n = math.sqrt(sum(a * a for a in j["axis"])) or 1.0
                tj = tuple(tj[i] + j["axis"][i] / n * val for i in range(3))
            frames[j["child"]] = compose(Rp, tp, Rj, tj)
            stack.append(j["child"])
    return frames


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--meshes", default=DEFAULT_MESHES)
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--units", choices=["mm", "m"], default="mm")
    ap.add_argument("--joints", nargs="*", type=float, default=None,
                    help="joint_0..joint_5 in radians (default all zero)")
    ap.add_argument("--gripper", type=float, default=0.0,
                    help="carriage travel in metres, 0=closed 0.04=open")
    args = ap.parse_args()

    out = args.out or os.path.join(
        os.path.dirname(args.urdf),
        f"wxai_assembled_{args.units}.stl")

    joints, visuals = parse_urdf(args.urdf)

    q = {}
    if args.joints:
        for i, v in enumerate(args.joints[:6]):
            q[f"joint_{i}"] = v
    q["left_carriage_joint"] = args.gripper

    frames = forward_kinematics(joints, q)
    scale = 1000.0 if args.units == "mm" else 1.0

    merged, missing = [], []
    print(f"  urdf   {args.urdf}")
    print(f"  meshes {args.meshes}")
    print(f"  units  {args.units}  (scale x{scale:g})\n")

    for link, vis in sorted(visuals.items()):
        path = os.path.join(args.meshes, vis["mesh"])
        if not os.path.exists(path):
            missing.append(vis["mesh"])
            continue
        if link not in frames:
            print(f"    {link:18} SKIPPED -- not connected to the tree")
            continue

        Rl, tl = frames[link]
        # A link's <visual><origin> is an extra transform inside the link.
        Rv, tv = compose(Rl, tl, rpy_to_mat(*vis["rpy"]), tuple(vis["xyz"]))

        tris = read_stl(path)
        for nrm, a, b, c in tris:
            merged.append((
                rotate(Rv, nrm),
                tuple(x * scale for x in apply(Rv, tv, a)),
                tuple(x * scale for x in apply(Rv, tv, b)),
                tuple(x * scale for x in apply(Rv, tv, c)),
            ))
        print(f"    {link:18} {len(tris):6} tris   origin "
              f"({tl[0] * scale:8.2f} {tl[1] * scale:8.2f} {tl[2] * scale:8.2f})")

    if missing:
        print(f"\n  MISSING MESHES: {', '.join(missing)}", file=sys.stderr)
    if not merged:
        print("  nothing assembled", file=sys.stderr)
        return 1

    write_stl(out, merged, name=f"wxai assembled ({args.units})")

    xs = [v[0] for t in merged for v in t[1:]]
    ys = [v[1] for t in merged for v in t[1:]]
    zs = [v[2] for t in merged for v in t[1:]]
    print(f"\n  wrote {out}")
    print(f"    {len(merged)} triangles, {os.path.getsize(out) / 1e6:.1f} MB")
    print(f"    bbox {args.units}:  x {min(xs):8.2f} .. {max(xs):8.2f}")
    print(f"                  y {min(ys):8.2f} .. {max(ys):8.2f}")
    print(f"                  z {min(zs):8.2f} .. {max(zs):8.2f}")
    print(f"    origin is base_link -- mate to this directly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
