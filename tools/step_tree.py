#!/usr/bin/env python3
"""Read a STEP assembly's component tree and each component's placement.

    ./step_tree.py assets/Mobile_Base.STEP
    ./step_tree.py assets/Mobile_Base.STEP --units m

A STEP assembly stores what a URDF needs -- which parts exist and where each one
sits relative to its parent -- but stores it as a graph of numbered entities
rather than anything you can read. This walks that graph and prints the tree
with a translation and rotation per component.

WHY NOT JUST OPEN IT IN CAD. Because the numbers have to end up in a text file
eventually, and reading them off a CAD GUI by hand is where transcription errors
come from. This is the same data, in the form the xacro wants.

WHAT IT DOES NOT DO. It does not invent joints. A STEP file has no concept of
"this is revolute about that axis" -- it is one rigid pose per part. What comes
out is the SKELETON: every component and its fixed transform. Turning the ones
that move into joints is a decision, made against docs/frames.md, not something
the file can tell you.

HOW THE GRAPH FITS TOGETHER, since the entity names are opaque:

    NEXT_ASSEMBLY_USAGE_OCCURRENCE   parent product -> child product ("A contains B")
    PRODUCT_DEFINITION               a product's definition, names it
    CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
                                     ties one of those occurrences to a transform
    ITEM_DEFINED_TRANSFORMATION      the transform: two AXIS2_PLACEMENT_3D
    AXIS2_PLACEMENT_3D               origin + z axis + x axis
"""
import argparse
import math
import re
import sys

ENT = re.compile(r"^#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*)\)\s*;?\s*$")
# STEP "complex entities": one id carrying several types at once, written
#   #3297 =( REPRESENTATION_RELATIONSHIP (...) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION (...) ... );
# The transform that positions every component in this assembly lives in one of
# these, so a parser that only handles the simple form finds no placements at
# all -- which is exactly what happened first time round.
COMPLEX = re.compile(r"^#(\d+)\s*=\s*\((.*)\)\s*;?\s*$")
REF = re.compile(r"#(\d+)")

# Only these are kept. The file is ~100 MB and mostly BREP faces we never touch.
WANTED = {
    # SolidWorks emits PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE, not
    # the bare PRODUCT_DEFINITION_FORMATION -- matching only the short name
    # silently resolved zero product names.
    "PRODUCT", "PRODUCT_DEFINITION", "PRODUCT_DEFINITION_FORMATION",
    "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
    "PRODUCT_DEFINITION_SHAPE", "NEXT_ASSEMBLY_USAGE_OCCURRENCE",
    "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
    "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
    "SHAPE_REPRESENTATION_RELATIONSHIP", "ITEM_DEFINED_TRANSFORMATION",
    "AXIS2_PLACEMENT_3D", "CARTESIAN_POINT", "DIRECTION",
}


def parse(path):
    ents = {}
    buf = ""
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            # Entities can wrap across lines; accumulate until the terminator.
            buf += raw.strip()
            if not buf.endswith(";"):
                continue
            m = ENT.match(buf)
            if m:
                eid, kind, args = int(m.group(1)), m.group(2), m.group(3)
                if kind in WANTED:
                    ents[eid] = (kind, args)
                buf = ""
                continue
            c = COMPLEX.match(buf)
            buf = ""
            if c and "REPRESENTATION_RELATIONSHIP" in c.group(2):
                # Keep the whole payload; the caller just scans it for refs.
                ents[int(c.group(1))] = ("COMPLEX", c.group(2))
    return ents


def strings(args):
    return re.findall(r"'((?:[^']|'')*)'", args)


def floats(args):
    return [float(x) for x in re.findall(r"[-+]?\d*\.\d+(?:[EeDd][-+]?\d+)?|[-+]?\d+\.", args)]


def clean(name):
    """STEP escapes non-ASCII as \\X\\hh or \\X2\\....\\X0\; show it readably."""
    n = re.sub(r"\\X2\\[0-9A-Fa-f]+\\X0\\", "?", name)
    n = re.sub(r"\\X\\[0-9A-Fa-f]{2}", "?", n)
    n = n.replace("''", "'").strip()
    return n or "(unnamed)"


def placement(ents, eid):
    """AXIS2_PLACEMENT_3D -> (origin, z_axis, x_axis)."""
    kind, args = ents.get(eid, (None, ""))
    if kind != "AXIS2_PLACEMENT_3D":
        return None
    refs = [int(r) for r in REF.findall(args)]
    out = []
    for r in refs[:3]:
        k, a = ents.get(r, (None, ""))
        out.append(tuple(floats(a)[:3]) if k in ("CARTESIAN_POINT", "DIRECTION") else None)
    while len(out) < 3:
        out.append(None)
    return out[0] or (0, 0, 0), out[1] or (0, 0, 1), out[2] or (1, 0, 0)


def mat_from_placement(p):
    """AXIS2_PLACEMENT_3D -> 4x4 as (R, t). z is the local Z, x the local X."""
    o, z, x = p
    def norm(v):
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return [c / n for c in v]
    z = norm(z)
    x = norm(x)
    d = sum(a * b for a, b in zip(x, z))          # re-orthogonalise; STEP allows slop
    x = norm([x[i] - d * z[i] for i in range(3)])
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    R = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    return R, list(o)


def relative(p1, p2):
    """The transform an ITEM_DEFINED_TRANSFORMATION actually means.

    It carries TWO placements, not a translation. The component's transform is
    M(p2) . M(p1)^-1 -- placement 1 undone, placement 2 applied. Subtracting the
    two origins, which is the obvious-looking shortcut, is only correct when
    both rotations are identity, and silently reports rpy=(0,0,0) for every
    component when they are not. That is what this function exists to avoid.
    """
    R1, t1 = mat_from_placement(p1)
    R2, t2 = mat_from_placement(p2)
    # inverse of (R1, t1) is (R1^T, -R1^T t1)
    R1i = [[R1[j][i] for j in range(3)] for i in range(3)]
    t1i = [-sum(R1i[i][k] * t1[k] for k in range(3)) for i in range(3)]
    R = [[sum(R2[i][k] * R1i[k][j] for k in range(3)) for j in range(3)]
         for i in range(3)]
    t = [sum(R2[i][k] * t1i[k] for k in range(3)) + t2[i] for i in range(3)]
    return R, t


def rpy_from_matrix(R):
    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    if sy > 1e-9:
        return (math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], sy),
                math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)


def rpy_from_axes(z, x):
    """z axis + x axis -> roll/pitch/yaw, the form a URDF <origin> wants."""
    def norm(v):
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return [c / n for c in v]
    z = norm(z)
    x = norm(x)
    # Re-orthogonalise x against z; STEP allows them to be slightly off.
    d = sum(a * b for a, b in zip(x, z))
    x = norm([x[i] - d * z[i] for i in range(3)])
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    R = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    if sy > 1e-9:
        return (math.atan2(R[2][1], R[2][2]), math.atan2(-R[2][0], sy),
                math.atan2(R[1][0], R[0][0]))
    return (math.atan2(-R[1][2], R[1][1]), math.atan2(-R[2][0], sy), 0.0)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step")
    ap.add_argument("--units", choices=["mm", "m"], default="mm",
                    help="SolidWorks exports mm; --units m divides by 1000")
    ap.add_argument("--filter", help="only show components matching this")
    ap.add_argument("--relative-to", metavar="NAME",
                    help="re-express every top-level placement in this "
                         "component's frame -- e.g. --relative-to SLATE. "
                         "This is what a URDF needs: poses relative to "
                         "base_link, not to whatever the CAD origin happens "
                         "to be.")
    args = ap.parse_args()

    print(f"  parsing {args.step} ...", file=sys.stderr)
    ents = parse(args.step)
    print(f"  {len(ents)} structural entities kept", file=sys.stderr)

    # product_definition -> readable product name
    pd_name = {}
    for eid, (kind, a) in ents.items():
        if kind != "PRODUCT_DEFINITION":
            continue
        for r in (int(x) for x in REF.findall(a)):
            k2, a2 = ents.get(r, (None, ""))
            if k2 and k2.startswith("PRODUCT_DEFINITION_FORMATION"):
                for r2 in (int(x) for x in REF.findall(a2)):
                    k3, a3 = ents.get(r2, (None, ""))
                    if k3 == "PRODUCT":
                        s = strings(a3)
                        if s:
                            pd_name[eid] = clean(s[0])
    # occurrence -> (parent_pd, child_pd)
    occ = {}
    for eid, (kind, a) in ents.items():
        if kind == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            refs = [int(x) for x in REF.findall(a)]
            if len(refs) >= 2:
                occ[eid] = (refs[0], refs[1])

    # occurrence -> transform, via CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
    xf = {}
    for eid, (kind, a) in ents.items():
        if kind != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            continue
        refs = [int(x) for x in REF.findall(a)]
        rel, pds = (refs + [None, None])[:2]
        target = None
        k, a2 = ents.get(pds, (None, ""))
        if k == "PRODUCT_DEFINITION_SHAPE":
            for r in (int(x) for x in REF.findall(a2)):
                if r in occ:
                    target = r
        if target is None:
            continue
        k, a3 = ents.get(rel, (None, ""))
        for r in (int(x) for x in REF.findall(a3)):
            k4, a4 = ents.get(r, (None, ""))
            if k4 == "ITEM_DEFINED_TRANSFORMATION":
                pr = [int(x) for x in REF.findall(a4)]
                if len(pr) >= 2:
                    p1, p2 = placement(ents, pr[0]), placement(ents, pr[1])
                    if p1 and p2:
                        xf[target] = (p1, p2)

    scale = 1.0 if args.units == "mm" else 0.001
    children = {}
    for eid, (par, ch) in occ.items():
        children.setdefault(par, []).append((eid, ch))
    all_children = {ch for _, ch in occ.values()}
    roots = [pd for pd in pd_name if pd not in all_children and pd in children]

    print(f"\n  {len(occ)} component placements, {len(pd_name)} named products")
    print(f"  units: {args.units}\n")

    def walk(pd, depth, seen):
        if depth > 6 or pd in seen:
            return
        for eid, ch in sorted(children.get(pd, []), key=lambda t: pd_name.get(t[1], "")):
            name = pd_name.get(ch, f"#{ch}")
            if args.filter and args.filter.lower() not in name.lower():
                walk(ch, depth + 1, seen | {pd})
                continue
            t = xf.get(eid)
            if t:
                # Argument order matters and the convention is not obvious.
                # In this SolidWorks export transform_item_2 is always identity
                # and transform_item_1 carries the placement, so the component's
                # pose in its parent is M(item_1) . M(item_2)^-1 -- i.e. p1
                # first. Passing them the other way round yields the INVERSE,
                # which looks plausible (real rotations, sane magnitudes) while
                # putting the scissor lift 2.5 m from the base it sits on.
                R, tr = relative(t[1], t[0])
                pos = tuple(v * scale for v in tr)
                rpy = rpy_from_matrix(R)
                print(f"{'  ' * depth}  {name[:46]:<46} "
                      f"xyz=({pos[0]:9.2f} {pos[1]:9.2f} {pos[2]:9.2f})  "
                      f"rpy=({rpy[0]:+.3f} {rpy[1]:+.3f} {rpy[2]:+.3f})")
            else:
                print(f"{'  ' * depth}  {name[:46]:<46} (no transform)")
            walk(ch, depth + 1, seen | {pd})

    if args.relative_to:
        # Collect the top-level placements, then express each in the chosen
        # component's frame: T_ref^-1 . T_component.
        top = []
        for r in roots:
            for eid, ch in children.get(r, []):
                if eid in xf:
                    R, t = relative(xf[eid][1], xf[eid][0])
                    top.append((pd_name.get(ch, f"#{ch}"), R, t))
        ref = next((e for e in top if args.relative_to.lower() in e[0].lower()), None)
        if ref is None:
            print(f"  no top-level component matching {args.relative_to!r}",
                  file=sys.stderr)
            return 2
        _, Rr, tr = ref
        Rri = [[Rr[j][i] for j in range(3)] for i in range(3)]
        print(f"  Top-level components, expressed in the frame of {ref[0]!r}:\n")
        print(f"    {'component':<40} {'xyz (' + args.units + ')':<30} rpy (rad)")
        print("    " + "-" * 88)
        for name, R, t in sorted(top):
            d = [t[i] - tr[i] for i in range(3)]
            trel = [sum(Rri[i][k] * d[k] for k in range(3)) * scale for i in range(3)]
            Rrel = [[sum(Rri[i][k] * R[k][j] for k in range(3)) for j in range(3)]
                    for i in range(3)]
            rpy = rpy_from_matrix(Rrel)
            print(f"    {name[:40]:<40} "
                  f"({trel[0]:8.3f} {trel[1]:8.3f} {trel[2]:8.3f})   "
                  f"({rpy[0]:+.4f} {rpy[1]:+.4f} {rpy[2]:+.4f})")
        return 0

    for r in roots:
        print(f"  ROOT: {pd_name.get(r, r)}")
        walk(r, 1, set())
    return 0


if __name__ == "__main__":
    sys.exit(main())
