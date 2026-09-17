"""Forward kinematics, Jacobian and damped-least-squares IK for the WXAI, from its URDF.

WHY THIS EXISTS. The controller has its own Cartesian IK, and on this rig it is
unusable on the elbow-negative branch: it refuses outright with the elbow folded
past about -150 deg, and at -130 deg it once swung the shoulder 130 deg for a
1 cm request before refusing. But the rig's rest pose lives on that branch and
must stay there, and teleop has to work from wherever the arm is. So the arm
container solves its own IK -- exactly what the middle arm has always done --
and streams JOINT velocities, which the controller accepts anywhere.

Damped least squares, not a pseudo-inverse: near a singularity the damping
turns "huge joint velocity for a tiny Cartesian step" into "slower Cartesian
motion", which is the behaviour a hand on a controller expects. Per-joint
velocity caps on top.

Only numpy and the standard library. The URDF is the vendor's follower model,
copied beside this file; its base_link is the controller's base frame (checked:
FK differs from the controller's reported EE only by the fixed tool offset).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

TIP_LINK = "link_6"


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rot(axis, th):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


class ArmKinematics:
    def __init__(self, urdf_path, tip=TIP_LINK, base="base_link", tool_xyz=(0.0, 0.0, 0.0)):
        # The controller reports and commands the TOOL point, not link_6. The
        # offset (link_6 frame) was measured from two live poses: FK(link_6)
        # versus the controller's own Cartesian readout. The Jacobian must be
        # taken at the same point the error is measured at, or a wrist rotation
        # shows up as a translation error and the loop chases it -- that is
        # exactly how the first joint-IK jog ran the forearm roll to +90 deg.
        self.tool_xyz = np.asarray(tool_xyz, float)
        root = ET.parse(urdf_path).getroot()
        joints = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain = []
        link = tip
        while link != base:
            j = joints[link]
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            axis = None
            if j.get("type") in ("revolute", "continuous"):
                axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            chain.append((xyz, _rpy(*rpy), axis))
            link = j.find("parent").get("link")
        self.chain = list(reversed(chain))          # base -> tip
        self.n = sum(1 for c in self.chain if c[2] is not None)

    def fk(self, q):
        """4x4 pose of the tip in the base frame."""
        T = np.eye(4); k = 0
        for xyz, R, axis in self.chain:
            Tj = np.eye(4); Tj[:3, :3] = R; Tj[:3, 3] = xyz
            T = T @ Tj
            if axis is not None:
                Rq = np.eye(4); Rq[:3, :3] = _rot(axis, q[k]); T = T @ Rq; k += 1
        Tt = np.eye(4); Tt[:3, 3] = self.tool_xyz
        return T @ Tt

    def jacobian(self, q, h=1e-6):
        """6xn geometric Jacobian (linear; angular), base frame, by central differences."""
        J = np.zeros((6, self.n))
        T0 = self.fk(q)
        for i in range(self.n):
            qp = np.array(q, float); qp[i] += h
            qm = np.array(q, float); qm[i] -= h
            Tp, Tm = self.fk(qp), self.fk(qm)
            J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * h)
            dR = Tp[:3, :3] @ Tm[:3, :3].T
            J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * 2 * h)
        return J

    def dls(self, q, v_cart, damping=0.05, max_dq=1.0, w_ang=0.3):
        """Joint velocities for a Cartesian velocity (base frame), damped, capped.

        w_ang weights the three orientation rows below the translation rows.
        With a straight wrist (this rig's rest pose) the angular rows are
        near-singular on their own, and unweighted they drag the damping up
        until translation barely moves either. Attitude only needs holding
        softly; position is what the operator is steering.
        """
        J = self.jacobian(q)
        W = np.array([1, 1, 1, w_ang, w_ang, w_ang], float)
        Jw = W[:, None] * J
        vw = W * np.asarray(v_cart, float)
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + (damping ** 2) * np.eye(6), vw)
        m = np.max(np.abs(dq))
        if m > max_dq:
            dq *= max_dq / m
        return dq

    def manipulability(self, q):
        s = np.linalg.svd(self.jacobian(q), compute_uv=False)
        return float(s.min() / s.max())
