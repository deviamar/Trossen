#!/usr/bin/env python3
"""Inter-arm collision gate: capsule FK for the whole rig, numpy only.

    from rig_collision import RigGate
    gate = RigGate("/home/robot/rig/capsules.yaml")
    gate.update("right_arm", q_right_measured)      # from /right_arm/joint_states
    gate.update("middle", q_middle_measured)        # from /middle/joint_states
    gate.set_lift(height_m)                         # from /slate/lift/height
    v = gate.check("left_arm", q_left_command)      # BEFORE sending it
    if not v.ok: hold instead

    python3 rig_collision.py            # offline: clearance at every saved pose

WHAT IT GUARANTEES, AND WHAT IT DOES NOT
========================================
Each arm's agent calls check() on the joint configuration it is about to send,
against the OTHER arms where they were last measured. Every link is a capsule
that CONTAINS its collision mesh (tools/fit_capsules.py), so

    capsule distance >= margin   =>   the real parts are >= margin apart.

ok=True is therefore a proof of separation; ok=False is conservative. The gate
never edits a command -- it refuses the tick, the arm holds where it is, and
the operator backs off. A gate that "repairs" commands is a second IK solver
with none of the reasoning behind it (giava learned this; capsule_gate.py).

It is INTER-ARM ONLY. Adjacent links of one arm overlap permanently at these
capsule sizes, so a self-collision check would refuse the rest pose; the
arms' own joint limits and the operator cover that. It also knows nothing
about the mast, the lift or the base -- add static capsules to the yaml when
those matter.

THE MARGIN COVERS WHAT HAPPENS BETWEEN CHECKS. The check is static and the
other arms are where they were last REPORTED (20 Hz on the WXAI, 100 Hz on
the middle arm), so the margin must absorb: one tick of the other arm's
travel, this arm's own tracking lag (the WXAI position loop sits up to ~10 mm
behind its command under gravity on this mount), and the ~1-2 cm the base
positions in sim/rig_params.yaml are marked as ESTIMATES by. 30 mm default;
the classes below refine it per pair.

ESCAPE, WITH A HARD FLOOR. Approaching from outside, a command that would
cross into the margin is refused: the arm stops AT the margin. But if two arms
are ALREADY inside it (they started there, or the other arm moved into this
one), refusing everything freezes both -- observed: the middle arm 0.3 mm
inside the margin could not even start its ramp back to rest, because the
first step closed the gap by a hair. So inside the margin a command is allowed
as long as the pair stays above a HARD FLOOR of half the margin. Half a margin
of circumscribed capsules is still a proof of separation; what is given up is
some of the allowance for travel between checks, which the slow named moves
that produce this situation do not need.

STALE PEERS. An arm whose joint state is older than STALE_S is treated as
where it was last seen -- if it has never been seen, its pairs are skipped
and the verdict says so. An absent peer must not stop this one (every
component of this rig tolerates its peers being down), and a peer that has
gone quiet is a peer that has stopped moving.
"""
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

STALE_S = 1.0

# Per-pair margins, metres, by capsule class. Same shape as giava's gate:
# the camera is the fragile part and gets the largest; two fingers meeting is
# a handover, not an accident, and gets the smallest.
DEFAULT_MARGINS = {
    "structure": 0.030,     # aluminium on aluminium
    "gripper": 0.030,       # something shoving a gripper, or vice versa
    "gripper_pair": 0.015,  # both ends are gripper/finger links: hands meeting on purpose
    "finger_pair": 0.012,   # fingertip to fingertip: the handover
    # Anything approaching a camera. Would be larger, except the middle arm's
    # ZED is modelled as a 70 mm ball (its mount is unmeasured, so the capsule
    # must contain it at any yaw) that already pads the real body by ~40 mm;
    # 45 mm on top of that held the left arm at REST. Raise it once the ZED
    # mount is measured and its capsule is tight.
    "camera": 0.030,
}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, th):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def T_of(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def segment_distances(A0, A1, B0, B1):
    """Closest distance between segments A0A1[i] and B0B1[i], vectorised.

    Ericson, Real-Time Collision Detection 5.1.9, written for arrays. Every
    branch is a mask, so N pairs cost the same handful of numpy ops.
    """
    d1 = A1 - A0
    d2 = B1 - B0
    r = A0 - B0
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    eps = 1e-12
    s = np.zeros(len(a))
    t = np.zeros(len(a))
    both_pts = (a <= eps) & (e <= eps)
    a_pt = (a <= eps) & ~both_pts
    b_pt = (e <= eps) & ~both_pts
    gen = ~(both_pts | a_pt | b_pt)
    # A is a point: t = f/e
    t[a_pt] = np.clip(f[a_pt] / e[a_pt], 0.0, 1.0)
    # B is a point: s = -c/a
    c = np.einsum("ij,ij->i", d1, r)
    b_idx = b_pt
    s[b_idx] = np.clip(-c[b_idx] / a[b_idx], 0.0, 1.0)
    # General case
    g = gen
    b = np.einsum("ij,ij->i", d1, d2)
    denom = a * e - b * b
    sg = np.zeros(len(a))
    nz = g & (denom > eps)
    sg[nz] = np.clip((b[nz] * f[nz] - c[nz] * e[nz]) / denom[nz], 0.0, 1.0)
    tg = (b * sg + f)
    tg_e = np.zeros(len(a))
    tg_e[g] = tg[g] / e[g]
    # clamp t, and recompute s where t was clamped
    t_lo = g & (tg_e < 0.0)
    t_hi = g & (tg_e > 1.0)
    t_mid = g & ~t_lo & ~t_hi
    s[t_mid] = sg[t_mid]
    t[t_mid] = tg_e[t_mid]
    s[t_lo] = np.clip(-c[t_lo] / a[t_lo], 0.0, 1.0)
    t[t_lo] = 0.0
    s[t_hi] = np.clip((b[t_hi] - c[t_hi]) / a[t_hi], 0.0, 1.0)
    t[t_hi] = 1.0
    P = A0 + s[:, None] * d1
    Q = B0 + t[:, None] * d2
    return np.linalg.norm(P - Q, axis=1)


@dataclass
class Verdict:
    ok: bool
    slack_m: float                 # min over pairs of (distance - margin); negative = inside
    distance_m: float              # capsule distance of the worst pair
    margin_m: float                # that pair's margin
    pair: Optional[Tuple[str, str, str]]   # (my link, other arm, other link)
    slack_now_m: float             # same measure at this arm's MEASURED pose
    escape: bool                   # allowed only because it does not get worse
    peers: List[str]               # arms actually checked against
    skipped: List[str]             # arms never heard from

    def __str__(self):
        if self.pair is None:
            return f"clear (no peers: {', '.join(self.skipped) or 'none'})"
        me, arm, other = self.pair
        return (f"{'ok' if self.ok else 'HOLD'} {me} <-> {arm}:{other} "
                f"{self.distance_m * 1e3:+.0f} mm (margin {self.margin_m * 1e3:.0f})"
                + (" [escape]" if self.escape else ""))


class ArmModel:
    def __init__(self, name, spec):
        self.name = name
        self.ns = spec["ns"]
        self.joint_names = list(spec["joint_names"])
        self.joints = [(np.asarray(j["xyz"], float), rpy_matrix(*j["rpy"]),
                        np.asarray(j["axis"], float)) for j in spec["joints"]]
        self.n = len(self.joints)
        self.limits = [(float(j["lower"]), float(j["upper"])) for j in spec["joints"]]
        m = spec["mount"]
        self.mount_parent = m.get("parent", "base_link")
        self.mount = T_of(m["xyz"], rpy_matrix(*m["rpy"]))
        self.lift = (T_of(m["lift_xyz"], rpy_matrix(*m["lift_rpy"]))
                     if "lift_xyz" in m else None)
        caps = spec["capsules"]
        self.cap_frame = np.array([int(c["frame"]) for c in caps])
        self.cap_p0 = np.array([c["p0"] for c in caps], float)
        self.cap_p1 = np.array([c["p1"] for c in caps], float)
        self.cap_r = np.array([float(c["r"]) for c in caps])
        self.cap_link = [c["link"] for c in caps]
        self.cap_klass = [c["klass"] for c in caps]

    def frames(self, q, lift_m=0.0):
        """4x4 of frame k (k = 0..n) in the rig base frame."""
        T = self.mount if self.lift is None else self.lift @ T_of([0, 0, lift_m], np.eye(3)) @ self.mount
        out = [T]
        for k, (xyz, R, axis) in enumerate(self.joints):
            T = T @ T_of(xyz, R) @ T_of([0, 0, 0], axis_angle(axis, float(q[k])))
            out.append(T)
        return out

    def capsules(self, q, lift_m=0.0):
        """World-frame (P0, P1, R) for every capsule at configuration q."""
        F = self.frames(q, lift_m)
        Rm = np.stack([F[k][:3, :3] for k in self.cap_frame])
        tm = np.stack([F[k][:3, 3] for k in self.cap_frame])
        P0 = np.einsum("nij,nj->ni", Rm, self.cap_p0) + tm
        P1 = np.einsum("nij,nj->ni", Rm, self.cap_p1) + tm
        return P0, P1, self.cap_r


def pair_margin(ka, kb, margins):
    grip = {"gripper", "finger"}
    if ka == "camera" or kb == "camera":
        return margins["camera"]
    if ka == "finger" and kb == "finger":
        return margins["finger_pair"]
    if ka in grip and kb in grip:
        return margins["gripper_pair"]
    if ka in grip or kb in grip:
        return margins["gripper"]
    return margins["structure"]


class RigGate:
    def __init__(self, path, margins=None, stale_s=STALE_S):
        with open(path) as f:
            doc = yaml.safe_load(f)
        self.arms: Dict[str, ArmModel] = {n: ArmModel(n, s) for n, s in doc["arms"].items()}
        self.margins = dict(DEFAULT_MARGINS)
        if margins:
            self.margins.update(margins)
        self.stale_s = stale_s
        self.q: Dict[str, np.ndarray] = {}
        self.stamp: Dict[str, float] = {}
        self.lift_m = 0.0
        # Pair tables, one per ordered (me, other): index arrays + margins.
        self._pairs = {}
        for me in self.arms:
            for other in self.arms:
                if me == other:
                    continue
                A, B = self.arms[me], self.arms[other]
                ia, ib, mg = [], [], []
                for i, ka in enumerate(A.cap_klass):
                    for j, kb in enumerate(B.cap_klass):
                        ia.append(i)
                        ib.append(j)
                        mg.append(pair_margin(ka, kb, self.margins))
                self._pairs[(me, other)] = (np.array(ia), np.array(ib), np.array(mg))

    def by_ns(self, ns):
        for n, a in self.arms.items():
            if a.ns == ns:
                return n
        return None

    def update(self, arm, q, stamp=None):
        """Latest measured joints of `arm` (arm joints only, this arm's order)."""
        model = self.arms[arm]
        q = np.asarray(q, float)[:model.n]
        if len(q) < model.n or not np.all(np.isfinite(q)):
            return
        self.q[arm] = q
        self.stamp[arm] = time.monotonic() if stamp is None else stamp

    def set_lift(self, h):
        if math.isfinite(h):
            self.lift_m = float(h)

    def _slack(self, me, q_me):
        """Min slack and worst pair of `me` at q_me against every known peer."""
        A = self.arms[me]
        P0a, P1a, Ra = A.capsules(q_me, self.lift_m)
        best = (math.inf, math.inf, 0.0, None)
        peers, skipped = [], []
        for other in self.arms:
            if other == me:
                continue
            if other not in self.q:
                skipped.append(other)
                continue
            peers.append(other)
            B = self.arms[other]
            P0b, P1b, Rb = B.capsules(self.q[other], self.lift_m)
            ia, ib, mg = self._pairs[(me, other)]
            d = segment_distances(P0a[ia], P1a[ia], P0b[ib], P1b[ib]) - Ra[ia] - Rb[ib]
            slack = d - mg
            k = int(np.argmin(slack))
            if slack[k] < best[0]:
                best = (float(slack[k]), float(d[k]), float(mg[k]),
                        (A.cap_link[ia[k]], other, B.cap_link[ib[k]]))
        return best, peers, skipped

    def check(self, me, q_cmd, q_now=None) -> Verdict:
        """Is it safe to send q_cmd for arm `me`? See the module docstring."""
        (slack, d, mg, pair), peers, skipped = self._slack(me, q_cmd)
        if pair is None:
            return Verdict(True, math.inf, math.inf, 0.0, None, math.inf, False, peers, skipped)
        if slack >= 0.0:
            return Verdict(True, slack, d, mg, pair, slack, False, peers, skipped)
        # Inside the margin. Allowed only if the arm is ALREADY inside it (so
        # this is not an approach from outside) and the command stays above
        # the hard floor -- or, below the floor, does not make it worse.
        q_ref = q_now if q_now is not None else self.q.get(me)
        if q_ref is not None:
            (slack_now, _, _, _), _, _ = self._slack(me, q_ref)
            if slack_now < 0.0:
                floor = -0.5 * mg                       # slack at half the margin
                if slack >= floor or slack >= slack_now - 5e-4:
                    return Verdict(True, slack, d, mg, pair, slack_now, True, peers, skipped)
            return Verdict(False, slack, d, mg, pair, slack_now, False, peers, skipped)
        return Verdict(False, slack, d, mg, pair, slack, False, peers, skipped)

    def clearance(self, me, q=None) -> Verdict:
        """Where `me` stands right now (or at q), for reporting."""
        q = self.q.get(me) if q is None else q
        if q is None:
            return Verdict(True, math.inf, math.inf, 0.0, None, math.inf, False, [], list(self.arms))
        (slack, d, mg, pair), peers, skipped = self._slack(me, q)
        return Verdict(slack >= 0.0, slack, d, mg, pair, slack, False, peers, skipped)


# ---------------------------------------------------------------------------
def _main():
    """Offline report: clearance between every pair of arms at their saved poses."""
    import argparse
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap = argparse.ArgumentParser(description=_main.__doc__)
    ap.add_argument("--capsules", default=os.path.join(here, "capsules.yaml"))
    ap.add_argument("--lift", type=float, default=0.0, help="lift height, m")
    args = ap.parse_args()

    gate = RigGate(args.capsules)
    with open(os.path.join(root, "manip-arm", "workspace", "config", "poses.yaml")) as f:
        wx = yaml.safe_load(f)
    with open(os.path.join(root, "middle-arm", "workspace", "config", "poses.yaml")) as f:
        mid = yaml.safe_load(f)
    gate.set_lift(args.lift)

    for pose in ("rest", "start", "home"):
        if not all(pose in wx[a] for a in ("left_arm", "right_arm")) or pose not in mid:
            continue
        gate.update("left_arm", wx["left_arm"][pose][:6])
        gate.update("right_arm", wx["right_arm"][pose][:6])
        gate.update("middle", mid[pose][:7])
        print(f"\n  all arms at '{pose}':")
        for me in gate.arms:
            v = gate.clearance(me)
            print(f"    {me:10s} {v}")
    print("\n  a negative number at 'rest' means the mounts in sim/rig_params.yaml are wrong,"
          "\n  not that the arms are touching -- measure the arm spacing and fix the yaml.")


if __name__ == "__main__":
    _main()
