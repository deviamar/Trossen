# Frame definitions

Every frame in the rig, defined precisely enough that two people measuring
independently get the same number. This exists because the previous version of
this task was a list of measurements with no statement of what they attached
to — which is unanswerable, not merely tedious.

Read this before measuring anything. The parameter table at the end is the only
part you actually have to fill in.

## Conventions

- **REP-103**: x forward, y left, z up. Right-handed. Metres and radians.
- **REP-105**: `world` → `odom` → `base_link` is the standard chain.
- A frame is defined by an **origin** (a physical feature you can point at) and
  an **orientation** (axes tied to physical features). "The middle of the plate"
  is a definition. "Roughly the centre" is not.
- Every transform below is `parent → child`, expressed as the child's origin in
  the parent's frame plus an `rpy` rotation, exactly as a URDF `<origin>`.

## Frame tree

```
world                     fixed. Where the robot was when the driver started.
└── odom                  driver's integration origin. Coincides with world at t0.
    └── base_link         SLATE. Drive-axle midpoint, at ground level.
        └── base_top      SLATE top plate, centre of the 500x500 surface.
            └── lift_base       scissor lift mounting plate, its centre.
                └── lift_platform    PRISMATIC. Lift top face. Travel = stroke.
                    └── mast_bar     the metal bar. Centre of its mounting face.
                        ├── left_arm/base_link
                        ├── right_arm/base_link
                        └── middle_arm/base_link
```

Everything above `base_link` is fiction we choose; everything below it is
physical and must match the machine.

---

## The frames, one at a time

### `world`
Fixed. Defined as coincident with `base_link` at the moment the base driver
started. The visualisation draws everything relative to this.

Not a survey point and not recoverable after a restart: restart the driver and
`world` moves to wherever the robot is standing. Anything drawn in `world` is
"where the robot thinks it is", never ground truth.

### `odom`
The driver's own integration origin, coincident with `world`. Wheel odometry
only — it accumulates error on every slip and carpet edge and nothing corrects
it. Kept separate from `world` so that a future correction (AprilTag, lidar) has
somewhere to live without redefining anything.

### `base_link` — SLATE
- **Origin**: midpoint of the line joining the two drive-wheel contact patches,
  projected to the **ground plane** (z = 0 at the floor).
- **Orientation**: +x toward the front of the base, +z up, +y = z × x (left).

**Why the drive axle and not the plate centre.** The driver integrates
differential-drive odometry about the drive axle; that is the point the machine
pivots around on a spin-in-place command. Put `base_link` anywhere else and a
pure yaw makes the real robot spin on the spot while the model swings through a
circle of the offset's radius. The divergence appears only when rotating, which
makes it unusually hard to attribute.

*Known*: wheel separation 392 mm, wheel diameter 165 mm, footprint
500 × 500 × 222 mm (`assets/slate_base/specs-technical_drawing.pdf`).

*Nothing defines this frame but us.* `interbotix_slate_driver` ships no URDF and
no description package — it declares `base_frame_name` defaulting to
`"base_link"` and publishes `odom → base_link` without ever saying where that is.

### `base_top` — SLATE top plate
- **Origin**: centre of the 500 × 500 mm top mounting surface, **on** the
  surface.
- **Orientation**: same as `base_link`.
- **Transform**: `(P_x, 0, P_z)` — fore/aft offset from the axle, and plate
  height above ground. Lateral offset is zero if the drive wheels are
  symmetric, which is worth confirming rather than assuming.

### `lift_base` — scissor lift mounting plate
- **Origin**: centre of the lift's base plate, on the face that contacts
  `base_top`.
- **Orientation**: +z up. +x along the lift's own long axis, which may not be
  the base's +x — hence a yaw term.
- **Transform**: `(L_x, L_y, 0)` and `yaw = L_yaw`. z is zero by construction:
  the plates are in contact.

### `lift_platform` — the moving top of the lift
- **Origin**: centre of the lift's top face, the one the bar bolts to.
- **Orientation**: same as `lift_base`. A scissor lift translates without
  rotating, which is exactly why it is a single prismatic joint.
- **Joint**: `prismatic`, axis `(0, 0, 1)`, `lower = 0`,
  `upper = L_stroke`.
- **Transform at zero**: `(0, 0, L_retracted)`.

`/slate/lift/height` drives this joint, so `L_retracted` and `L_stroke` are what
make that number mean something in the model.

### `mast_bar`
- **Origin**: centre of the bar's top (arm-mounting) face.
- **Orientation**: +y along the bar's length (arms are spread left–right), +z up.
- **Transform**: `(0, 0, B_h)` from `lift_platform`, assuming the bar is centred
  and square to the lift. If it is not, it needs its own x/y/yaw.

Skip this frame entirely if the arms bolt straight to the lift top.

### `left_arm/base_link`, `right_arm/base_link`, `middle_arm/base_link`
- **Origin**: **centre of the arm's flat bottom mounting face** — the surface
  that contacts the bar.
- **Orientation**: the arm's own URDF frame. +z is the waist rotation axis
  pointing up out of the base.

Verified from the vendor meshes, so this is not an assumption:

| Arm | Base mesh z-range | Waist axis | Base footprint |
|---|---|---|---|
| WXAI (left, right) | 0 → 60.2 mm | +57.25 mm, +Z | x −32.5…+37.2, y ±32.5 mm |
| wx250s (middle) | 0 → 78.7 mm | +72 mm, +Z | x ±102, y −102…+197.5 mm |

Both meshes start at z = 0, which means **the URDF origin is exactly on the
bottom mounting face** for both arm families. That is the point you measure to,
and it is unambiguous.

- **Transform**: `(A_x, A_y, A_z)` and `rpy = (A_roll, A_pitch, A_yaw)` per arm.

**`A_z` is zero if the arm bolts flat to the bar.** It is not zero if there is a
spacer, riser or adapter plate, and that is easy to forget.

**Orientation is the one to be careful with.** If an arm is mounted inverted, its
roll is π and everything below it mirrors. If the outer arms are angled inward,
that is a yaw. Both of your manipulators currently report their end effector at
z = −0.39 m in their own base frames — consistent with either a folded pose on an
upright arm or an arm hanging inverted, and the joint data alone cannot tell
those apart. Confirm by eye before writing a number.

---

## Parameters to determine

26 numbers, of which most are zero or fixed by symmetry. Marked ● where a real
value is needed, ○ where zero is very likely but worth confirming.

| # | Parameter | Frame pair | Meaning |
|---|---|---|---|
| ● | `P_x` | base_link → base_top | fore/aft, drive axle to plate centre |
| ○ | `P_y` | " | lateral. Zero if wheels are symmetric |
| ● | `P_z` | " | top plate height above ground (~0.222, confirm loaded) |
| ● | `L_x`, `L_y` | base_top → lift_base | lift plate centre on the top plate |
| ○ | `L_yaw` | " | zero if the lift is square to the base |
| ● | `L_retracted` | lift_base → lift_platform | lift top face height, fully down |
| ● | `L_stroke` | " | travel, fully down to fully up |
| ● | `B_h` | lift_platform → mast_bar | bar mounting face above lift top |
| ○ | `B_x`,`B_y`,`B_yaw` | " | zero if the bar is centred and square |
| ● | `A_y` ×3 | mast_bar → each arm | spacing along the bar |
| ● | `A_x` ×3 | " | fore/aft on the bar |
| ○ | `A_z` ×3 | " | zero unless there is a riser |
| ● | `A_roll/pitch/yaw` ×3 | " | **confirm by eye first** |

---

## How to get the numbers, best first

### 1. CAD assembly — do this if you can

You have STEP for everything: `slate_base/step/SLATE.STEP`,
`scissor_lift/scissor_lift.STEP`, `widowX-AI/step/`, `widowX-250/step/`. STEP
carries exact geometry, and you designed the lift in SolidWorks already.

Assemble with mates, then **read the transforms off** instead of measuring them.
A tape measure cannot resolve a 3D relative pose to better than a few mm and
cannot resolve angle at all; a mated CAD assembly is exact.

Two things to be careful about:

- **The CAD origin is not the URDF origin.** Assembling gives you part-to-part
  relationships in SolidWorks' frames. You still have to express them between
  the frames defined above — which is what the definitions are for. In practice:
  create a reference point/axis at each frame origin above, then measure between
  those, not between arbitrary part origins.
- **Units and the ground plane.** `base_link` is at the floor, not on the
  chassis. Include a floor plane in the assembly or you will lose `P_z`.

Blender works but is the wrong tool: no constraint solver, mesh-only, and unit
handling that makes mm/m errors easy. If SolidWorks is available, use it.

### 2. Let the robots measure themselves — best for arm-to-arm

The arms' own forward kinematics is exact (vendor URDF, factory calibration), so
the relative transform between two arm bases can be *solved for* rather than
measured.

**Touch test.** Bring both end effectors to touch at a single physical point,
several times, in different postures. At each touch, FK gives you the tip
position in each arm's own base frame. The unknown transform
`T(left_base → right_base)` is whatever makes those agree, and with 4+ well-spread
touches it is over-determined — solve least-squares and the residual tells you
how good it is.

This is far more accurate than a tape and it directly measures the quantity that
matters for bimanual work, which is the arms' relation to *each other* rather
than to the bar.

**For the base**: drive a known square and compare commanded to `/slate/odom`.
That calibrates odometry scale and any axle offset error, not the mounting.

I can write both of these as scripts once the URDF skeleton exists.

### 3. Tape measure — only where the other two cannot reach

Only `P_x` and `P_z` genuinely need a tape, because they reference the physical
wheel contact patch and the floor, which are in nobody's CAD.

---

## Approximate first, then tune — yes, this is the right workflow

Starting from approximations and correcting against reality is standard
kinematic calibration, not a shortcut.

1. **Fill every parameter with a best guess** from CAD and the drawings. Write
   them all in one YAML so the xacro reads them from a single place — the point
   is that each is a named number you can change without touching the model.
2. **Build the URDF and visualise it** next to the real rig in the same posture.
   Gross errors — a wrong sign, an inverted arm, a missing riser — are obvious
   here and are the ones worth catching before any arithmetic.
3. **Drive one axis at a time** and compare. Extend the lift 100 mm: does the
   model's bar rise 100 mm? Spin the base 90°: does the model pivot on the spot
   or swing? A swing is `P_x` and its radius *is* the error.
4. **Touch test for the arms** to solve arm-to-arm exactly.
5. **Update the YAML, repeat.**

The thing that makes this work is that each parameter has a distinct signature.
`P_x` shows up only when rotating. `L_retracted` shifts everything above it
uniformly. An arm's `A_yaw` shows up as its EE tracing an arc offset from the
model's. So the difference between model and reality is not one blurry error —
it decomposes, and each part points at the parameter that caused it.

Which is also why it is worth writing the frames down first: a parameter you
have not named is one you cannot tune.

## The WXAI mount remap, and where (0,0,0) is

Two separate things, easy to confuse:

**The remap** is fixed by how the arm is bolted on. `ARM_WORLD_X/Y/Z` in
`manip-arm/docker-compose.yml` name the arm axis each WORLD axis maps to:

    ARM_WORLD_X: "+z"     world +x  ->  arm +z
    ARM_WORLD_Y: "+y"     world +y  ->  arm +y
    ARM_WORLD_Z: "-x"     world +z  ->  arm -x

Applied in BOTH directions -- `cmd_pose` world->arm, `ee_pose` arm->world. It
has to be both: the jog tools anchor on `ee_pose` and add a delta, so converting
one direction only would put the anchor and the step in different frames and the
arm would walk diagonally instead of along the axis you pressed.

The determinant is checked at startup and must be +1. An odd number of sign
flips is a mirror, not a rotation: translations still look right while rotations
come out backwards, which is a miserable thing to debug.

**The origin** is where (0,0,0) sits, and it is NOT part of the remap. It is a
translation captured once and stored in `manip-arm/workspace/config/origin.yaml`,
keyed by arm. Position only -- never orientation, because zeroing a rotation
makes "level" mean whatever the wrist happened to be doing at startup.

It persists deliberately. Re-zeroing on every start would mean a fault silently
shifts every coordinate you have written down by however far the arm drifted
before it stopped. After a restart the agent reads the live end effector and
applies the ORIGINAL offset, so the frame outlives the process. Logged as
`origin restored from config/origin.yaml` -- a restored origin is
indistinguishable from a fresh one until the arm moves, so the log line is the
only way to tell.

To move the origin: zero keys `4`/`5`/`6` in `rig_key.py`, which re-capture and
re-save. To force a fresh capture at next start, delete that arm's key from the
file.
