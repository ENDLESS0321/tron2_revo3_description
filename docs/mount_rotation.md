# Bilateral hand and wrist-camera mount rotation (1.5.0)

Both hand/flange assemblies and both wrist-camera assemblies are rotated 180°
from the previous model around the forearm longitudinal centerline. In each
`wrist_roll_*_Link` frame this is the local Z axis passing through
`[-0.0317, 0, 0]` m. This is distinct from the actuated wrist-roll joint axis.
Only the orientations of `left_adapter_mount`, `right_adapter_mount`,
`left_wrist_camera_reference_joint`, and `right_wrist_camera_reference_joint`
change. Root positions, hand-to-flange joints, camera internal transforms,
meshes, actuated joint limits and the head camera remain unchanged.

## Reason for the update

The intended tabletop task holds the right palm down and the fingers pointing
in the robot's forward direction. Changing the mount orientation is intended
to leave greater joint-limit headroom when the arm reaches this posture, rather
than changing physical joint limits. The release verifies the installation
geometry; it does not quantify a larger joint-limit margin or feasible workspace.
Those comparisons require new IK and collision evaluations for this model.
Previous evaluations using different mount directions do not validate this release.

## Files and coordinates

- `assets/assembly.urdf`: the approved `assembly_bilateral_axis180.urdf`
  directly replaces the canonical model byte-for-byte. This is the default
  assembly URDF; all existing viewers keep using this path.
  SHA256 `537f31a798ddb05d1f29e2b5eeede63d47af3d9ff519f40907a24e757f5bbf44`.
- `assets/scene.xml`: matching MuJoCo mount orientations and base Z = 1.20035 m.
  The work-table center is at `[0.72, 0, 0.72535]` m and its half-height is
  0.025 m, giving top Z = 0.75035 m. Thus `1.20035 - 0.75035 = 0.45 m`.
- `assets/mount_rotation.json`: input hashes, changed joint transforms, geometry
  checks and the scene's 45 cm height relationship.
- `assets/manifest.json`: file sizes and SHA256 checksums used by the viewers.

The 1.4.0 release kept a normalized default and a separate exact variant.
Version 1.4.1 supersedes that arrangement: the default is now the exact approved
artifact, including its original world placement. The duplicate variant is
removed. Existing code importing `assets/assembly.urdf` needs no path change;
code importing the removed variant should switch to the canonical path.
The base, table and tabletop test cubes move upward by 0.35 mm relative to 1.4.0, preserving
their separation. No URDF normalization or internal geometric modification is
performed during this replacement.

The URDF contains the robot; the table is separate scene geometry. The table
preview places its center 0.60 m along base local +X, with zero lateral offset,
and its top 0.45 m below the base origin. The preview uses the exact URDF with
an explicit scene placement; it does not use its inherited world offset.
Both arms are at zero in the preview, so this image is not the palm-down task
posture, an IK feasibility result, or a physics rollout.

![New URDF and forward table](images/new_urdf_front_table.png)

The zero-pose comparison confirms unchanged hand-base positions and finger
directions, reversed palm normals, and unchanged camera-to-hand transforms.

![Original and updated model at arm zero](images/bilateral_zero_pose.png)

## Exact rotation convention and preserved connections

The table below compares installation transforms with the pre-1.4.0 model.
RPY follows URDF fixed-axis X/Y/Z convention: `R = Rz(yaw) Ry(pitch) Rx(roll)`.
The half-turn is applied in the parent wrist-link frame as
`R_new = Rz(pi) R_old`. The axis line passes through the mounting roots, so their
translations stay fixed. This changes the mounted branches, not the wrist joint
axis, zero angle, name or limit. The left/right assemblies retain their own
branches; no meshes or links are swapped.

| Joint | Previous RPY (rad) | Current RPY (rad) |
| --- | --- | --- |
| `left_adapter_mount` | `-1.57079632679 0 1.57079632679` | `-1.57079632679 0 -1.5707963268` |
| `right_adapter_mount` | `-1.57079632679 0 -1.57079632679` | `-1.57079632679 0 1.5707963268` |
| `left_wrist_camera_reference_joint` | `1.57079632679 0 -1.57079632679` | `1.57079632679 0 1.5707963268` |
| `right_wrist_camera_reference_joint` | `1.57079632679 0 1.57079632679` | `1.57079632679 0 -1.5707963268` |

Adapter-root translations remain `[-0.0317, 0, -0.0812]` m. Camera-root
translations remain `[-0.0317, -1.31006316906e-18, -0.0753]` m; the tiny Y value
is retained exactly from the approved artifact. Both hand-to-flange fixed joints
retain translation `[0, 0.023855, 0]` m and their original RPY:
left `[-1.57079632679, -1.57079632679, 0]`, right
`[-1.57079632679, 1.57079632679, 0]`.
The complete downstream finger chains and camera mechanical/optical frames are
preserved. Hand-local +X denotes the palm normal; hand-local +Z denotes the
finger direction. At the same arm configuration the mounting half-turn reverses
the palm normal while preserving the finger direction.

## Joint-limit headroom and verification

For a revolute joint at angle q with bounds [lower, upper], its angular headroom
is `min(q - lower, upper - q)`. The same world hand pose can require a different
arm configuration after a fixed mount rotation, so headroom can change even
though the joint bounds do not. Greater headroom is the design objective for
the specified tabletop posture, not a claim about every joint or every pose.

The release checks all asset hashes, loads both 58-DoF models, verifies the
45 cm table separation, and compares URDF/MJCF mount poses at zero and nonzero
arm configurations. The approved zero-pose comparison has hand-base position
error below 2.4e-13 m, finger-direction error below 1.4e-11, palm-normal dot
product approximately -1 and camera-to-hand transform error below 6.5e-15.
These geometry checks do not measure task success. Quantitative headroom or
workspace comparisons must use the same target positions, palm-down/fingers-
forward orientation, table/object geometry, joint bounds and collision rules
for both models. Existing IK trajectories and world camera extrinsics need
recomputation for the changed mount, while internal camera frames stay intact.

## RL and simulation-training asset requirement

For RL and simulation training, simplify **each flange to one rigid link and
one collision shape**: one for the left flange and one for the right flange.
The derived `assets/assembly_rl_convex.urdf` now satisfies this requirement;
the default assembly keeps its detailed collision geometry. The published assembly currently
has one `left_adapter_link` and one `right_adapter_link`, but each contains
48 URDF collision elements. The derived training URDF replaces those 48 elements with one convex mesh
per flange.

Keep each flange's parent mounting transform and hand-to-flange fixed transform
exactly as defined here. Consolidate only fixed flange parts, preserving the
original collision geometry inside the new convex envelope, as well as units,
total mass, center of mass and equivalent inertia; the convex envelope may fill
holes and concavities. Retain the original visual mesh if desired. Do not merge across
actuated arm or finger joints. A single convex collision mesh or conservative
primitive is preferable for this single-shape requirement, subject to clearance
checks. Merely concatenating meshes or enabling fixed-joint merging does not
ensure one collider: simulator import and automatic convex decomposition can
create multiple shapes from one URDF collision element.

Verify the imported simulator, not only the XML: each flange must have exactly
one rigid body/link and one collision shape after import. Compare hand poses,
joint limits, masses/inertias, table/object/self-collision clearance and representative
contacts with the assembly reference. A coarse convex approximation may fill
holes or concavities and change clearance, so it must not silently become the
reference for a larger feasible-workspace claim. Bind the derived training
URDF and collision asset hashes to the training configuration, and regenerate
IK/collision checks for that asset. The default assembly remains the detailed geometry reference. The new training
URDF is included; MuJoCo import confirms one collision shape on each flange,
while other simulation engines require their own importer check.

## Included single-convex training model (1.5.0)

Load `assets/assembly_rl_convex.urdf` for the flange-simplified model. Its source
is the exact canonical `assets/assembly.urdf`; the original assembly stays
byte-for-byte unchanged. Each flange already has one rigid link, so only its
48 collision elements are replaced by one element. No links or joints are
removed, and visual geometry, mass, center of mass, inertia, installation
transforms, world origin and joint limits are preserved.

The two new meshes are `assets/meshes/adapter_rl/left_flange_convex.stl` and
`assets/meshes/adapter_rl/right_flange_convex.stl`, in meters. The builder reads
all 48 meshes on each flange, applies any mesh scale and collision origin, then
computes the convex hull of their combined vertices in that flange's link frame.
It replaces the collision origins with identity because the vertices are already
in link coordinates. These are separate left/right hulls, not one hull combining
the two arms. Each output is a closed convex surface enclosing all source
collision vertices (maximum outside-plane error below 1e-8 m).

Rebuild from the repository root with the installed requirements:

```bash
.venv/bin/python tools/build_flange_convex.py
.venv/bin/python -m unittest discover -s tests -v
```

`assets/flange_convex.json` records source/output hashes, mesh hashes, collision
counts, bounds, hull volume and geometry validation. MuJoCo loads the model with
58 generalized positions and one collision shape per flange. Structural tests
verify that changes are confined to the flange collision geometry; the default
assembly hash remains the approved hash. The rebuild script regenerates the
training files and report; intentional geometry changes also require refreshing
`assets/manifest.json` checksums before the viewer integrity check can pass.

The convex envelope fills flange holes and concavities. It therefore reduces
collision-shape count but can increase the occupied collision region. It does
not prove faster training, larger workspace, successful grasping or physical
stability. Collision clearance and task-specific contact evaluation must be
redone with this exact training asset. The default `scene.xml` still represents
the detailed assembly; importing the training URDF is required to use the new
hulls. No simplified MJCF scene is supplied in this release.
