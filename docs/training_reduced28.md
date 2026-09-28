# Reduced 28-DoF training URDF (1.6.0)

`assets/assembly_rl_convex.urdf` is now an exact, byte-for-byte copy of the
supplied `assembly_bilateral_axis180_reduced28.urdf`. The path is retained for
existing training configurations, but its topology has changed: it is no
longer the 58-DoF, bilateral flange-only simplification shipped in 1.5.0.
The unchanged `assets/assembly.urdf` remains the full 58-DoF assembly model.

## Topology and active joints

| Property | Published training model |
| --- | --- |
| URDF links | 38, including the URDF world link |
| URDF joints | 37: 28 revolute and 9 fixed |
| Controlled chain | Right arm: 7 joints; right Revo3 hand: 21 joints |
| Visual geometries | 78 |
| Collision elements | 31, on 31 links |
| Referenced mesh files | 101; 31 newly added to this repository |
| Right flange collision | One convex mesh on `right_adapter_link` |
| Right palm collision | One convex mesh on `right_hand_base_link` |

The seven right-arm joint names, in proximal-to-distal order, are
`proximal_pitch_R_Joint`, `proximal_roll_R_Joint`, `proximal_yaw_R_Joint`,
`elbow_R_Joint`, `wrist_yaw_R_Joint`, `wrist_pitch_R_Joint`, and
`wrist_roll_R_Joint`. The right hand retains five thumb joints and four joints
on each of index, middle, ring and little fingers. Use joint names to map
commands; XML ordering or a previous 58-element vector is not a control contract.

The right-arm and right-hand revolute joint definitions, axes and limits are
preserved from the supplied derivation source. The right flange retains the
axis-180 mounting orientation, and the hand-to-flange transform stays intact.

## Baked inactive chains

Head and left-side joints are not controllable joints in this asset. Their
geometry is transformed into the nearest retained ancestor link at the following
fixed pose, then their inertials are combined using the parallel-axis theorem:

- Head yaw: 0 rad; head pitch: 0.35 rad.
- Left arm, ordered pitch/roll/yaw/elbow/wrist-yaw/wrist-pitch/wrist-roll:
  `1.23951905389645 0.00509655792019234 0.0706739400409262 -0.100750786889327 -0.337943364571909 -0.617288412539318 0.545372818680642` rad.
- Left-hand joints: all zero.

This keeps the frozen geometry rather than simply discarding the left arm and
head, but removes their independent runtime joints and link identities. Camera
geometry can remain visible while separate mechanical/optical frame links are
removed by this reduction; consumers must not assume the full assembly's named
camera frames or its MJCF camera setup exist in this URDF.

## Collision assets and coordinates

The supplied model uses boundaryfix collision meshes, a single right-flange
convex mesh, and a single right-palm convex mesh. All 101 mesh references are
relative to `assets/` and are shipped in the repository, including the 31 new
files. Mesh bytes are copied without re-export or renaming. The original
URDF bytes and their references are preserved exactly. Convex flange/palm
geometry may fill holes and concavities; use this model's collision boundaries
when checking contacts and clearance.

`world_to_base` stays at `[0, 0, 1.20035]` m. The default scene table top is
0.75035 m, so its separation from the base origin is 0.45 m. The training URDF
contains no table or manipulated object. The default `scene.xml` continues to
represent the detailed 58-DoF assembly, not this reduced training articulation.
For training, import this URDF and explicitly configure the world/base/table
layout; avoid applying the URDF's world offset a second time. Reuse of a 45 cm
layout does not establish that an arbitrary old IK sidecar matches this asset.

## Provenance and verification

Published URDF SHA256:

```text
56fdcc40198d38f075dd307f256d7b961ea7197958b617c550ad6bb8a9a3c313
```

`assets/training_reduced28.json` records the supplied filename, replacement hash,
per-mesh hashes and the supplied derivation report. That report traces this
model to the single-palm boundaryfix predecessor, with 58 moving joints reduced
to 28. It reports mass conservation error of about 1.28e-11 kg, global inertia
tensor error about 6.38e-11, and zero right-chain FK error over ten tested poses.
These are source-derivation results, not a new RL success evaluation.

Release tests independently verify the exact published hash, referenced meshes,
28-joint topology, retained right-joint definitions and right-chain FK relative
to the full assembly, and MuJoCo import with one collision shape on the right
flange and palm. MuJoCo loads 28 generalized positions, 28 joints and 31 collision
shapes. Other engines may fuse fixed links or interpret collision meshes
differently and need their own import inspection.

## Use and migration

```python
import mujoco
model = mujoco.MjModel.from_xml_path("assets/assembly_rl_convex.urdf")
assert model.nq == 28
```

Run release verification with `.venv/bin/python -m unittest discover -s tests -v`.
Close/reimport any simulator that cached the older model. Update action mappings,
body/frame references and robot configuration for this 28-joint topology; bind
IK/H5/runtime configuration to this URDF hash. Matching tensor sizes alone does
not establish checkpoint compatibility. This publication launches no training
and establishes no reward, performance, grasp or physical-success result.

The older bilateral convex builder and its old output report/meshes were removed
from the current tree to prevent it overwriting this supplied reduced model.
They remain available in the 1.5.0 Git history. The supplied reduced model is
published as an artifact; this repository does not claim to regenerate its
boundaryfix derivation with the removed flange-only builder.
