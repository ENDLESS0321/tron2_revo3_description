# Reduced 28-DoF training URDF (1.7.1)

`assets/assembly_rl_convex.urdf` is an exact, byte-for-byte copy of the supplied
`assembly_bilateral_axis180_reduced28_physicsfix.urdf`. The path is retained for
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
| Collision elements | 58, on 31 links |
| Referenced mesh files | 128 unique files; 28 palm-component meshes added and the former single palm convex mesh removed |
| Collision distribution | Base and 7 arm links: 8; palm components: 28; finger components: 21; flange: 1 |

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

The physicsfix model restores 28 palm-component collision meshes alongside the
single flange collision and detailed arm/finger collisions. Its 128 unique mesh
references are relative to `assets/`; compared with 1.7.0, 28 palm meshes were
added and the one single-palm convex mesh was removed. This changes collision
geometry, so physics behavior is not identical to the previous training model
or the full assembly. The historical `rl_convex` filename is retained for
configuration compatibility even though the palm now uses 28 components.

Five fingertip links have recovered inertials: each has mass 0.001 kg, zero COM,
and diagonal inertia `(1e-10, 1e-10, 1e-9) kg·m²`. The palm collision change and
these inertial changes were made together; their separate causal effects have
not been isolated. `right_palm` remains a placeholder without an inertial and
therefore still differs from the full-assembly baseline. Do not interpret the
historical derivation report's mass-conservation result as applying to this
physicsfix artifact.

`world_to_base` stays at `[0, 0, 1.20035]` m. The default scene table top is
0.75035 m, so its separation from the base origin is 0.45 m. The current
training setup uses a base Z of 0.957 m and table top of 0.507 m, also separated
by 0.45 m. The training URDF contains no table or manipulated object. The default
`scene.xml` continues to
represent the detailed 58-DoF assembly, not this reduced training articulation.
For training, import this URDF and explicitly configure the world/base/table
layout; avoid applying the URDF's world offset a second time. Reuse of a 45 cm
layout does not establish that an arbitrary old IK sidecar matches this asset.

## Provenance and verification

Published URDF SHA256:

```text
2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034
```

`assets/training_reduced28.json` records the supplied filename, replacement hash,
per-mesh hashes and derivation provenance. Historical source-derivation metrics
describe an earlier single-palm predecessor; they do not establish mass or
inertia conservation for this physicsfix model. The published physicsfix URDF
is currently being used by a four-GPU 4090D training run. Training is ongoing;
this publication does not claim reward, performance, grasp or physical-task
success, and the simultaneous collision/inertial edits do not isolate causality.

Release tests independently verify the exact published hash, referenced meshes,
28-joint topology, retained right-joint definitions and right-chain FK relative
to the full assembly, and MuJoCo preview compilation with 58 collision shapes.
The preview loader balances inertia in memory to allow a kinematic display; raw
URDF loading is rejected by MuJoCo. Other engines may fuse fixed links or
interpret collision meshes differently and need their own import inspection.

## Use and migration

```python
import sys
sys.path.insert(0, "viewers")
from _runtime import load_training_model
model = load_training_model()
assert model.nq == 28
```

MuJoCo cannot load the raw URDF directly because the supplied fingertip inertias
do not satisfy its inertia triangle check. The preview loader enables
`balanceinertia` on an in-memory `MjSpec` before compiling, so it can display the
model without changing the on-disk URDF or stepping physics. Compilation adjusts
inertias; this preview therefore does not establish that MuJoCo dynamics match
the Isaac training model.

Run release verification with `.venv/bin/python -m unittest discover -s tests -v`.
Close/reimport any simulator that cached the older model. Update action mappings,
body/frame references and robot configuration for this 28-joint topology; bind
IK/H5/runtime configuration to this URDF hash. Matching tensor sizes alone does
not establish checkpoint compatibility. The model is in an ongoing four-GPU
4090D training run; this publication makes no claim of reward, performance,
grasp or physical-task success.

The older bilateral convex builder and its old output report/meshes were removed
from the current tree to prevent it overwriting this supplied reduced model.
They remain available in the 1.5.0 Git history. The supplied reduced model is
published as an artifact; this repository does not claim to regenerate its
boundaryfix derivation with the removed flange-only builder.

## Repository preview and integrity checks (1.7.1)

```bash
./viewers/view_assembly.sh --model training
./viewers/view_assembly.sh --model training --check --output outputs/check/training
```

The reduced viewer imports this exact URDF, checks 28 DoF/zero render cameras,
and applies only the configured right-side display joints. `0` resets those
28 joints; `1` restores the display pose. Baked head/left geometry stays fixed.
The preview shows visual geometry without a table or payload and does no
physics stepping. `--check` writes `assembly_training_both_overview.png`.
The three-camera and CAD-part viewers remain full-model tools; training selection
there is rejected explicitly. `runtime.json` declares the two model paths,
expected topology, display poses and table/camera availability. Asset verification
checks both full and training mesh inventories against the manifest and provenance.

![Reduced28 display pose; forward kinematics, no physics stepping](images/training_reduced28_preview.png)
