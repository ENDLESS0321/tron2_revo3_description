# Reduced28 physicsfix training model

English | [简体中文](training_model.zh-CN.md)

## Artifact and scope

Use [`assets/assembly_rl_convex.urdf`](../assets/assembly_rl_convex.urdf) as the repository training-model path. It is a byte-for-byte supplied source artifact named `assembly_bilateral_axis180_reduced28_physicsfix.urdf`; that original filename is provenance metadata, not another checked-in file. [`assets/manifest.json`](../assets/manifest.json) records the SHA256 as `2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034`, plus the per-file hashes and source revisions. The full URDF hash is `537f31a798ddb05d1f29e2b5eeede63d47af3d9ff519f40907a24e757f5bbf44`.

The reduced artifact has 38 links, 37 joints (28 revolute and 9 fixed), 78 visuals, 58 collision elements on 31 links, and 128 unique mesh references. It contains the right arm and right hand as moving chains. Head, left arm, left hand and their geometry are baked into retained ancestors; their independent joint and link/frame identities are absent. This is a topology reduction, so consumers that need those original frames cannot recover them by name from this URDF.

## The 28 moving joints

The list below follows the right kinematic chain from robot base to fingertip branches. Names are exact and case-sensitive. Each of the five digits has its own named serial branch.

| Group | Joint names, in proximal-to-distal order |
| --- | --- |
| Right arm | `proximal_pitch_R_Joint`, `proximal_roll_R_Joint`, `proximal_yaw_R_Joint`, `elbow_R_Joint`, `wrist_yaw_R_Joint`, `wrist_pitch_R_Joint`, `wrist_roll_R_Joint` |
| Thumb | `right_thumb_CMP_joint`, `right_thumb_CMR_joint`, `right_thumb_MCP_joint`, `right_thumb_PIP_joint`, `right_thumb_DIP_joint` |
| Index | `right_index_MPR_joint`, `right_index_MCP_joint`, `right_index_PIP_joint`, `right_index_DIP_joint` |
| Middle | `right_middle_MPR_joint`, `right_middle_MCP_joint`, `right_middle_PIP_joint`, `right_middle_DIP_joint` |
| Ring | `right_ring_MPR_joint`, `right_ring_MCP_joint`, `right_ring_PIP_joint`, `right_ring_DIP_joint` |
| Little | `right_little_MPR_joint`, `right_little_MCP_joint`, `right_little_PIP_joint`, `right_little_DIP_joint` |

This table is a readable chain grouping, not a promise about a policy action vector. In the training URDF, the XML declarations for moving joints occur in the same grouped sequence shown above (arm, thumb, index, middle, ring, little); the seven arm joints are also the base-to-wrist chain. A runtime that enumerates joints using its own importer may assign a different index order. [`assets/runtime.json`](../assets/runtime.json) stores display poses as name/value mappings and specifies 28 expected DOF, but defines no action vector or policy mapping. Bind controls, IK, H5 trajectories and checkpoints by the exact joint names plus the URDF hash; do not infer compatibility from an old 58-value vector or matching tensor dimensions.

## Frozen pose and retained inertials

Geometry no longer articulated by this file is embedded at this frozen pose, with angles in radians:

- Head yaw `0`; head pitch `0.35`.
- Left arm in pitch, roll, yaw, elbow, wrist-yaw, wrist-pitch, wrist-roll order: `1.2395190538964451`, `0.0050965579201923406`, `0.0706739400409262`, `-0.10075078688932715`, `-0.33794336457190877`, `-0.6172884125393175`, `0.5453728186806424`.
- All left-hand joints are zero.

Inertials were combined during reduction. Each of the five right fingertip links has mass `0.001 kg`, zero inertial-origin translation (COM), diagonal inertia `(1e-10, 1e-10, 1e-9) kg·m²`, and zero products of inertia. `right_palm` is a placeholder link with no inertial. This artifact includes both the fingertip inertial repair and palm collision restoration; their separate effects on any training outcome have not been isolated.

## Collision inventory

| Group | Collision elements |
| --- | ---: |
| Base and seven right-arm links | 8 |
| Right hand base / palm components | 28 |
| Right finger components | 21 |
| Right adapter / flange | 1 |
| **Total** | **58** |

The retained `rl_convex` filename does not mean that all collision geometry is a single convex hull: the palm has 28 mesh components. Every referenced training mesh is included under `assets/meshes/`; the manifest covers all 128 unique references. The hash-named `boundaryfix_exact/` files and `right_adapter_single_convex.obj` are provenance-tracked simulation assets. The repository does not specify the mesh-generation procedure implied by “boundaryfix”.

## Coordinate placement

Distances below are meters; angles above are radians. The full and training URDFs retain the fixed `world_to_base` translation `[0, 0, 1.20035]`. In the full MuJoCo scene, the table top is `z = 0.75035 m`, 0.45 m below the robot origin. A separate physicsfix training layout uses base `z = 0.957 m` and tabletop `z = 0.507 m`, also 0.45 m apart. The training URDF itself has no table or manipulated object. Treat the URDF world offset, simulator base placement and environment table placement as separate configuration inputs; applying the full-scene offset on top of a training layout can double the translation.

The full scene XML describes the complete 58-DOF articulation. The reduced28 URDF is the training topology. The release viewer uses the reduced URDF only for a kinematic preview: it compiles an in-memory MuJoCo `MjSpec` with `balanceinertia` enabled because the supplied fingertip inertias violate MuJoCo's inertia triangle check. The published URDF bytes are not altered. This correction is a preview import accommodation and does not demonstrate MuJoCo/Isaac dynamics equivalence, training completion, hardware operation, or task success.

The repository's viewer helper can load this kinematic preview programmatically:

```python
import sys
sys.path.insert(0, "viewers")
from _runtime import load_training_model

model = load_training_model()
assert model.nq == 28
```
