# Reduced right-arm training assembly

This guide defines the reduced training URDF, its relationship to the full assembly, and the limits of the available checks. The active right-hand subtree and its collision meshes are taken from the official BrainCo Revo3 migration at commit `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`.

## Contents

- [Model scope and comparison](#model-scope-and-comparison)
- [Joint identity and order](#joint-identity-and-order)
- [Collision geometry and builder](#collision-geometry-and-builder)
- [Frozen geometry, placement, and limits](#frozen-geometry-placement-and-limits)

## Model scope and comparison

The training URDF keeps the established 28-DOF reduced topology and refreshes its active right hand from the official full model. The source of the reduced model's non-hand topology remains the pinned training template; its frozen head and left-side geometry stays baked into `base_Link`.

| Model | Links | Moving joints | Collision elements | Scope |
| --- | ---: | ---: | ---: | --- |
| Full `assets/assembly.urdf` | 114 | 58 | 280 | Both arms and hands plus head, cameras, and adapters |
| Training `assets/assembly_rl_convex.urdf` | 37 | 28 | 91 | Right arm and official right hand; nonactive chains remain frozen or removed |

The training model retains 9 non-hand collision elements, compared with 116 in the full model, and keeps all 82 collision elements of the official right-hand subtree. The reduced file has 91 collision elements total. The prior training model already used 28 palm and 21 finger collision components; its main reduction was removing or freezing nonactive chains, not collapsing the hand to hulls. The official hand now has 82 collision components, which remain separate. Its historical `rl_convex` filename does not mean that the palm or thumb is represented by one hull. In particular, the 32 official thumb `CMR` collision components remain separate, preserving their authored spaces and transforms.

The full and training URDF hashes are recorded in [`assets/official_revo3_migration.json`](../assets/official_revo3_migration.json) and [`assets/manifest.json`](../assets/manifest.json). Training consumers must bind to the current training URDF hash, exact joint names, and their own importer mapping.

## Joint identity and order

The active right hand has 21 actuated joints. The official order is thumb CMP, CMR, MCP, PIP, DIP, followed by index, middle, ring, and little, each MPR, MCP, PIP, DIP. The seven right arm joints retain their existing names. The explicit official mapping is recorded in [`assets/official_revo3_migration.json`](../assets/official_revo3_migration.json).

XML declaration order is not a policy action-vector contract. Runtime consumers should bind controls, IK, trajectories, and checkpoints by exact joint names and the URDF hash. Do not infer compatibility from an old action vector or matching tensor dimensions.

## Collision geometry and builder

Training preserves the existing non-hand collision model and uses the official full model's collision links and meshes for the active right hand. It does not merge multiple meshes into a link-wide hull or regenerate convex approximations. This preserves component spacing and avoids silently inflating the official contact envelope. Collision element counts and source mesh hashes are covered by the tests and asset manifest.

The official migration builder is [`scripts/build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py). It accepts the full or reduced URDF as a separate source, reads the official description and mesh closure, and writes a new output directory. Build each model from its own source URDF so the full and training mount transforms remain independent. The builder verifies the selected mount and non-hand structure before materializing output.

## Frozen geometry, placement, and limits

`base_Link` retains the existing 44 visuals that bake in the nonactive head and left-side pose; those visuals are legacy frozen geometry from the reduced training model. The training migration updates the active right hand and does not claim to regenerate the frozen opposite side from the current full assembly. The full model remains the reference when a task requires both hands or the original head and camera frames.

The training model keeps the existing root and right-hand mount. The tabletop-to-physical-base-origin gap is exactly `0.45 m`; see the [frame contract](../TABLETOP_FRAME_CONTRACT.md). The reduced URDF is not a table or object scene.

Current verification checks URDF topology, mesh hashes, exact right-hand source nodes, and relative right-hand FK at tested poses. These checks do not establish PhysX runtime collision behavior, dynamics equivalence, policy compatibility, training completion, collision-free motion, grasp success, or hardware readiness. Existing trajectories and checkpoints need revalidation against the current training URDF.
