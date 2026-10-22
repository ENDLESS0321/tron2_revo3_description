# Asset models and frames

This guide identifies the full and reduced robot assets and their frame-placement contract.

## Model selection

| File | Use | DOF |
| --- | --- | ---: |
| [`assembly.urdf`](assembly.urdf) | Full bilateral robot and camera frames | 58 |
| [`scene.xml`](scene.xml) | Full MuJoCo kinematic scene, table, six render cameras | 58 |
| [`assembly_rl_convex.urdf`](assembly_rl_convex.urdf) | Reduced right-arm topology with official right Revo3 hand | 28 |

## Frames and placement

URDF `world_to_base` remains unchanged at `[0, 0, 1.20035] m`. Scene physical base XYZ is `[-0.22, 0.60231100353792, 0.957]`; table top is at `0.507 m`, exactly `0.45 m` below the physical-base origin. Table size is `[0.7, 0.7, 0.4] m` and center is `[0.2764393782702718, 0.39253473731213084, 0.307] m`. When importing a URDF, apply root XYZ `[-0.22, 0.60231100353792, -0.24335]` and the scene base quaternion once. Root, wrist, flange, and physical-base origins are distinct.

The full hand mount transforms are preserved from the original full assembly. The reduced right-hand mount is preserved from its own source assembly, which has a different installation context. Do not transfer mount values between assets. Hand-joint identity mappings are recorded in [`official_revo3_migration.json`](official_revo3_migration.json).

## Mesh closure and provenance

URDF and MJCF mesh references are relative to `assets/`. `manifest.json` contains the release file inventory and hash checks. The full URDF references 404 unique mesh URIs; the reduced training URDF references 196. `scene.xml` uses derived MuJoCo STL copies for two DAE visual meshes; their source-to-derived mapping is recorded in `manifest.json`.

The tabletop height is fixed by [`../TABLETOP_FRAME_CONTRACT.md`](../TABLETOP_FRAME_CONTRACT.md). Official hand source commit, source and output hashes, joint order, mount transforms, and referenced mesh hashes are recorded in the migration JSON. License attribution and unresolved redistribution status are in [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
