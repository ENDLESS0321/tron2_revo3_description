# Asset models and frames

English | [简体中文](README.zh-CN.md)

## Select the model

| File | Use | DOF | Table and cameras |
| --- | --- | ---: | --- |
| [`assembly.urdf`](assembly.urdf) | Complete bilateral robot and camera mount/optical frames | 58 | No table or MuJoCo render-camera definitions |
| [`scene.xml`](scene.xml) | Full MuJoCo scene | 58 | Table and six RGB/depth rendering cameras |
| [`assembly_rl_convex.urdf`](assembly_rl_convex.urdf) | Reduced right arm and hand training articulation | 28 | No table, object, or camera frames/render cameras |

The URDF training artifact is the file in this repository. Its source filename, `assembly_bilateral_axis180_reduced28_physicsfix.urdf`, is provenance metadata only. See [training model details](../docs/training_model.md) for its frozen geometry, exact joint inventory, collisions, and inertial caveat.

## Frames and placement

Lengths in the robot URDFs and simulation scene are meters; joint angles are radians. The URDF `world_to_base` fixed joint places the robot base at `[0, 0, 1.20035] m`. In the full scene the table top is `z = 0.75035 m`, 0.45 m below that base origin. A separate documented physicsfix training layout uses base `z = 0.957 m` and table top `z = 0.507 m`, also separated by 0.45 m. These are distinct world placements: the training URDF contains no table, and an importing simulator must choose its world/base/table placement once rather than applying the full-model offset twice.

```mermaid
graph TD
  W[World] -->|world_to_base: z 1.20035 m in URDF/full scene| B[TRON2 base]
  B --> L[Left arm and Revo3]
  B --> R[Right arm and Revo3]
  T[Full-scene table top: z 0.75035 m] -. 0.45 m below base origin .-> B
  BT[Training-layout base z 0.957 m] -. 0.45 m above table top z 0.507 m .-> TT[Training-layout table]
```

Both hands and wrist-camera mounts use an axis-180 installation about the parent wrist link's local Z centerline, through `[-0.0317, 0, 0] m`; this is a mounting transform, not the actuated wrist-roll axis. Hand-to-flange translation is `[0, 0.023855, 0] m`; mount RPY is left `[-π/2, -π/2, 0]`, right `[-π/2, +π/2, 0]`.

## Mesh paths and integrity

URDF and MJCF mesh filenames are relative to `assets/`, for example `meshes/tron2/base_Link.STL`. Preserve the supplied `assets/meshes/` tree when relocating or copying a model, and resolve paths from the URDF/MJCF asset root rather than the current working directory. Referenced simulation meshes are in meters except where the URDF explicitly supplies a scale; the D455 source mesh is in millimeters and the URDF scales it by `0.001`. Manufacturing CAD has its own units: adapter/bracket CAD STL files are millimeters.

[`manifest.json`](manifest.json) is the release inventory: it records per-file SHA256 and byte length, upstream source revisions, 265 full-model unique mesh references, 128 training mesh references, and the source hash of the training artifact. Verify the hashes before accepting modified or copied assets. The runtime viewers resolve the relative paths and reject missing or changed manifest entries; they do not regenerate assets. Source and license qualifications are in [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
