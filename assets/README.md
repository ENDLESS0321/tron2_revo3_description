# Asset models and frames

English | [简体中文](README.zh-CN.md)

## Contents

- [Select the model](#select-the-model)
- [Frames and placement](#frames-and-placement)
- [Mesh paths and integrity](#mesh-paths-and-integrity)

## Select the model

| File | Use | DOF | Table and cameras |
| --- | --- | ---: | --- |
| [`assembly.urdf`](assembly.urdf) | Complete bilateral robot and camera mount/optical frames | 58 | No table or MuJoCo render-camera definitions |
| [`scene.xml`](scene.xml) | Full MuJoCo scene | 58 | Table and six RGB/depth rendering cameras |
| [`assembly_rl_convex.urdf`](assembly_rl_convex.urdf) | Reduced right arm and hand training articulation | 28 | No table, object, or camera frames/render cameras |

The URDF training artifact is the file in this repository. Its source filename, `assembly_bilateral_axis180_reduced28_physicsfix.urdf`, is provenance metadata only. See [training model details](../docs/training_model.md) for its frozen geometry, exact joint inventory, collisions, and inertial caveat.

## Frames and placement

The robot URDF `world_to_base` stays at `[0, 0, 1.20035] m`. Release 1.8.0 places the full scene robot and table in the same world layout as dex-retarget/dex-rl; only the scene placement changes. Lengths are meters and angles are radians.

| Parameter | Value |
| --- | --- |
| Physical base XYZ | `[-0.22, 0.60231100353792, 0.957]` |
| Physical base quaternion WXYZ | `[0.999048221581858, 0, 0, -0.043619387365336]` |
| Table center XYZ | `[0.2764393782702718, 0.39253473731213084, 0.307]` |
| Table full size XYZ | `[0.7, 0.7, 0.4]` |
| Table top Z / base-to-table gap | `0.507 / 0.45` |
| URDF root world Z when importing | `-0.24335` |

MuJoCo box `size` stores half extents, so the scene uses `[0.35, 0.35, 0.2]`. Test cubes rest on the revised tabletop. `runtime.json` records `world_layout`. When importing either robot URDF, apply the root position `[-0.22, 0.60231100353792, -0.24335]` and the base quaternion exactly once; the URDF contains no table.

```mermaid
graph TD
  W[World] -->|Explicit scene or import root transform| B[Physical TRON2 base: z 0.957 m]
  B --> L[Left arm and Revo3]
  B --> R[Right arm and Revo3]
  T[Table top: z 0.507 m] -. 0.45 m below base .-> B
```

Both hands and wrist-camera mounts use an axis-180 installation about the parent wrist link's local Z centerline, through `[-0.0317, 0, 0] m`; this is a mounting transform, not the actuated wrist-roll axis. Mount translations and physical joint limits are retained relative to the pre-rotation installation; the head camera is unchanged. Hand-to-flange translation is `[0, 0.023855, 0] m`; mount RPY is left `[-π/2, -π/2, 0]`, right `[-π/2, +π/2, 0]`.

## Mesh paths and integrity

URDF and MJCF mesh filenames are relative to `assets/`, for example `meshes/tron2/base_Link.STL`. Preserve the supplied `assets/meshes/` tree when relocating or copying a model, and resolve paths from the URDF/MJCF asset root rather than the current working directory. Referenced simulation meshes are in meters except where the URDF explicitly supplies a scale; the D455 source mesh is in millimeters and the URDF scales it by `0.001`. Manufacturing CAD has its own units: adapter/bracket CAD STL files are millimeters.

[`manifest.json`](manifest.json) is the release inventory: it records per-file SHA256 and byte length, upstream source revisions, 265 full-model unique mesh references, 128 training mesh references, and the source hash of the training artifact. Verify the hashes before accepting modified or copied assets. The runtime viewers resolve the relative paths and reject missing or changed manifest entries; they do not regenerate assets. Source and license qualifications are in [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).
