# Mesh inventory

This directory contains only mesh files required by current robot assets. URDF/MJCF paths are relative to `assets/`; per-file hashes are recorded in [`../manifest.json`](../manifest.json).

## Mesh groups

| Directory | Role |
| --- | --- |
| `tron2/` | TRON2 visuals and selected collisions |
| `hands/` | Official Revo3 hand visuals and collision geometry for both sides |
| `adapter_v2/` | Current bilateral adapter geometry |
| `wrist_camera_brackets/` | Aligned camera bracket geometry |
| `cameras/` | D405/D455 visuals |

## Collision and provenance notes

The full URDF references 404 unique mesh URIs; the reduced right-arm URDF references 196. Collision geometry follows the official hand model and may contain multiple meshes per link; the `rl_convex` filename is historical and does not assert that every collision shape is convex. Two official DAE visual meshes are converted to derived MuJoCo STL files for scene compilation. The source-to-derived mapping and source hashes are recorded in `assets/manifest.json` and `assets/official_revo3_migration.json`.

The official source revision and unresolved license status are documented in [`../../THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md). These assets do not certify mechanical properties or hardware safety.
