# Mesh inventory

English | [简体中文](README.zh-CN.md)

Mesh paths are referenced relative to `assets/` by the URDF and scene XML. Keep this directory tree beside those files. The authoritative per-file hashes and sizes are in [`../manifest.json`](../manifest.json); scope, model choice, units and frame placement are described in [`../README.md`](../README.md).

| Directory | Contents and role |
| --- | --- |
| `tron2/` | LimX TRON2 visual meshes and selected collision meshes |
| `revo3/` | BrainCo Revo3 hand visual and collision geometry |
| `adapter_v2/` | Revised bilateral adapter visuals and decomposed convex collision pieces |
| `wrist_camera_brackets/` | Supplied, aligned V3 bracket simulation meshes |
| `cameras/` | D405/D455 camera body visuals |
| `boundaryfix_exact/` | Hash-named meshes retained by the reduced28 physicsfix training asset |
| `right_adapter_single_convex.obj` | Training adapter collision mesh |

URDF visual and collision geometry may use different files: visual meshes preserve appearance while collision meshes are the simplified/convex shapes actually assigned to collision elements. The reduced28 URDF contains 58 collision elements, including 28 palm components; see [the training note](../../docs/training_model.md). The `boundaryfix_exact` directory name identifies its retained asset set, not a documented mesh-generation algorithm. Its 57 files, like the other distributed meshes, are individually hashed in the manifest and included in the training URDF's provenance. No stronger claim about the term “boundaryfix” or the original generation procedure is encoded here.

LimX, BrainCo, RealSense and user-provided CAD lineage, source revisions and licensing qualifications are recorded in [`../../THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md). These converted simulation meshes do not extend upstream licenses or establish mechanical or hardware validation.
