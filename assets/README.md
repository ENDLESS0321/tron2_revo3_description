# Robot assets

[English](README.md) | [简体中文](README.zh-CN.md)

Use assembly.urdf and scene.xml for the unchanged full visual assembly. Use the two URDFs under training/ for the model5200 training lineage:

- model5200_official_aligned_20261023_icp.source.urdf is the source model for retargeting and asset binding.
- model5200_official_aligned_20261023_importer_runtime.urdf is the Isaac importer input.

Both training URDFs keep the model5200 bytes except the 21 Revo3 hand joints, whose `<limit effort>` and `<dynamics damping friction>` hold PACE-identified values. training/manifest.json records both the current and the original model5200 SHA-256. Their relative mesh references resolve under training/meshes/; training/manifest.json verifies every file. The runtime model has 28 moving joints and 58 collisions. The full visual model remains unchanged.

From the repository root, run:

```bash
python3 -m unittest discover -s tests -v
```

This verifies hashes and closure. Importer overrides are documented in the training manifest; per-joint Revo3 hand dynamics from PACE identification are in training/revo3_pace_sysid.json. Neither is a hardware limit.
