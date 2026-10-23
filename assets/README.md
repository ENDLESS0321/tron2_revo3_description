# Robot assets

[English](README.md) | [简体中文](README.zh-CN.md)

Use assembly.urdf and scene.xml for the unchanged full visual assembly. Use the two URDFs under training/ for the model5200 training lineage:

- model5200_official_aligned_20261023_icp.source.urdf is the source model for retargeting and asset binding.
- model5200_official_aligned_20261023_importer_runtime.urdf is the exact Isaac importer input.

Both training URDFs keep their original SHA-256 identities. Their relative mesh references resolve under training/meshes/; training/manifest.json verifies every file. The runtime model has 28 moving joints and 58 collisions. The full visual model remains unchanged.

From the repository root, run:

```bash
python3 -m unittest discover -s tests -v
```

This verifies hashes and closure. Importer overrides are documented in the training manifest and are not hardware limits.
