# Release asset tests

English | [简体中文](README.zh-CN.md)

Run from the repository root after following the [installation instructions](../README.md#quick-start).

## Existing checks

```bash
.venv/bin/python -m unittest discover -s tests -v
```

[test_release_assets.py](test_release_assets.py) checks manifest digests and mesh references; full/training topology and model selection; exact approved URDF identity; mount transforms and URDF/MJCF agreement; the 45 cm base-origin/table-top gap; reduced-model joints, collisions, inertias and right-chain FK; and camera bodies, frames and palette. It also checks that training camera selection is rejected.

The suite compiles models and checks numerical relationships. Rendering checks below separately create images and inspect nonempty output:

```bash
./viewers/view_adapters.sh --check --output outputs/check/adapters
./viewers/view_camera_brackets.sh --check --output outputs/check/brackets
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

Part checks cover both sides and all four preset viewpoints. Offscreen rendering requires a working EGL or configured alternative backend. See [viewers](../viewers/README.md) and [camera output format](../docs/camera_outputs.md).

## Maintaining assets

1. Preserve the released source and make changes in an isolated branch or output.
2. Check URDF mesh paths, units, joint names and limits; synchronize corresponding full-scene transforms and runtime metadata when relevant.
3. Record reviewed new hashes, byte counts and source provenance in [manifest.json](../assets/manifest.json). The manifest covers assets, not every repository document.
4. Review tests with pinned hashes and geometry expectations. Change those expectations only for an intentional approved asset revision; do not just refresh hashes to hide an unexplained mismatch.
5. Run the unit suite and relevant rendering checks. Inspect resulting images and camera metadata before release, then synchronize both language versions of documentation.

A passing check verifies the stated asset/preview contracts. It does not certify hardware assembly, collision-free motion, physics equivalence, policy compatibility or manipulation success. Simulator caches must be reimported after an asset change.
