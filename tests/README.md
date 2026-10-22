# Asset verification

Run all checks from the repository root after installing `requirements.txt`.

## Unit tests

```bash
.venv/bin/python -m unittest discover -s tests -v
```

The suite verifies manifest hashes and mesh closure; full/reduced model topology and loading; preserved mounts, camera and world placement; 0.45 m tabletop gap; official hand subtree inertias and joint order; fingertip aliases; URDF-to-scene FK at nonzero hand poses; and rejection of a tampered mesh manifest.

## Viewer checks

```bash
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

These checks exercise kinematic preview and rendering only. They do not certify dynamics, trajectory clearance, policy compatibility, hardware operation, or task success. Run training only under a separately approved protocol; this asset task does not launch it.

## Asset maintenance

When rebuilding a hand subtree, use [`scripts/build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py), supply the exact source assembly and its mesh-root closure, and validate the generated mount/non-hand invariants. Update the migration record and manifest only after reviewing source commit, exact native joint order, all referenced mesh hashes, and tests. Keep source backups and transient build outputs outside the repository.
