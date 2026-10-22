# Asset verification

Run all checks from the repository root after installing `requirements.txt`.

## Contents

- [Unit tests](#unit-tests)
- [Viewer checks](#viewer-checks)
- [Asset maintenance](#asset-maintenance)

## Unit tests

```bash
.venv/bin/python -m unittest discover -s tests -v
```

The suite verifies manifest hashes and mesh closure; full/reduced model topology and loading; preserved mounts, camera and world placement; the 0.45 m tabletop gap; official hand inertias and joint order; fingertip aliases; URDF-to-scene and full-to-training FK at nonzero poses; and rejection of a tampered mesh manifest. It also checks the 58-DOF full model's 280 collision elements against the 28-DOF training model's 58, confirms that official joint, inertia, visual and DIP collision geometry is preserved, rejects an unpinned collision-reducer input, and checks a recorded signature for retained non-hand training elements.

## Viewer checks

```bash
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

These checks exercise kinematic preview and rendering only. They do not certify PhysX runtime contacts, dynamics, trajectory clearance, policy compatibility, hardware operation, or task success. These test commands do not launch training.

## Asset maintenance

When rebuilding a hand subtree, use [`scripts/build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py), supply the exact source assembly and its mesh-root closure, and validate the generated mount/non-hand invariants. Update the migration record and manifest only after reviewing source commit, exact native joint order, all referenced mesh hashes, and tests. For the 58-element training model, apply [`scripts/build_collision58.py`](../scripts/build_collision58.py) to the pinned 91-element migration output and verify the resulting collision report. Keep source backups and transient build outputs outside the repository.
