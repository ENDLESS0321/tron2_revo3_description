# TRON2 + Revo3 Assembly Model

This repository provides models, manufacturing CAD, and kinematic viewers for a dual-arm DACH_TRON2A robot with BrainCo Revo3 hands, adapters, camera mounts, and camera frames. The Revo3 hand subtrees now use the official source model at commit `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`; see the [migration record](assets/official_revo3_migration.json).

## Contents

- [Models](#models)
- [Quick start](#quick-start)
- [Verification](#verification)
- [Limits](#limits)

## Models

| Asset | Purpose | Moving joints |
| --- | --- | ---: |
| [Full assembly](assets/assembly.urdf) | Bilateral robot, official left/right hand kinematics and camera frames | 58 |
| [MuJoCo scene](assets/scene.xml) | Full kinematic preview with table and six rendering cameras | 58 |
| [Reduced training assembly](assets/assembly_rl_convex.urdf) | Existing reduced right-arm topology with official right-hand subtree | 28 |

The hand migration preserves each input assembly's arm, adapter, camera, non-hand transforms, and authored root-to-physical-base transform. Hand mount geometry is input-specific. Hand joints retain the official names and native order; mappings are recorded in the migration JSON. The table-to-physical-base gap remains 0.45 m under the [frame contract](TABLETOP_FRAME_CONTRACT.md).

## Quick start

On Ubuntu with Python 3.10, create an environment, install requirements, then launch a viewer:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
./viewers/view_assembly.sh
```

The full and reduced model preview commands are `./viewers/view_assembly.sh --model full` and `./viewers/view_assembly.sh --model training`.

## Verification

```bash
.venv/bin/python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
```

[`assets/manifest.json`](assets/manifest.json) verifies packaged file hashes and mesh closure. The migration record identifies the official source commit, source URDF hashes, output hashes, mounts, joint-order mappings, and mesh manifests.

## Limits

This is an asset and kinematic-preview update. It does not validate dynamics, collision-free trajectories, training compatibility, task success, hardware operation, or hardware limits. RL training is deferred. The official vendor tree did not include an applicable license file in the inspected source; attribution is provided, but redistribution permission remains unresolved. See [third-party notices](THIRD_PARTY_NOTICES.md).
