# TRON2 + Revo3 Assembly Models

This repository preserves the unchanged full visual assembly and packages the new robot assets for a fresh run using the original model_5200 training framework. This asset set is not bound to the historical model_5200.pt checkpoint, which used earlier 2776/f898 assets.

[English](README.md) | [简体中文](README.zh-CN.md)

## Contents

- [Choose a model](#choose-a-model)
- [Preview and verification](#preview-and-verification)
- [Limits](#limits)

## Choose a model

| Model | Use it for | Moving joints |
| --- | --- | ---: |
| [Full assembly](assets/assembly.urdf) | Bilateral visualization and the original full robot description | 58 |
| [Training source](assets/training/model5200_official_aligned_20261023_icp.source.urdf) | Retargeting and source-side asset binding | 28 |
| [Isaac runtime](assets/training/model5200_official_aligned_20261023_importer_runtime.urdf) | Reproducing the model5200 Isaac import | 28 |

The training source and runtime URDF are distinct files with their original bytes preserved. The runtime URDF removes five empty fixed tip links for importer compatibility; their sensor frames remain documented in [runtime derivation](assets/training/runtime_derivation.json). The [training manifest](assets/training/manifest.json) records hashes, mesh closure, geometry counts, mount height, and simulation overrides.

## Preview and verification

Install Python dependencies with python3 -m pip install -r requirements.txt, then run:

```bash
./viewers/view_assembly.sh --model full
./viewers/view_assembly.sh --model training
python3 -m unittest discover -s tests -v
```

The training viewer loads the exact runtime URDF for kinematic preview. [The frame contract](TABLETOP_FRAME_CONTRACT.md) fixes the tabletop to physical-base gap at 0.45 m.

## Limits

The package records a model and preview configuration. It does not certify trajectory clearance, dynamics, task success, hardware operation, or hardware limits. The hand importer overrides are 6.28 rad/s velocity, 0.35 Nm effort, and PD gains 3/0.1. They apply to the Revo3 hand only and are not hardware limits. The 1 kg world importer mass is separate from the 1.105000027 kg total mass of the 22 Revo3 hand bodies before startup DR. See [third-party notices](THIRD_PARTY_NOTICES.md).
