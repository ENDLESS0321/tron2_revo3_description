# Model5200 training robot

[English](training_model.md) | [简体中文](training_model.zh-CN.md)

The repository packages source and Isaac runtime robot URDFs for a fresh run using the original model_5200 training framework. They are not bound to the historical model_5200.pt checkpoint, which used the previous 2776/f898 assets. Both files and the complete relative mesh/texture closure are hash-verified in [assets/training/manifest.json](../assets/training/manifest.json).

## Contents

- [Select the URDF](#select-the-urdf)
- [Mount and simulation values](#mount-and-simulation-values)
- [Verify](#verify)

## Select the URDF

| File | Purpose | SHA-256 |
| --- | --- | --- |
| assets/training/model5200_official_aligned_20261023_icp.source.urdf | Retargeting/source binding | 6e3592a9b0a648cbc73f6c96aaac5884d640e8fc9f3c31810ed64f38bed4b103 |
| assets/training/model5200_official_aligned_20261023_importer_runtime.urdf | Exact Isaac runtime import and preview | 8e330b11ecafb4cfc4dbeaafa44441261a6e915256e2a804e3bf701535ae0a23 |

The source has 37 links and 28 moving joints. The runtime has 32 bodies including world and 28 moving joints; it removes five empty fixed tip links while preserving their virtual sensor frames in [runtime_derivation.json](../assets/training/runtime_derivation.json). Both contain 58 collisions: 28 on the palm, 21 on fingers, and 9 on the arm, base, and adapter. Mesh bytes and scales follow the model5200 training asset.

## Mount and simulation values

The tabletop top is 0.507 m, the physical base origin is 0.957 m, and the URDF root is -0.24335 m in the authored world frame. The physical-base to tabletop gap is exactly 0.45 m. The complete model and its mesh closure are unchanged.

The hand importer overrides are 6.28 rad/s velocity, 0.35 Nm effort, and PD gains 3/0.1. They apply to the Revo3 hand only. The 1 kg world importer mass is separate from the 1.105000027 kg total mass of the 22 Revo3 hand bodies before startup DR. These values do not establish hardware limits.

## Verify

From the repository root, run:

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check
```

These checks verify packaged bytes, relative URI closure, topology, and kinematic preview. They do not verify training replay, policy performance, dynamics, or hardware behavior.
