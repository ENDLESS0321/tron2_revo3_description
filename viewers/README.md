# Model viewers

[English](README.md) | [简体中文](README.zh-CN.md)

The viewer supports the unchanged full assembly and the exact model5200 Isaac runtime URDF.

```bash
./viewers/view_assembly.sh --model full
./viewers/view_assembly.sh --model training
./viewers/view_assembly.sh --model training --check
```

The training option reads assets/training/model5200_official_aligned_20261023_importer_runtime.urdf and its local assets/training/meshes closure. It is a kinematic preview. The displayed 28-joint robot does not validate dynamics, contact response, collision-free motion, training replay, or hardware limits. See [the training model record](../docs/training_model.md).
