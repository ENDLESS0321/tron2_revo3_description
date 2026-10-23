# 模型查看器

[English](README.md) | [简体中文](README.zh-CN.md)

查看器支持未修改的完整总成，以及 model5200 Isaac 实际运行时 URDF。

```bash
./viewers/view_assembly.sh --model full
./viewers/view_assembly.sh --model training
./viewers/view_assembly.sh --model training --check
```

training 选项读取 assets/training/model5200_official_aligned_20261023_importer_runtime.urdf 及其本地 assets/training/meshes 闭包。该功能仅用于运动学预览。28 关节机器人显示不验证动力学、接触响应、无碰撞运动、训练回放或硬件限值。详见[训练模型记录](../docs/training_model.zh-CN.md)。
