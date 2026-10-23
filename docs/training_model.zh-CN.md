# Model5200 训练机器人模型

[English](training_model.md) | [简体中文](training_model.zh-CN.md)

仓库现包含使用原 model_5200 框架从零训练所需的源 URDF 与 Isaac 运行时 URDF。新的 6e/8e URDF 资产与历史 model_5200.pt 检查点不兼容；该检查点绑定较早的 2776/f898 资产。两个新文件及其完整相对 mesh/texture 闭包均由[assets/training/manifest.json](../assets/training/manifest.json)进行哈希校验。

## 目录

- [URDF 选择](#urdf-选择)
- [安装与仿真参数](#安装与仿真参数)
- [验证](#验证)

## URDF 选择

| 文件 | 用途 | SHA-256 |
| --- | --- | --- |
| assets/training/model5200_official_aligned_20261023_icp.source.urdf | 重定向/源侧绑定 | 6e3592a9b0a648cbc73f6c96aaac5884d640e8fc9f3c31810ed64f38bed4b103 |
| assets/training/model5200_official_aligned_20261023_importer_runtime.urdf | Isaac 实际运行时导入与预览 | 8e330b11ecafb4cfc4dbeaafa44441261a6e915256e2a804e3bf701535ae0a23 |

源模型包含 37 个 link 和 28 个可动关节。运行时模型包含 32 个 body（含 world）和 28 个可动关节；它移除了五个空的固定指尖 link，并在[runtime_derivation.json](../assets/training/runtime_derivation.json)中保留虚拟传感器坐标系。两者均包含 58 个碰撞元素：掌部 28 个、手指 21 个、手臂/基座/转接件 9 个。mesh 字节、缩放和逐 link 碰撞数沿用 model5200 训练资产。模型保留现有掌部/拇指结构重叠；58 个碰撞元素只是几何数量统计，不表示整机几何或轨迹无碰撞。

## 安装与仿真参数

统一世界坐标系中，桌面顶面为 0.507 m，物理基座原点为 0.957 m，URDF root 为 -0.24335 m。物理基座到桌面的高度差精确为 0.45 m。完整模型及其 mesh 闭包保持原样。

手部导入器覆盖参数为 6.28 rad/s 速度、0.35 Nm 力矩和 3/0.1 PD 增益，仅适用于 Revo3 手部。1 kg 世界导入器质量与 1.105000027 kg Revo3 手部 22 个 body 在启动 DR 前的总质量分开记录。这些参数不构成硬件限值。

## 验证

在仓库根目录运行：

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check
```

训练查看器直接设置关节位置并调用 MuJoCo mj_forward 计算正向运动学，不执行动力学步进。这些检查验证打包字节、相对 URI 闭包、拓扑和运动学预览，不验证训练回放、策略性能、动力学或硬件表现。
