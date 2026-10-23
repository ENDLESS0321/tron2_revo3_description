# TRON2 + Revo3 总成模型

本仓库保留未修改的完整可视化总成，并为使用原 model_5200 训练框架的新一轮从零训练提供机器人资产。该资产组不绑定历史 model_5200.pt 检查点；历史检查点使用较早的 2776/f898 资产。

[English](README.md) | [简体中文](README.zh-CN.md)

## 目录

- [模型选择](#模型选择)
- [预览与验证](#预览与验证)
- [限制](#限制)

## 模型选择

| 模型 | 用途 | 可动关节 |
| --- | --- | ---: |
| [完整总成](assets/assembly.urdf) | 双臂可视化与原始完整机器人描述 | 58 |
| [训练源模型](assets/training/model5200_official_aligned_20261023_icp.source.urdf) | 重定向及源侧资产绑定 | 28 |
| [Isaac 运行时模型](assets/training/model5200_official_aligned_20261023_importer_runtime.urdf) | 复现 model5200 的 Isaac 导入 | 28 |

训练源 URDF 和运行时 URDF 是两个不同文件，均保留原始字节。运行时模型为兼容导入器移除了五个空的固定指尖 link；对应传感器坐标系记录在[运行时派生信息](assets/training/runtime_derivation.json)中。[训练 manifest](assets/training/manifest.json)记录哈希、mesh 闭包、几何数量、安装高度和仿真覆盖参数。

## 预览与验证

使用 python3 -m pip install -r requirements.txt 安装 Python 依赖，然后运行：

```bash
./viewers/view_assembly.sh --model full
./viewers/view_assembly.sh --model training
python3 -m unittest discover -s tests -v
```

训练查看器使用精确的运行时 URDF 进行运动学预览。[坐标系约定](TABLETOP_FRAME_CONTRACT.md)规定桌面到物理基座的高度差固定为 0.45 m。

## 限制

此仓库记录模型与预览配置，不代表轨迹净空、动力学、任务成功、硬件运行或硬件限位认证。手部导入器覆盖参数为 6.28 rad/s 速度、0.35 Nm 力矩和 3/0.1 PD 增益，仅适用于 Revo3 手部，且不是硬件限值。1 kg 世界导入器质量与 1.105000027 kg Revo3 手部 22 个 body 在启动 DR 前的总质量分开记录。详见[第三方声明](THIRD_PARTY_NOTICES.zh-CN.md)。
