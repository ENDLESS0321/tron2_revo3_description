# TRON2 + Revo3 机器人总成模型

本仓库提供 DACH_TRON2A 双臂机器人及 BrainCo Revo3 双手、转接件、相机支架和相机坐标系的模型、制造 CAD 与运动学查看器。Revo3 手部子树现使用官方源模型 commit `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`；详见[迁移记录](assets/official_revo3_migration.json)。

## 目录

- [模型](#模型)
- [快速开始](#快速开始)
- [验证](#验证)
- [限制](#限制)

## 模型

| 资产 | 用途 | 可动关节数 |
| --- | --- | ---: |
| [完整总成](assets/assembly.urdf) | 双臂机器人、官方左右手运动学和相机坐标系 | 58 |
| [MuJoCo 场景](assets/scene.xml) | 含桌面及六个渲染相机的完整运动学预览 | 58 |
| [降阶训练总成](assets/assembly_rl_convex.urdf) | 原降阶右臂拓扑及官方右手子树 | 28 |

手部迁移保留每个输入总成中的机械臂、转接件、相机、非手部变换和已编写的 root 到物理 base 变换。手部安装几何随各自输入总成而异。手部关节保留官方名称和原生顺序，映射记录在迁移 JSON 中。桌面到物理 base 的高度差仍为 0.45 m，遵守[坐标系约定](TABLETOP_FRAME_CONTRACT.md)。

## 快速开始

Ubuntu 与 Python 3.10 环境下创建虚拟环境、安装依赖并启动查看器：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
./viewers/view_assembly.sh
```

完整模型和降阶模型的预览命令分别为 `./viewers/view_assembly.sh --model full` 与 `./viewers/view_assembly.sh --model training`。

## 验证

```bash
.venv/bin/python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
```

[`assets/manifest.json`](assets/manifest.json) 校验打包文件哈希和 mesh 闭包。迁移记录列出官方源 commit、源 URDF 哈希、输出哈希、安装变换、关节顺序映射和 mesh 清单。

## 限制

本次更新仅涉及资产和运动学预览，不代表动力学、无碰撞轨迹、训练兼容性、任务成功或硬件运行验证，也不定义硬件限位。RL 训练暂缓。检查的官方供应商源码未包含适用的许可文件；已标注来源，但再分发授权仍未确认。详见[第三方声明](THIRD_PARTY_NOTICES.md)。
