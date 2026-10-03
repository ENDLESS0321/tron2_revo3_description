# 降阶28 physicsfix 训练模型

[English](training_model.md) | 简体中文

## 目录

- [资产与范围](#资产与范围)
- [28 个可动关节](#28-个可动关节)
- [固定姿态与保留惯性](#固定姿态与保留惯性)
- [碰撞体清单](#碰撞体清单)
- [坐标摆放](#坐标摆放)

## 资产与范围

仓库中的训练模型路径为[`assets/assembly_rl_convex.urdf`](../assets/assembly_rl_convex.urdf)。该文件是提供的源资产 `assembly_bilateral_axis180_reduced28_physicsfix.urdf` 的逐字节副本；原源文件名仅记录来源，并非另一个仓库文件。[`assets/manifest.json`](../assets/manifest.json) 记录其 SHA256：`2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034`，以及逐文件 hash 和上游版本。完整 URDF 的 SHA256 为 `537f31a798ddb05d1f29e2b5eeede63d47af3d9ff519f40907a24e757f5bbf44`。

降阶模型有 38 个 link、37 个 joint（28 个 revolute、9 个 fixed）、78 个 visual、分布于 31 个 link 上的 58 个 collision，以及 128 个唯一 mesh 引用。可动链为右臂与右手。头部、左臂、左手及其几何已固化到保留的祖先 link；原有独立关节和 link/frame 身份已移除。需要这些原始 frame 的程序无法再通过本 URDF 中的名称找到它们。

## 28 个可动关节

下表按右侧运动链从机器人基座到指尖分支排列。名称区分大小写且必须完全匹配。五个手指各自有一条独立的串联分支。

| 分组 | 关节名，按近端到远端排列 |
| --- | --- |
| 右臂 | `proximal_pitch_R_Joint`、`proximal_roll_R_Joint`、`proximal_yaw_R_Joint`、`elbow_R_Joint`、`wrist_yaw_R_Joint`、`wrist_pitch_R_Joint`、`wrist_roll_R_Joint` |
| 拇指 | `right_thumb_CMP_joint`、`right_thumb_CMR_joint`、`right_thumb_MCP_joint`、`right_thumb_PIP_joint`、`right_thumb_DIP_joint` |
| 食指 | `right_index_MPR_joint`、`right_index_MCP_joint`、`right_index_PIP_joint`、`right_index_DIP_joint` |
| 中指 | `right_middle_MPR_joint`、`right_middle_MCP_joint`、`right_middle_PIP_joint`、`right_middle_DIP_joint` |
| 无名指 | `right_ring_MPR_joint`、`right_ring_MCP_joint`、`right_ring_PIP_joint`、`right_ring_DIP_joint` |
| 小指 | `right_little_MPR_joint`、`right_little_MCP_joint`、`right_little_PIP_joint`、`right_little_DIP_joint` |

此表是便于阅读的运动链分组，不表示策略 action 向量顺序。训练 URDF 中可动 joint 的 XML 声明顺序与表格分组相同（右臂、拇指、食指、中指、无名指、小指）；右臂七个关节也是基座到腕部的连续链。不同运行时导入器枚举关节时可能采用不同索引顺序。[`assets/runtime.json`](../assets/runtime.json) 将展示姿态存为按名称索引的数值映射，并声明预期自由度为 28；它没有定义 action 向量或策略映射。控制量、IK、H5 轨迹和 checkpoint 应绑定精确关节名及 URDF hash；不能根据旧 58 维向量或张量维数相同推断兼容性。

## 固定姿态与保留惯性

此文件中不再可动的几何固化在以下姿态，角度单位为弧度：

- 头部 yaw 为 `0`，pitch 为 `0.35`。
- 左臂按 pitch、roll、yaw、elbow、wrist-yaw、wrist-pitch、wrist-roll 排列：`1.2395190538964451`、`0.0050965579201923406`、`0.0706739400409262`、`-0.10075078688932715`、`-0.33794336457190877`、`-0.6172884125393175`、`0.5453728186806424`。
- 左手所有关节均为零。

降阶时合并了惯性。右手五个 fingertip link 的质量均为 `0.001 kg`，惯性原点平移（质心）为零，对角惯量为 `(1e-10, 1e-10, 1e-9) kg·m²`，惯性积为零。`right_palm` 是没有 inertial 的占位 link。此资产同时包含指尖惯性修复与掌部碰撞恢复；它们对训练结果各自造成的影响尚未单独隔离。

## 碰撞体清单

| 分组 | 碰撞元素数 |
| --- | ---: |
| 基座与右臂七个 link | 8 |
| 右手基座／掌部分块 | 28 |
| 右手指分块 | 21 |
| 右侧 adapter／法兰 | 1 |
| **合计** | **58** |

保留的 `rl_convex` 文件名不代表所有碰撞几何都是一个凸包：掌部由 28 个 mesh 部件组成。训练 URDF 引用的 mesh 均随仓库置于 `assets/meshes/`，manifest 覆盖全部 128 个唯一引用。hash 命名的 `boundaryfix_exact/` 文件和 `right_adapter_single_convex.obj` 都是来源受追踪的仿真资产。仓库未说明“boundaryfix”所指网格生成流程。

## 坐标摆放

距离单位为米，角度为弧度。两个机器人 URDF 保留 `world_to_base = [0, 0, 1.20035]`。版本 1.8.0 将完整 MuJoCo 场景对齐到 physicsfix 训练布局：物理 base Z `0.957`、桌面 Z `0.507`、高度差 `0.45`。桌面完整尺寸为 `[0.7, 0.7, 0.4]`，中心为 `[0.2764393782702718, 0.39253473731213084, 0.307]`；场景 base 位置和四元数与 dex-retarget/dex-rl 一致。URDF 不包含桌面或物体。只应用一次显式导入 root 变换，详见[坐标系与摆放](../assets/README.zh-CN.md#坐标系与摆放)。

完整场景 XML 描述 58 自由度的完整 articulation。降阶28 URDF 对应训练拓扑。发布仓库中的预览工具只将降阶 URDF 用于运动学预览：由于提供的指尖惯量不满足 MuJoCo 的惯量三角检查，预览在内存中的 MuJoCo `MjSpec` 启用 `balanceinertia`。发布版 URDF 字节不会改变。该导入适配只是预览处理，不能证明 MuJoCo／Isaac 动力学等价、训练完成、硬件操作或任务成功。

仓库提供的 viewer helper 可通过以下方式加载运动学预览：

```python
import sys
sys.path.insert(0, "viewers")
from _runtime import load_training_model

model = load_training_model()
assert model.nq == 28
```
