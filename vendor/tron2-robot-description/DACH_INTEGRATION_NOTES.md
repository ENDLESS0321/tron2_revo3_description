# DACH_TRON2A 本地集成检查

检查对象是官方 `tron2a/DACH_TRON2A/urdf/robot.urdf`，commit
`9939c22e69d27653ec0ba8a505859a2903dd1a71`。原始文件未修改。
完整来源、文件 SHA256、字节数和实际所需网格列表见 `VENDOR_SOURCE.json`。

## 结构和安装接口

- 单根 `base_Link`，25 个 link，24 个 joint；14 个机械臂旋转关节和 2 个头部旋转关节。
- 每侧机械臂顺序（`S` 为 `L` 或 `R`）：`proximal_pitch_S_Joint` → `proximal_roll_S_Joint` → `proximal_yaw_S_Joint` → `elbow_S_Joint` → `wrist_yaw_S_Joint` → `wrist_pitch_S_Joint` → `wrist_roll_S_Joint`。
- 最后一个机器臂 link 为 `wrist_roll_L_Link` / `wrist_roll_R_Link`；最后一轴的 axis 均为局部 `(1,0,0)`。
- 使用该基础 URDF 替换夹爪时，移除 `grasper_L_Link`、`grasper_R_Link` 及其 `grasper_L_Joint`、`grasper_R_Joint`，保留两侧 `wrist_roll_*_Link`。
- 两侧原夹爪固定关节相对 wrist_roll 的变换均为 `xyz=(-0.0317,0,-0.0812)` 米、`rpy=(0,0,0)`。这是官方夹爪安装参考坐标；转接件相对它的安装旋转和手侧装配坐标仍须由转接件几何验证。
- 两侧 wrist_roll 网格的最低 Z 面为 `-0.0812` 米，与上述关节平面一致。URDF 没有另行命名的 flange/tool link。

## 头部和基座

| 关节 | 父 → 子 | origin xyz，米 | axis | 弧度范围 |
|---|---|---|---|---|
| `head_base_Joint` | `base_Link` → `head_base_Link` | `(0.04656,0.00009,0.26115)` | fixed | — |
| `head_yaw_Joint` | `head_base_Link` → `head_yaw_Link` | `(0,0,0.008)` | `(0,0,1)` | `[-pi/2,pi/2]` |
| `head_pitch_Joint` | `head_yaw_Link` → `head_pitch_Link` | `(0.051,0.03,0.097)` | `(0,1,0)` | `[-pi/4,pi/3]` |

上表 origin 的 rpy 都为零。两头部关节 effort 为 20，velocity 为 17.8，按原始 URDF 保留。

零关节姿态、以 `base_Link` 为参考：

| link / 参考点 | xyz，米 | 旋转 |
|---|---|---|
| `wrist_roll_L_Link` | `(0.03126,0.16758,-0.54469)` | I |
| `wrist_roll_R_Link` | `(0.03126,-0.16758,-0.54469)` | I |
| 原左夹爪坐标原点 | `(-0.00044,0.16758,-0.62589)` | I |
| 原右夹爪坐标原点 | `(-0.00044,-0.16758,-0.62589)` | I |
| `base_3_Link` | `(0.00056,-0.00009,-1.00235)` | I |
| `head_pitch_Link` | `(0.09756,0.03009,0.36615)` | I |

保留立柱和底座时，底座网格最低点在 `base_Link` 下方约 1.20035 米；
把根 link 放在 world Z=1.20035 米可使网格底面约在地面 Z=0。
此值来自 URDF 变换与 STL 顶点边界，不是额外标定结果。

## 网格路径和单位

源文件使用 `package://robot_description/tron2/DACH_TRON2A/meshes/NAME.STL`，
而本 checkout 的文件位于 `tron2a/DACH_TRON2A/meshes/NAME.STL`。
生成新的独立 URDF 时应重写 mesh filename 为输出 URDF 能解析的相对路径，
并处理原文件中的 `<mujoco><compiler meshdir="../meshes"/></mujoco>` 扩展。
不能直接按 package 前缀拼接 checkout 目录。

基础 `robot.urdf` 需要 26 个不同 STL，已全部取得，总计 52,516,284 字节。
移除原两夹爪后，新组合实际保留其中 24 个。原 URDF 没有额外 mesh scale；
STL 顶点尺寸与米制 URDF 一致。26 个文件均通过 binary STL 三角数/字节长检查。
额外 URDF 和 xacro 仅作源文件参考，未下载其 articulated gripper 或 camera wrist 额外网格。

## 已完成的静态检查与边界

- XML 可解析；link/joint 名称唯一；每个非根 link 有一个父关节；单个连通树。
- 所有 25 个 link 均含正质量、正定惯性，并满足主惯量三角不等式。
- 原总质量 61.302 kg；两侧原夹爪各 0.53 kg；移除两夹爪后为 60.242 kg，尚未计入转接件及 Revo3。
- 无 collision 的 link 是 `base_imu`、`head_base_Link`、`d435_Link`、`d435_optical_frame`。
- 官方 collision 是简化模型，不能作为完整外表面覆盖证明。例如 proximal_yaw visual 的 Z 范围约 `[-0.2611,0.0222]` 米，而其 collision STL 只有约 `[-0.0325,0.0222]` 米；wrist_roll 的 collision 也仅覆盖 visual 的部分区域。
- 本检查尚未进行动力学仿真、接触稳定性、整机自碰撞或实际转接装配验证。

## 上游许可证记录

已原样保留 `LICENSE`、`NOTICE`、`THIRD_PARTY_NOTICES.md`、`ASSETS.md`。
仓库整体声明 Apache-2.0；上游两份资产说明中仍存在 STL provenance 的 `TO CONFIRM`
标记。本次本地集成保留这些声明，未把它们改写为已完成确认。
