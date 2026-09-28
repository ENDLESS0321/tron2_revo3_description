# 28 自由度训练 URDF（1.6.0）

`assets/assembly_rl_convex.urdf` 现为用户指定的
`assembly_bilateral_axis180_reduced28.urdf` 的原样副本，逐字节一致。
沿用原训练路径，便于现有配置引用，但拓扑已改变：它不再是 1.5.0 的
58 自由度、仅简化双侧法兰的模型。`assets/assembly.urdf` 仍保留完整的 58 自由度装配模型。

## 拓扑与活动关节

| 项目 | 当前发布的训练模型 |
| --- | --- |
| URDF link 数量 | 38，包含 URDF world link |
| URDF joint 数量 | 37：28 个 revolute、9 个 fixed |
| 可控制关节链 | 右臂 7 个关节＋右 Revo3 手 21 个关节 |
| visual 几何数量 | 78 |
| collision 元素数量 | 31，分布在 31 个 link 上 |
| 引用网格数量 | 101，其中 31 个为仓库新增文件 |
| 右法兰碰撞 | `right_adapter_link` 上一个凸碰撞网格 |
| 右手掌碰撞 | `right_hand_base_link` 上一个凸碰撞网格 |

右臂从近端到末端的关节名依次为：`proximal_pitch_R_Joint`、`proximal_roll_R_Joint`、
`proximal_yaw_R_Joint`、`elbow_R_Joint`、`wrist_yaw_R_Joint`、
`wrist_pitch_R_Joint`、`wrist_roll_R_Joint`。
右手保留拇指 5 个关节，食指、中指、无名指和小指各 4 个关节。
控制映射应按关节名称建立，不能直接沿用旧的 58 维向量或假定 XML 顺序就是控制顺序。

右臂和右手的 revolute 关节定义、轴和限位沿用派生源文件；右法兰保留 axis-180
安装方向，手与法兰的连接变换保持原样。

## 固定链的烘焙处理

头部及左侧关节不再是此资产中可控制的关节。它们的几何在下述姿态下变换到最近的
保留祖先 link 坐标系，惯量使用平行轴定理合并：

- 头部 yaw = 0 rad，pitch = 0.35 rad。
- 左臂按 pitch／roll／yaw／elbow／wrist-yaw／wrist-pitch／wrist-roll 顺序为
  `1.23951905389645 0.00509655792019234 0.0706739400409262 -0.100750786889327 -0.337943364571909 -0.617288412539318 0.545372818680642` rad。
- 左手关节全部为零。

这保留了冻结姿态下的几何，不是直接丢弃左臂和头部，但它们不再拥有独立的运行时
关节及 link 身份。相机外观几何可以保留，而独立机械／光学 frame link 可能随约简被移除；
使用方不能假定完整装配版的相机 frame 名称及 MJCF 相机设置仍存在于本 URDF 中。

## 碰撞资产与坐标

指定文件使用 boundaryfix 碰撞网格、单个右法兰凸网格及单个右手掌凸网格。
全部 101 个网格引用均相对 `assets/`，已随仓库提供，包括 31 个新增文件。
网格内容直接复制，不重新导出或改名；URDF 内容和引用保持原样。
法兰／手掌凸碰撞几何可能填平孔洞及凹陷，接触与间隙检查应以此模型的边界为准。

`world_to_base` 仍为 `[0, 0, 1.20035]` m。默认场景桌面顶面为 0.75035 m，
比基座原点低 0.45 m。训练 URDF 本身不含桌子或被操作物体。
默认 `scene.xml` 仍对应完整 58 自由度装配模型，不是此约简训练模型。
训练时应导入本 URDF 并显式配置世界／基座／桌面布局，避免重复叠加文件的世界偏移。
同为 45 cm 布局，不表示任意旧 IK sidecar 与本资产兼容。

## 来源与验证

发布文件 SHA256：

```text
56fdcc40198d38f075dd307f256d7b961ea7197958b617c550ad6bb8a9a3c313
```

`assets/training_reduced28.json` 记录指定文件名、替换前后哈希、各网格哈希及源派生报告。
源报告追溯到 single-palm boundaryfix 前序模型，由 58 个活动关节约简为 28 个；
报告中的质量守恒误差约为 1.28e-11 kg，全局惯量张量误差约为 6.38e-11，
10 个右侧关节姿态的 FK 误差为零。这些是源派生结果，不是本次重新验证的 RL 成功结果。

发布测试独立检查文件哈希、网格引用、28 关节拓扑、保留的右侧关节定义、相对完整模型的
右侧 FK，以及 MuJoCo 导入后右法兰和右手掌各一个碰撞 shape。
MuJoCo 加载后有 28 个广义位置、28 个关节和 31 个碰撞 shape。
其他引擎可能合并固定 link 或采用不同网格解释，需要核对其实际导入结果。

## 使用与迁移

```python
import mujoco
model = mujoco.MjModel.from_xml_path("assets/assembly_rl_convex.urdf")
assert model.nq == 28
```

运行 `.venv/bin/python -m unittest discover -s tests -v` 验证发布资产。
重新导入此前缓存了旧模型的仿真场景；按 28 关节拓扑更新 action 映射、body／frame 引用
及机器人配置，并将 IK／H5／运行配置绑定到本 URDF 哈希。
仅张量尺寸匹配不能证明 checkpoint 兼容。本次发布不启动训练，也不报告 reward、
训练性能、抓取或物理操作成功结果。

旧双侧法兰凸包生成器及其输出报告／网格已从当前目录移除，避免覆盖指定的约简模型；
它们仍保留在 1.5.0 的 Git 历史中。本仓库以资产形式发布指定的约简模型，不声称能用已移除的
法兰专用生成器重建其 boundaryfix 派生流程。
