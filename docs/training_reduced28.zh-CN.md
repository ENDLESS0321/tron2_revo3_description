# 28 自由度训练 URDF（1.7.1）

`assets/assembly_rl_convex.urdf` 现为用户指定的
`assembly_bilateral_axis180_reduced28_physicsfix.urdf` 的原样副本，逐字节一致。
沿用原训练路径，便于现有配置引用，但拓扑已改变：它不再是 1.5.0 的
58 自由度、仅简化双侧法兰的模型。`assets/assembly.urdf` 仍保留完整的 58 自由度装配模型。

## 拓扑与活动关节

| 项目 | 当前发布的训练模型 |
| --- | --- |
| URDF link 数量 | 38，包含 URDF world link |
| URDF joint 数量 | 37：28 个 revolute、9 个 fixed |
| 可控制关节链 | 右臂 7 个关节＋右 Revo3 手 21 个关节 |
| visual 几何数量 | 78 |
| collision 元素数量 | 58，分布在 31 个 link 上 |
| 引用网格数量 | 128 个唯一文件；新增 28 个手掌组件网格，移除旧的单一手掌凸包网格 |
| 碰撞分布 | 基座和 7 个手臂 link：8；手掌组件：28；手指组件：21；法兰：1 |

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

physicsfix 模型恢复了 28 个手掌组件碰撞网格，同时保留单个法兰碰撞和手臂／手指的详细碰撞。
128 个唯一网格引用均相对 `assets/`。相较 1.7.0，新增 28 个手掌网格并移除旧单一手掌凸包，
因此碰撞几何发生变化，其物理行为不等同于旧训练模型或完整装配。为兼容既有配置仍保留
`rl_convex` 文件名，但当前手掌由 28 个组件组成。

五个指尖 link 恢复了惯量：各自质量 0.001 kg、质心为零，惯量对角项为
`(1e-10, 1e-10, 1e-9) kg·m²`。手掌碰撞改动和这些惯量改动同时实施，尚未隔离各自的因果影响。
`right_palm` 仍是没有 inertial 的占位 link，因此仍与完整装配基线不同。历史派生报告的质量守恒
结论不适用于此 physicsfix 资产。

`world_to_base` 仍为 `[0, 0, 1.20035]` m。默认场景桌面顶面为 0.75035 m，
比基座原点低 0.45 m。当前训练布局使用基座 Z=0.957 m、桌面顶面 Z=0.507 m，同样相差 0.45 m。
训练 URDF 本身不含桌子或被操作物体。
默认 `scene.xml` 仍对应完整 58 自由度装配模型，不是此约简训练模型。
训练时应导入本 URDF 并显式配置世界／基座／桌面布局，避免重复叠加文件的世界偏移。
同为 45 cm 布局，不表示任意旧 IK sidecar 与本资产兼容。

## 来源与验证

发布文件 SHA256：

```text
2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034
```

`assets/training_reduced28.json` 记录指定文件名、替换前后哈希、各网格哈希及派生来源。
历史源派生指标针对更早的单掌模型，不能证明 physicsfix 模型的质量或惯量守恒。
当前 physicsfix URDF 正用于 4090D 四卡训练；训练仍在进行，本次发布不宣称 reward、性能、抓取或
物理任务成功，同时实施的碰撞与惯量改动也未隔离因果关系。

发布测试独立检查文件哈希、网格引用、28 关节拓扑、保留的右侧关节定义、相对完整模型的
右侧 FK，以及 MuJoCo 预览编译后的 58 个碰撞 shape。预览加载器在内存中平衡惯量以支持运动学展示；
MuJoCo 会拒绝直接加载原始 URDF。
其他引擎可能合并固定 link 或采用不同网格解释，需要核对其实际导入结果。

## 使用与迁移

```python
import sys
sys.path.insert(0, "viewers")
from _runtime import load_training_model
model = load_training_model()
assert model.nq == 28
```

MuJoCo 直接加载原始 URDF 会因部分指尖惯量不满足惯量三角不等式而报错。预览加载器在内存中启用
`balanceinertia` 后编译模型，以支持无物理步进的展示；它不修改磁盘上的 URDF，但编译时会调整惯量。
因此该预览不能用于宣称 MuJoCo 动力学与 Isaac 训练相同。

运行 `.venv/bin/python -m unittest discover -s tests -v` 验证发布资产。
重新导入此前缓存了旧模型的仿真场景；按 28 关节拓扑更新 action 映射、body／frame 引用
及机器人配置，并将 IK／H5／运行配置绑定到本 URDF 哈希。
仅张量尺寸匹配不能证明 checkpoint 兼容。本次文档更新不启动训练，也不报告 reward、训练性能、抓取或物理操作成功结果。

旧双侧法兰凸包生成器及其输出报告／网格已从当前目录移除，避免覆盖指定的约简模型；
它们仍保留在 1.5.0 的 Git 历史中。本仓库以资产形式发布指定的约简模型，不声称能用已移除的
法兰专用生成器重建其 boundaryfix 派生流程。

## 仓库预览与完整性检查（1.7.1）

```bash
./viewers/view_assembly.sh --model training
./viewers/view_assembly.sh --model training --check --output outputs/check/training
```

约简模型查看工具核对原样 URDF 的 28 自由度／零个渲染相机，并用内存中的惯量平衡仅编译模型，
仅设置配置中的右侧
展示关节。`0` 复位这 28 个关节；`1` 恢复展示姿态，头部及左侧的烘焙姿态始终固定。
预览显示视觉几何，不含桌子或物体，不进行物理步进。
`--check` 输出 `assembly_training_both_overview.png`。三相机和 CAD 零件查看工具仍使用
完整模型，指定训练选项会明确报错。`runtime.json` 声明两种模型路径、预期拓扑、展示姿态及
桌面／相机可用性。完整性检查会核对完整与训练模型的网格清单、manifest 和来源记录。

![28 自由度模型展示姿态：正向运动学预览，未进行物理步进](images/training_reduced28_preview.png)
