# Model5200 训练机器人模型

[English](training_model.md) | [简体中文](training_model.zh-CN.md)

仓库现包含使用原 model_5200 框架从零训练所需的源 URDF 与 Isaac 运行时 URDF。新的 6e/8e URDF 资产与历史 model_5200.pt 检查点不兼容；该检查点绑定较早的 2776/f898 资产。两个新文件及其完整相对 mesh/texture 闭包均由[assets/training/manifest.json](../assets/training/manifest.json)进行哈希校验。

## 目录

- [URDF 选择](#urdf-选择)
- [安装与仿真参数](#安装与仿真参数)
- [PACE 辨识的手部动力学参数](#pace-辨识的手部动力学参数)
- [验证](#验证)

## URDF 选择

| 文件 | 用途 | SHA-256 |
| --- | --- | --- |
| assets/training/model5200_official_aligned_20261023_icp.source.urdf | 重定向/源侧绑定 | 68301474686571650d6614be7d30109e6d086f1a6497978605d3f105379667a8 |
| assets/training/model5200_official_aligned_20261023_importer_runtime.urdf | Isaac 运行时导入与预览 | 0e379bce7fdbe731f3447bdbc1c4a119d9a6ea71d79a27c46532404ef7976a7c |

源模型包含 37 个 link 和 28 个可动关节。运行时模型包含 32 个 body（含 world）和 28 个可动关节；它移除了五个空的固定指尖 link，并在[runtime_derivation.json](../assets/training/runtime_derivation.json)中保留虚拟传感器坐标系。两者均包含 58 个碰撞元素：掌部 28 个、手指 21 个、手臂/基座/转接件 9 个。mesh 字节、缩放和逐 link 碰撞数沿用 model5200 训练资产。模型保留现有掌部/拇指结构重叠；58 个碰撞元素只是几何数量统计，不表示整机几何或轨迹无碰撞。

## 安装与仿真参数

统一世界坐标系中，桌面顶面为 0.507 m，物理基座原点为 0.957 m，URDF root 为 -0.24335 m。物理基座到桌面的高度差精确为 0.45 m。完整模型及其 mesh 闭包保持原样。

手部导入器保留 6.28 rad/s 速度覆盖。手部逐关节增益、力矩限幅、附加惯量和摩擦改用下文的 PACE 辨识结果；原先统一的 0.35 Nm 力矩与 3/0.1 PD 增益仅作为已被替代的值保留在 manifest 中。1 kg 世界导入器质量与 1.105000027 kg Revo3 手部 22 个 body 在启动 DR 前的总质量分开记录。这些参数不构成硬件限值。

## PACE 辨识的手部动力学参数

[assets/training/revo3_pace_sysid.json](../assets/training/revo3_pace_sysid.json) 记录右手 21 个关节的仿真等效动力学参数，摘自 2026-11-01 的 PACE 辨识报告：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| kp, kd | N·m/rad, N·m·s/rad | 仿真控制器的力矩域 PD 增益，不是 SDK 增益；采集时 SDK 为 Kp 3.0 / Kd 0.1 |
| armature | kg·m² | 附加关节空间惯量 |
| friction | N·m | 干摩擦力矩 |
| viscous_damping | N·m·s/rad | 被动黏性损失，与 kd 分开 |
| effort_limit | N·m | 拟合得到的力矩限幅，不是硬件额定值 |

拟合假设：共享指令延迟 15 ms、PD 更新 5 ms、物理步长 2.5 ms、连杆质量取资产值、固定手掌、开启自碰撞。复用这些参数时应保持 5 ms PD 周期；报告显示预测结果对该周期敏感。

这些值是一套参数：以 fit_03 为基线，其中报告第 17 节重新拟合的 7 个字段（新候选 balanced_20261101_01）替换了 fit_03 的原值。报告表格为四舍五入后的显示值；完整精度的 best_parameters.json 不在本仓库中，sidecar 记录了其 SHA-256。速度限制、电流到力矩常数、其他 SDK 增益设置及接触参数均未辨识。

### 加载参数

```python
from revo3_pace_profile import load_profile
joints = load_profile()   # fit_03，其中第 17 节的 7 个字段已替换
```

`load_profile()` 返回全部 21 个手部关节的 `{关节名: {kp, kd, armature, friction, viscous_damping, effort_limit}}`，不再有参数集选项。

### 相对 fit_03 的改动

只有下面 7 个数与 fit_03 不同；其余 18 个关节，以及所有摩擦、黏性阻尼和力矩限幅都是 fit_03 的值。被替换的原值保留在 sidecar 的 `parameters.replaced_from_fit_03` 中。

| 关节 | 字段 | fit_03（已替换） | 当前值 |
| --- | --- | ---: | ---: |
| 拇指 CMP | armature (kg·m²) | 0.00532 | 0.004364 |
| 拇指 CMP | kp (N·m/rad) | 7.151 | 23.543235 |
| 拇指 CMP | kd (N·m·s/rad) | 0.339 | 0.698884 |
| 拇指 CMR | kp (N·m/rad) | 0.785 | 1.10393 |
| 拇指 CMR | kd (N·m·s/rad) | 0.033 | 0.042304 |
| 无名指 PIP | kp (N·m/rad) | 2.29 | 4.652494 |
| 无名指 PIP | kd (N·m·s/rad) | 0.134 | 0.120927 |

fit_03 使这三个关节的阶跃响应比初始模型更差。第 17 节的重新拟合修正了这一点，代价是拇指周期跟踪变差：

| 按原记录指令回放（RMSE，°） | fit_03 | 当前 |
| --- | ---: | ---: |
| 全 21 关节阶跃阶段 | 1.228 | 1.034 |
| 五指 4 Hz 阶段 | 1.164 | 1.352 |
| 拇指 4 Hz | 1.605 | 2.303 |
| 无名指 4 Hz | 1.241 | 1.086 |

第 17 节的重新拟合复用了旧阶跃数据，尚未在新采集数据上验证。

### 写入 URDF 的值

两个训练 URDF 的 21 个手部关节写入了其中 URDF 能表达的部分：`<limit effort>` 为 effort_limit，`<dynamics damping>` 为 viscous_damping，`<dynamics friction>` 为干摩擦。关节范围、`velocity="0"`、origin、惯性参数和 mesh 均未改动；manifest 保留了每个文件原 model5200 版本的 SHA-256。URDF 没有对应 kp、kd、armature 和指令延迟的字段，这些只来自 sidecar。如果 Isaac Lab 执行器配置也设置了 friction 或 viscous_friction，会覆盖导入器从 URDF 读到的值。

命令行查看：

```bash
python3 scripts/revo3_pace_profile.py              # 21 个手部关节参数表
python3 scripts/revo3_pace_profile.py --isaaclab   # DelayedPDActuatorCfg 字段字典
```

在 Isaac Lab 中，kp/kd/effort_limit/armature/friction/viscous_damping 分别对应 `DelayedPDActuatorCfg` 的 `stiffness`/`damping`/`effort_limit`/`armature`/`friction`/`viscous_friction` 字段。`sim.dt = 0.005` 时，15 ms 延迟即 `min_delay = max_delay = 3` 步。拟合中的 2.5 ms 物理子步在 PhysX 中没有直接对应项。

## 验证

在仓库根目录运行：

```bash
python3 -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check
```

训练查看器直接设置关节位置并调用 MuJoCo mj_forward 计算正向运动学，不执行动力学步进。这些检查验证打包字节、相对 URI 闭包、拓扑和运动学预览，不验证训练回放、策略性能、动力学或硬件表现。
