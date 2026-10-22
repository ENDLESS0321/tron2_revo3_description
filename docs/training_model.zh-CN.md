# 降阶右臂训练总成

训练模型具有 28 个可动关节和 58 个碰撞元素。它保留新版官方 Revo3 的运动学、惯量、视觉几何和安装变换，只对选定的同 link 凸碰撞分块进行合并。这是几何近似，并非恢复旧机器人模型。

## 目录

- [模型身份](#模型身份)
- [碰撞体分配](#碰撞体分配)
- [构建与验证](#构建与验证)
- [兼容性与限制](#兼容性与限制)

## 模型身份

| 资产 | Link 数 | 可动关节数 | 碰撞元素数 |
| --- | ---: | ---: | ---: |
| 完整模型 `assets/assembly.urdf` | 114 | 58 | 280 |
| 训练模型 `assets/assembly_rl_convex.urdf` | 37 | 28 | 58 |

官方来源 commit 为 `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6`。训练 URDF SHA-256 为 `49120fdadec38e67dd81a39f3197858a9a643f950f18b350c6239dee15afb8ea`。未简化的迁移输入具有 91 个碰撞元素，SHA-256 为 `623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f`。仓库只打包当前训练资产；完整模型保留官方碰撞几何。

全部 28 个关节定义、惯量、视觉几何、安装变换和空固定指尖别名保持原有含义。物理 base 必须始终比桌面高 0.45 m：桌面 Z=0.507 m、物理 base Z=0.957 m、URDF root Z=-0.24335 m；详见[坐标系约定](../TABLETOP_FRAME_CONTRACT.zh-CN.md)。

## 碰撞体分配

| 分组 | 官方迁移输入 | 当前训练模型 |
| --- | ---: | ---: |
| 非手部机器人几何 | 9 | 9 |
| 手掌 | 5 | 3 |
| 拇指 | 41 | 24 |
| 食指 | 9 | 6 |
| 中指 | 9 | 6 |
| 无名指 | 9 | 5 |
| 小指 | 9 | 5 |
| 总计 | 91 | 58 |

拇指 CMR 从 32 块降至 19 块。五指 DIP 的全部碰撞几何，包括橡胶表面，均保持不变。只合并属于同一刚性 link 的分块；合并凸包保守包围对应的源凸包，因此可能局部外扩。在检查的顶点、边中点和三角形中心上，最大外扩距离为 0.341512 mm。这是采样结果，并非连续 Hausdorff 上界。

[几何报告](../assets/training_collision_geometry.json) 记录每次合并、输出网格、来源哈希和采样误差。完整模型仍需要原始官方碰撞网格。

## 构建与验证

1. 使用 [`build_official_revo3_assembly.py`](../scripts/build_official_revo3_assembly.py) 生成具有 91 个碰撞元素的官方迁移模型，保留降阶模型自身的安装变换。
2. 在具有 NumPy、SciPy 和 trimesh 的环境中运行第二阶段简化器。输出必须是仓库外的新目录；输入必须是 91 碰撞元素的迁移输出，不是已经简化的默认模型。

```bash
python scripts/build_collision58.py \
  --urdf /absolute/path/to/migration91/assembly.urdf \
  --output /absolute/path/to/new_collision58 \
  --target 58
```

生成器输出 URDF、生成的凸网格、采样报告和共享来源网格的符号链接。正式分发资产将引用落实在 `assets/` 内，不含外部符号链接。训练证据记录生成器的依赖版本，因为凸包三角化可能随库版本变化。

3. 从仓库根目录验证打包模型与网格哈希：

```bash
python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model training --check --output /tmp/revo3-training-check
```

测试检查数量、网格闭包、保留的关节结构、未改变的 DIP 几何，以及非零姿态下与完整模型的 FK 一致性。查看器仅用于运动学预览。

## 兼容性与限制

具有 1081 个样本的 contact-v5 螺丝刀参考 H5 保持 SHA-256 `08ccf7990ace5dca291895ffc700eee7c8df4baa20e1b150ff3b37ab5f3442c7`。新求解的 270 帧手臂 IK 无 fallback，最大位置误差 2.090e-8 m、最大姿态误差 8.149e-7 rad、最大速度比 0.168706。该名义轨迹的离散检查中，跨指间距至少 2.129 mm、机器人—桌面间距至少 3.121 mm、机器人—物体间距至少 0.143 mm，以上检查均无采样穿透。DIP 接触间隙保持不变。这些结果绑定精确资产与采样轨迹；CCD、随机轨迹、接触力和真机成功尚未验证。

不得仅因张量维度相同就复用 checkpoint。必须绑定策略、归一化器、URDF、网格闭包、H5、新 IK 和仿真设置。如果 Isaac 导入器为空固定指尖 link 赋予隐式质量，需要单独验证的运行派生模型：仅移除这些空叶节点，并以 offset 实现其传感器坐标系；碰撞体数仍为 58。正式逻辑 URDF 保留这些别名。
