# 资产模型与坐标系

[English](README.md) | 简体中文

## 模型选择

| 文件 | 用途 | 自由度 | 桌面与相机 |
| --- | --- | ---: | --- |
| [`assembly.urdf`](assembly.urdf) | 完整双侧机器人及相机安装／光学 frame | 58 | 不含桌面和 MuJoCo 渲染相机定义 |
| [`scene.xml`](scene.xml) | 完整 MuJoCo 场景 | 58 | 含桌面及六个 RGB／深度渲染相机 |
| [`assembly_rl_convex.urdf`](assembly_rl_convex.urdf) | 降阶右臂与右手训练 articulation | 28 | 不含桌面、物体、相机 frame 或渲染相机 |

仓库中的训练资产就是此处的 URDF。源文件名 `assembly_bilateral_axis180_reduced28_physicsfix.urdf` 仅用于记录来源。冻结几何、完整关节清单、碰撞体和惯性注意事项见[训练模型说明](../docs/training_model.zh-CN.md)。

## 坐标系与摆放

机器人 URDF 和仿真场景中的长度单位为米，关节角度单位为弧度。URDF 的 `world_to_base` 固定关节将机器人基座放在 `[0, 0, 1.20035] m`。完整场景桌面顶面为 `z = 0.75035 m`，比基座原点低 `0.45 m`。另有独立记录的 physicsfix 训练布局：基座 `z = 0.957 m`、桌面顶面 `z = 0.507 m`，二者也相差 `0.45 m`。这些是不同的世界摆放；训练 URDF 不含桌面，导入模拟器时须只设置一次 world／base／table 关系，避免重复叠加完整模型的偏移。

```mermaid
graph TD
  W[世界] -->|URDF／完整场景 world_to_base：z 1.20035 m| B[TRON2 基座]
  B --> L[左臂与 Revo3]
  B --> R[右臂与 Revo3]
  T[完整场景桌面顶面：z 0.75035 m] -. 低于基座原点 0.45 m .-> B
  BT[训练布局基座 z 0.957 m] -. 高于桌面顶面 z 0.507 m 0.45 m .-> TT[训练布局桌面]
```

双手与腕部相机安装采用 axis-180：绕父级腕部 link 的局部 Z 纵向中心线旋转，轴线经过 `[-0.0317, 0, 0] m`。这是安装变换，不是可动腕部 roll 轴。相对于旋转前的安装配置，安装平移与关节物理限位保留，头部相机不变。手到法兰平移为 `[0, 0.023855, 0] m`；左侧安装 RPY 为 `[-π/2, -π/2, 0]`，右侧为 `[-π/2, +π/2, 0]`。

## Mesh 路径与完整性

URDF／MJCF mesh 文件名相对于 `assets/`，例如 `meshes/tron2/base_Link.STL`。复制或移动模型时保留 `assets/meshes/` 目录结构，并相对于模型资产根目录解析路径，而不是相对于当前工作目录。仿真 mesh 通常以米为单位；D455 源 mesh 以毫米为单位，URDF 通过 `0.001` scale 转为米。制造 CAD 使用独立单位：adapter／支架 CAD STL 为毫米。

[`manifest.json`](manifest.json) 是发布资产清单，记录每个文件的 SHA256 与字节数、上游版本、完整模型 265 个唯一 mesh 引用、训练模型 128 个 mesh 引用和训练源资产 hash。接收修改或拷贝后的资产时应校验 hash。运行时预览会解析相对路径并拒绝缺失或与清单不符的文件，不会自动重建资产。来源与许可说明见[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)。
