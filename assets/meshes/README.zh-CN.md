# Mesh 清单

本目录仅包含当前机器人资产需要的网格文件。URDF/MJCF 路径相对于 `assets/`；逐文件哈希记录在[`../manifest.json`](../manifest.json)。

## Mesh 分类

| 目录 | 用途 |
| --- | --- |
| `tron2/` | TRON2 视觉网格和选定碰撞网格 |
| `hands/` | 官方左右 Revo3 手部视觉与碰撞几何 |
| `adapter_v2/` | 当前双侧转接件几何 |
| `wrist_camera_brackets/` | 已对齐的相机支架几何 |
| `cameras/` | D405/D455 视觉网格 |

## 碰撞与来源说明

完整 URDF 引用 404 个唯一 mesh URI；降阶右臂 URDF 引用 196 个。碰撞几何遵循官方手部模型，每个 link 可能包含多个网格；`rl_convex` 文件名属于历史命名，并不表示所有碰撞体都是凸形。两个官方 DAE 视觉网格转换为派生 MuJoCo STL 文件，以便场景编译。源文件到派生文件的映射和源哈希记录在 `assets/manifest.json` 与 `assets/official_revo3_migration.json`。

官方源版本与尚未解决的许可状态见[`../../THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md)。这些资产不证明机械性能或硬件安全。
