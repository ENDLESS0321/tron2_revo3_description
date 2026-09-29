# Mesh 清单

[English](README.md) | 简体中文

URDF 和场景 XML 中的 mesh 路径均相对于 `assets/`。请将此目录结构与模型文件一起保留。每个文件的权威 hash 与字节数见[`../manifest.json`](../manifest.json)；模型用途、单位和坐标摆放见[`../README.zh-CN.md`](../README.zh-CN.md)。

| 目录 | 内容与作用 |
| --- | --- |
| `tron2/` | LimX TRON2 视觉网格及部分碰撞网格 |
| `revo3/` | BrainCo Revo3 手部视觉与碰撞几何 |
| `adapter_v2/` | 修订后的左右 adapter 视觉网格及分解的凸碰撞部件 |
| `wrist_camera_brackets/` | 提供的对齐 V3 支架仿真网格 |
| `cameras/` | D405／D455 相机外观网格 |
| `boundaryfix_exact/` | 降阶28 physicsfix 训练资产保留的 hash 命名网格 |
| `right_adapter_single_convex.obj` | 训练 adapter 碰撞网格 |

URDF 的视觉几何和碰撞几何可能使用不同文件：视觉网格用于外观，碰撞网格则为分解或凸化几何，并被实际放入 collision 元素。降阶28 URDF 共含 58 个 collision 元素，其中掌部为 28 个分块；详见[训练模型说明](../../docs/training_model.zh-CN.md)。`boundaryfix_exact` 目录名用于标识保留的资产集合，并不代表仓库记录了某种网格生成算法。目录中的 57 个文件和其他分发网格一样，都在 manifest 中单独记录 hash，并列入训练 URDF 的来源清单。仓库没有提供有关“boundaryfix”含义或原始生成过程的更强说明。

LimX、BrainCo、RealSense 和用户提供 CAD 的来源、版本与许可限制见[`../../THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md)。这些转换后的仿真网格不会扩展上游许可，也不代表经过机械或硬件验证。
