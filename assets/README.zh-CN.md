# 资产模型与坐标系

本说明介绍完整与降阶机器人资产，以及坐标系摆放约定。

## 模型选择

| 文件 | 用途 | 自由度 |
| --- | --- | ---: |
| [`assembly.urdf`](assembly.urdf) | 完整双臂机器人和相机坐标系 | 58 |
| [`scene.xml`](scene.xml) | 完整 MuJoCo 运动学场景、桌面和六个渲染相机 | 58 |
| [`assembly_rl_convex.urdf`](assembly_rl_convex.urdf) | 降阶右臂拓扑及官方右侧 Revo3 手 | 28 |

## 坐标系与摆放

URDF `world_to_base` 保持为 `[0, 0, 1.20035] m`。场景中物理 base 的 XYZ 为 `[-0.22, 0.60231100353792, 0.957]`；桌面高度为 `0.507 m`，比物理 base 原点低 `0.45 m`。桌面尺寸为 `[0.7, 0.7, 0.4] m`，中心为 `[0.2764393782702718, 0.39253473731213084, 0.307] m`。导入 URDF 时，将 root XYZ `[-0.22, 0.60231100353792, -0.24335]` 和场景 base 四元数应用一次。Root、腕部、法兰和物理 base 原点彼此不同。

完整模型的手部安装变换来自原完整总成并保持不变。降阶右手安装变换保留自其自身源总成，该总成具有不同安装上下文。禁止在两个资产之间互换安装参数。手部关节身份映射记录在[`official_revo3_migration.json`](official_revo3_migration.json)。

## Mesh 闭包与来源

URDF 和 MJCF 的 mesh 引用均相对于 `assets/`。`manifest.json` 包含发布文件清单及哈希校验。完整 URDF 引用 404 个唯一 mesh URI；降阶训练 URDF 引用 196 个。`scene.xml` 对两个 DAE 视觉网格使用派生的 MuJoCo STL 副本，源文件到派生文件的映射记录在 `manifest.json`。

桌面高度由[`../TABLETOP_FRAME_CONTRACT.md`](../TABLETOP_FRAME_CONTRACT.md)固定。官方手部源 commit、源与输出哈希、关节顺序、安装变换及引用 mesh 哈希记录在迁移 JSON 中。许可署名及尚未解决的再分发状态见[`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)。
