# TRON2 + Revo3 最终装配模型

[English](README.md) | 简体中文

查看 DACH_TRON2A 双臂、两只 BrainCo Revo3、最终手转接件和 V3 相机支架。
外观配色按参考图更新为 TRON2 深灰黑主体、红/青点缀、Revo3 银灰手部和紫色法兰。
三相机预览包含头部 D455、左右腕 D405 的 RGB 与米制深度。

`main` 提供完整装配、约简训练模型、查看工具和来源记录；`dev` 保留早期开发快照及 CAD
输入，已发布版本可从 Git 历史获取。运行不依赖 CAD 软件、开发目录、ROS 或在线下载模型。

## 安装方向配置（1.7.0）

左右手／法兰组件及左右腕部相机均沿前臂纵向中心轴旋转 180°。更新目的是在桌面操作的
掌心朝下、手指朝前姿态下获得更大的关节限位余量，物理限位不变。
详见[旋转说明、模型来源及预览](docs/mount_rotation.zh-CN.md)。
已将确认的新 URDF 原样覆盖到 `assets/assembly.urdf`，完整装配 URDF 使用此默认路径；
整机及相机预览加载匹配的 `scene.xml`，
训练预览加载约简 URDF。MuJoCo 场景同步更新，基座原点到桌面顶面的高度差保持 45 cm。

## 训练模型与预览（1.7.1）

`assets/assembly_rl_convex.urdf` 是 physicsfix 版约简模型
（`assembly_bilateral_axis180_reduced28_physicsfix.urdf`）的原样副本：右臂 7 个关节＋右手
21 个关节。58 个碰撞元素分布于基座、手臂、法兰、28 个手掌组件和 21 个手指组件。
完整装配模型和默认 MJCF 保持原样。复用 action 映射、body／frame 名称、IK 或 checkpoint
前，请阅读[训练拓扑与迁移说明](docs/training_reduced28.zh-CN.md)。当前正用于 4090D 四卡训练；
本次发布不宣称训练成功。MuJoCo 训练预览仅在内存中平衡指尖惯量以供展示，不进行物理步进，
也不修改发布的 URDF。

## 安装

已验证：Ubuntu 22.04、Python 3.10。在项目根目录运行；本机已有 `.venv` 时可直接启动。

```bash
sudo apt install python3-venv python3-tk libgl1 libegl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

三维窗口使用 GLFW；三相机窗口使用 Tk，离屏图像使用 EGL。无显示环境可用 `--check`。
EGL 不可用时，可在安装相应图形库后尝试 `MUJOCO_GL=osmesa`。

## 四个启动入口

所有入口集中在 `viewers/`。原根目录的重复启动脚本已移除。

| 命令 | 内容 |
| --- | --- |
| `./viewers/view_adapters.sh` | 只看左右手转接件 |
| `./viewers/view_camera_brackets.sh` | 只看左右相机支架，不包含相机机身 |
| `./viewers/view_assembly.sh` | 查看完整机器人、双手、转接件、支架及相机 |
| `./viewers/view_cameras.sh` | 同时显示三路 RGB 和三路深度，共六个画面 |

也可以从其他目录使用绝对路径启动。脚本会定位本仓库的环境和资产。
分发时须保留整个目录结构，不要只复制一个脚本或一个 URDF。

### 零件与整机操作

鼠标左键拖动旋转，右键拖动平移，滚轮缩放，关闭窗口退出。

- 零件窗口：`1` 左件、`2` 右件、`3` 并排；`F/B/T/U` 切换正面/背面/顶面/腕侧视角；`R` 复位。
- 零件窗口支持 `--side left`、`--side right`、`--side both`；左件橙色，右件蓝色。
- 整机窗口：`0` 零位、`1` 展示姿态、`R` 复位视角。只做正向运动学预览，不运行自由动力学。

整机入口支持 `--model full`（默认，58 自由度、含桌子）和 `--model training`
（28 自由度、不含桌子）。训练模式的 `0` 只复位保留的 28 个关节，已烘焙的头部和左侧
姿态保持固定；`1` 使用 `runtime.json` 中的右侧展示姿态。零件和相机查看工具使用完整资产，
不接受训练模型选项，因为其独立相机 frame 已被合并。所有预览均不进行物理步进。

```bash
./viewers/view_assembly.sh --model training
./viewers/view_adapters.sh --side left
./viewers/view_camera_brackets.sh --side right
```

### 三相机与保存

```bash
./viewers/view_cameras.sh --width 640 --height 480 --rate 8
```

点击“保存当前图像与深度”，默认写入 `outputs/<时间戳>/`；`--output` 可指定目录。
每路保存 RGB PNG、深度伪彩色 PNG、原始深度 `_depth_m.npy`，以及包含内参、光学
坐标系名称、时间戳和有效像素统计的 `capture.json`。

- 原始深度为 `float32`，单位米，表示光学坐标系 Z 距离；无效值为 `NaN`，显示为黑色。
- 头部 D455 深度预览使用其标称 0.6–6 m 理想范围，腕部为 0.07–0.5 m。
- 原始深度未与彩色图配准；D455 的两路光学原点和视场不同。
- 这是理想针孔仿真，不是实机采集；固定模型没有会令相机脱离安装孔的长度/倾角滑条。

## 无窗口检查

检查资产完整性、模型加载并实际渲染，不打开桌面窗口。零件检查覆盖左右件和多个视角。

```bash
./viewers/view_adapters.sh --check --output outputs/check/adapters
./viewers/view_camera_brackets.sh --check --output outputs/check/brackets
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

资产摘要不一致时不会自动重建或替换模型。请用 `git status` 检查改动，从可信提交
恢复相应资产后重试。

## 模型与 CAD 路径

```text
assets/
├── assembly_rl_convex.urdf  # 右臂＋右手训练模型，28 自由度
├── training_reduced28.json  # 来源、拓扑、网格哈希及验证
├── mount_rotation.json     # 完整装配安装方向来源
├── assembly.urdf           # 完整装配；米/弧度，58 个机器人自由度
├── scene.xml               # MuJoCo 场景，含六个 RGB/深度渲染视点
├── runtime.json            # 完整／训练模型选择、姿态、相机及零件配置
├── manifest.json           # 文件摘要与上游版本
├── meshes/                 # 实际引用的最终网格，含必要碰撞网格
└── cad/
    ├── adapters/           # left/right.3mf 与 left/right_print_mm.stl
    └── camera_brackets/    # left/right.step 与 left/right_mm.stl
viewers/                    # 四个入口与共用实现
licenses/                   # 上游许可、声明及许可状态依据
```

交结构同学：相机支架优先用 `assets/cad/camera_brackets/left.step` 和 `right.step`；
手转接件使用 `assets/cad/adapters/` 的 3MF/STL。CAD STL 单位为毫米，仿真网格为米。
相机机身不在支架制造文件中。

完整装配 URDF 包含 RealSense 官方 D455 机身网格，以及标称深度、彩色、红外和 IMU
坐标系；RGB/深度渲染使用 `scene.xml` 中的相机定义。

URDF 和 MuJoCo 场景中的 `base_Link` 原点均位于世界坐标 `z=1.20035 m`；
场景工作台上表面位于 `z=0.75035 m`，与该机器人原点的垂直距离恰好为 `0.45 m`。
URDF 本身不包含桌面几何体。

```python
import mujoco
model = mujoco.MjModel.from_xml_path("assets/scene.xml")
```

## 分支与使用边界

- `dev`：完整开发快照，不含本机虚拟环境和解释器缓存；包括原始 STP/3MF 与三个
  vendor 仓库的本地 Git 元数据。
- `main`：最终模型及四类查看功能。切换前请关闭窗口并保存本地修改。单文件恢复：
  `git restore --source dev -- <路径>`；完整开发版：`git switch dev`。
- 公开仓库：[ENDLESS0321/tron2_revo3_description](https://github.com/ENDLESS0321/tron2_revo3_description)。
  `main` 和 `dev` 均发布，完整开发快照与原始 CAD 也在公开历史中；从 `main` 工作树
  删除文件不会使历史中的文件变为私有。相邻手套程序、交付包、skill 交接包不包含在本仓库。
- 相机型号/内参是仿真选型与名义值，非实机标定。未完成实机装配、线缆/工具空间、
  全运动范围、强度或疲劳认证；原始贴合面的公差仍需结构同学核实。
- 不连接硬件，不发送控制命令；制造和实际抓取前需要另行验证。

许可及 BrainCo 尚未明确的许可状态见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
