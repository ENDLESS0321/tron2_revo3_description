# TRON2 + Revo3 装配模型

[English](README.md) | 简体中文

本仓库提供 DACH_TRON2A 双臂机器人、BrainCo Revo3 双手、手部法兰、V3 腕部相机支架、头部 D455 和两个腕部 D405 的模型、制造 CAD 与运动学预览工具。模型资产版本为 1.7.1，包含 physicsfix 训练 URDF；所需 mesh 均随仓库提供。

![完整装配及桌面：运动学预览](docs/images/new_urdf_front_table.png)

## 快速开始

已在 Ubuntu 22.04、Python 3.10 上验证；预览无需 CAD 软件、ROS 或额外模型下载。

```bash
git clone https://github.com/ENDLESS0321/tron2_revo3_description.git
cd tron2_revo3_description
sudo apt install python3-venv python3-tk libgl1 libegl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
./viewers/view_assembly.sh
```

无桌面环境时，将最后一行替换为 `./viewers/view_assembly.sh --check --output outputs/check/assembly`。离屏渲染仍需要可用的图形后端，详见[查看工具说明](viewers/README.zh-CN.md)。

## 选择模型

| 文件 | 用途 | 可动关节 | 桌面／渲染相机 |
| --- | --- | ---: | --- |
| [assets/assembly.urdf](assets/assembly.urdf) | 完整双侧装配 | 58 | URDF 不含桌面或渲染相机定义，但保留相机机身和坐标系 |
| [assets/scene.xml](assets/scene.xml) | 完整模型的 MuJoCo 预览 | 58 | 含桌面及 6 个 RGB／深度视点 |
| [assets/assembly_rl_convex.urdf](assets/assembly_rl_convex.urdf) | physicsfix 右臂／右手训练 articulation | 28 | 不含桌面或渲染相机定义 |

双侧手／法兰与腕部相机使用 axis-180 安装。旋转轴、坐标变换、米／弧度单位和场景布局见[资产说明](assets/README.zh-CN.md)。训练模型包含右臂 7 个和右手 21 个可动关节；接入 IK、控制映射或 checkpoint 前，请阅读[训练模型接入说明](docs/training_model.zh-CN.md)。

![降阶训练模型：运动学预览](docs/images/training_reduced28_preview.png)

## 查看入口

| 命令 | 显示内容 |
| --- | --- |
| `./viewers/view_adapters.sh` | 左右手法兰 |
| `./viewers/view_camera_brackets.sh` | 左右相机支架，不含相机本体 |
| `./viewers/view_assembly.sh` | 完整装配及桌面 |
| `./viewers/view_assembly.sh --model training` | 28 关节训练模型 |
| `./viewers/view_cameras.sh` | 三路 RGB 与三路深度 |

入口脚本固定使用仓库的 `.venv`，也支持从其他目录通过绝对路径启动。鼠标左键旋转、右键平移、滚轮缩放；装配窗口中 `0` 切换零位、`1` 切换展示姿态。完整参数、快捷键、保存路径与排错见[查看工具说明](viewers/README.zh-CN.md)。

## 按用途阅读

| 任务 | 文档 |
| --- | --- |
| 导入模型、理解坐标与布局 | [资产说明](assets/README.zh-CN.md) |
| 接入训练、IK 或控制映射 | [训练模型接入](docs/training_model.zh-CN.md) |
| 交给结构同学或选择制造文件 | [CAD 制造交付](assets/cad/README.zh-CN.md) |
| 使用查看工具和无窗口模式 | [查看工具](viewers/README.zh-CN.md) |
| 读取 RGB、深度与相机元数据 | [相机输出格式](docs/camera_outputs.zh-CN.md) |
| 理解视觉／碰撞网格与来源 | [网格说明](assets/meshes/README.zh-CN.md) |
| 修改资产后执行验证 | [测试与维护](tests/README.zh-CN.md) |

[文档索引](docs/README.zh-CN.md)提供上述入口的完整导航。

## 验证与资产身份

```bash
.venv/bin/python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

[manifest.json](assets/manifest.json)记录资产 SHA256、字节数和来源；预览先检查完整性，不会静默重建被修改的模型。[测试说明](tests/README.zh-CN.md)列出全部检查命令和维护流程。

## 仓库内容与使用边界

`main` 提供模型、制造 CAD 和查看工具；`dev` 保留早期开发快照。各目录入口为 [assets](assets/README.zh-CN.md)、[viewers](viewers/README.zh-CN.md)、[tests](tests/README.zh-CN.md) 和 [docs](docs/README.zh-CN.md)。截图和采集数据默认写入被 Git 忽略的 `outputs/`。

全部查看工具仅做运动学预览，不进行物理步进，也不发送硬件命令。训练预览在内存中平衡指尖惯量以满足 MuJoCo 编译要求；预览通过不能证明动力学一致、物理抓取成功或策略兼容。相机使用标称参数的理想针孔模型，深度未与彩色图配准。

制造文件、单位和未确认参数见 [CAD 说明](assets/cad/README.zh-CN.md)。结构强度、疲劳、线缆／工具间隙和硬件标定尚未认证。来源与许可条件见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)，BrainCo 的许可状态仍未解决。
