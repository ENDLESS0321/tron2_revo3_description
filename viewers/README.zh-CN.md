# 预览工具指南

[English](README.md) | 简体中文 · [相机采集文件](../docs/camera_outputs.zh-CN.md)

这些启动脚本对随仓库发布的模型执行正向运动学预览，不进行物理步进，也
不会连接机器人硬件。从仓库根目录运行；脚本会自行定位仓库和 `.venv`，不
依赖当前工作目录。

## 安装与启动

脚本直接调用 `.venv/bin/python`，因此必须先在仓库根目录创建该环境并安装
依赖。安装步骤见根目录 [`README.zh-CN.md`](../README.zh-CN.md#快速开始)。
脚本会清除继承的 `PYTHONPATH`，避免误载其他目录中的模块。

| 启动脚本 | 预览内容 | 支持的模型／参数 |
| --- | --- | --- |
| `./viewers/view_adapters.sh` | 左右手部法兰 | `--side left\|right\|both`，默认 `both` |
| `./viewers/view_camera_brackets.sh` | 左右相机支架，不含相机本体 | `--side left\|right\|both`，默认 `both` |
| `./viewers/view_assembly.sh` | 完整装配与桌面 | `--model full\|training`，默认 `full`；接受 `--side`，但不影响装配显示 |
| `./viewers/view_cameras.sh` | 头部 D455 与两个腕部 D405 的 RGB／深度视图 | `--width`、`--height`、`--rate`；仅完整模型 |

通用参数为 `--check` 和 `--output DIR`。`--check` 会离屏渲染图像（相机模式
会生成一组采集文件），不打开窗口并退出。默认输出目录是 `outputs/`。相机
尺寸默认 320×240，采集循环速率默认每秒 8 次；宽度范围 64–1280，高度
64–960，速率须大于 0 且不超过 60。尺寸与速率只作用于相机预览；`--side`
用于零件预览，`--model` 用于装配预览。`training` 只支持
`view_assembly.sh`，因为其相机 frame 已在降阶时固化。

```bash
./viewers/view_assembly.sh --model training
./viewers/view_adapters.sh --side left
./viewers/view_camera_brackets.sh --side right
./viewers/view_cameras.sh --width 640 --height 480 --rate 8
./viewers/view_assembly.sh --check --output outputs/check/assembly
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

## 操作与渲染

3D 窗口中，按住鼠标左键拖动可旋转，右键拖动可平移，滚轮可缩放。零件窗
口用 `1`、`2`、`3` 选择左侧、右侧、双侧；`F`、`B`、`T`、`U` 选择前、后、
顶、腕侧视角；`R` 重置视角。装配窗口用 `0` 选择零位、`1` 选择配置的展示
姿态，`R` 重置视角。降阶训练预览仅有保留的 28 个可动关节；固化的头部和
左侧几何不会移动。

3D 窗口使用 MuJoCo GLFW；相机窗口使用 Tk 和 MuJoCo EGL。交互窗口需要桌面
环境（`DISPLAY` 或 `WAYLAND_DISPLAY`）；无桌面时使用 `--check` 离屏生成输
出。离屏渲染仍要求图形运行环境可用。除非环境中已设置 `MUJOCO_GL`，脚本在
`--check` 和相机模式选择 EGL，在交互 3D 模式选择 GLFW。这里不假定存在适用
所有机器的 EGL／OSMesa 修复办法。

所有预览均为运动学预览。原始训练 URDF 的指尖惯量不满足 MuJoCo 惯量三角
检查。预览运行时会对内存中的 `MjSpec` 启用 `balanceinertia`；不会改写发布
的 URDF，也不能据此证明与 Isaac 的动力学等价。

## 故障排查

- **提示找不到 `.../.venv/bin/python`**：在仓库根目录创建 `.venv` 并安装
  `requirements.txt`；启动脚本固定使用该解释器。
- **提示 `No desktop display`**：使用 `--check` 离屏生成输出。无桌面的机器
  仍需可用的 EGL 图形环境才能渲染 MuJoCo 场景。
- **提示 `Asset changed` 或 `Missing or invalid asset`**：运行时会将资产
  SHA256 与 `assets/manifest.json` 比较。请恢复发布时的确切文件，或调查被
  修改的资产；预览不会重建资产或跳过校验。
- **MuJoCo 报惯量三角错误**：使用提供的训练预览入口。直接编译原始训练
  URDF 不会应用仅供预览的内存惯量平衡。

相机窗口的保存按钮和输出文件结构见
[`camera_outputs.zh-CN.md`](../docs/camera_outputs.zh-CN.md)。
