# 发布资产测试

[English](README.md) | 简体中文

先按[首页安装说明](../README.zh-CN.md#快速开始)准备环境，以下命令均从仓库根目录运行。

## 已有检查

```bash
.venv/bin/python -m unittest discover -s tests -v
```

[test_release_assets.py](test_release_assets.py)检查清单摘要与网格引用、完整／训练拓扑和模型选择、确认 URDF 的精确身份、安装变换及 URDF/MJCF 对齐、基座原点与桌面顶面的 45 cm 高度差、约简模型关节／碰撞／惯性和右侧链 FK，以及相机机身、坐标系和配色。还会检查训练模型的相机选项被正确拒绝。

单元测试编译模型并核对数值关系。下面的渲染检查另行生成图像，并检查输出非空：

```bash
./viewers/view_adapters.sh --check --output outputs/check/adapters
./viewers/view_camera_brackets.sh --check --output outputs/check/brackets
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

零件检查覆盖左右件和四个预设视角。离屏渲染需要可用的 EGL 或已配置的其他图形后端。详见[查看工具](../viewers/README.zh-CN.md)和[相机输出格式](../docs/camera_outputs.zh-CN.md)。

## 修改资产后的维护流程

1. 保留已发布来源，在独立分支或输出目录中修改。
2. 核对 URDF 网格路径、单位、关节名和限位；涉及完整模型时，同步相应场景变换和运行配置。
3. 将已核验的新摘要、字节数和来源写入 [manifest.json](../assets/manifest.json)。该清单覆盖资产，并不覆盖所有说明文档。
4. 检查测试中固定的 hash 和几何预期。仅对明确批准的资产版本更新这些预期，不能通过刷新 hash 掩盖不明来源的差异。
5. 运行单元测试及相关渲染检查；发布前检查图像和相机元数据，并同步中英文说明。

检查通过只验证所述资产与预览约束，不证明实机装配、无碰撞运动、动力学等价、策略兼容或操作成功。资产变化后，模拟器旧缓存需要重新导入。
