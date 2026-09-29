# CAD 交接：TRON2 + Revo3 转接件与腕部相机支架

[English](README.md) | 简体中文

本目录包含随模型发布的左右手部转接件和腕部相机支架。文件清单与哈希记录在 [`../manifest.json`](../manifest.json)；viewer 选择的文件、单位缩放和仅用于显示的旋转角记录在 [`../runtime.json`](../runtime.json)。核对 CAD 导出是否属于本次发布时，请以这两个文件为准。

![包含双手和腕部相机的装配预览](../../docs/images/new_urdf_front_table.png)

## 文件选择

| 部件 | 左侧 | 右侧 | 用途 |
| --- | --- | --- | --- |
| 手部转接件，V2 | [`adapters/left.3mf`](adapters/left.3mf) 或 [`adapters/left_print_mm.stl`](adapters/left_print_mm.stl) | [`adapters/right.3mf`](adapters/right.3mf) 或 [`adapters/right_print_mm.stl`](adapters/right_print_mm.stl) | 3MF 为打包模型文件；`_print_mm.stl` 是零件 viewer 选用的毫米制 STL。 |
| 腕部相机支架，V3 | [`camera_brackets/left.step`](camera_brackets/left.step) 或 [`camera_brackets/left_mm.stl`](camera_brackets/left_mm.stl) | [`camera_brackets/right.step`](camera_brackets/right.step) 或 [`camera_brackets/right_mm.stl`](camera_brackets/right_mm.stl) | STEP 为 CAD 交换文件；`_mm.stl` 是零件 viewer 选用的毫米制 STL。支架预览不包含相机本体。 |

发布版 runtime 将配置标记为“purple adapter V2 + camera bracket V3”。装配 URDF 使用对应的 `adapter_v2` 网格和 `supplied_aligned_v3` 支架网格。这些版本名称仅用于识别随包提供的配置，不代表其与其他版本或实际硬件已确认可互换。

## 左右侧与单位

“左”和“右”按机器人自身面向前方时的左右定义。从机器人正面观察时，不要按观察者看到的画面左右来选文件。应使用对应侧的文件，不要镜像一侧来生成另一侧。

本目录 CAD 导出按毫米尺度提供。导入 STL 时应选择毫米；若以米为单位的 viewer/导入器需要比例系数，则使用 `0.001`。仿真网格单位为米。`runtime.json` 对这些 CAD STL 使用 `0.001` 缩放以供显示。编辑或制造前，请核对导入单位并用已知尺寸检查；STL 文件本身不可靠地携带单位信息。

零件 viewer 对 `camera_brackets/right_mm.stl` 另设绕 Y 轴 180° 的显示旋转；左支架和左右转接件的显示旋转为零。这只是 viewer 的对齐设置，不会修改右支架 CAD 坐标，也不是制造指令。除非经过单独审查的机械变更要求修改，否则应保留所提供的 CAD 几何。可查看完整装配预览了解安装后的显示位置；该预览仅为运动学预览，不能证明实际装配适配或间隙满足要求。

## 仿真模型中已确认的安装信息

完整装配 [`../assembly.urdf`](../assembly.urdf) 以米为单位定义了机器人坐标系下的固定安装 frame。左右 adapter mount 分别连接到对应侧的 `wrist_roll_*_Link`，手部基座再连接到转接件。URDF 中转接件到手部基座的变换为平移 `(0, 0.023855, 0)` m；左侧 RPY 为 `(-π/2, -π/2, 0)`，右侧为 `(-π/2, +π/2, 0)`。左右 adapter mount 原点均为 `(-0.0317, 0, -0.0812)` m；左侧 RPY 为 `(-π/2, 0, -π/2)`，右侧为 `(-π/2, 0, +π/2)`。这些是仿真 frame 变换，不是完整装配图，也不能替代对 CAD 基准体系的核对。

URDF 在每侧腕部定义了 V3 支架和 D405 相机 frame。CAD 目录提供左右支架几何，但不包含相机本体。完整运动学链和预览背景请参阅精确 URDF 及模型 README：

- [完整装配与坐标说明](../../README.zh-CN.md)
- [英文完整装配说明](../../README.md)
- [完整装配运动学预览](../../docs/images/new_urdf_front_table.png)

## 机械交接状态

**本发布包已确认：** 左右 CAD 文件均存在；manifest 记录了文件字节数和 SHA-256；runtime 选择毫米制 STL 并记录 viewer 显示旋转；装配 URDF 提供具名的机器人坐标安装 frame 和仿真变换；完整模型预览展示了预期仿真布置。

**制造或安装到硬件前仍待确认：** 实际机器人/灵巧手/相机的版本与接口尺寸；基准定义及公差；紧固件规格、螺纹、长度、等级、数量和扭矩；材料及加工工艺；若采用打印，还需确定打印方向、支撑、填充/壁厚设置和后处理；实际配合、走线、工具间隙、承载能力、疲劳与安全审查。随包文件没有确立这些规格。不得根据网格、URDF、文件名或 viewer 设置推断或补写数值。

本包是 CAD/模型交接资料，不是制造工程图、装配适配保证、结构评估，也不构成硬件或制造认证。制造或安装前，应使用实际部件核验尺寸与接口，并由负责的机械工程人员完成审查。
