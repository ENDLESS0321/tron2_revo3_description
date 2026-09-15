# 原 STEP 支架：向内安装、腕部孔组修订 V2

此版现保留为历史版本。2026-09-15 的摆正/工具空间/第三孔修改见 `../supplied_brackets_aligned_v3/`；`view_supplied_camera_rig.sh` 默认入口已切换 V3，要查看本版请显式传入本版配置。

本版仅修改腕部连接区域，支架向内安装到用户指定的相邻小孔。机器人孔位不改；手和手转接件沿用上一版左 +90°、右 -90° 的装配姿态，未再次旋转。

## 交付文件

给结构同学优先提供两个精确实体 STEP：

- `camera_bracket_left_inward_v2.step`
- `camera_bracket_right_inward_v2.step`

同名 `_mm.stl` 是毫米网格，可用于检查/试制切片；STEP 是 CAD 修改的首选。没有把相机实体或螺钉合并进支架。两份最初的 `.stp` 原件均保留未修改。

## 修改内容与边界

- 补实原两处腕部通孔及沉孔，在新位置重新开 Ø3.3 mm 通孔、Ø6.5 mm 沉孔。
- 新孔仍在源 CAD 的 Y=-1.4 mm 高度；左件角度为 60°/120°，右件为 240°/300°。
- 支架分别向内旋转后，两侧均配准到机器人原转接件坐标系的 **330°/30° 小孔**，轴向位置 Y=-4.5 mm。没有使用附近的大固定孔。
- 沉孔底径向位置保持 R34 mm，名义座下径向壁厚约 3.5 mm；保留原腕部 R30.5 配合内圆。
- 为使新沉孔出口贯通，清除了连接区域外缘的一小段凸缘。全部变化限制在腕部连接区域内：源坐标 |Z|<35 mm、Y=-5.9…5.9 mm、R30.5…40 mm；原支撑段、颈部上段和相机板不重建、不拉伸、不调角。
- 相机后部两孔和相机与支架的相对安装位姿保持不变。
- 没有新增大避让孔。支架覆盖机器人 15° 位置的大转接件固定孔，沿用用户指定的安装思路。

## 验证结果

- 左右 STEP 均为单一有效实体，导出后重新导入通过；STL 均为封闭、正体积网格。
- 几何差集检查：腕部连接区域以外变化为 0，上部支撑/相机板保护区域变化为 0（CAD Boolean 容差 0.00001 mm）。STEP 序列化体积相对差约 0.000111% / 0.000051%，记录在 CAD manifest 中。
- 相机原两孔轴线几何差为 0；装配后的相机孔位数值残差小于 0.001 mm。
- 两侧腕部目标小孔轴向及切向数值残差小于 0.001 mm。这些是模型配准结果，不是实机制造公差承诺。
- 与上一版比对了 163 个非相机 link/joint 元素，全部保持一致，手和转接件的姿态参数也保持一致。
- MuJoCo 加载通过：58 个机器人自由度、6 个 RGB/深度渲染视点。固定 STEP 不允许用旧滑条只移动相机。
- 十个轴向截面中，支架与手转接件重叠为 0；支架与腕壳最大截面重叠约 0.0465 mm²，处于原始零名义间隙圆柱面的网格近似区域。尚未做制造间隙补偿，不能据此宣称实机零干涉。

## 结构与装配注意

本版是几何与装配验证件，不是强度/制造放行件。

- 原支架材料未核实，仿真惯量沿用实心铝 2700 kg/m³ 的估计；不能直接当作打印材料强度参数。
- 腕部螺纹规格、螺钉长度、有效啮合深度和实际头型需结构人员核实。
- 新孔局部的 Ø6.48 mm、R34.01…38.99 mm 检查包络内无实体阻挡。它只是避开重合面的几何测试，并未指定实际螺钉。
- 每侧有一个孔的长直 Ø6.5 mm 工具进出包络会遇到保留的相机板，需确认工具尺寸、角度和装配顺序；本版没有为此切掉相机板。
- 孔距缩短后紧固件受力分配发生变化，未完成载荷、振动、疲劳或打印工艺验证。

## 查看

在项目 `code/tron2_revo3_description` 中：

```bash
# 整机装配，原入口现已指向本修订版
bash view_supplied_camera_rig.sh

# 只显示左右相机支架，不显示相机、机器人或桌子
bash view_supplied_camera_rig.sh --brackets-only

# 查看相机 RGB/深度画面
bash view_cameras.sh --config config/camera_rig_supplied_step_inward_v2.json
```

旧原件装配版仍可用 `--config config/camera_rig_supplied_step_20260914.json` 显式打开。旧默认 `config/camera_rig.json` 没有覆盖。

新的 URDF：`urdf/tron2_dach_revo3_supplied_step_inward_v2.urdf`。

检查报告与图片位于 `reports/cameras/supplied_step_inward_v2/`，其中：

- `hole_alignment.png`：腕部网格截面和正确孔组；
- `left_mount.png` / `right_oblique.png`：整机近景；
- `left_wrist_rgb.png` / `right_wrist_rgb.png`：实际相机渲染；
- `validation.json`：装配检查数据。

相较旧版，掌心投影已进入两侧视野，掌面法向与看向镜头方向的夹角由约 86.6° 变为约 61°。但原板角和短支撑保留，因此画面仍偏近、掌面与手指不全在画面中央；本版不声称已获得理想抓取视角。

## 复现

```bash
.cad-venv/bin/python scripts/modify_supplied_wrist_mounts.py
env -u PYTHONPATH .venv/bin/python scripts/prepare_inward_bracket_rig.py
env -u PYTHONPATH .venv/bin/python scripts/check_inward_bracket_rig.py
```

本目录 `*_cad_manifest.json` 记录 CAD 修改与保护区域检查，`*_asset_manifest.json` 记录仿真网格、惯量估计和文件摘要。
