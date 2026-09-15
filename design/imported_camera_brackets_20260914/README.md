# 原始 STEP 支架装配检查版 — 2026-09-14

后续用户已授权修改腕部连接区域，已另行完成 `../supplied_brackets_inward_v2/`。`view_supplied_camera_rig.sh` 现指向该修订版；本目录保留原件检查和旧错误孔组选取记录。

本版完成了原始左右支架替换及手/转接件的 ±90° 装配预览，但**未通过可用手心视角检查**。没有覆盖默认 `config/camera_rig.json`，也没有修改两份原始 STP。

**用户纠正后的状态：本版选用了错误的腕部孔组。** 用户指定两侧均保持相邻的 330°/30° 小孔，不是下文记录的 120° 孔对。因此下文配准记录仅描述旧预览版，不能作为当前要求的验收。`reports/cameras/requested_inward_holes_20260914/` 中的实际截面对照表明：原 STEP 两孔在 R30.5 贴合面上的弦距 52.83 mm，指定两孔弦距 30.50 mm；将原支架向内转 90° 后，每孔仍相差 30°。当前仅完成核查，未修改支架几何或装配，等待确认是否允许修改支架腕部连接段以适配固定的机器人孔位。手与转接件的既有 90° 旋转保持不动。

## 文件结论

- `Tron2灵巧手相机支架-l.stp` 和 `-r.stp` 均可正常解析为标准 STEP，各有一个有效实体。
- 两者外包尺寸均约 **66.312 × 11.800 × 72.159 mm**。主体为 Z 方向镜像布局，不是一长一短。
- 镜像配准后对称差体积约 1.638 mm³，占单件体积约 0.019%；存在小局部差异，因此两件分别使用，没有拿同一个网格代替左右件。
- 相机不包含在支架实体内，装配使用独立的 D405 模型。

## 配准方式

以原手转接件坐标系为基准：+Y 指向手指，X/Z 为腕部径向。

原 STEP 到该坐标系的旋转为 `diag(-1,-1,1)`，位移为 `[0,-5.9,0] mm`。因此支架腕箍位于 Y=-11.8…0 mm，孔轴 Y=-1.4 mm 映射到腕部 Y=-4.5 mm。

- 左支架使用腕部 30°/150° 的小孔，右支架使用 210°/330° 的小孔。两孔径向夹角为 120°，不是旧自建侧支架的 60° 孔对。
- 支架通孔 Ø3.3 mm；腕部网格中的对应孔约 Ø2.5 mm。只确认轴线配准，不能据此确认螺纹、螺钉长度和有效啮合深度。
- 相机后部两孔中心距 20 mm；按 STEP 接触平面与官方相机孔位放置，孔位数值残差小于 0.001 mm。此为模型计算精度，不是制造精度。
- 从机械臂向指端看，左手与左转接件逆时针 90°，右手与右转接件顺时针 90°。对应腕部坐标 +Z 旋转为左 +90°、右 -90°。
- 仅修改手转接件根固定关节；手与转接件之间的相对位置、手内部关节及机器人姿态参数不变。
- 旋转后四个转接件固定孔仍对应 15°/105°/195°/285° 孔组，网格拟合切向偏差约不超过 0.051 mm。10 个轴向切片中防呆扇区重叠为零。
- 转接件与腕壳在非防呆区仍有旧模型已有的局部重叠；本次前后对照基本相同，不能宣称整体无干涉或实机可装。
- 支架内圆 R30.5 与腕壳名义外圆贴合，尚未施加制造装配间隙。截面还存在很小的网格重叠，需结构人员确认公差。

## 尚未解决：相机视角

使用原支架和上述 90° 旋转后，镜头虽在掌面正侧，但从掌心看向镜头的方向与掌面法向夹角约 **86.6°**，接近沿掌面侧视。掌心投影位于图像纵向边界之外，实际渲染受到拇指/手掌明显遮挡。

所以本版不能标记为“已能看清手心”。建议下一步保留腕部孔组和相机两孔，调整中间支撑的空间偏置与相机板朝向，将镜头移向新掌面法向一侧后重新瞄准抓取区域。仅把原径向支撑加长，并不一定解决侧视问题。此修改尚未实施，等待用户确认。

## 查看和复现

在 `code/tron2_revo3_description` 下运行：

```bash
bash view_supplied_camera_rig.sh
bash view_cameras.sh --config config/camera_rig_supplied_step_20260914.json
```

固定 STEP 版本禁止使用旧版长度/倾角滑条，以免只移动相机而使孔位脱离实体。

主要输出：

- `urdf/tron2_dach_revo3_supplied_step_20260914.urdf`
- `simulation/cameras/supplied_step_20260914/scene.xml`
- `reports/cameras/supplied_step_20260914/validation.json`
- `reports/cameras/supplied_step_20260914/left_assembly_1.png` 等装配图
- `reports/cameras/supplied_step_20260914/left_wrist_rgb.png`、`right_wrist_rgb.png`
- 本目录中的 `source_inspection.json`、`interfaces.json`、左右 `*_manifest.json`、`*_source_mm.stl`

重新生成：

```bash
.cad-venv/bin/python scripts/import_supplied_camera_brackets.py
env -u PYTHONPATH .venv/bin/python scripts/prepare_supplied_bracket_rig.py
env -u PYTHONPATH .venv/bin/python scripts/check_supplied_bracket_rig.py
```

支架惯量仅临时按实心铝密度 2700 kg/m³ 估计，未确认原件材质；相机和支架当前主要用于装配与成像预览，没有完成接触动力学、结构强度或实机紧固验证。不要把本检查版直接视为制造放行文件。
