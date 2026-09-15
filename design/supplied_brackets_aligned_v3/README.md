# V3：相机摆正、支撑居中、连接弧收短

对应 2026-09-15 用户的三处修改要求。原始 STP、V2 CAD、手和手转接件均保留；仅相机支架和相机相对安装位置更新。

## 修改结果

- 腕部两颗安装孔、支架根坐标、手及手转接件姿态不变。
- 消除原上部支撑的 12° 侧向偏转，使相机横边平行于手/转接件的横向基准。**这是横边摆正，不是清零所有倾角；原 10° 前后俯仰保留。**
- 上部支撑居中，并径向外移 10 mm，为螺钉工具杆与相机机身留空间。相机板、原上部颈段只作刚体变换，不拉伸；相机后部两孔仍为 20 mm 间距。
- 腕部连接弧收短为 88°（装配后绕正前方 ±44°），露出原被端部遮挡的第三颗固定孔。不是把整条连接段轴向削薄。
- 两安装孔继续使用 Ø3.3 mm 通孔、Ø6.5 mm 沉孔；沉孔底 R34 mm、轴向位置和方向保持不变。新弧内半径 R30.5、外半径 R35.5，轴向总宽 11.8 mm。
- 根部使用宽 25 mm、轴向厚 6.7 mm 的实心连接段及圆角，不增加贯穿主支撑的大避让孔。沉孔底到名义内圆仍约 3.5 mm。

## 交结构同学

优先使用本目录精确实体 CAD：

- `camera_bracket_left_aligned_v3.step`
- `camera_bracket_right_aligned_v3.step`

同名 `_mm.stl` 是毫米制封闭网格。相机和螺钉没有包含在这些支架文件中。

`*_cad_manifest.json` 记录尺寸、源文件摘要及 CAD 检查；`*_asset_manifest.json` 记录仿真网格和惯量假设。

## 验证与边界

- 左右件都是单一有效实体，STEP 重新导入通过，STL 封闭且正体积。
- 腕部两孔与目标 330°/30° 孔组配准，数值残差小于 0.001 mm；相机两孔数值残差小于 0.001 mm。数值精度不是制造精度承诺。
- URDF 中相机横边与手横向基准的夹角为 0°。163 个非相机 link/joint 元素与原装配一致。
- 每侧两颗安装螺钉外侧，Φ6.5 mm、从径向 R35.6 延伸 80 mm 的直线工具通道与支架、保守相机机身包络均无交叠。
- 转接件 105°/195°/285° 三颗固定螺钉分别通过 Φ9 mm 空间检查；285° 是此次重点露出的第三孔。夹在两安装孔之间的 15° 固定孔仍被覆盖，不宣称四孔都开放。
- 新根部与相机机身包络无新增交叠。原颈部与矩形相机包络约 2.45 mm³ 的交叠被保留并记录；包络包括实际圆角外壳不占用的角部，不能把它等同于精确相机实体的干涉体积。
- 保留原 R30.5 名义贴合内圆，尚未施加制造公差补偿。腕壳网格表面仍可有小量截面交叠；没有完成实机装配、线缆、运动全范围、疲劳或强度验证。
- 材料尚未核实，仿真惯量暂按实心铝 2700 kg/m³ 估计。实机螺纹、螺钉长度、头型及工具尺寸须复核；不要按该密度直接推断打印件强度。

## 查看

从项目 `code/tron2_revo3_description` 运行：

```bash
# 关闭旧窗口后重新启动，入口现指向 V3
bash view_supplied_camera_rig.sh

# 只显示左右相机支架
bash view_supplied_camera_rig.sh --brackets-only

# 三路 RGB/深度
bash view_cameras.sh --config config/camera_rig_supplied_step_aligned_v3.json
```

旧 V2 可通过 `bash view_supplied_camera_rig.sh --config config/camera_rig_supplied_step_inward_v2.json` 显式打开。旧默认 `config/camera_rig.json` 未覆盖。

新 URDF：`urdf/tron2_dach_revo3_supplied_step_aligned_v3.urdf`。

报告目录：`reports/cameras/supplied_step_aligned_v3/`。

- `left_front_alignment.png` / `right_front_alignment.png`：正面平行关系；
- `left_third_screw.png` / `right_third_screw.png`：第三孔开放情况；
- `hole_alignment.png`：实际网格截面，小孔红色、第三固定孔绿色；
- `validation.json`：孔位、姿态、相机与截面检查数据；
- `*_wrist_rgb.png`：实际相机渲染，不能据此声称所有抓取姿态都能看到目标。

## 复现

```bash
.cad-venv/bin/python scripts/design_aligned_camera_brackets_v3.py
env -u PYTHONPATH .venv/bin/python scripts/prepare_aligned_bracket_rig_v3.py
env -u PYTHONPATH .venv/bin/python scripts/check_inward_bracket_rig.py \
  --config config/camera_rig_supplied_step_aligned_v3.json \
  --report-directory reports/cameras/supplied_step_aligned_v3
```

CAD 生成器提供 `--radial-lift-mm`，默认 10；工具包络不通过时会拒绝交付该参数。当前界面仍按固定 STEP 展示，不提供会使相机脱离孔位的假滑条。
