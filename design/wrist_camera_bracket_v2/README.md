# 腕部相机支架 V2

V2按本次修改要求，去掉基座的两处9.5 mm避让孔，保留两个3.4 mm腕安装孔与6 mm支承座；整个沿+Y的横臂已删除。实心竖杆从基座顶面 **Z=35.8 mm沿+Z直接延长**，相机板以短颈和两侧实体过渡直接连接其顶端。V1原件和文件均保留。

默认参数为**竖杆80 mm、板角40°**，对应pivot `[0,0,115.8] mm`。这里的80 mm是新的径向竖杆长度，不是V1的水平臂长。

| 部分 | 规格 |
| --- | --- |
| 腕座 | 内R30.8、外R35.8 mm；两个孔θ30°/150°、Y=-4.5 mm，3.4 mm通孔与6 mm平底座；无大避让孔 |
| 竖杆 | 实心24×12 mm矩形，X=-12..12、Y=-8.5..3.5；纵向边圆角0.8 mm；长度仅沿Z改变 |
| 相机板 | 主板40×42×4 mm，两孔间距20 mm、孔径3.4 mm；板局部孔中心 `(±10,32,0)` |
| 板根过渡 | 短颈、两侧2.6 mm厚加强过渡，与竖杆顶部18 mm范围真实连接；没有新的长横向支撑臂 |

## 文件

在项目根目录执行 `bash view_camera_bracket.sh`，默认只看V2支架；`bash view_camera_robot.sh` 看整机装配。图像与参数调整用 `bash view_cameras.sh --config config/camera_rig.json`。请关闭旧窗口后再启动，新版本不会改变已打开窗口里的旧模型。旧V1仍可用 `bash view_camera_bracket.sh --design parametric_photo_reference_v1` 查看。

- `whole_bracket.step`：毫米制完整一体STEP。
- `whole_bracket_mm.stl`：毫米制完整一体STL，用于切片准备。
- `whole_bracket_m.stl`：同网格换算成米，用于仿真，不要当毫米模型打印。
- `parts/base.*`、`stem.*`、`plate.*`：同一整体的三语义分区，含STEP、根坐标毫米网格和body-local米制网格。它们不是三个没有紧固件便可独立装配的打印零件。
- `manifest.json`：实体尺寸、孔轴、估计质量/惯量、frame及文件指纹。
- `cad_preview.png`：蓝色竖杆/基座和绿色板的实际CAD预览。
- `bracket_overview.png`：不含相机的三维预览，蓝色连续竖段直接连接绿色板。
- `options/L080_T060/`：80 mm竖杆、60°板角对照版本。
- `checks/` 与 `parameter_validation.json`：边界和参数分离的导出实体检查。

相机本体和螺钉未混入支架STEP/STL，应独立装配。

## 参数化

在项目根目录执行：

```sh
.cad-venv/bin/python scripts/design_wrist_camera_bracket_v2.py \
  --length-mm 80 --plate-tilt-deg 40 \
  --output-directory design/wrist_camera_bracket_v2
```

支持长度20–150 mm、板角0–75°。长度改变竖杆Z跨度并上移板pivot；角度只重建相机板、短颈和相连加强段，基座与竖杆不转。修改参数须重新生成实体，不是把已有网格拉长或仅旋转悬空板。

`plate_frame.xyz_mm=[0,0,35.8+length_mm]`，姿态为Rx(板角)。板接触相机背面的平面为局部Z=-2 mm。D405的bottom-screw frame相对该plate frame：

```text
translation = [0, 53, -10.35] mm
rotation = [[0,1,0], [0,0,-1], [-1,0,0]]
```

相机光轴朝plate -Z，两背面安装点对应支架 `(±10,32,-2)` mm。

## 验证与边界

每次生成都会回读STEP检查单一有效实体，检查毫米/米制STL封闭及语义分区体积。已生成默认80/40、80/60、100/40及20/0、150/75边界样例；导出STEP的实体不变性另由 `check_parameter_invariants.py` 检查。

默认总质量约43.59 g，基于显式密度1250 kg/m³；这不是材料选定或称重结果。腕座对R30.5外圆预留0.3 mm径向设计间隙，实物贴合、原机螺纹、螺钉长度和相机背面啮合深度仍需核对。本次没有据CAD闭合性宣称材料、根部连接或整体强度已获认证。
