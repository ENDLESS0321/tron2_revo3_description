# V3：拇指侧两小孔安装座

默认竖杆 **80 mm**、安装板 **15°**。相机独立，实心24×12 mm竖杆、板孔和背面安装变换保留V2定义；V1/V2文件未修改。

V3实际重做了安装座：局部小孔改为 **60°/120°，夹角60°**，R30.5处孔中心距30.5 mm；圆弧座缩至52°..128°，内/外R仍为30.8/35.8 mm。它不是把V2的120°孔距座简单旋转。

装配时相对原adapter frame使用 **Ry(+90°)**，两孔对应所选拇指侧的 **330°/30°小孔**；二者之间15°的大连接件孔由座实体覆盖，不开避让孔。两只手使用相同的这一步根坐标旋转，具体URDF位姿由总装配置给出。

## 文件

- `whole_bracket.step`：标准毫米STEP，单一有效实体。
- `whole_bracket_mm.stl` / `whole_bracket_m.stl`：毫米/米网格；制造用毫米版，仿真用米版。
- `parts/base.*`、`stem.*`、`plate.*`：同一整体的三个语义分区，不是无紧固件可拼接的独立打印零件。
- `manifest.json`：尺寸、孔轴、frame、估计质量/惯量、文件指纹。
- `interface_validation.json`：从实际STEP读取两组径向圆柱面；两小孔轴线空腔与15°大孔对应区域实体覆盖检查。
- `cad_preview.png`：局部CAD坐标下的预览；实际安装还须施加Ry(+90°)。
- `options/L080_T040/`：80 mm / 40°旧板角对照；默认仍是15°。

## 生成

```sh
.cad-venv/bin/python scripts/design_wrist_camera_bracket_v3.py \
  --length-mm 80 --plate-tilt-deg 15 \
  --output-directory design/wrist_camera_bracket_v3
```

长度从局部Z=35.8沿+Z计算，plate pivot为`[0,0,35.8+L] mm`，板绕局部X倾转。局部板孔为`(±10,32,0) mm`，相机背面接触面Z=-2 mm；D405 bottom-screw frame相对plate frame的平移为`[0,53,-10.35] mm`，旋转矩阵为`[[0,1,0],[0,0,-1],[-1,0,0]]`。

腕孔仍为3.4 mm通孔、6 mm支承座；原机螺纹和螺钉长度未被认证。默认估计质量41.57 g基于1250 kg/m³假设，不代表材料已选定或实测质量；CAD封闭验证不等于实机适配或整体强度认证。
