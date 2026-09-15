# 最新侧装支架：板与支撑杆夹角15°

本版修正角度定义：用户参数15°表示绿色相机板与蓝色支撑杆之间的夹角为15°。内部CAD以横向+Y为参考，所以实际旋转是Rx(75°)=Rx(90°−15°)；不要把内部75°再当作界面输入。

## 保持的结构

- 两侧均安装到拇指侧小孔330°/30°，两孔夹角60°；新座局部孔60°/120°，孔心距30.5mm。
- 相邻大固定孔15°被实体覆盖，不作为支架安装孔，不增加大避让口。
- 实心24×12mm支撑杆，默认净长80mm，顶部基准[0,0,115.8]mm。
- 相机板背面两M3安装孔间距20mm；相机为独立模型，不包含在支架STEP/STL中。
- 没有恢复橙色横臂。

## 文件与查看

- `whole_bracket.step`：当前完整一体STEP，可交结构同学。
- `whole_bracket_mm.stl`：毫米制网格。
- `whole_bracket_m.stl`：仿真米制网格，不要按毫米打印。
- `parts/`：同一整体的三个功能分区。
- `bracket_overview.png`：实际几何的板—杆15°预览。
- `manifest.json` 同时记录用户夹角15°和内部平面旋转75°。

关闭旧窗口后，在项目根目录重新运行：

```bash
bash view_camera_bracket.sh
bash view_camera_robot.sh --config config/camera_rig.json
bash view_cameras.sh --config config/camera_rig.json
```

显式选择默认配置可避免加载以前保存的旧角度定义。当前用户夹角范围15°–90°，对应已支持的内部平面角75°–0°。修改夹角只重建绿色板及其短根部过渡；杆和安装座不转动。

重新生成默认交付：

```bash
.cad-venv/bin/python scripts/design_wrist_camera_bracket_v4.py --length-mm 80 --plate-tilt-deg 15
```

这是一体原型设计；原机螺纹、螺钉长度、材料/打印工艺及承载能力仍需实测。窄侧座不能直接沿用旧宽座的连接强度结论。旧版文件保留，不冒充原始STP复原。

## 验证

独立实体量角为15.00000268°，URDF两侧约15°；孔轴、小孔与大孔区分、相机后背配合已检查，见 `assembly_review.md`。三路ROS图像发布也通过，报告为 `../../reports/cameras/ros_topic_validation_side_bend_v4.json`。

当前示例姿态下，D405配置量程为0.07–0.5m，左右腕有效深度像素约15.4%/14.1%；其余多为超量程无效值。角度正确和可发布图像不等于当前姿态视野最佳，可在图像窗口继续结合抓取位置评估长度与夹角。
