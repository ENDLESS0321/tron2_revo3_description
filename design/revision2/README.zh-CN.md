# 转接件第 2 版：腕侧对孔、左右手背走线

本版修改实际三维实体，保留 TRON2 腕关节和 Revo3 手掌的安装姿态。原始第 1 版 3MF、v1 装配模型与其生成网格均保留。

## 打印文件

| 安装位置 | 3MF（建议优先使用） | STL（毫米，已摆放打印姿态） |
| --- | --- | --- |
| 机器人左手 | [adapter_left_v2.3mf](adapter_left_v2.3mf) | [adapter_left_v2_print_mm.stl](adapter_left_v2_print_mm.stl) |
| 机器人右手 | [adapter_right_v2.3mf](adapter_right_v2.3mf) | [adapter_right_v2_print_mm.stl](adapter_right_v2_print_mm.stl) |

左右指机器人自身的左/右手，和你面对机器人时的左右相反。

两份 3MF 均为标准单实体文件，单位为毫米；不是带打印机、材料和支撑参数的完整 Bambu 切片工程。已保留并烘焙原工程的 **1.01 缩放**，导入后用 **100%**，不要再乘一次 1.01。打印姿态包围尺寸约 **82.82 × 56.40854 × 46.213 mm**。

`*_assembly_mm.stl` 为装配坐标下的毫米网格；`*_m.stl` 为仿真使用的米制网格。打印请选上表文件，避免把米制文件当毫米导入。

[左右开槽对比图](adapter_pair.png) 使用相同相机角度比较两件；[腕端 V1/V2 叠图](analysis/v1_v2_wrist_sections.png) 中灰色为原件、橙色为新件、蓝色为官方腕端。

## 只看打印件

在桌面终端运行：

```bash
cd /home/endless/Documents/Project/Dexterous_Hand/code/tron2_revo3_description
bash view_parts.sh
```

默认只显示两个 v2 打印实体，无机械臂、灵巧手或碰撞代理。左件为橙色，右件为蓝色。

- `1`：只看左件，自动居中；`2`：只看右件；`3`：两件并排。
- `F` / `B`：看两侧走线槽；`T`：手侧俯视；`U`：腕侧仰视；`R`：复位视角。
- 鼠标左键拖动旋转，右键拖动平移，滚轮缩放；关闭窗口退出。

直接从单件开始可用 `bash view_parts.sh --side left` 或 `--side right`。
无桌面窗口的检查命令为 `bash view_parts.sh --check`，静态图片输出到 `reports/revision2/parts_viewer/`。
这些操作不改打印文件或装配模型。需要查看整个 v2 机器人时，运行 `bash view_robot.sh`。

## 腕侧怎样修改

从左右官方腕 STL 分别提取接口，确认应匹配的四个较大径向孔位于：

`θ = 15°、105°、195°、285°，孔轴高度 Y = -4.500 mm`

角度定义为 `atan2(adapter Z, adapter X)`，图中 X 向右、Z 向上；增加 θ 对应标准右手系绕 Y 的负角旋转。旁边另有一组 2.5 mm 小孔，未将它们当作转接件的目标孔。

原插入段由四瓣组成。若只在原瓣上偏转钻孔，新的孔位会落在较薄的区域，螺母座也不能正常对应。因此本版只将肩台以下的四个插入瓣连同圆孔、六角螺母腔转位 **15°**，上方非圆肩台、套筒和整手安装方向不转动。

原工程有 1.01 比例，原孔高会产生约 0.045 mm 的轴向偏差。为了保留圆孔和螺母腔的原形，本版把插入四瓣整体向肩台方向抬高 **0.045 mm**，微量嵌入原实心肩台。孔高随之对齐，外露插入深度由约 10.403 mm 缩为 **10.358 mm**。没有通过拉伸孔、改变手模型或调整腕关节来对孔。

源圆孔名义直径 4.2 mm、六角螺母腔对边 6.9 mm 保留；烘焙比例后分别为约 **4.242 mm、6.969 mm**。官方腕端大孔直筒直径约 4.5 mm。

## 防呆口单独检查

实机照片所示的防呆形态，与官方腕内孔约 **30°** 的凸台相符；没有从倾斜照片反推毫米尺寸。该凸台和四个大孔不是同一特征。

已对实际新 STL 和左右官方腕模型分别检查 13 个非空截面，键区约为 θ=22.766°–37.234°。这些截面上键区实体相交面积均为零，最小采样径向间隙约 **1.58 mm**。这支持当前四瓣转位不会顶住该凸台；没有将整件镜像，或把防呆方位跟走线槽一起翻转。

这里验证的是该凸台的几何让位，不是唯一插装方向的机械防错认证，也不是完整三维连续净空证明。完整孔/键证据在 [实际 V2 配合验收](analysis/v2_fit_validation.md)。

## 左右走线槽

沿用当前装配约定，Revo3 的 `hand +X` 为掌心，`hand -X` 为手背：

- 左件：手背对应 adapter **+Z**，保留原侧的 15 mm 名义走线槽。
- 右件：手背对应 adapter **−Z**。先补回原件自带的 +Z 侧 10 mm 旧槽，再在 −Z 开 15 mm 名义槽。

烘焙比例后的槽宽为 **15.15 mm**。两个零件均只有手背一侧开槽，掌心一侧为实体。没有旋转整个上部，也没有改变手侧安装孔、肩台和腕侧孔组的左右对应关系。

## 已完成的检查

- 每件 STL/3MF 均为一个连通、封闭、法向一致的正实体；保留原有拓扑孔数。
- 独立解析 3MF ZIP/XML，确认毫米单位、单 object、单 build item、无重复缩放。
- STL 转换至打印姿态时有 2 个面因输出浮点精度坍缩为零面积；仅去掉这两个零面积面后，重载保持封闭，几何对比最大表面差约 0.0000045 mm。
- 每侧 60 个槽内采样点均为空，60 个掌心侧对应点均为实体；槽边和下缘采样保持实体。
- 手侧上部排除两处槽窗口后，和原件的双向差约 0.00378 mm³，属网格离散量级。
- 四孔高度拟合最大误差约 0.000061 mm；实测孔径约 4.2416 mm；螺母对边约 6.969 mm。
- 源 CAD 孔本身约 0.1035°、0.0501 mm 的微小偏轴被保留并记录，没有声称加工级零误差。
- 原始 3MF SHA-256 仍为 `b933846237df86a8982ca9a4c61340965368c19dab714b5a3358b8cb2e903b4b`。

独立报告为 [制造文件验收](manufacturing_validation.json) 与 [孔/防呆配合验收](analysis/v2_fit_validation.json)。`design_manifest.json` 是生产阶段清单；上述独立报告才是后续验收结果。

## 仍需实物确认

本次保留原工程 1.01 打印补偿，腕接口整圈仍有原量级的局部 CAD 干涉；键区无相交不能解释为整个接口零干涉。不同材料、收缩和实际尺寸会影响插入力，建议先做试装，再确认最终打印补偿。此版未重新设计手侧那组约 0.57 mm 的轴向孔位差，也未验证螺钉长度、锁紧力、打印强度和线缆弯曲空间。

## v2 仿真装配

配套文件独立于 v1：

- [v2 组合 URDF](../../urdf/tron2_dach_revo3_v2.urdf)
- [v2 安装配置](../../config/assembly_v2.json)
- [v2 装配预览](../../reports/revision2/assembly_overview.png)

在项目根目录打开：

```bash
env -u PYTHONPATH .venv/bin/python scripts/preview_assembly.py \
  --urdf urdf/tron2_dach_revo3_v2.urdf \
  --scene simulation/revision2/scene.xml \
  --report-dir reports/revision2 --viewer --skip-render
```

查看器仍是运动学模式，不会连接真机或启用抓取控制。

## 再生成

在项目根目录使用：

```bash
env -u PYTHONPATH .venv/bin/python scripts/redesign_adapters.py
env -u PYTHONPATH .venv/bin/python design/revision2/check_manufacturing.py
env -u PYTHONPATH .venv/bin/python design/revision2/analysis/check_v2_fit.py
env -u PYTHONPATH .venv/bin/python scripts/render_adapter_revision2.py
env -u PYTHONPATH .venv/bin/python scripts/build_revision2_urdf.py
```

再生成会更新本版派生产物，原始第 1 版 3MF 不会被修改。生成器的输入、源模型与输出哈希保存在各 JSON 清单中。本版是网格修改成果，没有伪装为恢复了原始 STEP 参数化设计。
