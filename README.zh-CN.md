# DACH_TRON2A + 打印转接件 + 双 Revo3

第一版目标是验证装配与运动学。已获得两家的官方模型，将用户的 3MF 转接件接到 TRON2 双腕，再安装对应的左、右 Revo3。未安装交接包里的 skill，未连接硬件。

## 快捷查看当前 v2

在本项目目录运行 `bash view_parts.sh`，只显示左右打印件：`1` 左件、`2` 右件、`3` 并排；鼠标左拖旋转、右拖平移、滚轮缩放。`F/B` 切两侧、`T` 俯视、`U` 看腕端、`R` 复位。启动时也可加 `--side left` 或 `--side right`。

运行 `bash view_robot.sh` 查看完整 v2 机器人。两种模式相互独立，均不连接硬件。打印件模式直接读取最终毫米制打印 STL；详细说明见 [第 2 版说明](design/revision2/README.zh-CN.md)。

相机支架当前默认V4：安装在侧面330°/30°两小孔，覆盖中间的大固定孔；绿色板与蓝色杆的实际夹角为15°，界面也输入15°。`bash view_camera_bracket.sh` 单独查看，`bash view_camera_robot.sh --config config/camera_rig.json` 看整机，`bash view_cameras.sh --config config/camera_rig.json` 看RGB-D并重建参数。请关闭旧窗口后重开。见 [最新STEP与角度定义](design/wrist_camera_bracket_v4/README.md)，旧版保留。

## 先看结果

- [独立 URDF](urdf/tron2_dach_revo3.urdf)：相对 mesh 路径，固定世界底座；使用时保留同项目 `meshes/` 目录。
- [ROS 包路径版 URDF](urdf/tron2_dach_revo3.ros.urdf)：使用 `package://tron2_revo3_description/...`，供安装此 ROS 2 description 包后使用。
- [MuJoCo 场景](simulation/scene.xml)：由同一 URDF 导出，保留模型物理参数，未添加执行器。
- [整体装配图](reports/assembly_overview.png)、[左腕特写](reports/mount_left.png)、[右腕特写](reports/mount_right.png)。橙色为用户打印件。
- [安装参数](config/assembly.json)：左右腕的旋转和平移分别可调，无须改上游 URDF。
- [关节分组](config/joint_map.json)：按名称列出 7+7 臂轴、2 头轴、21+21 手轴；不是硬件 SDK 电机索引映射。

生成模型含 82 个 link、81 个 joint，其中 58 个 revolute 和 23 个 fixed。保留 DACH 的头部、相机坐标、立柱和底座；移除两只原始夹爪；保留两只 Revo3 的全部主动关节、原点、轴和限位。

## 在本机打开

专用 `.venv` 已建立，未更改系统 Python 包。下面的 `env -u PYTHONPATH` 用来避免终端预先加载的 ROS Python 路径干扰仿真环境。

```bash
cd /home/endless/Documents/Project/Dexterous_Hand/code/tron2_revo3_description
env -u PYTHONPATH .venv/bin/python scripts/preview_assembly.py --viewer --skip-render
```

查看器使用正运动学保持姿态，不执行无驱动器的重力自由下落。`0` 显示零位，`1` 显示适度屈肘，`C` 切换碰撞代理显示，`F` 切换轻微手指往复；可在 MuJoCo 关节面板调整姿态。关闭窗口退出。直接让无执行器的 `scene.xml` 做动力学积分不会自动保持站立/抓取姿态。

重新校验并输出静态图片：

```bash
env -u PYTHONPATH .venv/bin/python scripts/validate_description.py
env -u PYTHONPATH .venv/bin/python scripts/preview_assembly.py
```

默认离屏渲染为 EGL；当前机器已实际渲染成功。其他机器可根据图形环境选择脚本的 `--backend`。

## 装配基准与已知差异

URDF 位置单位为米、角度为弧度。`xyz/rpy` 是子坐标系在父坐标系中的位姿。

```text
world → base_Link
        ├─ 左臂 7 轴 → wrist_roll_L_Link → left_adapter_link → left_hand_base_link → 左手 21 轴
        ├─ 右臂 7 轴 → wrist_roll_R_Link → right_adapter_link → right_hand_base_link → 右手 21 轴
        └─ 头部 2 轴、立柱、底座、相机坐标
```

两侧腕到打印件均先采用 `xyz=(-0.0317,0,-0.0812)`、`rpy=(-π/2,0,0)`。位置与官方原夹爪参考原点、腕 STL 的端面一致；打印件选名义 CAD `(0,-12.6,0)` 肩台中心作为原点，保留 3MF 的 `1.01` 打印缩放。打印平台的旋转和平移已去掉，零件相对装配变换保留。

Revo3 根部前 12 mm 为插入段；让 `hand Z=12 mm` 肩台落到套筒口沿，得到打印件到 hand base 的平移 `(0,0.023855,0)`。两手方向分别设为掌心相向、拇指朝机器人前方。转接件几何沿用同一份，没有凭空镜像成另一种制造件。

3MF 中有两个正实体和一个 `negative_part`。生产转换执行实体并集再扣除负体，正确保留减料槽。生成 STL 封闭、法向一致、重载体积匹配。完整过程见 [转换报告](meshes/adapter/adapter_report.json) 和 [几何分析](inspection/README.md)。

当前仍需保留的机械差异：

- 手侧两组螺孔对当前官方 hand CAD 的轴向偏差约 0.06 mm、0.57 mm；不是已验证可直接锁紧的机械配合。
- 机器人侧孔位存在绕轴角度差；1.01 放大后，六个截面中局部径向干涉最大约 0.29 mm，相交面积约占夹持截面的 0.1–1.5%。这可能涉及打印补偿与实物收缩，不能直接以原始 CAD 判断实际装配力。此版未修改打印件设计。
- 安装旋转、真实插入量、公差、螺钉和线缆空间仍需与实物对应。
- 打印件质量按实心 PLA、密度 1240 kg/m³ 计算，每只约 0.10348 kg。实际填充率、材料、螺钉未知，不能当作实测惯性。

可检查 [手掌配合截面](inspection/mount_fit_sections.png)、[右腕配合截面](inspection/right_wrist_fit_sections.png)、[左腕配合截面](inspection/left_wrist_fit_sections.png)。

## 验证范围

[URDF 检查](reports/urdf_validation.json) 检查单树、link/joint 唯一性、上游主动关节一致性、全部 mesh 文件、质量/惯性以及 `check_urdf` 解析。

[MuJoCo 检查](reports/mujoco_validation.json) 检查 58 自由度加载、每个关节的独立有界运动、全部 link 正运动学、腕/转接件/手的固定关系、质量及惯性保留，并记录图片与源文件哈希。

这是装配和运动学通过，尚不是动力学抓取环境：

- 当前零位和展示姿态均有 8 个负距离接触，归并为 `base_Link` 与左右 `proximal_pitch_*_Link` 两对，最深约 15.55 mm。这些源模型肩部/机身代理接触未被屏蔽，后续需按机械结构核对碰撞过滤，不能直接据此运行稳定接触控制。
- TRON2 部分碰撞代理只覆盖局部外形，头部底座和相机等也未提供完整碰撞覆盖。
- 打印件使用 48 块 CoACD 凸代理；分块达到数量上限，未保证达到所请求的凹度阈值。五个套筒中轴采样点未被代理封堵，但没有证明整个套筒或小螺孔的连续净空。
- 已去掉固定 palm/tip 标记上的零质量或不满足惯量三角不等式的占位惯性，保留这些坐标系；没有改动真实活动 link 的惯性。
- 生成的 Revo3 网格文件名含左右侧及几何类别前缀，避免 MuJoCo 把同名 `base_link.STL`、`part_000.obj` 等复用到错误手掌。网格字节未变，额外逐 body 校验导入后的网格 SHA 对应关系。
- BrainCo 当前参数网页与锁定 URDF 的部分关节限位不同。此版保留锁定模型的定义，后续按真实设备版本核对。
- 尚未配置执行器、控制增益、接触过滤、摩擦、桌面抓取任务、触觉或动捕手套。

## 再生成与后续修改

完整再生成命令：

```bash
env -u PYTHONPATH .venv/bin/python scripts/convert_adapter.py
env -u PYTHONPATH .venv/bin/python scripts/build_adapter_collision.py
env -u PYTHONPATH .venv/bin/python scripts/build_description.py
env -u PYTHONPATH .venv/bin/python scripts/validate_description.py
env -u PYTHONPATH .venv/bin/python scripts/preview_assembly.py
```

这些命令会更新本项目的生成文件。原始 3MF 和 `vendor/` 上游来源不被修改。

只改装配角度/平移时，修改 `config/assembly.json` 后从 `build_description.py` 开始即可。如果改打印缩放、原点或密度，先给转换器传对应参数，使转换报告和配置一致，然后重建碰撞代理。

在另一台机器建立环境，可用 `python3 -m venv .venv`，再执行 `.venv/bin/python -m pip install -r requirements.txt`。模型查看需要 `urdf/`、`meshes/` 和 `simulation/`；完整再生成还需要 `vendor/`、脚本、配置及原始 3MF（可用转换器的 `--source` 指定）。

建议下一阶段按顺序推进：先确认左右手实际安装方向与腕角，再核对碰撞代理/允许接触对，然后配置 58 轴仿真执行器和基础位置保持，最后添加桌面、简单物体、单手抓取和双臂协作。动捕手套放在关节名称与控制接口稳定之后对接。

## 官方来源

- [LimX TRON2 description](https://github.com/limxdynamics/tron2-robot-description)，commit `9939c22e69d27653ec0ba8a505859a2903dd1a71`，使用 `tron2a/DACH_TRON2A/urdf/robot.urdf`。
- [BrainCo description](https://github.com/BrainCoTech/brainco-description)，commit `f332a6f0dc944e26b82976b637074b03f7ee8a2c`，使用 `revo3_system/urdf/revo3_left.urdf` 与 `revo3_right.urdf`。
- [BrainCo Revo3 参数](https://www.brainco-hz.com/docs/revolimb-hand/revo3/parameters.html)、[官方下载中心](https://www.brainco-hz.com/docs/revolimb-hand/revo3/download.html)。

来源文件、哈希与许可声明保留在 `vendor/` 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。BrainCo 资产仓库当前没有明确 LICENSE 文件，此项目未将其重新声明为 Apache-2.0，也未对外发布资产。
