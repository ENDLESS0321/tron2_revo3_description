# TRON2 相机型号、官方模型与 RGBD 坐标研究

检查日期：2026-09-11。当前选择是腕部 D405、头部 D435i 的仿真配置；这不等于已经确认当前这台 TRON2 的实物相机型号。本次没有连接相机或机器人，也没有操作用户现有 viewer 进程。

## 型号证据与 LimX 模型版本差异

[LimX 官方 TRON2 SDK 文档](https://www.limxdynamics.com/en/documents/856848486581276672)的 §4.2 明确把 D435i 归在 **Waist Camera（腰部相机）**。§4.1 的头部/腕部说明另指向[官方链接的飞书指南](https://cwjgfm21di.feishu.cn/docx/NZ68dQnIHoANwOxZXzQczQSQnIg)，本次公开访问被重定向到登录页。因此，不能把“TRON2 有 D435i”扩写为“TRON2 头部一定是 D435i”，也没有公开资料足以确认腕部一定是 D405。

当前固定的 LimX description commit 是 `9939c22e69d27653ec0ba8a505859a2903dd1a71`。同一版本的不同描述文件存在实际差异：

| 来源与参考帧 | 父 link | xyz，米 | rpy，弧度 |
|---|---|---|---|
| `urdf/robot.urdf` 的 `d435_Link` | `base_Link` | `(0.0968,0.01759,-0.00409)` | `(0,2.7227,0)` |
| `xacro/robot.urdf.xacro` 的 `d435_Link` | `head_pitch_Link` | `(0.058,-0.0308,0.0408)` | `(0,0,0)` |
| `urdf/robot_grasper.urdf` 的 `left_camera` | `wrist_roll_L_Link` | `(0.0664,-0.0012,-0.0979)` | `(1.781790,1.451946,-2.929144)` |
| 同文件的 `right_camera` | `wrist_roll_R_Link` | 同左腕 | 同左腕 |

上述文件均在 `vendor/tron2-robot-description/tron2a/DACH_TRON2A/`。头相机位置有[官方 xacro 源码依据](https://github.com/limxdynamics/tron2-robot-description/blob/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A/xacro/robot.urdf.xacro)；腕参考来自[官方夹爪版 URDF](https://github.com/limxdynamics/tron2-robot-description/blob/9939c22e69d27653ec0ba8a505859a2903dd1a71/tron2a/DACH_TRON2A/urdf/robot_grasper.urdf)。这些相机 link 只有占位质量/小方块或完全没有外观，没有给出具体 RealSense 型号，也没有说明该原点是镜头、光学原点还是底部螺孔。

因此，集成时保留现有胸腰 `d435_Link`，另建随 `head_pitch_Link` 转动的头相机；可以用官方 xacro 的位置作为有来源的参考，但相机安装基准的解释仍应标为仿真假设。腕参考姿态若解释为 ROS 相机本体坐标，其 +X 在 wrist 坐标中约为 `(-0.1159,-0.0250,-0.9929)`，朝向腕端；这只是坐标计算，不能反过来证明它就是某一设备的校准相机帧。

头部两轴的实际链保持不变：`base_Link -> head_base_Link` 位移 `(0.04656,0.00009,0.26115)`；到 `head_yaw_Link` 再加 `(0,0,0.008)`；到 `head_pitch_Link` 再加 `(0.051,0.03,0.097)`。现有 generated URDF 的胸腰相机不会随这两轴运动。

## 上游有没有直接可用的 RGBD 传感器

- 当前使用的 `robot.urdf` 无 Gazebo 相机插件，也没有可直接运行的 RGBD 传感器。
- DACH xacro 添加了 `depth_camera_link` 并调用 `macros.xacro` 中的 Gazebo 宏。ROS1 分支有 RGB camera 与 depth camera 插件，通用参数为 640×480、30 Hz、水平 FOV≈60°、近远裁剪 0.1–10 m；ROS2 分支也有相机配置。这些参数不是 D405/D435i 实机标定。
- 官方 `xml/robot.xml` 只有外部跟随视角 `track` 相机；`d435_Link` 和 optical body 是参考坐标，没有绑定真实头/腕 RGBD camera。其 `<sensor>` 主要是 IMU、关节位置/速度/驱动力，不会自动产生这三路 RGBD。

## RealSense 官方资产与可用派生文件

使用 [RealSense 官方 ROS 仓库](https://github.com/realsenseai/realsense-ros)，commit `9a11121700cb4780e273e34141f6402fe184321d`，description 包版本 4.58.4。仓库已迁移至 RealSenseAI 组织；保留了原始 `LICENSE`、`NOTICE.md` 和文件 SHA。

| 型号 | 官方来源 | 本地直接可用的米制外观 |
|---|---|---|
| D405 | `_d405.urdf.xacro`、毫米 `d405.stl` | `vendor/realsense-description/generated/realsense_d405_native_axes_m.stl` |
| D435i | `_d435i.urdf.xacro` 继承 `_d435.urdf.xacro` 和 IMU 模块；复用米制 `d435.dae` | `vendor/realsense-description/generated/realsense_d435i_native_axes_m.stl` |

D405 仅做毫米到米的 0.001 缩放，3892 面。D435 DAE 的唯一 scene node 没有附加变换，18 个 triangle geometry 实例按原顶点坐标汇总为 231186 面 STL；原始 DAE 仍保留，STL 不携带材质。两种派生网格保留原生 mesh 轴，不能省略下表的 visual origin。

同目录 `d405_nominal.urdf`、`d435i_nominal.urdf` 是静态 nominal 展开，等效于 `use_nominal_extrinsics=true`、不添加 USB plug。没有安装 xacro、SDK 或驱动。再生成方法是运行该 vendor 目录内的 `derive_nominal_assets.py`；派生输入输出哈希见 `generated/manifest.json`。

## 相机内部 nominal TF

以下 transform 均表示 child 坐标到 parent 坐标的变换，单位米。数值直接来自已固定的[官方 D405 xacro](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d405.urdf.xacro)、[D435 xacro](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d435.urdf.xacro)及其 D435i IMU 扩展。上游明确说明这些是近似 nominal 外参，实际驱动会发布每台设备的校准结果。

| 变换 | D405 xyz | D435i xyz |
|---|---|---|
| bottom_screw_frame → camera_link | `(0.01085,0.009,0.021)` | `(0.0106,0.0175,0.0125)` |
| camera_link → depth_frame | `(0,0,0)` | `(0,0,0)` |
| camera_link → infra1_frame | `(0,0,0)` | `(0,0,0)` |
| camera_link → infra2_frame | `(0,-0.018,0)` | `(0,-0.050,0)` |
| camera_link → color_frame | `(0,0,0)` | `(0,0.015,0)` |
| camera_link → accel / gyro_frame | 无 | `(-0.01174,-0.00552,0.0051)` |
| camera_link → visual mesh | `(0.0038,-0.009,0)` | `(0.0043,-0.0175,0)` |

除 visual 外，上表各帧旋转为零。两个 visual 均使用 `rpy=(π/2,0,π/2)`。每个传感器 frame 到相应 optical frame 的旋转均为 `rpy=(-π/2,0,-π/2)`：ROS 本体轴是 X前/Y左/Z上，optical 是 X右/Y下/Z前。MuJoCo camera 使用 -Z前/+Y上，因此从 optical frame 到渲染 camera frame 额外施加 `Rx(π)`。

如果把 LimX 的参考位置解释为 camera_link，而不是 bottom_screw_frame，就不应再把 bottom_screw_to_camera_link 平移重复加一次。该解释必须在新增装配配置中明确。

## 外形、重量与安装孔

已按 PDF 阅读流程渲染并查看官方 [2025-10 D400 数据手册](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf)的尺寸表和工程图。表 3-52/3-54 位于第 69 页；D435/D435i 图 10-9 位于第 140 页；D405 图 10-13 位于第 146 页。

| 项目 | D405 | D435i |
|---|---|---|
| 名义宽×高×深 | 42×42×23 mm | 90×25×25 mm；xacro/工程图深度约 25.05 mm |
| 数据手册名义重量 | 58 g，±10%；现产品页取整为 60 g | 75 g，±10% |
| 底部安装 | 1/4-20 UNC；D405 最大插入 5 mm | 1/4-20 UNC |
| 背面安装 | 两个 M3×0.5，间距 20 mm，最大插入 4 mm | 两个 M3，间距 45 mm，最大插入 3 mm |

D405 的 USB 侧还有两个 M2×0.4 固定孔，间距 18 mm、最大插入 3 mm，应与背面 M3 孔区分。用官方 bottom_screw_frame 和名义外尺寸换算，背面 M3 中心可表示为 D405 `(-8.35,±10,21)` mm，D435i `(-10.15,±22.5,12.5)` mm；孔由后向内沿本体 +X。这里是工程图与官方基准共同导出的名义坐标，不是当前打印支架的测量值。

上游两款 xacro 都沿用 72 g 及同一套惯量，并明确把它们标为不可靠。因此派生 nominal URDF 保留这些值只为忠实记录源模型，集成动力学时不应把它们当作已校准相机惯量。若使用数据手册质量配合均匀盒体估算，也应单独标记为估算。

## FOV、内参与三路 RGBD 的含义

[D405 官方规格](https://www.realsenseai.com/products/stereo-depth-camera-d405/)给出深度及 RGB FOV 87°×58°、理想范围 7–50 cm；RGB 来自左侧深度成像器的 ISP，nominal RGB/depth 原点一致。它没有独立 RGB 相机、IR projector 或 IMU。[D435i 官方规格](https://www.realsenseai.com/products/depth-camera-d435i/)给出深度 FOV 87°×58°、RGB FOV 69°×42°、理想范围 0.3–3 m；深度是 global shutter，独立 RGB 为 rolling shutter。

`config/camera_model_catalog.json` 提供两款的模型、米制网格、visual origin、TF、color/depth FOV、建议裁剪范围及 640×480 的 nominal K。K 用针孔公式 `fx=W/(2*tan(hFOV/2))`、`fy=H/(2*tan(vFOV/2))`，主点取 `((W-1)/2,(H-1)/2)`；它不是出厂 CameraInfo。实际 K、畸变和外参随设备及分辨率模式变化，通常由 librealsense stream profile 或 ROS CameraInfo 给出，本次未读取硬件。

三路仿真 RGBD 可以输出同步 RGB 与以光学 Z 为单位的米制几何深度。D405 nominal 可以共用一个视点；D435i 原始 RGB 与 depth 存在约 15 mm 侧向位移且 FOV 不同。若从 color optical 视点渲染深度，应称为**与 RGB 对齐的理想几何深度**，不冒充原始立体深度输出。

理想深度不包含真实双目匹配失败、纹理不足、IR 照明、遮挡重投影空洞、畸变、rolling-shutter/曝光运动影响以及量化噪声。这样仍可直接用于验证三路视野、坐标、RGBD 接口和几何算法；与实机成像的差异保留在元数据中即可。
