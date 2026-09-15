# 三相机 RGB-D 与相机支架工作状态

已按用户更新的路径，参考照片新建三段式一体相机支架，并接入双腕、独立 D405 相机和三路 RGB-D。原 STP 的对比留待以后可读时补做，不再阻塞本版新设计。当前不是对原 STP 的复原，也不声称知道原两件的长短或左右关系。

当前默认是支架V4：安装到侧面330°/30°两小孔，覆盖相邻15°大固定孔；无橙色横臂。用户角度15°现在明确表示绿色板与蓝色杆的实际夹角，内部CAD旋转为75°。旧窗口需关闭后重开，并显式使用默认配置，避免旧保存配置中的角度定义。

## 当前完成到哪里

| 用户目标 | 当前证据与状态 |
| --- | --- |
| 腕相机支架安装到两个小孔 | 新设计实体已接入左右腕，腕座两条非平行孔轴与官方小孔对照；原机螺纹和实际装配公差仍待实测 |
| 判明两份 STP 是左右件还是长短件 | 按新指示延期；本版使用左右共用的新设计，通过安装姿态对应两侧孔位 |
| 改支撑杆长度、相机铁片倾角 | 实际重建 STEP/STL：改变蓝色竖段长度，或重建绿色安装片及根部过渡；相机独立，腕座不动 |
| 相机型号、模型和 TF | 已调研官方来源并提供 D405/D435i 官方几何及 nominal 光学 TF；TRON2 实装型号尚需确认 |
| 头部与左右腕均输出图像 | 已实际渲染三路 RGB + 米制深度；GUI参数变化会改变对应图像 |
| 图像发布验收与环境依赖 | 已通过 ROS 2 两进程订阅验收，以及独立已知平面/投影几何验收 |

## 直接看图，不需要 ROS

在桌面终端运行：

```bash
cd /home/endless/Documents/Project/Dexterous_Hand/code/tron2_revo3_description
bash view_cameras.sh --config config/camera_rig.json
```

窗口分三列显示头部、左腕、右腕的 RGB 和深度色图。深度黑色为无效或超出所配置量程；色图仅用于显示，保存的 `*_depth_m.npy` 是浮点米制数据。

左右各有两个独立参数：

- 长度：从固定腕座顶面Z=35.8 mm起，沿支架局部+Z改变实心竖段长度，带动顶端绿色板和相机，腕座不动。
- 板与杆夹角：围绕杆顶的局部X轴重建绿色板及短根部过渡，竖段与腕座不转动。用户夹角β对应内部Rx(90°−β)；15°就是板与杆15°，不是补角75°。

默认杆净长80mm、板—杆夹角15°，pivot仍为[0,0,115.8]mm。修改参数会生成CAD变体并重载场景，首次约需数秒。当前长度20–150mm、用户夹角15°–90°为几何范围，不是强度许可。最新文件见 [V4交付](../design/wrist_camera_bracket_v4/README.md)。下方旧V2截图与报告为历史对照，当前V4报告另存，不混用。

只看支架运行 `bash view_camera_bracket.sh`；看整机、支架及独立相机运行 `bash view_camera_robot.sh`。保存用户参数后可给后者加 `--config config/camera_rig_user.json`，新开窗口查看对应构型；已打开的独立窗口不会自动联动。

“保存相机参数”写入独立的 `config/camera_rig_user.json`，不改默认配置。下次加载保存值：

```bash
bash view_cameras.sh --config config/camera_rig_user.json
```

无桌面窗口时运行 `bash view_cameras.sh --check`，输出在 `outputs/cameras/viewer_check/`；长度与角度的对照图在相邻的 `viewer_length_check/`、`viewer_tilt_check/`。

[V2支架默认 RGB-D](../outputs/cameras/vertical_bracket_check/viewer_check/rgbd_montage.png) · [增加竖段长度](../outputs/cameras/vertical_bracket_check/viewer_length_check/rgbd_montage.png) · [改变绿色板倾角](../outputs/cameras/vertical_bracket_check/viewer_tilt_check/rgbd_montage.png)

## ROS 2 发布和验证

MuJoCo 负责渲染，ROS 2 负责把图像和 TF 传给其他节点。仅看本地窗口不需要 ROS；测试下游订阅才启动下面的桥接。

终端 A 先启动检查器：

```bash
cd /home/endless/Documents/Project/Dexterous_Hand/code/tron2_revo3_description
bash check_camera_topics.sh --timeout 60
```

终端 B 启动发布器：

```bash
cd /home/endless/Documents/Project/Dexterous_Hand/code/tron2_revo3_description
bash publish_cameras.sh
```

持续发布用 Ctrl+C 停止；有限测试可加 `--frames 90`。保存过 GUI 参数时，两个命令都加相同的 `--config config/camera_rig_user.json`。GUI和发布器是两个独立实例；GUI的临时滑块变化不会自动修改另一个正在运行的发布器。

默认使用隔离 `ROS_DOMAIN_ID=94` 和仅本机发现，不进入默认域。查看话题：

```bash
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 ros2 topic list --no-daemon
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 ros2 run rqt_image_view rqt_image_view
```

每个 `head`、`left_wrist`、`right_wrist` 有四个话题：

```text
/tron2_sim/<name>/color/image_raw       rgb8
/tron2_sim/<name>/depth/image_raw       32FC1，米制光轴 Z
/tron2_sim/<name>/color/camera_info
/tron2_sim/<name>/depth/camera_info
```

另有 `/clock`、`/tf`、`/tf_static`。深度中的 NaN 表示无效/超量程，保留原始浮点值，不拿伪彩色图当深度消息。

真实验收使用两个进程：在新实体80/40构型下发布了30个三路bundle，检查器取得至少3个完整共同时间戳，检查12个图像/信息话题、编码/尺寸/stride/字节数、K/R/P、NaN与有效范围、共同时间戳、时钟以及world到六个光学坐标的精确时刻TF。传输为best-effort，未宣称无丢帧。

[V2 ROS 发布验收](../reports/cameras/ros_topic_validation_vertical_v2.json) · [V2发布日志](../reports/cameras/ros_camera_publisher_vertical_v2.log) · [详细桥接协议](ros_camera_bridge.md)

## 相机模型与坐标

已有生成 URDF 中 `d435_Link` 是 `base_Link` 下的机身固定占位，并不随头关节转动。相同官方仓库中的 xacro 却给出了 `head_pitch_Link` 下的头部参考位置，两者不一致。本版保留旧机身标记，另加明确命名的头部与腕部相机：

```text
head_pitch_Link → head_camera_mount_frame → head_camera_link → color/depth optical frames
wrist_roll_L/R_Link → camera_mount_frame → rod_end_frame → camera_plate_frame
                                                     → camera_link → color/depth optical frames
```

光学坐标统一为 +X 向右、+Y 向下、+Z 向前；MuJoCo 渲染相机的 -Z 向前通过独立 Rx(π) 转换。D435i 的 color/depth 保留不同视场角和约 15 mm 外参位移，发布的是原始两视点图像，未冒充像素对齐深度。

本版使用 D405（腕）和 D435i（头）作为显式仿真选型。LimX公开SDK明确确认的 D435i 是腰部相机；公开的头/腕指南链接需要登录，不能把实装型号当作已确认。具体证据、原始模型、安装孔和版本哈希见 [相机研究](camera_research.md) 与 [模型目录](../config/camera_model_catalog.json)。

- [带相机模型/TF的 URDF](../urdf/tron2_dach_revo3_cameras.urdf)
- [MuJoCo 六个 color/depth 渲染相机场景](../simulation/cameras/scene.xml)
- [默认参数](../config/camera_rig.json)

这些相机是理想针孔几何成像，不模拟真实双目匹配、曝光、噪声、材质失效或设备标定。当前 K 由标称 FOV 和输出尺寸计算；适合视角研究，不能代替真机内参。

## 几何验证

独立脚本使用六个视点、两种分辨率、已知世界平面和 35 点彩色投影网格验证：

- 正确的光学轴、URDF正运动学、半像素主点约定；
- 光轴 Z 的米制深度；
- 渲染图像与发布 K 的投影一致性；
- 左侧长度增加 20 mm 与板角增加 15°的作用范围：左图变化，头部/右侧位姿和像素不变。

光学与图像验证见相关报告；V2的实体孔位、原大孔补实、竖段方向和相机独立配合见 [V2装配评审](../design/wrist_camera_bracket_v2/assembly_review.md)。数字几何通过不等于实物材料、疲劳或紧固强度认证，且V1横梁公式不能直接用于V2竖杆，需看V2专门的强度评审。

## 是否还需要安装

当前机器已具备并验证：项目 `.venv` 的 MuJoCo/NumPy/Pillow/Tk，ROS 2 Humble 的 rclpy、Image/CameraInfo、TF、Clock，以及 rqt_image_view、RViz2。看仿真图像和运行本地发布测试不需要再装 Isaac Sim 或 RealSense 硬件驱动，也不需要 cv_bridge。

独立 `.cad-venv` 已用于真实参数化设计、STEP导出及回读验证。当前文件见 [V2支架交付说明](../design/wrist_camera_bracket_v2/README.md)，包括完整一体STEP、三语义分区、毫米STL、80/60选项、参数不变性和强度评审。原STP可读后再单独比较，不会自动覆盖本版设计。
