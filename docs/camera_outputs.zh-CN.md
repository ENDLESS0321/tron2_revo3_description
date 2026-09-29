# 仿真相机采集文件

[English](camera_outputs.md) | 简体中文 · [预览工具指南](../viewers/README.zh-CN.md)

`./viewers/view_cameras.sh` 从完整装配的三个固定相机位姿渲染图像：`head`
（D455）、`left_wrist`（D405）和 `right_wrist`（D405）。这是本地 MuJoCo
仿真，不会连接真实设备，也不是经过标定的相机数据流。在安装 `.venv` 后
从仓库根目录运行：

```bash
./viewers/view_cameras.sh --width 640 --height 480 --rate 8 --output outputs/cameras
```

在窗口点击 **保存当前图像与深度**，会在 `--output`（默认 `outputs/`）下
创建以当前日期时间命名的目录：

```text
<capture>/
  head_rgb.png
  head_depth.png
  head_depth_m.npy
  left_wrist_rgb.png
  left_wrist_depth.png
  left_wrist_depth_m.npy
  right_wrist_rgb.png
  right_wrist_depth.png
  right_wrist_depth_m.npy
  capture.json
```

RGB PNG 和着色深度 PNG 便于查看；NPY 文件保存未着色的原始深度数组。
`capture.json` 结构如下（数值为示意）：

```json
{
  "stamp_ns": 66666667,
  "depth_unit": "meter",
  "cameras": {
    "head": {
      "name": "head",
      "label": "头部 D455",
      "model": "D455",
      "color_frame": "head_camera_color_optical_frame",
      "depth_frame": "head_camera_depth_optical_frame",
      "color_fov_deg": [90, 65],
      "depth_fov_deg": [87, 58],
      "depth_range_m": [0.6, 6.0],
      "color_K": [[320.0, 0, 319.5], [0, 376.7, 239.5], [0, 0, 1]],
      "depth_K": [[337.2, 0, 319.5], [0, 433.0, 239.5], [0, 0, 1]],
      "rgb_shape": [480, 640, 3],
      "rgb_std": 42.1,
      "valid_depth_pixels": 123456
    }
  }
}
```

`cameras` 对象包含三个相机名称。每条记录复制运行配置（去掉仅供内部渲染
使用的 ID），并附加 `color_K`、`depth_K`、`rgb_shape`、`rgb_std` 和
`valid_depth_pixels`。内参矩阵为 3×3 理想针孔模型，依据配置中的标称水平、
垂直 FOV 和请求图像分辨率计算。JSON 保存计算所得的实际矩阵，以及每个相
机的 FOV、量程和 frame。它们只是仿真标称参数，不是硬件标定结果；采集文
件不导出相机到世界坐标的外参，也没有单独的标定文件。

RGB 和深度图像均为请求的宽高。RGB 数组为 `uint8`，形状
`(height, width, 3)`。原始深度数组为 `float32`，形状 `(height, width)`，
表示光学坐标系的 Z 距离，单位米。超出配置量程、非有限或没有几何体的像素
会写为 `NaN`；计算前请先筛出有限值。深度由独立的深度相机渲染，没有配准
到彩色图。头部 D455 的标称量程为 0.6–6.0 m，腕部 D405 各为 0.07–0.5 m。

`stamp_ns` 是合成的模型时间，单位纳秒：采集代码每次将仿真 tick 增加
`1 / fps`，`assets/runtime.json` 中配置的 `fps` 为 15。它不是 Unix／纪元
时间，也不是测得的实际采集时间。GUI 的 `--rate` 独立控制循环等待间隔，
默认每秒 8 次；它不会改变模型时间 tick 的频率。不能用 `stamp_ns` 或采集
顺序推断其与外部数据流同步，或推断实际墙钟时间。

最小读取示例：

```python
import json
from pathlib import Path
import numpy as np
from PIL import Image

capture = Path("outputs/cameras/20260929_120000_123456")
meta = json.loads((capture / "capture.json").read_text())
rgb = np.asarray(Image.open(capture / "head_rgb.png"))
depth_z_m = np.load(capture / "head_depth_m.npy", allow_pickle=False)
valid = np.isfinite(depth_z_m)
print(meta["stamp_ns"], meta["cameras"]["head"]["color_K"])
print(rgb.shape, rgb.dtype, depth_z_m.shape, depth_z_m.dtype, int(valid.sum()))
```

无桌面环境下可用 `--check` 生成同一组文件，并检查三路相机均有非空 RGB
与有效深度：

```bash
./viewers/view_cameras.sh --check --width 640 --height 480 --output outputs/check/cameras
```

渲染前启动脚本会检查资产校验和。校验失败表示文件缺失或与发布清单不符；
应恢复或调查相应资产，不要把该运行产物视为有效采集。
