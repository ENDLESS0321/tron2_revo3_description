# Simulated camera captures

English | [简体中文](camera_outputs.zh-CN.md) · [Viewer guide](../viewers/README.md)

`./viewers/view_cameras.sh` renders three fixed camera poses from the full
assembly: `head` (D455), `left_wrist` (D405), and `right_wrist` (D405). It is a
local MuJoCo simulation, not a device connection or calibrated camera stream.
Run it from the repository root after installing the required `.venv`:

```bash
./viewers/view_cameras.sh --width 640 --height 480 --rate 8 --output outputs/cameras
```

The GUI's **保存当前图像与深度** button creates a timestamp-named directory
under `--output` (default `outputs/`). Each capture contains:

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

RGB PNG and colorized depth PNG are for convenient viewing. The NPY files hold
the uncolorized depth arrays. `capture.json` has this structure (values shown
here are schematic):

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

The `cameras` object contains all three names. Each camera record copies its
runtime configuration except internal renderer IDs, then adds `color_K`,
`depth_K`, `rgb_shape`, `rgb_std`, and `valid_depth_pixels`. Intrinsic matrices
are 3×3 pinhole matrices computed from the configured nominal horizontal and
vertical FOV at the requested resolution. The exact computed matrices and
per-camera FOV/range/frame values are written to the JSON. These are nominal
simulation parameters, not hardware calibration results. The capture does not
export camera-to-world extrinsics or a separate calibration file.

Both RGB and depth images have the requested width and height. RGB arrays are
`uint8` with shape `(height, width, 3)`. Raw depth arrays are `float32`, shape
`(height, width)`, and contain optical-frame Z distance in meters. Values
outside the configured range, non-finite values, and absent geometry become
`NaN`; inspect finite values before calculations. Depth is rendered from a
separate depth camera and is not registered to the color image. The nominal
ranges are 0.6–6.0 m for the head D455 and 0.07–0.5 m for each wrist D405.

`stamp_ns` is synthetic model time in nanoseconds: the capture code advances a
simulation tick by `1 / fps`, with `fps` configured as 15 in
`assets/runtime.json`. It is not Unix/epoch time or measured wall-clock capture
time. The GUI's `--rate` controls an independent loop wait interval (default 8
iterations per second); it does not change the model-time tick rate. Do not use
`stamp_ns` or capture order to claim synchronization with external streams or
actual wall-time timing.

Minimal reader:

```python
import json
from pathlib import Path
import numpy as np

capture = Path("outputs/cameras/20260929_120000_123456")
meta = json.loads((capture / "capture.json").read_text())
rgb = np.asarray(__import__("PIL.Image", fromlist=["open"]).open(capture / "head_rgb.png"))
depth_z_m = np.load(capture / "head_depth_m.npy", allow_pickle=False)
valid = np.isfinite(depth_z_m)
print(meta["stamp_ns"], meta["cameras"]["head"]["color_K"])
print(rgb.shape, rgb.dtype, depth_z_m.shape, depth_z_m.dtype, int(valid.sum()))
```

For a headless smoke capture, `--check` saves the same file set and validates
that all three cameras have nonempty RGB and valid depth:

```bash
./viewers/view_cameras.sh --check --width 640 --height 480 --output outputs/check/cameras
```

The launch scripts verify asset checksums before rendering. A checksum failure
means a file is absent or differs from the published manifest; restore or
investigate the asset rather than treating generated captures as valid.
