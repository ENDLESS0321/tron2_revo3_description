# ROS 2 RGB-D bridge

The bridge publishes the head, left-wrist and right-wrist MuJoCo camera streams.
It uses the project Python 3.10 environment with the already installed ROS 2 Humble
packages. No `cv_bridge` or additional installation is needed. The current default
uses the user-authorized photo-reference bracket CAD, with the camera kept as a
separate model. Length and plate angle rebuild the solid geometry. This is a new
design, not a reconstruction of the unavailable original bracket; ROS publication
does not establish physical fit or material strength. Legacy frame-only configs
can still be used explicitly.

Run both commands from the project directory, in separate terminals. Start the
checker first so it can discover the publishers before the finite capture run:

```sh
.venv/bin/python scripts/check_camera_topics.py --width 640 --height 480 --timeout 60
```

```sh
.venv/bin/python scripts/camera_ros_bridge.py --width 640 --height 480 --frames 90
```

Both tools accept `--config PATH`. The bridge also accepts `--rate 30` and
`--startup-wait 2`. `--frames 0` runs until interrupted. The rig owns simulation
time and advances it using its configured `fps` (currently 15 Hz); `--rate` only
limits wall-clock publication rate. Use matching config and width/height arguments
in the checker. Both tools default to
`ROS_DOMAIN_ID=94` and set `ROS_LOCALHOST_ONLY=1` before creating their own ROS
contexts. They do not launch device drivers, discover the default domain, or
modify existing ROS nodes. Use a different agreed non-default domain with
`--domain-id` if domain 94 is already in use.

The scripts append these existing Humble paths in-process, while retaining the
project environment's NumPy 1.26.4 and MuJoCo 3.13.0:

```text
/opt/ros/humble/local/lib/python3.10/dist-packages
/opt/ros/humble/lib/python3.10/site-packages
```

The standalone bridge selects EGL for rendering and does not open a desktop
window. Its image conversion fills `sensor_msgs/Image` directly, avoiding the
NumPy ABI dependency of `cv_bridge`.

## Topics and timing

For each `{camera}` in `head`, `left_wrist`, `right_wrist`:

| Topic | Payload |
| --- | --- |
| `/tron2_sim/{camera}/color/image_raw` | `rgb8`, contiguous RGB bytes |
| `/tron2_sim/{camera}/depth/image_raw` | `32FC1`, little-endian optical-axis depth in meters |
| `/tron2_sim/{camera}/color/camera_info` | Color intrinsics and color optical frame |
| `/tron2_sim/{camera}/depth/camera_info` | Depth intrinsics and depth optical frame |

Images and CameraInfo use sensor-data QoS (best effort, volatile). `/tf` uses the
normal dynamic TF broadcaster; `/tf_static` is transient-local and sent once.
`/clock` uses reliable, volatile QoS. Each complete capture publishes one simulation
stamp across all images, CameraInfo, dynamic TF, and `/clock`. CameraInfo carries
zero distortion, identity rectification and a projection matrix formed from K.
Color and depth can have separate K and separate optical frame IDs; the current
rig provides synchronized RGB-D with separate color/depth optical frames and
intrinsics. It does not claim pixel alignment between the separate sensors.
Frame names are taken from the rig,
not invented by the bridge. Optical axes must be +X right, +Y down, +Z forward.

## In-process interface

`scripts/camera_rig.py` supplies:

```python
source = create_source(width=640, height=480, config_path=None)
source.camera_specs  # list of 3 dictionaries
source.static_transforms  # list of fixed camera/optical transforms
bundle = source.step_and_capture()
source.close()
```

Each camera specification contains `name`, `width`, `height`, and either a shared
`frame_id`/`K` or stream-specific `color_frame_id`, `depth_frame_id`, `color_K`,
`depth_K`. K can be a flat list of 9 values or a 3x3 array.

Each bundle contains `stamp_ns` (nonnegative integer simulation nanoseconds),
`frames` keyed by the three camera names, and `transforms`. Each camera frame has
`rgb` (`uint8[H,W,3]`) and `depth` (`float32[H,W]`, meters), with optional per-frame
`color_K`/`depth_K` overrides. `NaN` denotes invalid or out-of-range pixels and is
preserved in the `32FC1` payload, consistent with the
[ROS depth-image convention](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst).
The bridge rejects infinities, negative finite values, and frames without any
positive finite depth. It checks valid pixels against `depth_range_m` when supplied
by the camera specification; these ranges are configured simulation choices,
not measured hardware limits. Each transform
uses `parent`, `child`, `xyz` in meters and `quat_xyzw`, with a unit quaternion.
A TF child must have one parent and must not appear in both static and dynamic TF.

`CameraPublisher(camera_specs, static_transforms, topic_prefix, context=...)`
and its `publish_bundle(bundle)` method can be called by an existing simulation
process. Importing the bridge does not initialize ROS, start rendering, or change
the graphical backend. The embedding application owns its executor and cleanup.

## Verification

`check_camera_topics.py` writes `reports/cameras/ros_topic_validation.json`
(override with `--output`). It subscribes through DDS and requires at least three
complete nonzero timestamps across every camera and both streams. It checks
resolution, encoding, endianness, row stride, byte count, NaN/valid fractions,
absence of infinite/negative finite depth, configured depth ranges, optical
frame naming, CameraInfo K/R/P, paired stamps/frame IDs, `/clock`
correspondence, nonempty dynamic/static TF, unit quaternions, unique TF parents,
acyclic TF, and exact-time `world`-to-optical lookup. Stamp zero is excluded from
the exact-time count because ROS interprets TF lookup at zero as “latest”.

The report stores payload statistics and calibration/TF snapshots, not raw image
buffers. Depth-to-world reprojection and camera pose correctness must be verified
by the separate rig tests. A passing transport report is not physical calibration
or a hardware readiness claim.
