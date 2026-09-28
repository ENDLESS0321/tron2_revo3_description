# TRON2 + Revo3 Assembly Model

English | [简体中文](README.zh-CN.md)

Models, manufacturing CAD and kinematic viewers for a DACH_TRON2A dual-arm robot
with BrainCo Revo3 hands, hand adapters, V3 wrist-camera brackets, a head D455
and two wrist D405 cameras. Release 1.7.1 includes the exact physicsfix training
URDF. All model meshes are bundled; CAD software, ROS and additional downloads
are unnecessary for the viewers.

## Choose a model

| Asset | Purpose | Moving joints | Table / rendering cameras |
| --- | --- | --- | --- |
| [`assets/assembly.urdf`](assets/assembly.urdf) | Complete bilateral assembly | 58 | Neither included in the URDF |
| [`assets/scene.xml`](assets/scene.xml) | Matching full MuJoCo preview | 58 | Table and 6 RGB/depth viewpoints |
| [`assets/assembly_rl_convex.urdf`](assets/assembly_rl_convex.urdf) | Physicsfix right-arm/hand training articulation | 28 | Neither included |

Both hand/flange assemblies and wrist cameras use the axis-180 installation:
180° around the parent wrist link's local Z centerline through `[-0.0317, 0, 0]`
m. This installation axis differs from the actuated wrist-roll axis. Mount
translations and physical joint limits are retained; the head camera is unchanged.
The hand-to-flange translation is `[0, 0.023855, 0]` m, with left RPY
`[-π/2, -π/2, 0]` and right RPY `[-π/2, +π/2, 0]`.

![Full assembly and table: kinematic preview](docs/images/new_urdf_front_table.png)

## Training topology, collisions and coordinates

The training model in this repository is
[`assets/assembly_rl_convex.urdf`](assets/assembly_rl_convex.urdf). It is a
byte-for-byte copy of the supplied source file
`assembly_bilateral_axis180_reduced28_physicsfix.urdf`; that source filename
is provenance metadata, not an additional file in this repository. Use
`assets/assembly_rl_convex.urdf` when loading or configuring the training model.
It has 38 links, 37 joints (28 revolute + 9 fixed), 78 visual geometries,
58 collision elements on 31 links, and 128 unique mesh references. The retained
`rl_convex` filename supports existing configuration paths; the palm uses
28 component collisions.

| Collision group | Elements |
| --- | ---: |
| Base and seven right-arm links | 8 |
| Right hand base / palm components | 28 |
| Right finger components | 21 |
| Right adapter / flange | 1 |
| Total | 58 |

The seven arm joints, in proximal-to-distal order, are
`proximal_pitch_R_Joint`, `proximal_roll_R_Joint`, `proximal_yaw_R_Joint`,
`elbow_R_Joint`, `wrist_yaw_R_Joint`, `wrist_pitch_R_Joint`, and
`wrist_roll_R_Joint`. The hand has five thumb joints and four joints on each
other finger. Map controls by joint name and bind IK/H5/runtime configurations
to the URDF hash. Old 58-element mappings and matching checkpoint tensor sizes
do not establish compatibility with this 28-joint model.

Head and left-side geometry is baked into retained ancestors, with inertials
combined during reduction. Their independent joints and link/frame identities
are removed. The frozen pose, in radians, is:

- Head yaw `0`, head pitch `0.35`; left hand joints all zero.
- Left arm, ordered pitch/roll/yaw/elbow/wrist-yaw/wrist-pitch/wrist-roll:
  `1.2395190538964451 0.0050965579201923406 0.0706739400409262 -0.10075078688932715 -0.33794336457190877 -0.6172884125393175 0.5453728186806424`.

Each of the five fingertips has mass `0.001 kg`, zero COM and diagonal inertia
`(1e-10, 1e-10, 1e-9) kg·m²`, with zero off-diagonal terms. `right_palm` remains
a placeholder without an inertial. This artifact combines the fingertip
inertial repair and palm collision restoration; their separate effects on
training have not been isolated.

Both URDFs set `world_to_base` to `[0, 0, 1.20035]` m. The full scene table top
is `0.75035 m`, exactly `0.45 m` below the robot origin. The physicsfix training
layout uses base Z `0.957 m` and table top `0.507 m`, also a `0.45 m` gap.
Configure world/base/table placement explicitly when importing the training
URDF, avoiding a second application of its world offset. It contains no table
or manipulated object, and `scene.xml` describes the full articulation.

![Reduced training articulation: kinematic preview](docs/images/training_reduced28_preview.png)

## Install and launch

Tested on Ubuntu 22.04 with Python 3.10. From the repository root:

```bash
sudo apt install python3-venv python3-tk libgl1 libegl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

| Command | View |
| --- | --- |
| `./viewers/view_adapters.sh` | Left/right hand adapters |
| `./viewers/view_camera_brackets.sh` | Left/right brackets, excluding camera bodies |
| `./viewers/view_assembly.sh` | Full assembly and table |
| `./viewers/view_cameras.sh` | Three RGB and three depth views |

```bash
./viewers/view_assembly.sh --model training
./viewers/view_adapters.sh --side left
./viewers/view_camera_brackets.sh --side right
./viewers/view_cameras.sh --width 640 --height 480 --rate 8
```

Launch scripts locate the repository environment and assets automatically,
including when invoked by absolute path. Preserve the directory structure.
GLFW supplies 3D windows; Tk and EGL supply the camera window. Use `--check`
without a desktop; `MUJOCO_GL=osmesa` is an alternative if the corresponding
libraries are installed.

Mouse: left-drag rotates, right-drag pans, scroll zooms. Parts: `1/2/3` selects
left/right/both, `F/B/T/U` selects front/back/top/wrist-side, `R` resets.
Assembly: `0` selects zero pose, `1` display pose, `R` resets the view. Training
mode changes only the 28 retained joints; baked geometry stays fixed. Parts and
camera viewers support the full model only.

All viewers use forward kinematics without physics stepping. MuJoCo rejects
the raw training URDF because its supplied fingertip inertias violate the
inertia triangle check. The training preview enables `balanceinertia` in an
in-memory `MjSpec`; it preserves the published URDF bytes and does not establish
MuJoCo/Isaac dynamics equivalence. For a programmatic kinematic preview:

```python
import sys
sys.path.insert(0, "viewers")
from _runtime import load_training_model
model = load_training_model()
assert model.nq == 28
```

The camera window's **保存当前图像与深度** button saves RGB PNGs, depth PNGs,
raw `_depth_m.npy` and `capture.json` (intrinsics, frames, time and valid-pixel
statistics) in `outputs/<timestamp>/`; `--output` chooses another directory.
Raw depth is optical Z in meters, `float32`, with invalid pixels as `NaN`.
Depth is not registered to color. Nominal depth ranges are D455 `0.6–6 m` and
D405 `0.07–0.5 m`. Cameras are ideal pinhole simulations with nominal fields
of view, rather than calibrated hardware streams.

## Verification and asset identity

```bash
.venv/bin/python -m unittest discover -s tests -v
./viewers/view_adapters.sh --check --output outputs/check/adapters
./viewers/view_camera_brackets.sh --check --output outputs/check/brackets
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

[`assets/manifest.json`](assets/manifest.json) records per-file SHA256, byte
counts and upstream revisions. Viewers check integrity before loading; they
never silently regenerate a changed asset. [`assets/runtime.json`](assets/runtime.json)
selects models, topology, poses, parts and camera settings. Exact URDF SHA256:

```text
assets/assembly.urdf
537f31a798ddb05d1f29e2b5eeede63d47af3d9ff519f40907a24e757f5bbf44
assets/assembly_rl_convex.urdf
2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034
```

Tests cover integrity, topology, joint definitions, right-chain FK, mounting,
table height and preview loading. Compilation and kinematic images do not
establish physical grasp success or policy compatibility. Reimport simulators
that cached an older asset.

## Repository contents and CAD

```text
assets/       URDFs, MuJoCo scene, runtime configuration and integrity manifest
  assembly.urdf           Full bilateral assembly (58 moving joints)
  assembly_rl_convex.urdf  Physicsfix training model (28 moving joints)
  scene.xml               Full-model MuJoCo scene
  runtime.json            Model selection and viewer settings
  manifest.json           Asset hashes and source provenance
  meshes/     Referenced visual and collision meshes
  cad/
    adapters/        left/right.3mf and left/right_print_mm.stl
    camera_brackets/ left/right.step and left/right_mm.stl
viewers/      Four launch scripts and shared implementation
tests/        Release asset verification
docs/images/  Full and training preview images linked above
licenses/     Upstream licenses, notices and licensing evidence
```

For manufacturing, use the bracket STEP files and adapter 3MF/STL files.
CAD STLs use millimeters; simulation meshes use meters. Bracket manufacturing
files exclude camera bodies. Mechanical strength, fatigue, cable/tool clearance
and hardware calibration have not been certified. The viewers send no hardware
commands.

Generated captures, logs, environments and interpreter caches are ignored.
Usage and model notes are kept in these paired READMEs; earlier development
reports remain in Git history. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
for source and licensing qualifications, including BrainCo's unresolved license.
