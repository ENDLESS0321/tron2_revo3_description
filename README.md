# TRON2 + Revo3 Assembly Model

English | [简体中文](README.zh-CN.md)

Visualize the DACH_TRON2A dual-arm robot, two BrainCo Revo3 hands, the final hand
adapters, and the V3 camera brackets. The appearance follows the graphite TRON2
and silver Revo3 references, with red/cyan accents and purple hand flanges. The
three-camera viewer renders RGB and metric depth from a head-mounted D455 and two
wrist-mounted D405 cameras.

`main` contains the full assembly, reduced training asset, viewing tools and
provenance metadata. `dev` preserves the earlier development snapshot and CAD
inputs; released revisions remain available in Git history.
Running the viewers does not require CAD software, ROS, development
directories, or additional model downloads.

## Mount configuration (1.7.0)

Both hand/flange assemblies and both wrist cameras now turn 180° around the
forearm longitudinal centerline. The aim is greater joint-limit headroom for
tabletop palm-down, fingers-forward operation; physical limits stay unchanged.
See [rotation details, model provenance and previews](docs/mount_rotation.md).
The approved generated URDF directly replaces `assets/assembly.urdf` byte-for-byte.
Full-assembly URDF imports use this canonical path; the full preview and cameras
load its matching `scene.xml`. Training preview loads the reduced URDF.
The MuJoCo scene is synchronized, retaining the 45 cm base-to-table-top gap.

## Training model and preview (1.7.1)

`assets/assembly_rl_convex.urdf` is an exact copy of the physicsfix reduced
28-DoF model (`assembly_bilateral_axis180_reduced28_physicsfix.urdf`): 7
right-arm plus 21 right-hand joints. Its 58 collision elements cover the base,
arm, flange, 28 palm components and 21 finger components. The full assembly
and default MJCF remain unchanged. Read
[training topology and migration](docs/training_reduced28.md) before reusing
action mappings, body/frame names, IK or checkpoints. The model is being used
in a four-GPU 4090D training run; this release does not claim training success.
MuJoCo's training preview balances the supplied fingertip inertias in memory
for display; it does not step physics or change the published URDF.

## Installation

Tested on Ubuntu 22.04 with Python 3.10. Run these commands from the repository
root. If the local `.venv` is already set up, skip to the launch commands.

```bash
sudo apt install python3-venv python3-tk libgl1 libegl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

The 3D windows use GLFW. The camera window uses Tk, with EGL for offscreen
rendering. Use `--check` when no desktop display is available. If EGL is
unavailable, you can try `MUJOCO_GL=osmesa` after installing the corresponding
graphics libraries.

## Four Launch Commands

All entry points are in `viewers/`. The duplicate launch scripts previously
located at the repository root have been removed.

| Command | What it shows |
| --- | --- |
| `./viewers/view_adapters.sh` | Left and right hand adapters only |
| `./viewers/view_camera_brackets.sh` | Left and right camera brackets only, without camera bodies |
| `./viewers/view_assembly.sh` | Complete robot, hands, adapters, brackets, and cameras |
| `./viewers/view_cameras.sh` | Three RGB views and three depth views in one window |

You can also launch these scripts using absolute paths from another directory.
They locate this repository's environment and assets automatically. Preserve
the complete directory structure when distributing the project; do not copy
only a script or a URDF file.

### Part and Assembly Controls

Drag with the left mouse button to rotate, drag with the right button to pan,
and use the scroll wheel to zoom. Close the window to exit.

- Part windows: `1` selects the left part, `2` the right part, and `3` both.
  `F/B/T/U` select front/back/top/wrist-side views; `R` resets the view.
- Part windows also accept `--side left`, `--side right`, or `--side both`.
  The left part is orange and the right part is blue.
- Assembly window: `0` selects the zero pose, `1` the display pose, and `R`
  resets the view. This is a forward-kinematics preview, not free dynamics.

The assembly entry accepts `--model full` (default, 58 DoF and table) or
`--model training` (28 DoF, no table). In training mode, `0` resets only the
28 retained joints; the baked head/left pose stays fixed. `1` applies the
right-side display pose from `runtime.json`. Parts and camera viewers use the
full assets; they reject the training selection because its camera frames are
baked away. All previews use forward kinematics without physics stepping.

```bash
./viewers/view_assembly.sh --model training
./viewers/view_adapters.sh --side left
./viewers/view_camera_brackets.sh --side right
```

### Camera Views and Saving Data

```bash
./viewers/view_cameras.sh --width 640 --height 480 --rate 8
```

Click **保存当前图像与深度** (Save current images and depth) to save a snapshot
under `outputs/<timestamp>/`. Use `--output` to choose another directory.
Each camera saves an RGB PNG, a false-color depth PNG, and a raw depth
`_depth_m.npy` file. The accompanying `capture.json` contains camera intrinsics,
optical frame names, a timestamp, and valid-pixel statistics.

- Raw depth is `float32`, in meters, and measures optical-frame Z distance.
  Invalid values are `NaN` and appear black in the depth visualization.
- The D455 head depth preview uses its nominal 0.6–6 m ideal range; the wrist
  range is 0.07–0.5 m.
- Raw depth is not registered to the color image. In particular, the D455
  color and depth streams have different optical origins and fields of view.
- These are ideal pinhole simulations, not live hardware streams. The fixed
  model has no length/tilt sliders that could detach cameras from their mounts.

## Headless Checks

These commands verify asset integrity, load the models, and render images
without opening a desktop window. Part checks cover both sides and multiple
viewpoints.

```bash
./viewers/view_adapters.sh --check --output outputs/check/adapters
./viewers/view_camera_brackets.sh --check --output outputs/check/brackets
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

If an asset checksum does not match, the viewer will not automatically rebuild
or replace the model. Inspect local changes with `git status`, restore the
relevant asset from a trusted commit, and try again.

## Model and CAD Files

```text
assets/
├── assembly_rl_convex.urdf  # Reduced right-arm/hand training model, 28 DoF
├── training_reduced28.json  # Source/topology/mesh hashes and verification
├── mount_rotation.json     # Full-assembly installation provenance
├── assembly.urdf           # Complete assembly; meters/radians, 58 robot DOFs
├── scene.xml               # MuJoCo scene with six RGB/depth rendering viewpoints
├── runtime.json            # Full/training selection, poses, cameras and parts
├── manifest.json           # File checksums and upstream revisions
├── meshes/                 # Referenced final meshes, including collision meshes
└── cad/
    ├── adapters/           # left/right.3mf and left/right_print_mm.stl
    └── camera_brackets/    # left/right.step and left/right_mm.stl
viewers/                    # Four entry points and their shared implementation
licenses/                   # Upstream licenses, notices, and licensing evidence
```

For mechanical engineering handoff, prefer
`assets/cad/camera_brackets/left.step` and `right.step` for the camera brackets.
Use the 3MF/STL files in `assets/cad/adapters/` for the hand adapters. CAD STL
files use millimeters; simulation meshes use meters. Camera bodies are not
included in the bracket manufacturing files.

The full assembly URDF includes the official RealSense D455 body mesh, nominal depth, color,
infrared and IMU frames, and the D405 wrist-camera frames. RGB/depth rendering
uses the camera definitions in `scene.xml`.

The `base_Link` origin is at world `z=1.20035 m` in both the URDF and MuJoCo
scene. The scene's work-table top is at `z=0.75035 m`, exactly `0.45 m` below
that robot origin. The URDF itself does not include the table geometry.

```python
import mujoco
model = mujoco.MjModel.from_xml_path("assets/scene.xml")
```

## Branches and Limitations

- `dev`: the complete development snapshot, excluding local virtual
  environments and interpreter caches. It includes original STP/3MF files and
  local Git metadata for the three vendor repositories.
- `main`: the final models and four viewing functions. Close running windows
  and save local changes before switching branches. Restore an individual file
  with `git restore --source dev -- <path>`, or use `git switch dev` for the
  complete development version.
- Public repository:
  [ENDLESS0321/tron2_revo3_description](https://github.com/ENDLESS0321/tron2_revo3_description).
  Both `main` and `dev` are published, including the development snapshot and
  original CAD in their shared history. Removing a file from the `main` working
  tree does not make its historical contents private. The adjacent glove
  application, delivery bundle, and skill handoff are not part of this repository.
- Camera models and intrinsics are simulation selections and nominal values,
  not hardware calibration. Physical assembly, cable/tool clearance, the full
  motion range, structural strength, and fatigue have not been certified.
  Tolerances at original mating surfaces still require mechanical review.
- The viewers do not connect to hardware or send control commands. Additional
  validation is required before manufacturing or physical grasping.

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for licensing information,
including BrainCo's unresolved license status.
