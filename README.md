# TRON2 + Revo3 Assembly Model

English | [简体中文](README.zh-CN.md)

Models, manufacturing CAD and kinematic viewers for a DACH_TRON2A dual-arm robot with BrainCo Revo3 hands, hand adapters, V3 wrist-camera brackets, a head D455 and two wrist D405 cameras. Asset release 1.7.1 includes the physicsfix training URDF; all referenced meshes are bundled.

![Full assembly and table: kinematic preview](docs/images/new_urdf_front_table.png)

## Quick start

Tested on Ubuntu 22.04 with Python 3.10. Viewers require no CAD software, ROS or additional model downloads.

```bash
git clone https://github.com/ENDLESS0321/tron2_revo3_description.git
cd tron2_revo3_description
sudo apt install python3-venv python3-tk libgl1 libegl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
./viewers/view_assembly.sh
```

Without a desktop, replace the last line with `./viewers/view_assembly.sh --check --output outputs/check/assembly`. Offscreen rendering still needs a working graphics backend; see the [viewer guide](viewers/README.md).

## Choose a model

| Asset | Purpose | Moving joints | Table / rendering cameras |
| --- | --- | ---: | --- |
| [assets/assembly.urdf](assets/assembly.urdf) | Complete bilateral assembly | 58 | No table or rendering camera definitions; camera bodies and frames are retained |
| [assets/scene.xml](assets/scene.xml) | Full-model MuJoCo preview | 58 | Table and 6 RGB/depth viewpoints |
| [assets/assembly_rl_convex.urdf](assets/assembly_rl_convex.urdf) | Physicsfix right-arm/hand training articulation | 28 | No table or rendering camera definitions |

Both hand/flange assemblies and wrist cameras use the axis-180 installation. See the [asset guide](assets/README.md) for the rotation axis, transforms, meter/radian units and layouts. The training model retains seven arm and 21 hand joints; read the [training integration guide](docs/training_model.md) before reusing IK, control mappings or checkpoints.

![Reduced training articulation: kinematic preview](docs/images/training_reduced28_preview.png)

## Viewer entry points

| Command | View |
| --- | --- |
| `./viewers/view_adapters.sh` | Left/right hand adapters |
| `./viewers/view_camera_brackets.sh` | Left/right brackets, excluding camera bodies |
| `./viewers/view_assembly.sh` | Full assembly and table |
| `./viewers/view_assembly.sh --model training` | 28-joint training model |
| `./viewers/view_cameras.sh` | Three RGB and three depth views |

Scripts use the repository's `.venv` and also work when invoked by absolute path from another directory. Left-drag rotates, right-drag pans and scrolling zooms; `0` selects zero pose and `1` the display pose in the assembly window. See the [viewer guide](viewers/README.md) for parameters, shortcuts, output paths and troubleshooting.

## Read by task

| Task | Guide |
| --- | --- |
| Import models and understand frames/layouts | [Assets](assets/README.md) |
| Integrate training, IK or control mappings | [Training model](docs/training_model.md) |
| Hand off CAD or select manufacturing files | [Manufacturing CAD](assets/cad/README.md) |
| Run viewers or headless checks | [Viewers](viewers/README.md) |
| Read RGB, depth and camera metadata | [Camera outputs](docs/camera_outputs.md) |
| Understand visual/collision mesh provenance | [Meshes](assets/meshes/README.md) |
| Validate changes to assets | [Tests and maintenance](tests/README.md) |

The [documentation index](docs/README.md) links these guides.

## Verification and asset identity

```bash
.venv/bin/python -m unittest discover -s tests -v
./viewers/view_assembly.sh --model full --check --output outputs/check/assembly
./viewers/view_assembly.sh --model training --check --output outputs/check/training
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

[manifest.json](assets/manifest.json) records SHA256, byte counts and provenance. Viewers check integrity and never silently regenerate changed models. See the [test guide](tests/README.md) for all checks and the maintenance workflow.

## Repository contents and limits

`main` supplies models, manufacturing CAD and viewers; `dev` retains an earlier development snapshot. Directory guides are available for [assets](assets/README.md), [viewers](viewers/README.md), [tests](tests/README.md) and [docs](docs/README.md). Generated captures default to Git-ignored `outputs/`.

All viewers use kinematics without physics stepping or hardware commands. The training preview balances fingertip inertia in memory to satisfy MuJoCo compilation; a passing preview does not establish dynamics equivalence, physical grasp success or policy compatibility. Cameras are ideal pinhole models with nominal parameters, and depth is not color-registered.

See the [CAD guide](assets/cad/README.md) for manufacturing files, units and pending specifications. Strength, fatigue, cable/tool clearance and hardware calibration have not been certified. Source and licensing qualifications are in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), including BrainCo's unresolved license status.
