# Viewer guide

English | [简体中文](README.zh-CN.md) · [Camera captures](../docs/camera_outputs.md)

These launchers provide forward-kinematics previews of bundled assets. They do
not step physics or connect to robot hardware. Run commands from the repository
root; each launcher resolves the repository and its `.venv` independently of
the current directory.

## Install and launch

The launchers invoke `.venv/bin/python` directly, so that environment must
exist and have the dependencies installed. See the root
[`README.md`](../README.md#quick-start) for setup. The launchers also
remove inherited `PYTHONPATH` to avoid loading unrelated modules.

| Launcher | Preview | Supported model / parameters |
| --- | --- | --- |
| `./viewers/view_adapters.sh` | Left/right hand adapters | `--side left\|right\|both` (default `both`) |
| `./viewers/view_camera_brackets.sh` | Left/right camera brackets, without camera bodies | `--side left\|right\|both` (default `both`) |
| `./viewers/view_assembly.sh` | Full assembly and table | `--model full\|training` (default `full`); `--side` is accepted but has no display effect |
| `./viewers/view_cameras.sh` | Head D455 and two wrist D405 RGB/depth views | `--width`, `--height`, `--rate`; full model only |

Common options are `--check` and `--output DIR`. `--check` renders offscreen
images (or a camera capture set) and exits without opening a window. Output
defaults to `outputs/`. Camera options default to 320×240 and 8 capture-loop
iterations per second; width must be 64–1280, height 64–960, and rate greater
than 0 and at most 60. These dimensions and rate affect camera preview only;
`--side` affects the parts previews, and `--model` selects the assembly only.
Training model is valid only with `view_assembly.sh`; its camera frames were
baked away.

Examples:

```bash
./viewers/view_assembly.sh --model training
./viewers/view_adapters.sh --side left
./viewers/view_camera_brackets.sh --side right
./viewers/view_cameras.sh --width 640 --height 480 --rate 8
./viewers/view_assembly.sh --check --output outputs/check/assembly
./viewers/view_cameras.sh --check --output outputs/check/cameras
```

## Controls and rendering

In 3D windows, drag with the left mouse button to rotate, drag with the right
button to pan, and use the scroll wheel to zoom. Parts windows use `1`, `2`,
`3` for left, right, and both sides; `F`, `B`, `T`, `U` select front, back,
top, and wrist-side views. `R` resets the parts view. Assembly uses `0` for the
zero pose, `1` for the configured display pose, and `R` to reset its view.
The reduced training preview has only its retained 28 movable joints; baked
head and left-side geometry does not move.

The 3D viewer uses MuJoCo's GLFW window. The camera GUI uses Tk and MuJoCo's
EGL renderer. An interactive run requires a desktop (`DISPLAY` or
`WAYLAND_DISPLAY`); without one, use `--check` to render offscreen. Offscreen
rendering still requires a working graphics runtime. The scripts select EGL
for `--check` and camera mode, and GLFW for interactive 3D mode unless
`MUJOCO_GL` is already set. No universal EGL/OSMesa repair is assumed here.

All views are kinematic. The published training URDF has fingertip inertias
that fail MuJoCo's inertia triangle check when compiled as-is. For the preview,
the runtime enables `balanceinertia` on an in-memory `MjSpec`; it does not
rewrite the URDF and does not establish dynamics equivalence with Isaac.

## Troubleshooting

- **`.../.venv/bin/python: No such file or directory`**: create `.venv` at the
  repository root and install `requirements.txt`; the scripts intentionally use
  that interpreter.
- **`No desktop display`**: run with `--check` for offscreen output. A headless
  machine still needs a functioning EGL graphics setup for MuJoCo rendering.
- **`Asset changed` or `Missing or invalid asset`**: the runtime compares
  bundled asset SHA256 values with `assets/manifest.json`. Restore the exact
  released file set or investigate the changed asset; the viewer does not
  regenerate assets or bypass the checksum.
- **MuJoCo inertia triangle error**: use the provided training viewer path.
  Directly compiling the raw training URDF does not apply the preview-only
  in-memory inertia balancing.

The camera save button and output files are documented in
[`camera_outputs.md`](../docs/camera_outputs.md).
