#!/usr/bin/env bash
set -euo pipefail
CAMERA_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$CAMERA_PROJECT_DIR"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
# Match the current assembled-robot viewer. An explicit --config in "$@"
# can still select a historical or saved configuration without changing it.
exec env -u PYTHONPATH "$CAMERA_PROJECT_DIR/.venv/bin/python" "$CAMERA_PROJECT_DIR/scripts/view_camera_images.py" \
  --config "$CAMERA_PROJECT_DIR/config/camera_rig_supplied_step_aligned_v3.json" "$@"
