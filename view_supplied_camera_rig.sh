#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH "$project_dir/.venv/bin/python" "$project_dir/scripts/view_camera_robot.py" --config "$project_dir/config/camera_rig_supplied_step_aligned_v3.json" "$@"
