#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH "$project_dir/.venv/bin/python" "$project_dir/scripts/camera_ros_bridge.py" --rate 15 "$@"
