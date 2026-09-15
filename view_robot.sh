#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH "$project_dir/.venv/bin/python" "$project_dir/scripts/preview_assembly.py" \
  --urdf "$project_dir/urdf/tron2_dach_revo3_v2.urdf" \
  --scene "$project_dir/simulation/revision2/scene.xml" \
  --report-dir "$project_dir/reports/revision2" --viewer --skip-render "$@"
