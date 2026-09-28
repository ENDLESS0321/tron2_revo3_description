#!/usr/bin/env bash
set -euo pipefail
# Select --model full (default) or --model training for the reduced28 preview.
viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec env -u PYTHONPATH "$viewer_root/.venv/bin/python" "$viewer_root/viewers/_viewer.py" --mode assembly "$@"
