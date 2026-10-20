#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/humble/setup.bash
source "$task_root/lio_ws/install/setup.bash"
set -u
exec python3 scripts/fly_tunnel.py "$@"
