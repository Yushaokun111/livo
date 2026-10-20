#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export GZ_SIM_RESOURCE_PATH="$project_dir/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if ! command -v gz >/dev/null; then
  echo 'Gazebo Harmonic is required (gz sim 8).' >&2
  exit 1
fi
if [[ ! -f "$project_dir/worlds/metro_tunnel.sdf" ]]; then
  python3 "$project_dir/scripts/generate_tunnel.py"
fi
# Pass --server for no GUI, --software for Mesa software rendering.
args=()
for arg in "$@"; do
  case "$arg" in
    --server) args+=(-s --headless-rendering) ;;
    --software) export LIBGL_ALWAYS_SOFTWARE=1 ;;
    *) args+=("$arg") ;;
  esac
done
exec gz sim -r "$project_dir/worlds/metro_tunnel.sdf" "${args[@]}"
