#!/usr/bin/env bash
# Build the independent official ArduPilot checkout inside this project.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
mkdir -p build .runtime/tmp
export CCACHE_DISABLE=1 TMPDIR="$task_root/.runtime/tmp"
export WAFLOCK="$task_root/build/apm-waf.lock"
export GIT_DIR="$task_root/dependencies/apm4.6.3/.git"
export GIT_WORK_TREE="$task_root/dependencies/apm4.6.3"
python3 dependencies/apm4.6.3/waf configure --top="$GIT_WORK_TREE" --out="$task_root/build/apm" --board=sitl --no-submodule-update
python3 dependencies/apm4.6.3/waf copter --top="$GIT_WORK_TREE" --out="$task_root/build/apm" -j4
unset GIT_DIR GIT_WORK_TREE
python3 - <<'PY'
import hashlib, json, subprocess
from pathlib import Path
root=Path.cwd(); source=root/'dependencies/apm4.6.3'
def git(*args):
    return subprocess.check_output(['git','-C',str(source),*args],text=True).strip()
file=root/'config/flight_provenance.json'
data=json.loads(file.read_text())
data['ardupilot']={'source':str(source),'repository':git('remote','get-url','origin'),
    'commit':git('rev-parse','HEAD'),'status':git('status','--porcelain')}
tags=git('tag','--points-at','HEAD').splitlines()
if tags: data['ardupilot']['tags']=tags
binary='build/apm/sitl/bin/arducopter'
data[binary]={'sha256':hashlib.sha256((root/binary).read_bytes()).hexdigest()}
file.write_text(json.dumps(data,indent=2)+'\n')
PY
cmake -S /home/qwer/gz_ws/src/ardupilot_gazebo -B build/gazebo -DCMAKE_BUILD_TYPE=Release
cmake --build build/gazebo --target ArduPilotPlugin -j2
gcc -Wall -Wextra -shared -fPIC scripts/gazebo_log_redirect.c -o build/libgazebo_log_redirect.so -ldl
python3 scripts/prepare_flight.py
set +u
source /opt/ros/humble/setup.bash
source "$task_root/lio_ws/install/setup.bash"
set -u
cmake --build lio_ws/build/fast_lio -j2
cmake --install lio_ws/build/fast_lio
