#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "${SCRIPT_DIR}/../.." && pwd)

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_ROOT}"

colcon build --symlink-install "$@"
