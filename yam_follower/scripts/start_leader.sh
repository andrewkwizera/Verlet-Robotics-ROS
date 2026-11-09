#!/usr/bin/env bash
# Role-specific startup; defaults to READ-ONLY. Additional launch args are passed unchanged.
set -eo pipefail
cd "$(dirname "$(realpath "$0")")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/nuc_network_env.sh
exec ros2 launch yam_bringup yam_leader_nuc.launch.py "$@"
