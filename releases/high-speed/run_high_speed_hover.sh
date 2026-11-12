#!/usr/bin/env bash
set -eo pipefail
release=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "$release/../.." && pwd)
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$release/host/sanity_zenoh:$workspace/Sanity/src/ros2_sanity_ws/src/udp_broadcaster${PYTHONPATH:+:$PYTHONPATH}"
cd "$workspace"
exec "$workspace/.venv-sanity-deploy/bin/python" -m sanity_zenoh.indi_hover --loco-root "$workspace/loco_mjx" "$@"
