#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  if ip -4 -o address show | rg -q ' 10\.0\.0\.200/'; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_robotlab.xml"
  fi
fi
exec /usr/bin/python3 -u "$SCRIPT_DIR/bifilar_inertia.py" "$@"
