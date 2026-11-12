#!/usr/bin/env bash
set -eo pipefail
ws=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(cd -- "$ws/../../.." && pwd)
for arg in "$@"; do
  if [[ "$arg" == --fly ]]; then source "$ws/scripts/sanity_clock_preflight.sh" "$@"; break; fi
done
source /opt/ros/jazzy/setup.bash
if [[ -f "$ws/install/setup.bash" ]]; then source "$ws/install/setup.bash"; fi
export PYTHONPATH="$ws/src/sanity_zenoh:$ws/src/udp_broadcaster${PYTHONPATH:+:$PYTHONPATH}"
cd "$workspace"
exec "$workspace/.venv-sanity-deploy/bin/python" -m sanity_zenoh.indi_hover \
  --config "$ws/src/sanity_zenoh/config/sanity15_full_indi_estimator_hover.json" \
  --loco-root "$workspace/loco_mjx" "$@"
