#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash; fi
export PYTHONPATH="$root/Sanity/src/ros2_sanity_ws/src/sanity_zenoh:$root/Sanity/src/ros2_sanity_ws/src/udp_broadcaster:$root/scripts${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
"$root/.venv-sanity-deploy/bin/python" -m pytest -q \
  Sanity/src/ros2_sanity_ws/src/sanity_zenoh/test \
  loco_mjx/experiments/sanity_pitch_mpc/test_flight_prediction.py \
  loco_mjx/experiments/sanity_pitch_mpc/test_pitch_mpc.py tests "$@"

export PYTHONPATH="$root/releases/high-speed/host/sanity_zenoh:$root/Sanity/src/ros2_sanity_ws/src/udp_broadcaster:$root/scripts${PYTHONPATH:+:$PYTHONPATH}"
exec "$root/.venv-sanity-deploy/bin/python" -m pytest -q releases/high-speed/test "$@"
