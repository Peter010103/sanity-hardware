#!/usr/bin/env bash
# Install/build only. Never starts ROS, room services, serial clients or flights.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
python_bin=${SANITY_SETUP_PYTHON:-/usr/bin/python3}
"$python_bin" -m venv --system-site-packages .venv-sanity-deploy
.venv-sanity-deploy/bin/python -m pip install -r requirements-dev.txt
.venv-sanity-deploy/bin/python Sanity/src/ros2_sanity_ws/scripts/install_sanity_acados.py --python "$root/.venv-sanity-deploy/bin/python" --jobs 4
