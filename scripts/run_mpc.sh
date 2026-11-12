#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root/.venv-sanity-deploy/bin/python" "$root/scripts/run_mpc.py" "$@"
