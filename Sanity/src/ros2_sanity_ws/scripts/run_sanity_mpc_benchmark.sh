#!/usr/bin/env bash
set -eo pipefail
scripts=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "$scripts/../../../.." && pwd)
cd "$workspace"
exec "$workspace/.venv-sanity-deploy/bin/python" "$scripts/run_sanity_mpc_benchmark.py" "$@"
