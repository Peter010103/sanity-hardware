#!/usr/bin/env bash
set -eo pipefail
scripts=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$scripts/run_sanity_mpc_benchmark.sh" "$@"
