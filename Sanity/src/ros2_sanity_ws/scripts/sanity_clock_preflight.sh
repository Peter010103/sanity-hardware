#!/usr/bin/env bash
# Sourced by mission wrappers before ROS/Zenoh startup. Does not change DDS interfaces.
sanity_clock_preflight() {
  unset SANITY_CLOCK_PREFLIGHT_JSON
  local arg scripts receipt
  for arg in "$@"; do
    case "$arg" in
      --sim|--plan-only|--prepare-only|--control|--control=*|--help|-h) return 0 ;;
    esac
  done
  scripts=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
  receipt=$(mktemp /tmp/sanity_clock_preflight_XXXXXXXX.json)
  if ! /usr/bin/python3 "$scripts/sync_sanity_clock.py" --wait 45 --receipt "$receipt"; then
    rm -f -- "$receipt"
    echo 'Clock preflight failed; no mission controller or Zenoh connection was started.' >&2
    return 1
  fi
  SANITY_CLOCK_PREFLIGHT_JSON=$(cat -- "$receipt")
  export SANITY_CLOCK_PREFLIGHT_JSON
  rm -f -- "$receipt"
}
sanity_clock_preflight "$@"
