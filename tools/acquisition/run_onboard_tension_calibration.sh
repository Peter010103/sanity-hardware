#!/usr/bin/env bash
# Interactive bench calibration. Never issues aircraft commands.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_root="$(cd -- "$script_dir/../../../.." && pwd)"
base_default_g="${SANITY_HANGING_BASE_MASS_G:-77.03}"
read -r -p "Drone weight including everything hanging from the mount (g) [$base_default_g]: " base_g
base_g="${base_g:-$base_default_g}"
printf 'Sanity09 hanging-drone calibration; base mass = %s g.\n' "$base_g"
printf 'Keep motors unpowered; start with the drone fully supported (sensor unloaded).\n'
printf 'Enter measured added totals. You will LOAD them in order, then UNLOAD them in reverse.\n'
read -r -p 'Ascending added totals in g, starting at 0 (e.g. 0,5,10,15,20): ' added_g
output="$task_root/outputs/strain_gauge/tension_calibration_$(date +%Y%m%d_%H%M%S_%N)"
exec python3 "$script_dir/calibrate_onboard_tension.py" calibrate \
  --quick --base-mass-g "$base_g" --added-g "$added_g" \
  --sensor-id sanity09_printed_mount_01 \
  --notes "350ohm taped gauge; 3x352ohm completion; fixed mount, hanging drone; measured base${base_g}g" \
  --output "$output"
