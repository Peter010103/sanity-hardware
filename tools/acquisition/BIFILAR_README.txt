Bifilar collection and analysis

Start with docs/inertia.md for the setup, latest masses/geometry, plots,
fixture correction, current URDF values and limitations.

From the repository root, reproduce the packaged dataset without ROS/hardware:
  python3 tools/inertia/analyze.py
Outputs: build/inertia/ (PNG/PDF figures, CSV traces and JSON diagnostics).
Add --update-results only when deliberately refreshing committed results.

Collect a new yaw trial with ROS 2 OptiTrack map -> RigidBody already running:
  bash tools/acquisition/run_bifilar.sh --collect-only --axis yaw --mass-g 87.38 --length-cm 64.2 --top-cm 15.6 --bottom-cm 13.2
Pitch/roll: remount the chosen drone axis vertically; change --axis and use
--bottom-cm 15.5. Allow the10-second countdown, gently twist5-10degrees at the
beep, then release without sideways pushing and leave untouched45seconds.
--countdown 15 --duration 60 allows a longer approach/recording.

A unique build/bifilar/sanity_bifilar_<timestamp>_<id>_<axis>/ directory contains
poses.csv, original config.json and capture_receipt.json. Collection-only with
no --mass-g leaves mass unconfirmed. Never apply old fixture subtraction to a
changed marker or mounting setup without updating reviewed inputs.

The launcher sources /opt/ros/jazzy/setup.bash and preserves existing ROS_DOMAIN_ID,
RMW_IMPLEMENTATION and DDS-profile environment settings. When using Fast DDS
with no supplied profile and this PC has10.0.0.200, it selects the included
DDS profile (mocap peer10.0.0.251). Other hosts should set their own DDS profile.
No network profile, FC configuration, arming or aircraft commands are changed.

Generic single-recording analysis, giving COMBINED suspension-axis inertia only:
  python3 tools/acquisition/bifilar_inertia.py --analyze /path/to/new/run --fit-start 10 --fit-end 20
It replaces derived fits/plots in that run; poses.csv is preserved. Do not run
it directly inside the curated raw-data archive. The batch inertia tool adds
fixture/marker and parallel-axis corrections for the packaged dataset.

Offline collector tests:
  python3 tools/acquisition/bifilar_inertia.py --self-test
