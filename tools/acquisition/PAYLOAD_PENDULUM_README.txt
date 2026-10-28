Suspended payload pendulum recording — scripts prepared24October2026

These scripts were written for the user to run; they were not executed by the
assistant. Source inspection only, no synthetic/ROS/hardware tests performed.

Confirmed setup:
  rigid body45g_mass, origin approximately at cube geometric center;
  payload+hook+markers44.73g; cable1.34g; combined suspended mass46.07g;
  pivot-to-hook cable length525mm for this fixed-support experiment;
  cube45mm per side; hook55mm above cube base;
  nominal hook-to-COM32.5mm, fixed pivot557.5mm above the initial cube center.
The mocap-origin offset and true printed-part COM/inertia are approximate.
A uniform-cube inertia is only a labelled nominal proxy in the analysis.

The flight attachment is under the battery. Its body-relative3D offset remains
unmeasured. This fixed-support script does not infer that aircraft offset.
A hovering aircraft has a MOVING pivot and requires a different analysis using
its pose and measured attachment transform; do not analyze flight as fixed support.

From /home/hw527/Projects/drones/Sanity/sanity-hardware:

1) Fixed support, X-direction release:
  tools/acquisition/run_payload_pendulum.sh record --axis x \
    --output ../../outputs/payload_pendulum_x_01

2) Fixed support, Y-direction release:
  tools/acquisition/run_payload_pendulum.sh record --axis y \
    --output ../../outputs/payload_pendulum_y_01

The output directory must be new. Repeat with _02 etc. when needed.
Recording opens ROS subscriptions only; no serial/Zenoh/control interface.
It uses ROS Jazzy, domain0 by default, /tf and parent map. Existing environment
ROS_DOMAIN_ID/RMW/DDS overrides are respected. --parent and --body override frame
names. System Python needs NumPy/SciPy, as in the existing bifilar recorder;
offline analysis additionally needs Matplotlib. No packages are installed.

Operator sequence:
  Wait for tracking; leave the mass hanging still and press Enter.
  Three seconds of rest are recorded to infer the fixed support position.
  Displace5–10degrees in the labelled direction and hold it; press Enter.
  During the ten-second countdown keep it held. At RELEASE NOW/the beep,
  release without pushing. Leave it untouched for20seconds.
  Ctrl-C retains partial data and records an incomplete-capture receipt.

3) Analyze each recording yourself:
  tools/acquisition/run_payload_pendulum.sh analyze ../../outputs/payload_pendulum_x_01
  tools/acquisition/run_payload_pendulum.sh analyze ../../outputs/payload_pendulum_y_01

The analysis directory must be new too; for a revised window/offset use:
  tools/acquisition/run_payload_pendulum.sh analyze ../../outputs/payload_pendulum_x_01 \
    --output ../../outputs/payload_pendulum_x_01/analysis_v2 \
    --start-after-release 1.0 --axis auto

The raw positions/quaternions are saved without smoothing by this recorder.
Source timestamps determine frequency; host timestamps mark the operator cue.
The cue is not the exact physical release instant. Default analysis starts0.5s
later; change it if a hand remains on the mass longer.

Coordinate geometry:
World Z is assumed up. At rest, the cube COM is nominally its geometric center
and the hook is vertically32.5mm above it. Initial rest orientation converts
that vector into a fixed rigid-body lever arm, so arbitrary mocap axis alignment
is allowed. Later orientations rotate the hook offset correctly.
The fixed pivot is inferred once from initial rest, never updated from moving
samples. --origin-to-center-mm DX DY DZ is an optional body-frame correction
from mocap origin to cube center; zero is the current uncalibrated assumption.

Files:
  parameters.json: measured values, assumptions, missing battery attachment,
    recorder/fit-source hashes;
  poses.csv: raw selected TF poses, source and host timestamps, experiment phase;
  events.jsonl: rest/release/finish cues;
  capture_receipt.json: completion, tracking counts/drops and errors;
  analysis/result.json: fit period/decay, conditional damping, geometry residuals;
  analysis/derived.csv: cable angles, hook-length residual, approximate COMs;
  analysis/pendulum_review.pdf and .png: measured motion and fit diagnostics.

The shared, existing bifilar fit routine is reused for a damped angular signal;
no bifilar spacing/stiffness/inertia formula is used here. Failed model-fit
criteria are retained in result.json. A single-mode fitted damping is only a
conditional estimate, particularly when the cube rotates relative to the cable.
The scripts never write MPC parameters, flight configuration or calibration.
