# Coordinated Sanity pitch flips

Run from `/home/hw527/Projects/drones`. This is an experimental full-attitude MPC
extension; the existing hover/line/policy runners remain separate. Default is
listing only. The command never starts a physical mission without `--fly` and
`--frames-confirmed`.

```bash
# List Sanity body frames and registered receiver identities.
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh --list

# Native loco_mjx/MJX simulation: takeoff, one common-clock flip, recovery, landing.
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh --sim --laps 1 --episodes 2

# Explicit subset, independent of which real bodies are currently visible.
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --sim --drones sanity09 sanity18 --laps 1 --voltage 7.1

# Software-only asymmetric packet-arrival pause; publisher/controller keep running.
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --sim --stall-device sanity22 --stall-ms 128 --laps 1
```

`--sim` uses currently visible registered Sanity `/tf` child frames when available.
With no live bodies, or an explicit `--drones` selection, it uses registered
identities. Poses are synthetic parallel lanes, not captured arena placements.
The printed/saved plan states the membership and placement source. The independent
MJX worlds share the reference clock/world placement but do not simulate inter-drone
contacts or aerodynamics. No Zenoh command session, serial interface or arming exists
in the physics subprocess. Discovery, when used, is read-only ROS input.

The registry is `config/fleet_hover_registry.yaml`. Receiver ID equals the body-name
suffix; `sanity09` uses09/.109, with no01 fallback. Unknown/uncommissioned bodies
are listed and excluded. An explicitly requested body must be registered. The
script defaults to one flip per aircraft; `--laps`, `--bottom`, `--direction`,
`--voltage`, `--episodes`, `--seed`, timing/model stress flags and simulation-only
`--lane-spacing` are available in `--help`.

## Controller and sequence

Each body has its own existing `PitchMission`, fitted-rate observer, force/selected-
command history, PD state and `PitchWorker` acados solver. This is the full-attitude
pitch controller, not the upright tilt-constrained hover MPC. It uses the existing
69.85g shared pitch model and measured small-rate response fit, collective dynamics,
run-in reference and command adapter. It does not inherit the upright MPC's new XY
bias observer. Models/calibrations are inherited assumptions for other aircraft,
not newly identified per-drone dynamics. No outgoing CTBR slew/filter is added.

All aircraft take off together to0.5m, move behind their own loop centre, and hand
from PD to MPC independently. After every MPC hover gate passes, the group schedules
one future monotonic epoch0.5s ahead. All previews use that epoch; no state/history
is reset and no aircraft silently starts early or joins later. The sequence is level
run-in at2.4m/s, C4 pitch entry, one elliptical pitch loop, exit, level braking,
upright hover recovery, then PD landing. Nominal reference X extent is about
−1.779..+1.779m relative to each takeoff centre, height0.475..1.740m. Existing hard
height2m and horizontal2m limits remain by default, as do25ms solver/40ms freshness/
1s lease checks. The optional horizontal/corridor bypasses are described below. The group-ready hold can wait45s, while each mission retains its120s bound.

SZC3 wire mapping uses thrust1e-4N and rates1e-3rad/s, linear600deg/s RC scaling,
with pitch authority9.5rad/s. Each receiver must acknowledge SZC3 while disabled,
report fresh FC/matching calibration/ownership, and complete an enabled-zero
handshake before positive commands. The existing ordinary SZC2 interface remains
unchanged. The fleet MJX adapter explicitly uses the correct SZC3 rate precision;
older single-aircraft simulator callers keep their previous default wire scale.

### Optional countdown readiness bypass

`--skip-prestart-readiness-check` bypasses the group landing triggered by losing a
hover-readiness criterion after the common start time has been scheduled. All
aircraft must still satisfy the initial readiness barrier and stable dwell before
a countdown is scheduled. With the flag, a brief angular-rate/other hover-gate
excursion during that countdown leaves the scheduled common start unchanged.
The first such excursion is logged as `prestart_readiness_loss_bypassed`, with
the aircraft and blocking gates. The selected mode is saved in the mission plan.

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --fly --frames-confirmed --laps 1 --bottom 0.5 \
  --skip-swept-path-check --skip-prestart-readiness-check
```

Both bypasses are optional and off by default. The countdown flag does not disable
explicit landing requests, participant failures, state/MPC/command/link freshness,
contact-distance disarm or hard flight limits. It does not change selected CTBR
commands or reset controller/history state. It also applies to `--sim`.

### Optional path-corridor and horizontal-boundary bypasses

`--skip-path-corridor-check` disables the group landing requested when a drone
moves more than 0.20 m from its full path sweep. The first crossing for each drone
is still recorded as `path_corridor_exceeded_bypassed`, with its distance/threshold.

`--skip-horizontal-limit` disables the 2 m horizontal distance stop from takeoff
in PD positioning, MPC control and PD landing. Horizontal distance and bypass mode
are recorded in each drone's control log. This changes the distance guard, not the
MPC objective, selected CTBR outputs or controller/history state.

To use all four opt-in bypasses discussed for the latest runs:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --fly --frames-confirmed --laps 1 --bottom 0.5 \
  --skip-swept-path-check --skip-prestart-readiness-check \
  --skip-path-corridor-check --skip-horizontal-limit
```

All flags default off and are saved in the mission plan; `--sim` uses the same
settings. The altitude ceiling/floor, freshness, receiver interlock, contact-distance
disarm and operator emergency stop remain active. The initial position/command-match
readiness gates and 1.5 m MPC reference tracking-error stop also remain active.
These flags do not force a flip when PD has not reached the required handover state.

## Space and failure handling

Hover spacing alone is insufficient. Before physical arming, fresh stationary homes
are captured and the full takeoff/transfer/run-in/flip/return/landing sweeps checked.
The initial implementation uses parallel world X-Z planes. Full disjoint path tubes
reserve0.15m body radius plus0.20m tracking allowance per drone and0.02m sampling
allowance: minimum swept-centre distance must exceed0.72m. Thus two nearby homes
along X may be invalid even if their hover points were separated. No automatic move
into rows or unrequested layout change is made. Synthetic six-aircraft lanes are
1.2m apart and span6m across Y; that is not verification of space in your arena.

By default this conservative swept-path test covers any relative phase within the
declared tubes. `--minimum-path-separation` can reduce the required centre-path
clearance (see below). It does not assume simultaneous radio arrival. Runtime
corridor/separation checks request group recovery/landing, and direct contact-distance
violation disarms.
These checks do not guarantee collision avoidance after arbitrary failures or a
lost aircraft leaving its tube. Normal MPC recovery continues during an active
maneuver before PD is used; there is no inverted PD fallback. A member losing
state/MPC/link freshness stops immediately; healthy members finish the existing
planned maneuver/recovery then land. No lost-link controlled-landing guarantee.

First Ctrl+C during flight requests group landing after the planned sequence reaches
upright recovery. Before takeoff it cancels startup. Second Ctrl+C or SIGTERM
immediately disarms. The operator socket bypasses discovery:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --control RUN_DIR --action emergency-stop
```

## Physical entry point

After aligning all selected body frames (x-forward,y-left,z-up), verifying the full
path space and commissioning the receivers, inspect the current layout first:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh --plan-only --laps 1
```

Physical command (not exercised in this implementation task):

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --fly --frames-confirmed --laps 1 --bottom 0.5
```

### Closer starting layouts

To relax the starting-layout check, set the minimum clearance between the **full
swept centre paths**. For example, require more than 0.50 m instead of the default
0.72 m:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --fly --frames-confirmed --laps 1 --bottom 0.5 \
  --minimum-path-separation 0.50
```

This uses the captured `/tf` starting positions and still checks the entire
takeoff/run-in/flip/recovery/landing sweep. It does not move drones into a row.
Homes separated along X can still fail when the run-in/return paths overlap.
Preview the current layout without opening a command link:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --plan-only --minimum-path-separation 0.50
```

Smaller clearance reduces the tracking-error reserve. At 0.50 m, only 0.09 m per
drone is reserved after the 0.15 m body radius and 0.02 m sampling allowance.
The unchanged runtime corridor remains 0.20 m, so those wider runtime tracking
bands can overlap; the reduced setting does not guarantee collision avoidance.
The lower configurable bound is 0.32 m (airframe diameter plus sampling allowance,
with zero tracking-error reserve). Intersecting paths remain rejected when the
swept-path gate is enabled. Handover,
freshness, controller outputs, flight limits and contact-distance disarm retain
their settings. The chosen clearance, default, per-pair distances and reserved
margins are saved in the plan and runtime/simulation reports.

Software geometry tests exercise 0.60 m parallel lanes: rejected by the default,
accepted with 0.50 m clearance. This is not a flight validation at that spacing.

### Skip swept-path overlap rejection

To bypass this specific preflight gate for a run:

```bash
Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh \
  --fly --frames-confirmed --laps 1 --bottom 0.5 \
  --skip-swept-path-check
```

The flag is off by default. It bypasses pairwise swept-path overlap rejection in
planning and runtime revalidation (and in `--sim`). Distances, failed clearance
comparisons and the bypass flag are still recorded; the console reports the gate
as skipped. `--plan-only --skip-swept-path-check` previews an overlapping layout
without a command connection. By default the runtime 0.20 m individual path corridor,
0.30 m contact-distance emergency disarm, freshness, handover and flight-envelope
checks remain active. Controller outputs are unchanged. With this gate skipped,
intersecting nominal paths are allowed and no collision-free layout is established.

It preserves the existing NTP/DDS/interface setup, records raw `/tf` once to MCAP,
uses one shared UDP control/TCP diagnostic transport, and keeps per-drone control/
transmit timers independent of inference. The isolated mocap worker uses the same
pitch10ms pose/20ms velocity/50ms angular-rate filters. It retains each registry
marker transform; it does not copy the old Sanity09-only ground-tilt calibration.

Each experiment contains source/config hashes, membership/homes, common timing,
raw MCAP, selected-command histories, state/reference/command/MPC input/result logs,
receiver/FC events, individual assessments and shutdown acknowledgements. Unix
stamps remain for video alignment. Failed and aborted missions cannot pass solely
because landing/disarm completed.

## Validation boundary

Native MJX results and plots: `outputs/sanity_fleet_pitch_20261026/`. Tests verify
barrier/clock behavior, independent histories, pre-start cancellation, no inverted
PD fallback, complete sweeps and simulation/hardware separation. A separate real
ROS/200Hz TF/MCAP test uses mocked command interfaces and stalled shadow inference;
it validates timers/startup/manual landing, not flight dynamics or real radios.

High-rate dynamics remain an extrapolation of small upright flight fits. Simulated
success does not establish simultaneous real-radio/power reliability, inverted
mocap visibility, aerodynamic interactions or physical high-rate authority.


Current evidence:30/30 matched-model flips across5 six-aircraft episodes,including
7.1V andone requested128ms arrival pause,all followed by recovery/landing. The
50Hz pause held six ticks(120ms),with140ms between new commands. MotionRMSE
1.343–1.433cm,maxrootheight1.7293m,minseparation1.1874m;86 regressions pass.
The final causal-log episode distinguishes the prior MPC candidate used for
selection from the new request/response submitted that tick. `command` is selected
CTBR; `applied` is after explicit queue/stall but before the modeled firmware/RC
conversion. These are not measured FC thrusts. Registry hover regions describe
the previous hover experiment;flip clearance is assessed from the displayed full
sweeps and the pitch controller's2m root-height/horizontal envelope.
