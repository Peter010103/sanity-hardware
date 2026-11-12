# MPC flight runbook

See the [latest butterfly update](latest-butterfly.md) for the additional six-eight
3D recipes, current paired evidence and scoped emergency-controller changes.

Run commands from the checkout root. All supplied top-level recipes default to
planning; the live flags are explicit. The scripts do not flash firmware, install
room services, automatically change FC gains, or select another aircraft when an
identity check fails.

## 1. Install on the flight workstation

The tested host is Ubuntu 24.04, Python 3.12 and ROS 2 Jazzy. Install ROS Jazzy with
`rclpy`, TF/geometry messages, the ROS bag CLI and MCAP storage plugin available.
The hardware-independent plan/solver tests do not need a running ROS graph.

```sh
git clone --single-branch --branch mpc-flight-experiments \
  git@github.com:Peter010103/sanity-hardware.git sanity-mpc-experiments
cd sanity-mpc-experiments
./scripts/setup.sh
./scripts/run_mpc.sh --list
./scripts/test.sh
```

Setup requires Git, a C/C++ compiler, CMake, Make and Python venv/pip. It creates
`.venv-sanity-deploy` with system packages so it can import the distribution's ROS
Python modules. `SANITY_SETUP_PYTHON` overrides `/usr/bin/python3`; use a Python
compatible with the installed ROS distribution. acados, HPIPM and BLASFEO are built
locally in `.deps/acados-v0.6.0`; CasADi is pinned to 3.7.2 and Zenoh to 1.10.1.
The Python tests use pytest 8 to avoid the older ROS launch-testing plugin's pytest
9 hook incompatibility; the test script also disables unrelated plugin autoload.

The first solver preparation generates and compiles a cached solver. Repeat
startup loads the matching cache. `SANITY_MPC_CACHE_DIR` changes the upright solver
cache; the pitch controller uses its own hash-keyed cache under the user cache
directory. A cache hit is not a check of hardware/frame correctness.

## 2. Prepare the equipment and local configuration

Use the FC/ESP wiring and firmware described in [hardware](hardware.md). The
baseline examples select receiver/body `9/sanity09` for the historical PID paths
and `15/sanity15` for the native PID/INDI comparison. They are examples tied to
specific recorded hardware, not a universal new-aircraft calibration.

```sh
cp config/lab.example.json config/lab.json
```

Edit the following fields for the current session:

| Field | Required meaning |
|---|---|
| `host_ip` | Workstation LAN address that ESP firmware connects to and Zenoh binds |
| `mocap_ntp_server` | Actual clock reference used by the ROS mocap source |
| `tf_topic`, `parent_frame` | Live ROS TF topic and world/map frame |
| `pid.device`, `indi.device` | Explicit receiver IDs for the selected aircraft |
| `tracked_frame` | Exact rigid-body TF child, e.g. `sanity15` |
| `expected_uid` | Actual native FC MSP UID; the native benchmark refuses a mismatch |
| `ground_m` | Current root height on the takeoff surface in the calibrated world frame |
| `world_from_map_xyzw` | Map-to-controller-world quaternion, XYZW, unit norm |
| `marker_to_base_xyz`, `marker_to_base_xyzw` | Measured marker/body transform, not a visual guess |
| `collective_profile` | Calibration matching the installed ESP conversion/calibration ID |
| `clock_endpoint` | Native receiver's observation clock endpoint, normally UDP 7461 |
| `pid_flip_marker_to_base_xyzw` | Flip-specific measured calibration; the included value is historical |

Local `config/lab.json` and generated `config/local/` are ignored by Git. The
launcher records the effective lab settings in each new run. For the historical
PID trajectory, `ground_m` must be measured/updated before flight. The pitch
mission captures the starting measured root Z for takeoff/landing, while its
reference bottom is an absolute world height. The modern benchmark refreshes its
ground from the current disabled bench immediately before flight.

For inspection without running a mission:

```sh
.venv-sanity-deploy/bin/python scripts/configure_lab.py \
  --lab config/lab.json --recipe indi-lemniscate-r2-3 \
  --output config/local/indi-preview.json
```

Do not use a stale ground height or the historical pitch quaternion merely because
it let another aircraft pass a gate. Physical mounting, marker orientation, motor
ordering, pole count, rate mapping and calibration identity must match the actual
selected hardware. None of the scripts substitutes another receiver ID.

## 3. Mocap, time and shared-room ownership

Provide a live ROS 2 `/tf` stream from the room's motion-capture system, with
correct source timestamps and the configured parent/child frame. A Motive/NatNet
or other vendor-to-ROS bridge is room infrastructure and must already be configured
for the actual system. The flight code consumes TF directly; it does not require
a learned-policy bridge or a separate ROS command publisher.

Keep the existing ROS domain/interface/QoS configuration appropriate to that TF
publisher. The flight subscriber uses best-effort TF input. Confirm the actual
rigid-body frames and inspect the complete path for visibility, including inverted
markers for flips, entry/exit regions and recovery paths. A stationary body being
visible does not establish complete path coverage.

Clock startup requires three fresh comparisons whose conservative absolute offset
bounds are below 1 ms. The wrappers only verify the selected NTP source; they do
not change system time services automatically. Arrange the workstation and mocap
source's clock synchronization before the session. The archived `sync_sanity_clock.py
--ensure` can change the host's time-service configuration and must only be used
as an intentional administrator action while missions are idle.

Before flight inspect both workstation and any acquisition/bridge hosts:

```sh
ps -eo user,pid,ppid,lstart,args
ss -lntup
```

Check for stale controllers, ROS nodes/daemons, Zenoh routers or listeners, NatNet
clients, pose bridges, bag recorders, sensor and preview servers. Identify ownership
before stopping unfamiliar processes; preserve other users' active experiments.
Stop your earlier task's processes gracefully and verify the relevant ports and
devices have been released. Do not use a broad name-based `pkill` in the shared room.

The top-level live launcher records a read-only inventory and uses an advisory
single-session lock. It tracks its own child processes, including new child process
groups, using a Linux subreaper and PID handles, and cleans up only those children.
It does not own an independently started room mocap bridge or daemon. Keep a host/
PID record for infrastructure you start, and stop it before handover unless someone
has explicitly requested continued operation.

## 4. Preview and prepare before a live session

```sh
./scripts/run_mpc.sh pid-circle-smoke --plan-only
./scripts/run_mpc.sh pid-circle-smoke --prepare-only
./scripts/run_mpc.sh pid-flip-1 --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-3 --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-4 --prepare-only
```

Plans print the actual reference, bounds and chosen model. They create a new
`outputs/` directory with `launch_plan.json` and a resolved local config, without
opening ROS, Zenoh, serial or aircraft interfaces. Preparation starts only the
isolated solver worker and closes/reaps it after warmup. It opens no command
transport. The high-rate and ordinary packages are separate so their command/
envelope rules cannot silently mix in one Python interpreter.

## 5. MPC + PID ordinary trajectory flight

Commission the PID aircraft/receiver with the matching ordinary CTBR calibration,
rate profile, source freshness and disabled/enable-zero handshake first. The
legacy trajectory path itself checks fresh mocap/receiver state, records MCAP,
warms the solver, then executes its existing takeoff/handover sequence.

```sh
./scripts/run_mpc.sh pid-circle-smoke \
  --lab-config config/lab.json --fly --frames-confirmed
```

Sequence: PD takeoff → PD reposition → gated MPC blend → timed circle/lemniscate →
PD braking/return/descent → low-height dwell → spool-down/disarm. A first Ctrl+C
requests PD landing and keeps recording. A subsequent interrupt escalates shutdown.
There is no automatic rearm.

The recorded faster recipes are explicit:

```sh
./scripts/run_mpc.sh pid-circle-1p5 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-lemniscate-1p5 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-circle-2p2 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-circle-3 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-circle-5 --lab-config config/lab.json --fly --frames-confirmed
```

Choose the intended recipe rather than editing a shared global default. `pid-circle-3`
selects the recorded 5 m/s² / 30° reference-planning envelope. `pid-circle-5` selects
the high-bank predictor/guard settings and 4 s smoothstep ramps. Physical reference
footprint is not sufficient emergency-braking space; see [results](results.md).

## 6. MPC + native INDI trajectory flight

The same native benchmark can select PID or INDI on the commissioned Sanity15
controller. Both require the expected UID, the native permission/status protocol,
RPM/observation health and the actual mode acknowledgement. `limits.pid_only=false`
in the common config does not mean PID is selected; `inner_controller` and the FC
acknowledgement determine the actual inner controller.

```sh
./scripts/run_mpc.sh indi-lemniscate-r2-3 \
  --lab-config config/lab.json --bench-check --frames-confirmed
./scripts/run_mpc.sh indi-lemniscate-r2-3 \
  --lab-config config/lab.json --fly --frames-confirmed
```

The flight invocation repeats its own disabled bench. It records at least three
seconds, requires a recent continuous stationary confirmed mocap window, derives
the current ground height, saves `configuration.json`, and then starts the finite
flight. A separate earlier bench directory is evidence, not a permanent readiness
token. Disabled bench commands cannot arm or issue positive flight output.

The normal INDI path requires actual full native authority before climb. Low-rate
pose/velocity input still comes from desktop mocap; onboard estimation is only
logged. Soft faults request the saved bounded landing behavior; stale/hard faults
retain their existing immediate-disable actions.

Useful variants:

```sh
./scripts/run_mpc.sh paired-pid-0p8 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh indi-lemniscate-1p5 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh indi-lemniscate-r2-1p5 --lab-config config/lab.json --fly --frames-confirmed
```

`indi-lemniscate-r2-4` uses the separate high-speed runtime, high-rate SZC6 mapping
and speed-specific model/envelope. The installed receiver must acknowledge the
matching capability while disabled. Merely raising an ordinary host rate cap cannot
provide it. The 5/6 m/s recipes are included for study and explicit experiments;
their evidence and recovery limitations are documented separately. They are not
promoted to successful physical recipes by passing a solver test.

## 7. PID-inner pitch flips

First qualify hover with the full-attitude controller and the flip-specific frame
calibration:

```sh
./scripts/run_mpc.sh pid-flip-1 --prepare-only
./scripts/run_mpc.sh pid-flip-1 --hover-only \
  --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-flip-1 \
  --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-flip-10 \
  --lab-config config/lab.json --fly --frames-confirmed
```

The path requires PID inner control and acknowledged SZC3 high-pitch capability,
not the INDI trajectory command selector. It gates handover and stable MPC hover
before scheduling run-in and loops. Allocate space for X about ±1.779 m around the
captured launch origin, Z about 0.475–1.740 m in the configured world frame, physical
aircraft radius, tracking error and recovery.

**Interrupt semantics differ from ordinary trajectories:** the first Ctrl+C asks
the existing pitch mission to finish the planned loop sequence, recover upright
and land. It may finish remaining requested loops. A second interrupt or SIGTERM
requests immediate disable. Do not expect upright PD landing while inverted.
The November INDI flip candidate is not substituted for these flown PID recipes.

## 8. Record, assess and release the room

Output layers are intentional:

| Path | Meaning |
|---|---|
| Top-level `launch_plan.json`, `local_config.*` | Named recipe, effective lab settings and exact executed command |
| Top-level `lab_process_inventory.json`, `owned_processes.json` | Pre-existing process inventory and task-owned PID/exit record |
| Legacy `flight/controller/` | Control/reference/history JSONL, worker diagnostics and shutdown receipt |
| Legacy `flight/mocap/` | Finalized raw TF MCAP bag |
| Pitch `flight/events.jsonl`, `flight/receipt.json` | Full-attitude control, raw/estimated TF, wire/status and stop outcome |
| Native `capture/bench/`, `capture/flight/` | Disabled bench and live event/native observation records |
| Native `capture/ground_reference.json`, `configuration.json`, `run_summary.json` | Current measured ground, actual flight configuration and outcome |

```sh
.venv-sanity-deploy/bin/python scripts/analyze_flight.py outputs/ACTUAL_RUN \
  --output outputs/ACTUAL_RUN_ANALYSIS
```

The analyzer accepts the top-level run or supported nested capture and produces
`metrics.json`, `tracking.csv`, `tracking.png` and `tracking.pdf`. It scores full
MPC authority and moving reference only, against the same tick's recorded causal
state. Original historical reports may use a slightly different moving predicate;
the output states its method and source hash. No blackbox decoder is needed for
this host tracking analysis. Detailed native tuning analysis additionally requires
a compatible trace/blackbox decoder supplied explicitly to the archived analysis tool.

Before handover, verify physical rotor stop, disarmed FC, disabled shutdown ACK,
closed/finalized recordings and no cleanup errors. Verify the task's solver,
recorders, ROS/controller children and any infrastructure you started have exited,
have not restarted, and released their devices/ports. The process ledger is evidence
of process exit, not a physical stop sensor. Preserve another user's active room
services. Remove motor power before touching the aircraft.
