# Sanity MPC flight experiments

Run and study the Sanity trajectory-tracking and pitch-flip experiments with
**MPC + Betaflight PID** or **MPC + native rotational INDI**. This is the independent
`mpc-flight-experiments` orphan branch of
[sanity-hardware](https://github.com/Peter010103/sanity-hardware).

The branch contains the controller sources, trajectory generators, fitted models,
transport/receiver code, firmware build patch, named flight scripts, analysis tools
and operating instructions. It needs no files, actor checkpoints or training
checkout from the original workspace. Public dependencies are installed separately
at the versions described below. Live flights need the matching physical aircraft,
receiver firmware, ROS 2 mocap stream, network and calibrated frames.

| Read | Purpose |
|---|---|
| [Runbook](docs/flights.md) | Installation, local configuration, solver preparation, bench checks, flight commands, recording and shutdown |
| [Connections and firmware](docs/hardware.md) | FC/ESP wiring, protocols, controller selection and reproducible builds |
| [Controller design](docs/controllers.md) | Models, MPC costs/constraints, asynchronous workers, INDI prediction and flip reference |
| [Flight evidence](docs/results.md) | Successful flights, interrupted attempts, metrics and limitations |
| [Validation](VALIDATION.md) | Offline checks performed on this standalone package |
| [Source provenance](SOURCE_MANIFEST.json) | Original sources and their hashes before packaging adaptations |

## Latest update

The later **[3D butterfly flights and paired PID/INDI comparison](docs/latest-butterfly.md)**
are now included, with named `indi-butterfly-3/4/5` and `pid-butterfly-3/5` recipes,
the activated roll-model/braking releases, exact outcomes and the final report.
Both3m/s systems and the INDI4m/s run completed six eights; neither5m/s attempt completed.

## Flight recipes

All recipes default to **offline planning**. `--prepare-only` builds and warms the
actual solver without ROS, network or aircraft access. A physical mission requires
both `--fly --frames-confirmed` and an explicit local lab configuration.

```sh
./scripts/setup.sh
./scripts/run_mpc.sh --list
./scripts/run_mpc.sh pid-circle-smoke
./scripts/run_mpc.sh pid-circle-smoke --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-3 --plan-only
./scripts/run_mpc.sh pid-flip-1 --plan-only
```

`setup.sh` creates `.venv-sanity-deploy` with the system Python (Ubuntu 24.04 / ROS
Jazzy uses Python 3.12), installs dependencies and builds acados v0.6.0 at commit
`503364817c872d474ab5bed219c26760ac267769`. CMake, Make, a C/C++ compiler, Git,
Python venv/pip and network access are needed for setup and initial solver builds.
It starts no room services and flashes no hardware.

| Recipe | Reference | Evidence |
|---|---|---|
| `pid-circle-smoke` | 0.3 m/s, R0.5, 2 laps | Small commissioning reference |
| `pid-circle-1p5` | 1.5 m/s, R1, 5 laps | Completed physical flight |
| `pid-lemniscate-1p5` | 1.5 m/s, R3.2, 5 laps | Completed physical flight |
| `pid-circle-2p2` / `pid-circle-3` | 2.2 m/s R1.5 / 3 m/s R2, 5 laps | Completed physical flights |
| `pid-circle-5` | 5 m/s R2, 5 laps, 4 s smoothstep ramps | Two completed references; measured cruise about 4.28 m/s |
| `pid-flip-1` / `pid-flip-10` | Full-attitude run-in pitch loops | One-loop and two ten-loop physical flights |
| `paired-pid-0p8` | Native benchmark, PID selected on the same aircraft as INDI | Matched-comparison recipe |
| `indi-lemniscate-1p5` | R3.2, shaped angular schedule, 3 laps | Updated fitted predictor/reference; coverage must be checked |
| `indi-lemniscate-r2-1p5` / `…-3` | R2, 1.5 / 3 m/s, 3 laps | Completed physical flights on 12 November |
| `indi-lemniscate-r2-4` | R2, 4 m/s, 3 laps, SZC6 | Completed physical flight after an earlier envelope stop |
| `indi-lemniscate-r2-5` / `…-6` | R2, 5 / 6 m/s, SZC6 | Experimental: included 5 m/s attempts stopped; 6 m/s is prepared/simulated |

A reference speed is a requested trajectory parameter, not proof that the aircraft
reached it. See [results](docs/results.md) for measured speeds and run identifiers.
The November INDI flip preparation is documented as a candidate; the reproduced
physical flip recipes here use the successful **PID-inner** controller.

## Physical flight example

Read the [runbook](docs/flights.md) and [hardware requirements](docs/hardware.md),
then copy and edit the lab configuration for the actual equipment:

```sh
cp config/lab.example.json config/lab.json
# Edit host IP, mocap server/frame, receiver ID, FC UID, ground and calibration.

./scripts/run_mpc.sh indi-lemniscate-r2-3 --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-3 \
  --lab-config config/lab.json --bench-check --frames-confirmed
./scripts/run_mpc.sh indi-lemniscate-r2-3 \
  --lab-config config/lab.json --fly --frames-confirmed
```

The native benchmark launcher records a disabled bench, confirms current receiver/
FC health, measures fresh stationary ground and only then starts the requested
flight. The historical PID trajectory/flip paths retain their own handshake and
mission sequencing. Their separate commissioning steps are in the runbook.

Every invocation creates a new directory under `outputs/`. Use `--output NEW_DIR`
to choose one. The top-level `launch_plan.json` records the exact command, recipe
and effective lab configuration. Live runs record process ownership; only children
started by that invocation are cleaned up. Existing lab infrastructure is preserved.

```sh
.venv-sanity-deploy/bin/python scripts/analyze_flight.py outputs/ACTUAL_RUN \
  --output outputs/ACTUAL_RUN_ANALYSIS
./scripts/test.sh
.venv-sanity-deploy/bin/python scripts/verify_package.py
```

The analyzer writes same-time tracking metrics, CSV and position/velocity PNG/PDF
plots for the historical and newer log formats. It does not fit away delay or bias.

## Source layout

The original relative source layout is retained so the tested runtime paths remain
recognizable. The short `scripts/` entrypoints provide the standalone interface.

- `Sanity/src/ros2_sanity_ws/src/sanity_zenoh/`: flight supervisors, state estimation,
  worker IPC, native command transport, models and regression tests.
- `loco_mjx/experiments/sanity_ctbr_sim2sim/`: upright CTBR MPC, INDI rate prediction,
  disturbance model, ordinary trajectories and PD takeoff/landing control.
- `loco_mjx/experiments/sanity_pitch_mpc/`: full-attitude pitch MPC and run-in reference.
- `releases/high-speed/`: isolated 4–6 m/s host/solver/envelope and SZC6 changes.
- `firmware/receiver/`: ESP32-S3 receiver source and pinned Pico build helpers.
- `firmware/fc/`: complete patch against pinned Betaflight 4.5.1, build script and
  reference settings; firmware installation is a separate operator action.
- `evidence/`, `media/`: curated flight receipts, exact historical plans and figures.

The host controls from desktop mocap. The optional onboard ESKF is logged and is
not the position-control state source. Older fleet/RL/simulator notes in
`docs/archive/` are historical context; use this branch's runbook for supported commands.

[Licences and attribution](LICENSES.md).
