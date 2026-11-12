# Standalone package validation — 12 November 2026

Validation was performed without connecting to the aircraft, receiver, FC serial
port, mocap server or lab command listeners. No physical flight, firmware flash,
FC setting change or room-service startup was performed for this packaging task.

- A fresh single-branch clone installed the published requirements, rebuilt
  acados independently and passed the same610+85 tests (5 optional skips).
- A new Python 3.12 environment installed the published requirements.
- acados v0.6.0 was fetched at the pinned revision, with its pinned HPIPM/BLASFEO
  submodules, and built into this checkout. Its Python wrapper and Tera renderer
  were installed successfully.
- All 15 named recipes generated offline plans from an unrelated working directory.
- Actual isolated solver preparation passed for upright PID, fitted INDI, isolated
  high-speed INDI and full-attitude PID pitch MPC. Workers were closed/reaped.
- Host/model/mission/launcher/analysis suite: **610 passed, 5 skipped**. Optional
  training/fleet paths and optional environment-specific tests are outside this
  branch's single-aircraft runtime scope. Tests include native mocap C decoding,
  measured-state freshness, selected-command history, MPC deadline/failure behavior,
  mode acknowledgement, launch boost, reference derivatives and descendant cleanup.
- Separate isolated high-speed/SZC6 suite: **85 passed**, including 48 actual
  mission guard checks and real native-C receiver parsing/mapping cases.
- The complete FC patch applied to a fresh pinned official Betaflight checkout.
  Both native-capable and plain PID-reference HEX builds succeeded with the pinned
  GCC Arm 10.3.1 toolchain. No firmware was installed.
- Native current FC adapter tests passed in optimized and ASan/UBSan builds against
  the rebuilt source, including RPM/numeric/voltage/authority faults and isolated
  timing-transient recovery. Synthetic replay limits are engineering checks, not
  reconstruction of an unrecorded physical FC tick.
- The included host analyzer was exercised on original PID 5 m/s-reference,
  ten-loop PID flip, and successful INDI 3/4 m/s recordings. It generated source-
  hashed metrics/CSV and 2D PNG/PDF plots. No fitted temporal/bias alignment was used.

Re-run the software checks with:

```sh
./scripts/setup.sh
./scripts/test.sh
./scripts/run_mpc.sh pid-circle-smoke --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-3 --prepare-only
./scripts/run_mpc.sh indi-lemniscate-r2-4 --prepare-only
./scripts/run_mpc.sh pid-flip-1 --prepare-only
```

The receiver's complete ESP-IDF application was not rebuilt during this packaging
task; the matching clean SDK revision, sanitized configuration template, Pico
patches and builder are included. Native parser tests do not establish radio/UART
latency, high-rate airborne behavior or hardware compatibility.

Packaging changes are confined to this orphan branch: local path resolution,
explicit lab/collective configuration, read-only clock verification, a named recipe
interface, process ownership/cleanup and host tracking analysis. Controller models,
trajectory definitions, native guards and physical command bounds remain the
selected implementation snapshots. Tests that previously assumed the old workspace
now resolve included fixtures; stale assertions of an identical PID/INDI predictor
were updated to the explicitly different modern predictors.

Generated sources/libraries, caches, private configs, build images and full raw
flight logs are excluded from Git. The source manifest records original copied
bytes before adaptations; `PACKAGED_FILES.json` records the final committed payload
hashes (excluding that manifest itself). Historical source/configuration paths in
`evidence/` are provenance, not runtime dependencies.
