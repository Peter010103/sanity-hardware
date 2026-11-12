# Validation — 12 November 2026

This branch was packaged and checked on the workstation without connecting to
the Jetson, flight controller, PSU or load cells. No experiment, motor command,
GPIO change, lab service or background acquisition was started for this task.

Checks:

- Native C readers compiled with `-O2 -Wall -Wextra -Werror -shared -fPIC`.
- Python modules compiled successfully.
- Offline pytest suite: **71 passed, 4 subtests passed**, including simulated MSP,
  pseudo-terminal protocol exchanges, fake GPIO, calibration fitting, sensor/
  telemetry faults, best-effort stop/disarm, optional torque faults, command
  limits, read-only behaviour, standalone previews and current execution gates.
- PSU dependency fetched at the documented revision and the included patch
  applied successfully to a clean upstream checkout.
- Included stopped/direct/Acro plans previewed without device access.
- A fresh single-branch clone independently fetched/patched the PSU dependency,
  rebuilt both readers, passed all 71 tests and previewed every included plan.
- Relative documentation links and the orphan branch's complete tracked file
  list checked. The final branch has one root commit and no parent history.

Run the checks after installation:

```sh
. .venv/bin/activate
python -m pip install -r requirements-dev.txt
./scripts/setup-psu.sh  # once; refuses to overwrite an existing dependency
make native
make test
python -m compileall -q tools
```

The source manifest contains original hashes before packaging adaptations. The
standalone version removes hard-coded `/tmp` and Jetson home-directory imports,
uses local configuration/dependency paths, requires current operator declarations,
and holds the experiment lock across PSU setup, capture and cleanup. It also
keeps RC writes out of the read-only MSP query table when Acro is imported.
Legacy one-off campaign entrypoints were omitted; their classes are retained as
the dependencies of the full-range runners.

Compilation on this workstation does not validate Jetson GPIO timing, pinmux,
privilege setup, the 80 SPS modification, FC/RX_MSP arming, source behaviour,
mechanical ratings, calibration or a physical emergency stop. Rebuild on the
actual Jetson, then follow the stopped bring-up and operating procedure before
powered use. Existing historical observations in the wiring guide apply only to
those dated sessions.
