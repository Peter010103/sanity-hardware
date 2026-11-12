# Sanity thrust test stand

Standalone acquisition software and operating instructions for the Sanity
whole-aircraft thrust / yaw-reaction stand. This is the **`thrust-test-stand`
orphan branch** of [sanity-hardware](https://github.com/Peter010103/sanity-hardware):
it has its own root commit and needs no files or history from `main`.

The package includes the single-cell battery-powered bench logger, the later
three-HX711 / regulated-supply direct-motor and armed Acro collectors, calibration,
CSV analysis, native GPIO readers, example plans and offline tests. It does not
require ROS, Zenoh, NatNet or an active motion-capture session.

| Start here | Contents |
|---|---|
| [Connections](docs/wiring.md) | Jetson header pins, Grove signals, load-cell terminals, FC and PSU connections |
| [Operation](docs/operation.md) | Installation, controller identity, calibration, stopped probe, powered runs and cleanup |
| [Single-cell reference](docs/single-cell.md) | Battery-powered presets, data files, timing and stop behaviour |
| [Examples](examples/) | Low-command direct/Acro stages, stopped probe and historical calibration |
| [Validation](VALIDATION.md) | Offline checks and limits of this packaging validation |
| [Source manifest](SOURCE_MANIFEST.json) | Original source paths and hashes before packaging changes |
| [Licences](LICENSES.md) | MIT code, CC BY documentation/reference data and separate GPL PSU dependency |

## What is required

- Secured whole-drone stand, one axial load cell and (for the three-channel path)
  two opposing torque-reaction cells; accessible physical motor-power disconnect.
- NVIDIA P3768 / Tegra234 developer-kit carrier. The three-channel collector retains
  the campaign's eight-core Orin NX layout (CPUs 1/4, CPU7 governor path), temporary
  pinmux changes and real-time worker scheduling. Do not assume other carriers or
  JetPack versions are compatible. The single-cell native reader uses the P3768
  pin mapping without those three-channel scheduling requirements.
- Grove HX711 modules, 3.3 V digital supply, individual data/clock pairs. The
  three-channel collector requires high-rate hardware configuration on all ADCs.
- BETAFPVF405 flight controller with the inspected Betaflight **4.5.1 / MSP 1.46**,
  four DSHOT300 outputs and bidirectional RPM telemetry. Other firmware/boards
  are rejected by the existing profile checks.
- FC USB MSP connection, or an already configured transparent serial bridge that
  exposes the same MSP endpoint. The bridge firmware is not part of this package;
  direct FC USB is the simplest direct-mode connection. Armed Acro additionally
  needs a functioning RX_MSP path that permits arming, with the documented AUX map.
- For regulated runs: **Multicomp Pro 72-13350 V2.3**, its USB serial connection,
  and the verified rear **30 A** output terminals and suitably rated power leads.
  The flight battery must be disconnected. The runner retains the historical
  30 A source limit; this is not a verified rating for the fixture or aircraft.

All commands without `--execute` on the bench wrappers and full-range collectors
print plans without opening devices. `betaflight_msp.py` is a read-only live
inspection tool; `betaflight_motors.py` performs live read-only checks even in its
dry-run mode. The HX711 raw/calibration commands acquire live data immediately.

## Install and preview

On the acquisition Jetson, clone **this branch**, then run from the repository root:

```sh
git clone --single-branch --branch thrust-test-stand \
  git@github.com:Peter010103/sanity-hardware.git sanity-thrust-test-stand
cd sanity-thrust-test-stand
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
./scripts/setup-psu.sh
make native

python tools/run_repeated_steps.py
python tools/full_campaign.py examples/probe.json
python tools/full_campaign.py examples/direct-stage.json
python tools/full_acro.py examples/acro-stage.json
```

A C compiler and Linux GPIO headers are needed. Jetson.GPIO must be available to
this interpreter for the single-cell calibration/raw tools and pinmux helper;
`--system-site-packages` exposes an existing system installation. Install it using
the instructions for the actual Jetson/JetPack if missing. The native C backend
uses Linux GPIO character devices directly and does not link against libgpiod.

`setup-psu.sh` fetches tenma-serial at revision
`d1b73c25b61d308349acdfd8695041d0ea05ec09` and applies the included
[72-13350 patch](patches/tenma-72-13350.patch). This preserves the campaign's model
recognition, single-channel handling and current-setpoint precision. An unpatched
pip package is not an equivalent dependency. Acquisition imports the local
checkout; it does not install or overwrite a global PSU library.

Follow [operation](docs/operation.md) before adding `--execute`. Each checkout
needs its own inspected FC profile and reviewed calibration in `config/`.

## Measurements and limitations

MSP motor commands are integer software counts: **1000 is STOP**, 1001–2000 are
motor-test commands. They are not physical PWM widths or RC throttle. Direct
mode bypasses the armed controller; Acro mode includes RX_MSP processing,
controller, mixer and voltage compensation. Never substitute one input for the other.

The single-cell presets cap at 1450. The three-channel runner accepts explicit,
finite plans up to 2000, but the included examples use 1250 with a 1300 ceiling.
A low command is not a mechanical safety guarantee. Secure the propeller-equipped
rig, validate its load path and set the force envelope for the actual installation.

Every cell retains raw counts and its own stopped baseline. The two torque cells
still use the campaign's provisional shared scale **21036.44535 counts/N**;
independent torque calibration is not silently inferred from the thrust calibration.
The historical geometry assumes 60 mm cell spacing and 30 mm perpendicular arms.
Do not publish absolute yaw torque without checking calibration, sign, arms and
load paths. The regulated collector uses a relative thrust baseline and does not
apply the single-cell historical absolute-load-range guard after rewiring.

There is **no independent motor-test hardware watchdog** in this host path.
Ctrl+C and normal faults trigger best-effort stop/disarm and PSU-off cleanup;
USB/host failure, SIGKILL or power loss can prevent delivery. Keep the physical
power cut reachable, verify the rotors stop, and finish the shutdown procedure.

The packaging checks ran without lab devices or motor power. Historical campaign
code and wiring observations are retained as provenance; they do not certify the
current rig or the packaged version's powered behaviour.
