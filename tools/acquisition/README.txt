SANITY BENCH EXPERIMENTS — LOGGER v2

Installed location: ~/sanity_thrust_test on unitree@192.168.123.15.
Run from that directory. Acquisition uses Python's standard library and the
existing MSP, HX711 native reader, libhx711.so, fc_profile.json, and
loadcell_calibration/calibration.json. No new acquisition packages are needed.

Purpose
-------
Collect better resolved command-to-reported-RPM dynamics, repeated up/down
edges, held-out sequences, mounted force checks, and (conditionally) individual
rotor static data. These tests do not identify pure motor delay separately from
USB, FC processing, ESC response, and telemetry delay. They do not measure yaw
reaction torque or aircraft inertia.

Each powered script is a separate finite test. Use a freshly charged, healthy
battery for each, as planned; give its actual identity and charge cycle a unique
--battery-id. Changing batteries changes pack identity as well as voltage, so
record capacity/type/charge state with --note where useful. No script discharges
a battery deliberately. Do not run the scripts simultaneously.

QUICK START — exact commands
----------------------------
cd ~/sanity_thrust_test

Any script without --execute only prints its plan and opens NO devices:
python3 run_repeated_steps.py

1. Read-only acquisition check (no motor commands, even STOP):
python3 probe_acquisition.py --execute

2. Main repeatability/direction test, battery B01:
python3 run_repeated_steps.py --battery-id B01 --execute --bench-ready

3. Smaller-step test, battery B02:
python3 run_small_steps.py --battery-id B02 --execute --bench-ready

4. Held-out multilevel test, battery B03:
python3 run_multilevel.py --battery-id B03 --execute --bench-ready

5. Individual motors, battery B04 — only after the fixture check below:
python3 run_individual_motors.py --battery-id B04 --execute --bench-ready --off-axis-checked

Optional: test just M3 on its own battery instead of the full four-motor sequence:
python3 run_individual_motors.py --motor 3 --battery-id B04_M3 --execute --bench-ready --off-axis-checked

6. Short reference pair, battery B05:
python3 run_reference_pair.py --battery-id B05 --note "fresh pack reference" --execute --bench-ready

7. Mounted scale check: physically disconnect motor battery; have 100 g ready:
python3 run_weight_check.py --execute --battery-disconnected --mass-g 100 --position centre

The battery labels above are examples; replace them with your own labels.
If time is limited: acquisition check, mounted weight check, repeatability,
small steps, multilevel, then individual rotors if the fixture qualifies.
The short reference is useful for checking reproducibility across packs/sessions.

What the powered commands do
---------------------------
After device/profile checks, every powered run records 2.5 seconds stopped,
establishes a load baseline, executes its sequence AUTOMATICALLY, commands STOP,
records 2.5 seconds stopped, and attempts STOP three more times during cleanup.
There is no manual Enter prompt during powered sequences. Leave the rig alone.

run_repeated_steps.py:
  1250 warmup for 2.5 s, then four 1250 <-> 1450 up/down pairs.
  High/low dwells: 1.57/1.83, 1.69/1.91, 1.76/1.63, 1.88/1.72 seconds.
  Different dwell lengths help avoid repeatedly sampling the same sensor phase.
  First three pairs are identification data; reserve the last pair for validation.
  Nominal capture 21.49 s plus startup/transaction overhead.

run_small_steps.py:
  1300 warmup for 2.5 s, then three 1300 <-> 1400 pairs using the first six
  dwells above. Last pair is held out. Nominal capture 17.89 s plus overhead.
  Compare direction and amplitude dependence with the main repeated-step test.

run_multilevel.py:
  1350 -> 1250 -> 1350 -> 1450 -> 1350 -> 1300 -> 1350 -> 1400 -> 1350.
  Every level lasts 2 s. Nominal capture 23 s plus overhead.
  Reserve this sequence to evaluate predictions from a model fitted elsewhere.
  It is a short deterministic multilevel sequence, not a broadband PRBS test.

run_individual_motors.py:
  In hardware-output order M1, M2, M3, M4, run only that motor at
  1250 -> 1350 -> 1450, each for 2 s. Other outputs stay at STOP (1000).
  Stop all motors for 2 s between motors and require zero RPM before the next.
  Repeat M1 at 1350 for 2 s at the end to expose drift/order effects.
  Nominal capture 39 s plus overhead. --motor N selects only one motor (13 s).
  These are direct hardware output numbers, not unverified front/rear positions.

  --off-axis-checked asserts that the mounting restrains single-rotor roll/pitch
  moments AND the load cell measures the intended axial force with an off-centre
  force. Check the load path using a known force applied along the measurement
  axis at the rotor locations (battery disconnected). The weight-check script
  can record each location via --position M1, etc., where mechanically suitable.
  Merely placing a weight anywhere on the platform does not establish this.
  If the fixture fails this check, skip individual thrust claims; collective
  force and individual RPM remain the supported measurements from other tests.
  Single-rotor results describe the mounted configuration and may differ from
  all-rotor operation due to airflow and interactions.

run_reference_pair.py:
  1250 for 2.5 s -> 1450 for 2 s -> 1250 for 2 s.
  Nominal capture 11.5 s plus overhead. Compare RPM, force and measured voltage
  with the same levels in other runs. A fresh pack alone does not provide a
  controlled voltage-dependence experiment; it does improve repeatability checks.

run_weight_check.py:
  Never opens the FC serial port or sends motor commands.
  Records 3 s baseline, then three manual add/remove cycles of the known weight.
  After each action, remove your hand and press Enter; record 3 s settled data.
  Reports the measured change against m*9.80665 N/kg and zero-return drift.
  q or Ctrl+C exits. Keep the fixture and load path the same as in thrust tests.
  Enter is an operator marker, NOT the exact instant the force was applied.
  This checks mounted scale/zero return; it does not by itself identify a precise
  HX711 delay or motor response. The full native force time series is retained.

probe_acquisition.py:
  Five seconds of HX711 and read-only RPM/battery/status polling with stopped
  outputs and zero reported RPM. --seconds changes duration (maximum 30 s).
  Use this first to check wiring, permissions, calibration and actual throughput.
  All three timed channels and raw payloads are stored as in powered tests.
  A successful stopped probe cannot establish telemetry freshness while spinning.

Stopping and limits
-------------------
--bench-ready means propellers/configuration are prepared for a secured thrust
bench test and a physical battery disconnect is reachable. Close Betaflight
Configurator, old bench scripts and other FC/HX711 clients before starting.
The new tools use an advisory lock with each other; older tools do not share it.

Ctrl+C, SIGTERM, terminal hangup, detected sensor/serial/logging errors, force
cutoff, loss of expected RPM, or battery warning all lead to STOP attempts.
STOP is 1000 in this FC's MSP motor-test interface. It is not an arming command
or a literal zero-valued motor packet. Commands are sent at transitions only.
Each STOP attempt awaits an acknowledgement; a received ACK is not proof of
physical rotor stop. Verify the rotors have stopped before approaching the rig.

There is NO independent hardware watchdog in this path. Host freeze, SIGKILL,
power loss or USB failure can leave motor-test output active. Python cleanup
cannot guarantee delivery in those cases. Use the physical battery disconnect.

All presets cap commands at 1450; they never approach the previous 1750 test.
Default cutoff is absolute load change >2.0 N from the stopped baseline.
The full calibrated load must also remain inside the measured calibration range.
This cutoff is a conservative run setting, not a certified fixture/load rating.
Default thrust sign is +1, matching the previous mounted tests. --thrust-sign -1
changes reported thrust direction without weakening the absolute force cutoff.

Require profile match, four motors, DShot telemetry, disarmed FC, stopped outputs
and zero RPM before starting. Abort for missing/invalid force, HX711 timing fault,
force data older than 0.35 s, battery/status data older than 0.45 s, commanded
motor RPM still zero after 1 s, FC battery warning or unknown/armed FC status.
Battery and armed-status queries each target 5 Hz; blocking transactions and
host scheduling mean these are software checks, not hard real-time guarantees.
FC voltage/current meter calibration and the physical motor pole count remain
independently unverified. A battery marked OK by the FC is not a capacity test.

Data and timestamps
-------------------
Every execution creates a NEW bench_<preset>_<UTC timestamp> directory in the
current working directory. Existing directories are never overwritten.
Use --output PATH to choose another fresh directory.

  metadata.json: full plan, battery label/note, FC profile, source hashes,
                 units, clock conventions, limits and operator declarations.
  calibration_reference.json: exact calibration used for that run.
  commands.csv: command vectors, host send-start and ACK timestamps.
  phases.csv: phase names, fit/validation roles, completed flag and boundaries.
  rpm.csv: per-motor FC RPM, derived omega, invalid-telemetry fraction,
           ESC auxiliary values when provided, raw MSP payload, request/reply time.
  force.csv: counts, calibrated load, stopped-baseline change, signed force,
             native read/ready timestamps, clock-pulse bound and validity.
  battery.csv: voltage, current, mAh, cell count and battery state with timestamps.
  status.csv: disarmed/armed status and raw payload with timestamps.
  weight_checks.csv: reference-weight deltas, errors and zero-return shifts.
  events.jsonl: baseline, errors, manual markers, best-effort cleanup STOP records.
  summary.json: completion/error state, row counts, baseline drift and stop ACKs.

HX711 runs in a separate worker, so slow force conversions do not gate RPM reads.
All channels use the same host CLOCK_MONOTONIC nanoseconds. Force ready-observed
time is not the instant the physical force changed. RPM request/reply times bound
the host exchange, not the FC's internal sampling instant. Data may be cached.
Raw independent channels are kept rather than assigning them one shared row time.
Requested RPM polling is 60 Hz; actual rate is limited by USB/MSP and safety
queries. No inference of sub-sample motor delay is justified by dense requests.

The FC reports mechanical RPM using its configured pole count (currently 12).
omega_reported_rad_s = reported_RPM * 2*pi/60.
erpm_derived = reported_RPM * configured_poles/2; this is reconstructed, not a
second independent speed measurement. If physical poles differ, the mechanical
scale and derived thrust coefficient need correction. Zero ESC voltage/current/
temperature fields do not establish functioning sensors for those quantities.

Collective axial force is the full aircraft's measured force, never automatically
an independently measured per-motor thrust. For static collective comparisons,
use sum(omega_i^2) rather than 4*(mean(omega_i))^2. Do not pair transient force
and RPM as simultaneous samples or mistake HX711/fixture lag for motor lag.

Analysis (on the Jetson or the local analysis computer)
------------------------------------------------------
python3 analyze_bench_experiment.py bench_repeatability_YYYYMMDD_HHMMSS

Writes timing_report.json, phase_summary.csv and, if matplotlib is installed,
overview.png. No plotting dependency is needed to acquire data or write summary
tables. Analyze on the local computer if the Jetson has no matplotlib.
The report includes achieved sample rates, request durations, intersample gaps,
RPM value changes, force baseline drift and capture/stop outcome.
Phase tables use the final 0.75 s and retain fit/validation roles. These are
endpoint windows, not automatic proof of settling. Inspect the trace before
fitting. Incomplete phases are explicitly marked.

Do not feed these multi-edge independent-channel files to the old single-step
fit script. First inspect capture quality, then fit command->RPM dynamics with
actual event times and direction-dependent response if supported; evaluate on
held-out cycles/levels. Force data can test static gain and slower combined
sensor/fixture response, not independently resolve fast motor delay at 10 SPS.

Validation
----------
python3 -m unittest discover -s . -p 'test_*.py'
Tests use fake devices and verify command bounds, read-only modes, independent
sampling, and stop handling under Ctrl+C, lost ACKs, sensor/battery/status faults
and failed data writes. See BENCH_EXPERIMENTS_VALIDATION.json for actual results
and any real stopped acquisition check. No powered experiment was run while
installing these scripts. Existing scripts, measurements and calibrations are
preserved.
