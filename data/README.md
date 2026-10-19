# Sanity calibration data

Actuator measurements were collected in-house on **2026-10-13**; bifilar inertia measurements on **2026-10-19**. See the [inertia guide](../docs/inertia.md) for the current mass/inertia and complete measurement workflow. [runs.csv](runs.csv) indexes the experiments; `raw/` contains recordings and `processed/` contains reduced tables and fit results.

## Bifilar inertia

The [raw 19 October recordings](raw/bifilar/20261019) contain yaw, pitch and roll poses, capture receipts, original configs and the nominal fixture DXF. The [processed inputs and results](processed/inertia/20261019) retain corrected geometry, explicit mass/COM assumptions, fits and validation. Original configs predate later geometry corrections; use the processed inputs for reproduction.

```sh
python3 tools/inertia/analyze.py
```

## Actuator experiments

| Recording | Kind | Status | Used for |
|---|---|---|---|
| [bench_individual_20261013_040641](raw/experiments/bench_individual_20261013_040641) | individual | completed | individual force/speed identification + reference evaluation |
| [bench_multilevel_20261013_040554](raw/experiments/bench_multilevel_20261013_040554) | multilevel | completed | held-out evaluation before final refit |
| [bench_probe_20261013_040054](raw/experiments/bench_probe_20261013_040054) | probe | completed | acquisition timing diagnostic only |
| [bench_probe_20261013_040425](raw/experiments/bench_probe_20261013_040425) | probe | interrupted | acquisition timing diagnostic only |
| [bench_reference_20261013_040736](raw/experiments/bench_reference_20261013_040736) | reference | completed | held-out evaluation before final refit |
| [bench_repeatability_20261013_040435](raw/experiments/bench_repeatability_20261013_040435) | repeatability | completed | fit + last-cycle model selection |
| [bench_small_steps_20261013_040516](raw/experiments/bench_small_steps_20261013_040516) | small_steps | completed | fit + last-cycle model selection |
| [bench_weights_20261013_040817](raw/experiments/bench_weights_20261013_040817) | weights | completed | mounted 100 g scale check only |
| [thrust_sweep_20261013_025451](raw/sweeps/thrust_sweep_20261013_025451) | legacy_collective_sweep | failed | ten settled static points only |
| [motor_step_20261013_031851](raw/steps/motor_step_20261013_031851) | legacy_step | failed | excluded from fast-response fitting |
| [motor_step_20261013_032116](raw/steps/motor_step_20261013_032116) | legacy_step | completed | excluded from fast-response fitting |
| [motor_step_20261013_032625](raw/steps/motor_step_20261013_032625) | legacy_step | completed | excluded from fast-response fitting |
| [loadcell_weight_calibration](raw/loadcell) | nominal 0..1000 g weights | complete | force calibration: 840 samples / 21 levels |
| [mass_summaries](raw/mass_readings.csv) | battery absent/present weighing | summaries only | 39.40 g / 66.09 g mass evidence |

## From measurements to parameters

1. **Load-cell scale:** nominal 100 g increments from 0 to 1000 g and back to zero; 40 accepted samples at each of 21 levels. Ascending means identify the gradient; descending levels assess hysteresis. Raw counts and timing are in `raw/loadcell/readings.json`, and the applied calibration is `calibration.json` in that directory. Rated capacity/reference-weight accuracy are unconfirmed.
2. **Bench acquisition:** USB MSP external motor counts command either all four outputs or one output at a time. The aircraft is mounted; measured axial load is baseline-subtracted. Voltage/current and RPM come from Betaflight; force comes from HX711. Individual-run off-axis validity was declared by the operator, not independently quantified.
3. **Steady-state data:** 41 settled phases plus ten earlier sweep levels. The processed table includes commands, voltage, thrust, force variation and mean squared angular speed for each output, along with the run type and training/evaluation split.
4. **Model fitting:** thrust versus squared speed, command-to-speed mapping with voltage, and speed response to command steps. Response fits use independently timestamped RPM: 104 rotor transitions across 32 command events. The earlier sequential step logs lack the timing resolution needed for these fits.
5. **Evaluation:** the last repeated cycles are used to select model complexity; multilevel and reference runs are held out for evaluation. The selected models are then refitted to all runs. The high-speed sweep contributes only to the static thrust fit.

## File guide and units

| Files | Contents |
|---|---|
| `raw/experiments/*/commands.csv`, `phases.csv` | M1–M4 external motor counts and phase/send/ACK times; 1000 stop, not literal zero or PWM microseconds |
| `raw/experiments/*/rpm.csv` | FC mechanical RPM and separate monotonic request/reply times in ns; configured 12 poles unconfirmed |
| `raw/experiments/*/force.csv` | Raw ADC counts, axial load in N, validity and separate read-ready/completion times |
| `raw/experiments/*/battery.csv`, `status.csv` | FC voltage (V), current (A), consumed charge and status fields where available; meter calibration unverified |
| `metadata.json`, `summary.json`, `events.jsonl` in each run | Setup declarations, plan, timing, completion/failure and stopping records |
| [static_points.csv](processed/actuator/static_points.csv) | 51 reduced force/speed observations, including individual versus collective context and fit partitions |
| [edge_fits.csv](processed/actuator/edge_fits.csv) | Per-edge dynamic diagnostics |
| [fit_report.json](processed/actuator/fit_report.json) | Full fit results, candidate comparisons, uncertainty and input hashes |
| `processed/actuator/*comparison.json` | Candidate-model and reserved-data scores |
| `processed/actuator/*heldout_predictions.csv` | Time-matched measured/predicted speed for reserved runs |
| [loadcell samples](processed/loadcell/loadcell_samples.csv) / [level means](processed/loadcell/loadcell_level_means.csv) | Load-cell calibration samples and averages |
| [mass_readings.csv](raw/mass_readings.csv) | Saved weighing summaries; individual mass samples are unavailable |
| [parameters.csv](provenance/parameters.csv) | Measured/calculated/configured/unknown parameter provenance |

Use each channel's timestamps to align samples. Absolute times use Unix ns; elapsed times use monotonic ns. Calibration uses 9.80665 m/s². Thrust sign +1 follows the bench's positive axial-load change. Stand, duct and ground effects and absolute force uncertainty have not been measured separately.

The earlier sweep includes battery-warning and critical readings and ended after severe voltage sag. Its flags are retained in the static fit data. Dividing collective thrust by four gives an equal-share estimate, not an individual motor measurement. The load-cell response filter describes the measurement system and should not be applied to simulated thrust.

## Reproduce

From the repository root:

```sh
python3 tools/reduce_loadcell.py
python3 tools/fit_actuators.py --bootstrap 100
python3 tools/verify_refit.py
```

Fit results go to `build/actuator_fit/`. Raw-file checksums are listed in [raw_manifest.json](provenance/raw_manifest.json); paths recorded inside the original run metadata refer to the collection machine.

[Actuator parameters](../parameters/actuators.json) · [Model details](../docs/technical-notes.md#actuator-equations) · [Hardware interface](../docs/technical-notes.md#hardware-control) · [Licensing](../LICENSES.md).
