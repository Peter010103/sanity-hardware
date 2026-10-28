# Onboard quarter-bridge tension calibration

27 October 2026. Bench measurement only; no tension feedback in a flight controller yet.

## Hardware and current firmware

The current repository firmware is `Sanity/src/sanity_receiver/zenoh_receiver`:
XIAO ESP32-S3, unified MSP on UART4, normal Zenoh with BOOT-selected USB
passthrough. Sanity09's 26 October installation uses receiver ID9, 10.0.0.109,
udp-split/minimal, host 10.0.0.200, GPIO4 TX → FC RX4 and GPIO3 RX ← FC TX4.
The older `main/combined.c` UDP firmware is not the deployed receiver.

The new optional HX711 task is disabled by default. Enable `--hx711` in
`build_comms_firmware.py`. It sends read-only JSON datagrams to host UDP7460,
independent of Zenoh command/status messages. No automatic tare, force feedback,
FC command or gain changes are added. BOOT passthrough starts no HX711 task.

| HX711 breakout | XIAO ESP32-S3 |
|---|---|
| VCC / digital supply | 3V3, for a breakout specified to operate at 3.3 V |
| GND | GND |
| DOUT / DT | **D0 = GPIO1** |
| PD_SCK / SCK | **D1 = GPIO2** |

These pads are unused by the current firmware. Verify no extra physical accessory
is wired to them. GPIO3/4 remain reserved for the FC; GPIO19/20 are USB, GPIO0 is
BOOT, GPIO21 is the indicator. Do not connect 5 V logic directly to the ESP.
Identify the actual breakout's VCC/DVDD/AVDD connections before applying power;
board labels and load-cell wire colours vary. Excitation is from E+/E− on that
board; measure it. The HX711 connection is a GPIO clock/data protocol, not I2C.

### Bridge topology

For one 350 Ω gauge and three 352 Ω completion resistors, use a complete bridge:

```text
                  E+ (excitation)
                   /          \
          gauge Rg=350       R3=352
                 /              \
          A+ midpoint          A− midpoint
                 \              /
               R2=352        R4=352
                   \          /
                  E− (return)
```

With this particular assignment,

`Vdiff = Vexc [352/(Rg+352) − 352/(352+352)]`.

At Rg=350 Ω the initial output is about **0.0014245 × Vexc**, or 4.70 mV
if excitation is 3.3 V. If excitation equals AVDD, this consumes about 36.5%
of one side of the A128 ADC range, whose ideal limit is ±AVDD/(2×128).
Actual resistor tolerance adds imbalance. Subtracting a digital zero does not
recover analog headroom. Probe raw counts before loading; saturation is rejected.
Use stable matched completion resistors and strain-relieved wiring. Do not swap
excitation with signal terminals or infer terminal identity from colours.

RATE low gives nominal10 samples/s; high gives nominal80 samples/s, selected by
board hardware, not software SCK frequency. Start at10 SPS for static calibration;
80 SPS can later characterize pickup transients. Recheck noise/zero/scale after a
rate or supply change. Identify the actual RATE wiring before modifying it.
A 350 Ω bridge draws roughly Vexc/350 (9.4 mA at3.3 V), plus ADC consumption.

A **taped gauge on printed plastic is an experimental transducer**. Calibration
identifies its present force response; it cannot remove tape slip, changing
strain transfer, creep, temperature sensitivity or cross-axis bending. A bonded
gauge and stable flexure may be needed if checks fail. There is no independently
established mount capacity. Start only within a separately justified conservative
mechanical load. The recorder does not impose a declared maximum-load check.

## Experiment: fixed mount, hanging drone

All drone weight must pass through the same attachment and strain path that
will carry cable force. Fix the mount above the drone, with the drone hanging
freely below. Match the operational load axis and moment arm. A different
bending direction or attachment point needs another calibration.

1. Motors unpowered/disconnected; remove propellers. Power the ESP/HX711 without
   motor output. If USB powers the ESP, support its cable with a slack loop so it
   neither supports drone weight nor pulls on the mount.
2. Weigh the **complete hanging base assembly**: drone, battery if fitted,
   ESP/HX711, hangers and anything else below the sensing section. Do not reuse
   the old69.85 g model mass; electronics/mount additions change it.
3. Warm up120 s unloaded. Record a true unloaded reading with the drone fully
   supported or detached. An empty drone is not an unloaded sensor.
4. Hang the base, then add accurately weighed totals (example0,5,10,15,20 g).
   Wait for motion to stop before pressing Enter. Every point has a10 s countdown,
   then5 s settling and10 s recording. The reference total is what is hanging
   now, not the latest increment added.
5. Remove weights through the same levels and repeat the full cycle twice.
6. Optional: supply `--check-added-g 7,17` to record independent check weights.
   The standard launcher skips these points.
7. Hold the hanging base constant for60 s to expose creep, then completely unload
   for the final zero-return check. Keep wiring and mounting unchanged.

Reference force is positive tension:

`T_i = (m_base + m_added,i) × g`, with masses in kg and default g=9.80665 m/s².

For unloaded points T=0. Fit ascending level means plus the first unloaded point:

`counts_i = b + k T_i + residual_i`, then **`T = (counts − b)/k`**.

A negative k is valid: wiring and gauge location determine the sign. Descending,
check, creep and final unloaded points are held out of the fit. Both raw counts
and the fitted intercept are kept; there is no live auto-zero that could erase
real preload. Absolute tension is reported, rather than only added-weight force.

For a108 g plate supported by three vertical equal-share cables, expected
per-cable force is0.353 N (36 g equivalent). Hanging a70 g drone starts above
that load: add a **weights-only calibration/check around20–60 g equivalent** using
the same axis and fixture. The script can also use a measured small hanger as its
base mass. The interval between unloaded and the lightest measured loaded state
is not independently validated by a straight line. Lower-range checks, changed
orientation/routing, remounting repeatability and battery-powered drift matter
before multi-aircraft pickup.

## Prepared firmware and commands

A compiled **disarmed bench image** is at:

`outputs/strain_gauge/sanity_onboard_tension_20261027/firmware_iram_recovery/`

It inherits Sanity09's latest Wi-Fi, identity and UART settings, adds HX711
telemetry, and enables the existing disarmed-only transport-test guard. It cannot
run an enabled flight mission. **It was flashed to Sanity09 on27 October2026 and the physical HX711 stream was verified.** Restore the recorded
flight image before flying; do not use this calibration image for deployment.
The installed Sanity09 receiver now has HX711 support in this disarmed bench image.
No aircraft control command is needed for HX711 acquisition.

To flash when ready, identify the actual ESP port (do not assume ttyACM0 is the
right aircraft), then from the workspace:

```bash
source /home/hw527/Projects/kine2go/deployment/esp-idf/export.sh
cd /home/hw527/Projects/drones/outputs/strain_gauge/sanity_onboard_tension_20261027/firmware_iram_recovery
python -m esptool --chip esp32s3 -p YOUR_ESP_PORT --baud 460800 \
  write-flash @flash_args
```

The original flight image/flash_args are retained under
`outputs/sanity09_id09_20261026/firmware/` (application SHA
8784e8c76106055059fc7486d84bd189179ff0e33b7a358cb6e1b58e2de3e557).
Flashing either image restarts the ESP. The current physical installation receipt is `outputs/strain_gauge/sanity_onboard_tension_20261027/recovery/flash_receipt_noinline.json`; no aircraft commands or FC settings changes were sent.

Bring up acquisition first, with the host reachable at10.0.0.200 and UDP7460
available. Outputs must be new directories:

```bash
cd /home/hw527/Projects/drones
python3 Sanity/sanity-hardware/tools/acquisition/calibrate_onboard_tension.py probe \
  --duration 30 --output outputs/tension_probe_01
```

Review observed sample rate, flagged packets, raw headroom, sample gaps and clock
pulse timing in summary.json/raw.jsonl. A changing signal alone does not prove
valid strain transfer. A missing HX711/readiness timeout stops collection.

Then, in zsh, enter your measured values (the added-weight lists below are
examples and must match your weighed totals):

```zsh
read 'BASE_G?Complete hanging base mass (g): '
python3 Sanity/sanity-hardware/tools/acquisition/calibrate_onboard_tension.py calibrate \
  --base-mass-g "$BASE_G" --added-g 0,5,10,15,20 \
  --sensor-id sanity09_printed_mount_01 \
  --notes '350ohm taped gauge; 3x352ohm completion; fixed mount, hanging drone' \
  --output outputs/strain_gauge/tension_calibration_01
```

Change the source/device options if measuring another aircraft:
`--device-id 14 --source-ip 10.0.0.114`. The firmware must also be built from
that aircraft's config. Do not flash the Sanity09 image onto another drone.

Results: raw.jsonl (including invalid ADC status samples), session.json (all
accepted measurement samples and stage statistics), calibration.json,
calibration.png and calibration.pdf. Partial sessions survive interruption.
Matplotlib is needed for plots; capture/fitting use the standard library.

Offline refit and timed calibrated recording:

```bash
python3 Sanity/sanity-hardware/tools/acquisition/calibrate_onboard_tension.py fit \
  outputs/strain_gauge/tension_calibration_01/session.json --output outputs/tension_refit_01
python3 Sanity/sanity-hardware/tools/acquisition/calibrate_onboard_tension.py monitor \
  --calibration outputs/strain_gauge/tension_calibration_01/calibration.json \
  --duration 60 --output outputs/tension_hold_01
```

Monitor records force.csv and a summary; it does not change calibration or control
anything. Values outside the tested range are explicitly marked.

## Deciding whether it is useful

Inspect independent check error, ascending/descending hysteresis, repeatability,
unloaded return drift, within-point noise and the60 s creep trace. For an initial
0.353 N application, a provisional measurement goal is check error/hysteresis/drift
below roughly0.01 N and noise well below that; this is an engineering target,
not a proven sensor specification or automatic flight acceptance. Compare the
measured signal/noise to the desired pickup threshold and include false-positive
trials with unloaded cables, cable motion and snags. Repeat at operating battery
power and after remounting. No tension threshold has been installed in flight code.

Counts are not smoothed for calibration. Timestamped readiness observations use
the ESP monotonic clock; they are not the internal ADC measurement midpoint and
include digital-filter latency. Host arrival times are recorded separately. Do not
claim synchronized force/mocap latency from UDP arrival alone. Firmware masks
interrupts only for each short HIGH pulse, checks a45 us software upper bound,
keeps SCK low while waiting, resets faults and discards settling conversions.
This still requires a scope/logic-analyzer pulse check and measured link timing
under the final workload before enabling acquisition during flight.

## Taut/slack use of the 27 October pull data — proposed, not deployed

The latest assembly was reported as a gauge bonded to the printed mount with a
thin layer of 5-minute epoxy, replacing the earlier taped arrangement. The run
`outputs/strain_gauge/tension_pull_20261027_233416_538834141` contains 22,383
healthy samples at an observed 90.31 Hz, with no sequence gaps or ADC reset
changes. Use measured timestamps; nominal HX711 RATE settings are not a timing
contract. Acquisition health does not establish mechanical force accuracy.

The data support investigating a load-engagement indicator, not calibrated force
or a validated near-zero detector. All six loaded holds still decrease, and the
unloaded references recover. For the two 93.38 g trials, subtracting only the
preceding unloaded reference gives 118,249 and 91,818.5 counts; interpolation
using future unloaded references gives 93,217.8 and 93,239.3 counts. One between-
reference zero change is 40,422 counts. Do not copy the offline interpolation or
the previous run's -583.53 counts/s gradient into live control.

### Causal evidence and baseline handling

Use a trailing robust filter on raw counts, with a configured loading polarity
established on the same mount and load direction. Define the signed evidence
`q(t) = polarity * (filtered_raw(t) - last_verified_unloaded_baseline)` in counts.
Never label this value as newtons. A high-pass/change feature can help locate an
engagement event but cannot establish sustained tension by itself.

Update the zero only from an independent `unloaded_reference_verified` input:
for example, operator-confirmed unloading on the bench, or a separately validated
slack fixture configuration with a known clear cable path. A small strain reading,
a classifier's own SLACK output, or a plate resting on the floor is insufficient
authority to tare. Armed/slack vibration and mount orientation may change the
response and must be checked before treating a flight condition as a zero.

Keep baseline age, noise and a conservative uncertainty budget. That budget must
account for baseline drift and loaded creep/hysteresis, not just ADC noise. No
reliable long-duration bound can be inferred from the six ten-second loaded
holds. An expired or unvalidated uncertainty horizon produces UNKNOWN; it must
not silently tare the signal or retain TAUT forever.

Configure two count thresholds `H_on > H_off`, time-based persistence and a
bounded transition grace. Strong loaded evidence requires `q-U > H_on`, where U
is the uncertainty budget; low evidence requires `q+U < H_off`. Maintain
hysteresis only while health and confidence remain valid. Health failures
override persistence immediately. Counts, uncertainty bounds, dwell times and
grace are experimental settings until validated at operating loads; this run
does not establish flight thresholds.

### Packet and time validity

Keep this read-only UDP7460 stream independent of command/status transport.
Validate schema, device/source, channel, gain, pins, ADC range and packet fields.
Firmware flags are 1=missing/not ready, 2=saturated, 4=clock/stuck-data fault,
8=settling. Any nonzero flag means UNKNOWN. A boot or reset-counter change
invalidates classification and the old baseline; start a new independently
verified zero. Sequence gaps/reordering, source-time discontinuity, stale input
or invalid high-pulse timing clear transition persistence and produce UNKNOWN
until clean evidence recovers. Duplicate packets cannot refresh freshness.

Use ESP `ready_us` differences for acquisition/filter durations and host monotonic
time for the receive watchdog. Expiration must be checked even when no packets
arrive. `ready_us` is an observed readiness time, not the internal conversion
midpoint. Host arrival does not establish alignment with mocap. Bound end-to-end
latency and establish a verified clock relationship before fast sensor/geometry
fusion; otherwise withhold an aligned-state claim. Do not assume 80 Hz or convert
persistence to a fixed number of samples.

### Fuse strain evidence with mocap

For each cable, compute hook-to-mount distance using both rigid-body poses and
the actual attachment offsets. Use the raw pose-derived geometric residual and
its uncertainty, not a distance that an observer has already forced to equal the
assumed cable length. The length interval and offset uncertainty must support any
claim of definite geometric slack.

| Evidence | Fused cable state |
| --- | --- |
| Sustained loaded evidence and consistent unobstructed cable geometry | TAUT |
| Low strain evidence and independently clear geometric slack | SLACK |
| Loaded indication with a distance too short for the nominal straight cable | UNKNOWN; investigate snag, load path, geometry or baseline |
| Low reading but geometry near taut length | UNKNOWN; could be low tension or creep |
| Missing/stale/faulted/reset data, unbounded baseline uncertainty or unresolved timing | UNKNOWN |

Positive strain alone cannot exclude a snag. During fixture length calibration,
bar motion and consistent hook distance over several angles provide additional
engagement evidence; strain amplitude still must be validated for the fixture's
actual small load.

### Use in pickup and navigation

First log the three-state estimate in shadow mode, with reason codes, q, baseline
age/uncertainty, packet health and mocap residual. Sanity09 is the only verified
sensor stream here; do not infer measured states for the other two aircraft.

After validation, use the state to schedule reference progress. A SLACK cable
allows slow bounded take-up while position tracking is good. A newly TAUT cable
stops additional individual take-up as the other aircraft catch up. All aircraft
remain actively controlled. Require consistent engagement, settled tracking and
actual plate clearance before a coordinated lift/taut-model MPC handover.
UNKNOWN pauses further loading; persistent uncertainty uses phase-aware recovery.
It must not directly cut throttle, remove payload feedforward in one step, or
substitute the existing airborne leveling routine for a slack floor-start abort.

In flight, a confirmed unexpected slack transition can suppress further mission
progress and request coordinated recovery. A sensor fault must not be interpreted
as physical cable release or immediately reset the observer to a free/slack model.
Keep hard attitude/separation guards independent. Quantitative force feedback,
equal-tension regulation and load-sharing optimization remain disabled until
force calibration is independently validated.

### Required lower-load evidence and firmware status

Repeat true zero versus the intended ~24 g load, and ~36 g equivalent per cable
for the 108 g plate. Include smaller positive preloads and gentle ramps to assess
the near-zero boundary, randomized repeats, mission-length loaded holds,
post-unload recovery, changes of cable direction and motor/operating-mode effects.
Use a light-load fixture preserving the operational strain path: retaining the
78.38 g hanging base and adding 24 g does not test a 24 g force. Record physical
transition timing separately; Enter confirms a hold after setup, not the exact
instant tension appeared. Reserve runs for testing thresholds and report false
taut/slack decisions, UNKNOWN coverage, onset delay and premature loss of TAUT.

The last inspected flash receipt remains `bench_only=true` and its build is
`test_only=true`, HX711 enabled. This assessment has not queried live firmware,
flashed a receiver, changed flight code or enabled tension-dependent output.
Normal flight and HX711 coexistence must be verified on an appropriate flight
build before deployment. Preserve all original raw and offline-corrected files.

## Sources (hardware)

- [Seeed XIAO ESP32-S3 pinout](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).
- [AVIA HX711 datasheet](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/73/HX711.pdf): bridge range, gain, RATE, serial timing and settling.
- [ESP32-S3 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf): GPIO electrical limits.
- Current firmware README/profiles and `outputs/sanity09_id09_20261026/{RESULTS.txt,validation.json}` for deployed configuration. Build receipt/software-only evidence is under `outputs/strain_gauge/sanity_onboard_tension_20261027/`.


## Physical connection check —27 October2026

Sanity09 ESPMAC64:e8:33:50:9c:34/8MB confirmed and the bench image flashed
with3regions verified. Measured2694healthy samples over29.996s (89.779Hz),
70.5%ADC headroom, no healthy-window sequence gaps/send-drop increments or
converter resets. Initial Wi-Fi startup produced252send failures before the
healthy window; startup data are preserved. Max clock-HIGH software bound43us
(limit45us), max ready-to-read span770us; scope timing remains unverified.
Raw counts are not force calibration. Next: measured full hanging base mass
and the loading/unloading/check/creep sequence. Raw data, plot and receipts:
`outputs/strain_gauge/sanity_onboard_tension_20261027/hardware/`.


Measured complete hanging base mass supplied by user: **77.03g**
(reference tension0.7554062495N at9.80665m/s²). Use `--base-mass-g 77.03`. Added weights must still be
weighed. Separate check weights are optional; no maximum-load input is required.
Setup metadata is in `hardware/hanging_setup.json` in the evidence directory.


For the recorded77.03g setup, use the interactive launcher from the workspace:

```bash
./Sanity/sanity-hardware/tools/acquisition/run_onboard_tension_calibration.sh
```

It asks only for the measured added-mass list,
then runs the sequence with the recorded77.03g base and a new timestamped output
directory. It does not send aircraft commands. Override the base for another
measured assembly using `SANITY_HANGING_BASE_MASS_G=...` before the command.


### Simplified prompts —27 October2026

The standard launcher now asks only for added totals in grams. The recorded
base77.03g is used automatically. No separate check-weight list or maximum
permitted-load input is required; the declared maximum-load gate was removed.
Unloaded readings, loading/unloading cycles and raw ADC validity handling remain
part of the measurement sequence. Existing historical results are unchanged.


### Short collection is now the launcher default —27 October2026

At user request the launcher selects `--quick`:2s initial acquisition,1s settling
and3s recording per point, one ascending pass, and unloaded readings before
and after. No countdown, repeated sweeps or60s creep hold. With five added
levels (e.g.0,5,10,15,20g), seven readings take about30s automatic time total,
plus time spent arranging weights. More/fewer weight levels change that total.
The complete hanging base remains77.03g and the wireless UDP endpoint is unchanged.
The full characterization sequence is still available by invoking the Python
calibration command without `--quick`. A quick run does not measure hysteresis
or long-term creep; its output remains an experimental calibration candidate.
This timing change does not resolve a missing-data/connection fault.


The launcher now first prompts for the complete hanging drone weight in grams.
Press Enter to reuse77.03g (or `SANITY_HANGING_BASE_MASS_G` if supplied). The
entered mass is used in force references, session metadata and notes. It then
asks for added totals and retains the quick collection timing.


### Transient ADC recovery —27 October2026

Calibration now waits for a complete clean source-clock window after faults.
It discards every affected fragment and all firmware settling samples, waits
the configured settling interval again, then records3s at the same weight.
A changed converter-reset counter also invalidates a fragment when the fault
packet itself was lost. No fixed80Hz sample-count assumption is used.

Raw fault packets stay in raw.jsonl; accepted points contain recovery diagnostics
and session.json includes accepted/failed measurement attempts. Completed weights
are saved immediately. Recovery adds time to the nominal30s run. If no clean
window can be obtained within the requested duration plus30s recovery allowance,
the program pauses at that same weight and asks Enter to retry; Ctrl-C retains
the partial session. A source/ESP boot change still stops to avoid mixing calibrations.
Persistent saturation/connection failure cannot produce an accepted point.

The old frequent error was the software clock-HIGH timing guard (e.g.46–47us
against45us) triggering resets, not sample-rate mismatch or measured ADC clipping.
The replacement firmware keeps read/clock code and GPIO get/set functions in IRAM,
uses a fixed240MHz CPU-cycle timing bound, and retains the45us guard. Build:
`outputs/strain_gauge/sanity_onboard_tension_20261027/firmware_iram_recovery/`.
Build/mocked recovery tests do not establish physical pulse timing; inspect the
installation receipt and post-flash readout results under `recovery/`.


Final timing image installed and checked27October2026: application SHA
`eb6f1504ee8be8ea22670c5563c4c3e91fe2b226dd9300ba017c431c82634fe0`,
three flash regions verified. Clock loop explicitly `noinline` and ELF placement
verified inIRAM, along withGPIO get/set. Final60s wireless check recorded5409
healthy samples at90.124Hz, zeroADC flags/resets/sample gaps and max measured
software HIGH bound3us. The45us guard remains. This is a readout test, not
force calibration or an oscilloscope pulse measurement. Restart the same
launcher for calibration; transient failures now retry clean data automatically.
Evidence: `outputs/strain_gauge/sanity_onboard_tension_20261027/recovery/validation.json`.


### Current default: short LOAD and UNLOAD sweep —27 October2026

The launcher now explicitly prompts LOAD through ascending added totals and
UNLOAD through the same levels in reverse (the maximum is already measured
at the turning point). True unloaded references bracket the run. Entered
drone mass remains the complete hanging assembly. Bridge identity is one350ohm
gauge and three352ohm completion resistors; their fixed imbalance is represented
by the calibration offset, not by assuming zeroADCcounts at zero force.

Quick mode targets about30s automatic time:2s initial readout,1s settling per
point, and the remaining budget split across readings (minimum1s each). For
0,5,10,15,30,40g:13readings at1.154s each. Weight changes and ADC recovery add
time. No repeated cycles or60s creep hold are added. Prompts say LOAD/UNLOAD
and state the target total, including the drone. ADC recovery remains enabled.

New outputs response_models.json/png/pdf compare degree1/2/3 forward curves
counts(force) for each recorded direction. They include held-out-force-level
errors, monotonicity and raw-count hysteresis. They are diagnostic; no nonlinear
force conversion is automatically installed. Legacy calibration.json remains
a linear experimental candidate. Higher trainingR² alone does not validate a
nonlinear inverse, especially across the unmeasured gap between zero and the
lightest hanging load. Firmware need not be reflashed for these host changes.

Review of210431run: all8 accepted windows ~90Hz with noADCflags/resets/gaps;
clock bound3us. LinearR²0.249 and62.6g equivalent RMS error. Zero changed44819
counts,2.18times the change from0to40g added. QuadratictrainingR²0.957 improves
fit but is nonmonotonic over0..1.148N and has worse full-range held-out error.
Current force conversion is not recommended; loading/unloading repetition is
needed to assess direction dependence and whether a nonlinear model is justified.
Original recording/calibration preserved; derived review in that run's analysis_review/.


### Free recording while touching the sensor

No weight entry, unloaded stage, calibration fit or warm-up prompts:

```bash
./Sanity/sanity-hardware/tools/acquisition/run_onboard_tension_recording.sh
```

Touch/release the sensor freely and pressCtrl-C when done. Alternatively pass
`--duration 30` to stop after30s. A timestamped outputs/strain_gauge/tension_touch_* directory
contains raw.jsonl, samples.csv, summary.json and time_series.png/pdf. All parsed
fault/settling packets are retained; they are marked rather than treated as
valid readings. Recording continues through missing data and ESP restarts.
The plot uses host monotonic arrival time; ESP ready/read times and boot/sequence
identity are also saved. No smoothing, taring, force conversion or aircraft
commands. Same receiver9/.109→host UDP7460; no firmware change required.


### Guided manual pull/release recording

```bash
./Sanity/sanity-hardware/tools/acquisition/run_onboard_tension_pull_test.sh
```

Waits for healthy live HX711 data, then5s get-ready,5s hands-off baseline,
and3cycles of PULL GENTLY AND HOLD for3s / LET GO for5s. ~34s after first
healthy data, then automatic stop/plot. Keep the mount fixed and pull along
the same axis with a roughly similar gentle effort each time. There is no
calibrated force target. High1000Hz tone = pull; low440Hz tone = release.
Console cues always print; `--mute` disables audio. Playback uses paplay/aplay
asynchronously, with terminal-bell fallback. No sound was played in validation.

Customize, for example `--cycles 5 --pull-s 4 --release-s 6`. Other options:
`--baseline-s`, `--get-ready-s`, optional recording limit `--duration`. Ctrl-C
saves a partial run and plot. Routine timing follows actual cue issue times,
so a delayed loop does not skip phases or shorten the next hold/release.

Results in outputs/strain_gauge/tension_pull_TIMESTAMP: raw.jsonl, samples.csv (cue phase
and cycle attached), events.csv (actual/planned host times and cue lateness),
protocol.json, summary.json, time_series.png/pdf. Pull/release intervals are
shaded on the plot. These are cue times; actual force onset includes human
reaction time, and audio start was not hardware timestamped. No force conversion,
filtering, tare or aircraft output. Same wireless endpoint and existing firmware.


### Known-weight input and consolidated output directory

All strain-gauge experiment results now live under `outputs/strain_gauge/`.
Existing26result/evidence folders were moved intact, retaining their names.
New raw, guided and calibration runs create subfolders within this parent.
An explicit `--output` still selects any new directory you supply. The relocation
manifest maps historical absolute paths to current paths; historical CSV/JSON
contents were preserved. Archived IDFbuild caches retain original absolute paths;
use a fresh output for recompilation. Relative flash_args and verified binaries
remain usable from their new firmware folder.

For repeated known-mass loading, use measured total ADDED mass, including the
added hanger. Remove all added mass between trials, using the same load point:

```bash
./Sanity/sanity-hardware/tools/acquisition/run_onboard_tension_pull_test.sh \
  --weights-g 10 --cycles 3 --pull-s 5 --release-s 20
```

The script cues LOAD10g for5s and UNLOAD for20s, three times, after5s ready
and5s baseline (~85s). Replace10with the actual measured grams. A list such as
`--weights-g 5,10,20` or `--weights-g 5 10 20` tests each mass independently,
then repeats the whole list `--cycles` times. `--weight-g` is an alias.

If a known mass remains attached throughout, specify `--base-load-g`, e.g.
77.03g when the complete drone stays hanging during both load and unload.
The requested added/total masses and nominal quasistatic reference force
`(base+added)*9.80665/1000` are saved with cues and samples. They are target
values, not measured force: attachment/removal has human delay. Raw ADC
counts remain uncalibrated. Plot labels show the requested mass.
Same firmware/wireless endpoint; no flash or aircraft output is required.


### Enter-confirmed known-weight setup

Known-weight mode now waits for Enter before each baseline, loaded hold and
unloaded recovery interval. Attach/remove the requested mass, stop touching
the fixture, then press Enter. Each timer begins at confirmation; operator
setup time is additional to the nominal timed duration. Same command/options.
Raw acquisition continues while waiting, so manipulation transients are saved.
Grey plot intervals and setup_* CSV phases mark preparation; their reference
force/mass fields are blank. Confirmed intervals carry the requested mass.
Events record setup prompts, Enter confirmation times and waiting duration.
Queued Enters from previous steps are discarded. If standard input closes,
the recorder saves the partial data/plot rather than automatically proceeding.
Hand-pull and free recording retain their existing automatic timing. Optional
`--auto-advance` restores fixed-timer known-weight cues for unattended tests.
No firmware change or physical output; all results stay under outputs/strain_gauge/.

## Causal acquisition support prepared —27 October2026 (unflashed)

The firmware/host timing contract is documented in
`Sanity/src/sanity_receiver/zenoh_receiver/HX711_TIMING.md`. Raw signed counts,
`sanity_hx711_v1` and flag bits1/2/4/8 are preserved. Additive `timing_v=1`
metadata links the separate acquisition boot ID to receiver-health boot ID,
brackets the polled ready observation, records shift/acquisition/dispatch stages,
and counts bounded queue losses. It does not synchronize strain and mocap or
measure ADC group delay. `read_us` is the pre-format/dispatch timestamp, NOT
clocking completion. Software reset counters cannot prove every isolated ADC
power dip. The host must withhold aligned-state claims without measured bounds.

Acquisition and UDP delivery now use separate low-priority tasks and a bounded
nonblocking queue. Optional sensor failures leave the receiver running; no sensor
code accesses FC controls. Four post-reset conversions and600ms settling remain;
flagged/settling values cannot become healthy. Explicit separate bench and
flight-compatible profiles are built, not flashed. The installed image's last
receipt remains bench-only; no change removes that image's guard.

`hx711_timing.py` provides packet-continuity/watchdog diagnostics only; baseline,
loading polarity, causal filtering, hysteresis, dwell and geometry fusion remain
host estimator work. Diagnostics deliberately assert no clock alignment or cable
state. Raw CSV/JSONL retain the new metadata. Historical recordings are unchanged.
No tare, fixed drift correction, counts-to-newtons inverse, cable threshold or
sensor-triggered flight action has been installed.

Evidence/artifacts: `outputs/strain_gauge/hx711_causal_support_20261027/`.
The connected ESP was only observed through passive UDP reception, without serial
reset, flash, probes or aircraft commands. This checks the existing legacy stream,
not physical behavior of the new build. Actual flight-image coexistence remains
unverified. Only Sanity09 has measured strain evidence.

### Next experiment: actual low loads and held-out decisions

Replace the78.38g hanging assembly with a light fixture that preserves the same
strain path/attachment lever arm. For a24g TOTAL test the old hanging base must
be fully supported/removed. Weigh the complete load INCLUDING hanger/cable below
the sensor. Record true unloaded before and after every hold; no hanging mass
may remain during a labelled zero. Separately establish conservative mechanical
limits; these tests are not quantitative force calibration.

Use measured totals0, small positive preloads (e.g.2,5,10g),24g and36g.36g is
only the equal-share vertical equivalent of108g/3; angled/unequal cables may differ.
Reserve at least eight whole repetitions per load across two separate sessions:
four development, two validation and two untouched held-out repetitions. Fix the
split before threshold tuning; interleave/randomize loads, including unloaded-only
and manipulation-without-load trials. Record the actual order/total mass. For
short initial transitions use20–30s holds/recovery; include repeated120–180s
loaded holds and90–120s unloaded recovery (or the entire intended mission length
if longer). Keep full raw manipulation transitions and independently timed video
or another onset reference; Enter confirms a hold after setup, not exact onset.

Existing receive-only guided recorder supports each measured load separately:

```bash
# Light fixture ONLY,24g total including its hanger; no78.38g hanging base.
Sanity/sanity-hardware/tools/acquisition/run_onboard_tension_pull_test.sh \
  --weights-g 24 --base-load-g 0 --cycles 8 --pull-s 120 --release-s 90
```

Change24 to each weighed TOTAL (or record a chosen randomized list of totals).
This command records sensor data/cues only; no aircraft commands. Actual load
application/removal remains manual and must match the entered mass. Use explicit
run notes for preload, orientation, session and held-out split. Do not relabel an
unknown manual pull as a known mass.

Repeat relevant cable directions/angles and mount orientations with a verified
unloaded reference at each changed setup. Compare USB versus normal battery/
regulator power, warm-up/temperature, cable/USB-wire motion and vibration. Later,
separately authorized secured/props-removed motor-mode and actual receiver-load
qualification should measure power/RF/timing effects. This task sends no motor
or flight commands and supplies no tension-dependent control behavior.

Freeze polarity, causal filter, uncertainty horizon, thresholds and elapsed-time
persistence using development/validation trials only. Replay held-out trials in
source-time order with receive watchdog gaps; never interpolate using future zeros.
Report false TAUT on verified-unloaded periods, false SLACK on verified-loaded
periods, UNKNOWN coverage/duration (overall AND by load/mode), onset/release delay
relative to an independent physical transition reference, and premature TAUT loss
throughout long loaded holds. Include packet loss/reordering, resets, stale gaps
and unresolved clock alignment in those results; do not exclude UNKNOWN to
inflate conditional accuracy. A declining loaded value must not automatically
produce SLACK, and a positive value cannot distinguish normal engagement from a
snag without geometry/path checks. No flight threshold is validated by the
existing78–118g data.


### Sanity09 installation update — 28 October 2026

The flight-compatible HX711 image was explicitly requested, flashed and verified
on MAC64:e8:33:50:9c:34. Application SHA256:
`5c857329bf3d32cffbc23d2e1ae0483a5c38d14c6d738cd1787554e1e2443554`.
All three flash regions verified. Passive10s readout:899 healthy timing_v1
samples,89.69Hz, noADC faults or sequence gaps; reset counter1, max software
HIGH bound3us. No aircraft commands/FC settings writes or enabled flight were
performed by the flashing/check procedure. Full previous8MB flash was backed up
privately before writing. Receipt and rollback instructions:
`outputs/strain_gauge/sanity09_hx711_flight_flash_20261028/`.

Current single-drone mass supplied by user:78.41g drone+cable, excluding payload.
Retaining1.34g cable gives77.07g aircraft-only;23.86g payload retained. Attachment
[0,0,-29]mm from FC/body frame. New dedicated ROS profile estimates cable length
in settled hover from490mm initial seed (user subsequently requested capture400..600mm). Supervisor is shadow
only; no force calibration or live taut/slack threshold has been validated.

One bench launcher `scripts/run_sanity09_cable_bench.sh` combines guided sensor
tare/loading with DISABLED receiver-workload checks; it does not arm. After a
passing bench result, `scripts/run_sanity09_cable_flight.sh` starts the dedicated
PID hover commissioning flight. Both live in `Sanity/src/ros2_sanity_ws`.
