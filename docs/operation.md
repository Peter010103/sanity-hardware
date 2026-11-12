# Operating the thrust stand

Commands below run from the repository root unless they explicitly change directory.
Inspect the [mechanical assembly](mechanical.md) and complete the
[wiring](wiring.md) first. Use one controller process for the stand.

## 1. Release the shared equipment

Before acquiring devices, inspect the Jetson and any controlling workstation for
leftover bench scripts, FC serial clients, HX711 readers, PSU clients, ROS nodes
and daemons, Zenoh, NatNet/pose bridges, recorders, controllers and preview servers.
A useful first inspection is:

```sh
ps -eo user,pid,ppid,lstart,args
fuser -v /dev/ttyACM* /dev/ttyUSB* /dev/gpiochip* 2>/dev/null
```

Identify ownership before stopping unfamiliar processes. Stop your old experiment
processes and their children gracefully, close Betaflight Configurator, and
confirm they exited and released the required devices/ports. Preserve other
users' active experiments. The bench runners share
`/tmp/sanity_bench_experiments.lock`; older standalone tools and external clients
may not honour it. Never remove a lock file to bypass an active owner.

Record the host and PID of each experiment and helper you start. These tools run
in the foreground and do not install daemons or services.

## 2. Select and inspect the flight controller

Use stable serial paths and verify both device identities:

```sh
. .venv/bin/activate
python tools/betaflight_msp.py list
ls -l /dev/serial/by-id/
export SANITY_FC_PORT=/dev/serial/by-id/usb-Betaflight_Betaflight_STM32F405_0x8000000-if00
export SANITY_PSU_PORT=/dev/serial/by-id/usb-Nuvoton_KORAD_USB_Mode_00002B240000-if00
mkdir -p config runs
python tools/betaflight_msp.py status --port "$SANITY_FC_PORT" --inspect \
  > config/fc_profile.json
```

The inspect command reads configuration; it does not configure or arm the FC.
Check the saved UID, board, firmware, motor protocol, motor count/poles and
telemetry setting against the intended aircraft. `tools/fc_profile.json` is a
historical test fixture, not a configuration to flash to an aircraft. The regulated
runner compares live configuration against `config/fc_profile.json` on every
snapshot and before capture. Optionally set `SANITY_FC_UID` for an additional
explicit UID check. Do not bypass a mismatch by repeatedly capturing a profile
without understanding the change.

Direct FC USB supports the disarmed direct-motor path. Acro runs need RX_MSP
and a transport over which the FC permits arming. A normal native-USB connection
may inhibit arming. The original campaign also used an ESP transparent serial
bridge to the MSP-configured UART; this repository does not flash that bridge or
change UART configuration. Do not clear unrelated arming faults to make a test run.

For Acro, the runner requires RX_MSP enabled, ARM on AUX1 in the 1700–2100 range
(MSP mode tuple `0,0,32,48`), and only the explicitly checked ANGLE rule on AUX2
if present. It commands centred roll/pitch/yaw, AUX2=1500, and validates the actual
RX channel map and armed mode flags. It reads the original
`vbat_sag_compensation`, changes only that value for a requested 0/100 trial,
saves/reboots if changed, and attempts restoration after disarming. This is an
actual persistent FC setting change; inspect the restoration receipt after a run.

## 3. Configure GPIO and calibrate the thrust cell

Disconnect motor power physically. With only the thrust module being read,
check the single-cell clock pad on the verified P3768 carrier:

```sh
sudo .venv/bin/python tools/hx711_pinmux.py status
sudo .venv/bin/python tools/hx711_pinmux.py enable-clock
python tools/hx711_jetson.py raw --backend native --duration 5
```

`enable-clock` only handles physical pin 31, retains an on-disk backup beside the
helper, and does not alter boot configuration. Stop the reader before changing
or restoring the pad. The three-channel runner configures all three clock pads
for its own session and restores their observed entry values.

Apply known axial loads with the actual fixture/preload unchanged. The maximum
below is an example; choose a reference range supported by the cell and rig:

```sh
python tools/calibrate_loadcell.py --max-mass-g 300 --step-g 100 \
  --load-cell-id thrust --rig-id sanity-stand \
  --output runs/loadcell-calibration
python tools/plot_loadcell_calibration.py runs/loadcell-calibration/readings.json
```

Review residuals, hysteresis, force sign and zero return. Only after review, copy
`runs/loadcell-calibration/calibration.json` to
`config/loadcell_calibration.json`. The historical reference file in `examples/`
is for provenance/offline reproduction and is not an automatic calibration of
a newly assembled stand. A measured range must include the selected cutoff; do
not enlarge recorded calibration bounds merely to pass preflight.

For a mounted weight check, still with motor power disconnected:

```sh
python tools/run_weight_check.py --calibration config/loadcell_calibration.json \
  --output runs/weight-check --mass-g 100 --position centre \
  --execute --battery-disconnected
```

Check known loads at motor locations before individual-motor claims: the fixture
must restrain roll/pitch moments and measure off-centre axial forces correctly.
Calibrate torque sensitivities and loading signs independently before absolute
torque use. The raw acquisition retains the historical provisional torque scale.

## 4. Single-cell battery-powered tests

Use the first DOUT/SCK pair only. Follow the complete
[single-cell guide](single-cell.md) for preset meanings and limits.

```sh
python tools/probe_acquisition.py --port "$SANITY_FC_PORT" \
  --profile config/fc_profile.json --calibration config/loadcell_calibration.json \
  --output runs/single-probe --execute

python tools/run_repeated_steps.py --port "$SANITY_FC_PORT" \
  --profile config/fc_profile.json --calibration config/loadcell_calibration.json \
  --output runs/repeatability --battery-id PACK_ID --execute --bench-ready
```

The probe sends no motor commands, including STOP. The repeatability test starts
automatically after checks and a stopped baseline. All single-cell presets stop
at 1450 or below. `--off-axis-checked` is additionally required for the individual
preset. A battery run does not use the PSU and must never have the PSU paralleled
with the flight battery.

## 5. Three-channel PSU-controlled tests

This retains the tested campaign's P3768 / eight-core Orin NX assumptions,
including `/sys/devices/system/cpu/cpu7/cpufreq/scaling_governor`, CPU1 host work,
CPU4 FIFO70 ADC worker and `/dev/cpu_dma_latency`. Root privileges are needed for
`/dev/mem`, governor changes and FIFO scheduling. Inspect the carrier/JetPack and
pad values rather than applying this to an arbitrary Jetson. Ordinary user
permissions on GPIO devices alone are insufficient.

All three modules need their high-rate RATE modification and separate wiring.
Each clock pulse still has a 50 microsecond rejection bound. Sequential sampling
does not imply simultaneous conversion. A failed thrust ADC aborts. Setting
`allow_torque_gaps: true` explicitly allows failed torque channels to be disabled
for the remainder of that run; missing rows remain missing. Acro permits optional
torque channels. Balanced-yaw plans require both torque channels.

Configure the **rear 30 A terminals**, rated leads and reachable physical power
cut, disconnect the flight battery, and manually verify the supply starts OFF at
the intended 30 A limit. Only this identified PSU model and source voltages
7.6/7.8/8.0/8.2 V are accepted. The script sets voltage and switches the supply;
it does not create a fast host voltage regulator. A stopped probe powers the FC
from the PSU but sends no positive motor commands or arming commands.

Preview and edit an example plan first. Set its label, explicit command ceiling,
force limit, phases and multi-motor maximum for the actual rig. The declarations
in the JSON describe the required setup; execution also requires fresh CLI flags.
Use a new Python process for every plan.

```sh
python tools/full_campaign.py examples/probe.json
sudo env SANITY_FC_PORT="$SANITY_FC_PORT" SANITY_PSU_PORT="$SANITY_PSU_PORT" \
  .venv/bin/python tools/full_campaign.py examples/probe.json \
  --execute --battery-disconnected --rear-terminals-confirmed
```

After checking stopped data, force/torque signs, sample rates and mounted response,
a finite direct stage commands M1 at 1250, then STOP:

```sh
sudo env SANITY_FC_PORT="$SANITY_FC_PORT" SANITY_PSU_PORT="$SANITY_PSU_PORT" \
  .venv/bin/python tools/full_campaign.py examples/direct-stage.json \
  --execute --bench-ready --battery-disconnected --rear-terminals-confirmed \
  --off-axis-checked
```

For the separately verified armed RX_MSP path, the example arms at idle, applies
1250 collective RC throttle, then disarms:

```sh
sudo env SANITY_FC_PORT="$SANITY_FC_PORT" SANITY_PSU_PORT="$SANITY_PSU_PORT" \
  .venv/bin/python tools/full_acro.py examples/acro-stage.json \
  --execute --bench-ready --battery-disconnected --rear-terminals-confirmed
```

`--bench-ready` means the propeller-equipped aircraft is secured and the physical
cutoff is reachable. `--off-axis-checked` records the single-motor fixture check.
The PSU limit is 30 A; source current telemetry is slow and is not an independent
fast overcurrent cutoff. FC delivered-bus voltage must remain within the retained
2S 6.6–8.4 V bounds. The FC warning state is logged in PSU mode; critical voltage
still stops the test. Battery-powered presets treat battery warnings as faults.

The three-channel relative thrust guard uses a causal median of three samples.
Raw samples remain unfiltered in the CSV. Absolute load offset is historical;
this path bypasses the old absolute-load-range guard and relies on a fresh
baseline, relative force cutoff, ADC validity and the physical fixture limits.

## 6. Read results and shut down

Each regulated execution creates `runs/fullrange_<label>_<UTC timestamp>/` and a
sibling `_inputs/` directory. `SANITY_RUN_ROOT` overrides `runs/`.
`SANITY_CALIBRATION` and `SANITY_FC_PROFILE` override the configuration paths.
Pass any overrides through `sudo env` as shown for the serial devices.

Captures include raw force/torque, per-motor RPM, FC output readback, battery/status,
IMU, command/phase timing and sequential PSU V/I/status. Acro adds RC commands and
readback. Metadata contains calibration references, limits and source information;
`_inputs/receipt.json` records stop observations, GPIO/governor restoration, supply
cleanup and any configuration restore errors. Never overwrite prior runs.

```sh
python tools/analyze_bench_experiment.py runs/ACTUAL_CAPTURE_DIRECTORY
```

The analyzer generates timing and phase summaries plus an optional plot. It
summarizes the shared v2 channels; inspect `torque.csv`, `supply.csv`, FC outputs
and the receipt separately for regulated runs. Last-0.75-second phase summaries
are endpoint windows, not proof of settling. Request/reply timestamps do not
identify the FC's internal sampling time. Supply voltage is measured at the PSU;
delivered FC voltage is a separate measurement. Whole-drone thrust is not an
independent per-motor measurement.

On completion or Ctrl+C:

1. Verify physical rotor stop and disarmed FC. Use the physical power cut if either
   is uncertain; an acknowledged STOP packet is not a physical stop measurement.
2. Check the final receipt: `fc_stopped_observed` must be true, final supply output
   OFF, and no cleanup/restoration errors. Inspect pad `restore_ok` values.
   Configuration restoration can fail; check compensation independently if reported.
3. Turn the source OFF physically and disconnect motor power before approaching or
   altering the rig. Verify the actual source output as well as its setpoint.
4. Close all started readers, serial clients and helpers gracefully. Confirm the
   recorded PIDs and their children exited, did not restart, and released devices
   and network ports. Check both the Jetson and controlling workstation.
5. If you enabled the single-cell clock manually, stop its reader and restore it:
   `sudo .venv/bin/python tools/hx711_pinmux.py restore`. The three-channel runner
   restores the pad values it found; it does not undo a separate earlier helper
   change. Preserve its backup until restoration is verified.

Host freeze, USB loss, SIGKILL or power loss can defeat Python cleanup. If the
receipt is missing/incomplete, physically remove motor power before recovery and
inspect the recorded pad/governor/configuration state. Do not hand over stale
processes or unverified powered equipment.
