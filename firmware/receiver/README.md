# Sanity: shared Zenoh/MSP command link

The installed XIAO ESP32-S3 image supports **CTBR and direct motor bench control
through the same UART4 MSP connection**. Selecting the host command mode requires
no Betaflight configuration change, ESP reflash, wiring change or SBUS connection.
Switch while disabled and with the FC confirmed disarmed. Direct motor flight is
not implemented: stock Betaflight's motor-test interface remains bench-only.

| Host mode | Input in model frame | Host conversion | ESP → Betaflight |
| --- | --- | --- | --- |
| `ctbr` | Total thrust N; body rates rad/s, FLU | Physical-unit packet and calibration check; host preview | ESP RPM/FC-voltage inverse → `MSP_SET_RAW_RC` (200); Betaflight Acro |
| `direct_thrust` | Four thrusts N, RR/FR/FL/RL | Supplied actuator inverse, live battery voltage, motor permutation | `MSP_SET_MOTOR` (214); **disarmed motor bench** |

The standard experiment launcher now sends physical CTBR units for ESP-side
conversion. Legacy RC-channel packets and direct motor-bench counts remain
supported. Direct motor conversion still runs on the host. UART4 stays non-inverted **115200 8N1** in both modes.
A host Zenoh peer listens on TCP 7447; the ESP connects as a Zenoh-Pico client.
Discovery is disabled; no separate router process is required for normal use.
The older UDP implementation remains separate.


## RPM and FC-voltage CTBR calibration — 21 October 2026

`main/actuator_profile.h` and ROS `config/sanity09_esp_collective.json` are generated
from the same 84 steady Acro100 observations in the retained 20 October campaigns.
`python scripts/generate_collective_profile.py` reproduces both artifacts. It uses
measured **delivered FC voltage**, not the PSU-source-voltage input of the earlier
selected bench fit. Recorded configuration: Acro, voltage compensation100,
RC smoothing ON, linear600deg/s rates. No Betaflight configuration is written.

Define `omega_eq=sqrt(sum(omega_i^2)/4)`. The selected model is
`omega_eq=c0+c1*q+c2*z+c3*q^2+c4*q*z+c5*z^2`, with
`q=(RC-1500)/400`, `z=(V_FC-7.4)/0.8`; total thrust is
`4*kT*omega_eq^2`, with effective armed `kT=3.067640401220797e-8`.
This armed/rig coefficient is separate from the isolated-motor constant.
The generator selects the simplest positive monotonic surface within5% of the
best combined command/voltage holdout score. Held-out-command thrust RMS0.02497N;
held-out-source-voltage RMS0.02867N. These are bench validation results.

The ESP inverts this monotonic model by integer-RC search over measured RC1100–1900.
There is no CTBR voltage-range interlock and no bypass flag is needed. Live FC
voltage remains telemetry and the model input; missing/stale telemetry uses8V.
For evaluation only, voltage is clamped to6.618–8.172V; reported telemetry is
unchanged. Commands below/above modeled force clamp to the RC endpoints; exact
zero stays RC1000. The measured joint hull remains in the calibration as provenance,
but is not a runtime restriction. Some runtime RC/voltage combinations are therefore
extrapolations beyond the measured joint hull; the original held-out scores do not
qualify those combinations. No second sag-compensation correction is added.

The selected observed Acro RPM rise L/tau24.90/42.57ms and fall18.23/68.86ms are
retained in the JSON for model comparison. They include FC/filter/telemetry timing;
no extra RPM feedback loop or inverse dynamic filter is added on the ESP. This
firmware does not add live RPM polling. Nonzero-moment mixing, loaded rate-loop
response and battery conditions outside measured coverage remain unvalidated.

Physical packets retain40bytes: magic/version `SZC2`/2, CTBR mode1, signed body
rates at1e-4rad/s in the first three16-bit fields, total thrust at1e-4N in the last.
Header bytes30–31 carry the low16bits of the calibration ID; the host checks the
full32-bit ID returned by the ESP before enabling. Epoch, session, sequence,
lease and explicit disabled→enabled-zero→positive handshake are retained.
Legacy `SZC1` count packets remain accepted; direct motor bench remains unchanged.

Status `SZS2`/2 is48bytes: original36-byte fields followed by32-bit calibration ID
and four16-bit applied RC/motor-count outputs. Flag bit3 indicates unavailable or
unsupported RPM-model voltage (informational fallback/clamp), separately from failsafe bit1. Host supports both
old36-byte and new48-byte status. The shared profile is content-hash checked.
The existing1s command watchdog, FC checks, UART4 wiring, network settings and
BOOT-selected passthrough are retained.

The standard `run_sanity_experiment.sh` live/shadow path selects the new profile;
other legacy sender scripts must opt into `FirmwareCollective` and
`SanitySender(physical_units=True)`. Simulation/training previously using the fixed
8V-source inverse must explicitly select the new calibration for an exact match.
No policy was retrained or powered motor test performed by this firmware update.

## Stamp-P4 with C6 add-on

Use `scripts/build_comms_firmware.py --board stamp-p4` with an existing receiver private base configuration. The P4 build imports only application settings, regenerates the SoC settings, and verifies the resolved board pin map before writing a successful build receipt. It targets S013 rev1.x silicon at 360MHz, 16MB flash and ESP-Hosted 2.12.13 over SDIO. The default builder board remains `xiao-s3`.

User wiring (9 November 2026): blue LPG8/GPIO8 is the separate FC Serial RX wire; white LPG10/GPIO10 is P4 TX to FC RX4; yellow LPG12/GPIO12 is P4 RX from FC TX4. Common GND and regulated 5V to 5VIN are required. The wide VBUS pad is directly connected to USB VBUS; use 5VIN for FC power so FC 5V is not tied to the PC USB supply. GPIO8 is unused/high impedance in the unified MSP profile; control and telemetry use UART4 MSP at 115200 8N1. Do not enable a separate SBUS output without matching the FC configuration.

The tested C6 factory firmware is ESP-Hosted 2.12.1; the host driver is pinned to the compatible 2.12 series. The project excludes the driver constructor and explicitly initializes/connects it after the receiver boot-mode selection. The C6 plugs into SDIO: CLK43/CMD44/D0-3=45-48/reset42. Wi-Fi starts only after normal receiver mode is selected. USB passthrough uses GPIO35 pulled to GND briefly during the four-second startup window. This bare module has no XIAO status LED, so the profile disables that LED. Hold GPIO35 low across reset only when entering the ROM flashing bootloader. Reset uses EN to GND. A standalone radio check does not qualify the FC link or flight. P4 HX711 builds are rejected until their timing is qualified.

## Current wiring, power and passthrough

| Flight controller | Seeed XIAO ESP32-S3 |
| --- | --- |
| TX4 | GPIO3 / D2 (ESP RX) |
| RX4 | GPIO4 / D3 (ESP TX) |
| GND | GND |

ESP USB powers the ESP only. Power the FC through its own USB or flight battery.
For configuration using PC → ESP USB → FC UART4:

1. Press and release **RESET**.
2. During the first **four seconds of LED blinking**, press **BOOT** briefly
   (at least 80 ms; half a second is enough).
3. A **steady LED** means USB passthrough. Connect Configurator to the ESP USB
   device at **115200 baud**. No Wi-Fi, Zenoh or control task runs in this mode.
4. Reset without pressing BOOT to return to Zenoh. The host selects CTBR/direct
   through its command stream. The choice of passthrough is not saved.

Holding BOOT while resetting enters the ROM flashing bootloader instead.
The LED indicates the selected USB/Zenoh mode, not connectivity or arming.
Configurator 10.10.0 hides Espressif devices by default: Options → **Show all
serial devices (for manufacturers or development)**. Select USB JTAG/serial debug
unit, or use Manual Selection with the stable ESP path:

```text
/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:50:9C:34-if00
```

Close other programs using that serial port. Passthrough forwards binary MSP/CLI
bytes and configuration writes at 115200; the USB baud dropdown does not change
the FC UART baud. USB logging is disabled to preserve the stream. Pending bytes
are retained on partial writes; a bridge driver error stops forwarding and blinks
the LED until reset. The original ESP 8 MB flash backup is private in
`outputs/sanity_esp_passthrough_20261016/`; it may contain saved credentials.

**Use direct FC USB for full CLI backups and FC firmware flashing/DFU.** UART4
`dump all` was truncated to 587 bytes versus 38,987 over direct FC USB on this
Betaflight build. Short MSP replies, including 290-byte jumbo BOXNAMES, passed
300 passthrough checks. A likely cause of truncation is an FC UART bulk-buffer
overwrite; this is an inference from the observed output and Betaflight source,
not an electrical UART capture.

## One-time Betaflight configuration

The installed FC is Betaflight **4.5.1 / MSP API 1.46**, BETAFPVF405, UID
`2a0039000a47333135373835`. The following settings were saved and read back:

```text
serial 3 1 115200 57600 0 115200
feature -RX_SERIAL
feature RX_MSP
set serial_update_rate_hz = 1000
save
```

Identifier 3 means UART4. In Configurator, UART4 has **MSP on / Serial RX off**;
the receiver source is **MSP**, not Serial RX/SBUS. Other receiver sources are
disabled; other serial ports and unrelated features are preserved. USB VCP MSP
stays enabled. ARM remains AUX1 high, while AUX2 high keeps the existing
ANGLE-on-low range inactive. The conflicting AUX2-high BLACKBOX ERASE range was
removed during the earlier SBUS setup. No rate/PID/idle/failsafe tuning changed.

`scripts/uart4_mode.py msp --expected-uid ...` prints a plan. Add
`--apply --output /path/to/NEW_private_backup_directory` to save it. The tool
checks UID/version, disarmed state and zero RPM, captures a complete native-USB
backup and rollback commands, then verifies the saved serial/feature settings.
Quiesce the ESP before this one-time setup; the deployment held it in its flashing
bootloader. **Do not run this tool to select CTBR versus direct mode.** Its older
`ctbr`/`msp-bench` targets are retained for legacy images only.

Betaflight 4.5.1 processes one MSP command per port per serial-task invocation.
The original 100 Hz setting can queue control plus telemetry commands; 1000 Hz
raises processing capacity. The 22-byte eight-channel RC frame takes **1.91 ms**
on a 115200 8N1 UART. This is wire time, not measured host-to-motor latency. Host
transmit/output scheduling, FC scheduling and Wi-Fi add delay. The earlier `wgb`
measurements had status gaps approaching 200 ms; this setup is not a flight
latency qualification.

## Firmware build

Requires ESP-IDF ESP32-S3 tools. Tested with the installed ESP-IDF development
checkout (`v6.1-dev-7447-g2067f3ae32b`, local environment named 6.2). Zenoh-Pico and
Python Zenoh are pinned to **1.10.1**. From this directory:

```sh
. /path/to/esp-idf/export.sh
python scripts/prepare_pico.py /tmp/sanity-zenoh-pico-1.10.1
idf.py -B build-xiao-msp \
  -D SDKCONFIG="$PWD/build-xiao-msp/sdkconfig" \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;profiles/xiao_s3_uart4.defaults;profiles/robotlab.defaults;sdkconfig.msp.defaults;/absolute/path/to/receiver.private.defaults' \
  -D ZENOH_PICO_SOURCE=/tmp/sanity-zenoh-pico-1.10.1 build
idf.py -B build-xiao-msp -p YOUR_ESP_SERIAL_PORT flash
```

The deployed network is **robotlab**, with static ESP IPv4 **10.0.0.109/24**,
gateway **10.0.0.1**, and Zenoh host **tcp/10.0.0.200:7447**. The PC is connected
by Ethernet and currently holds 10.0.0.200 through DHCP; if that address changes,
update the ESP endpoint and rebuild. The ESP's static IP is independent of the
server endpoint. Default startup remains Zenoh; the BOOT passthrough selection,
shared MSP mode and all FC settings are unchanged.

`profiles/robotlab.defaults` contains only the SSID and static network settings.
Keep credentials and the server endpoint in the private defaults file referenced
above (outside the repository; restrict access with `chmod 600`):

```text
CONFIG_SANITY_WIFI_PASSWORD="your exact Wi-Fi password"
CONFIG_SANITY_ZENOH_ENDPOINT="tcp/10.0.0.200:7447"
```

The supplied password was configured with its literal asterisks. It is not stored
in this README or the public profile. Generated SDKCONFIG files, binaries and
flash backups can contain credentials; the deployment directory is private.

Static IPv4 is optional (`CONFIG_SANITY_STATIC_IP=y`); set its address, netmask
and gateway through the `SANITY_IPV4_*` options. The firmware stops DHCP before
applying those settings. Builds without this option retain DHCP. The older
`profiles/wgb.defaults` remains available for the earlier open-network setup.
Generic builds must configure Wi-Fi/endpoint, device ID, pins and unified MSP in
menuconfig. Use a fresh SDKCONFIG/build directory when changing defaults; existing
SDKCONFIG values take precedence. The XIAO profile sets 8 MB flash, GPIO4/3,
BOOT GPIO0, active-low LED21, four-second selection window and clean USB bytes.
The optional `profiles/passthrough_default.defaults` reverses the startup choice.

`prepare_pico.py` pins commit `e1ab223a28aaebb5dec1e70d98eab152332f777a` and applies
`patches/zenoh-pico-1.10.1-gcc16.patch` for bounded copies/initialized decode
variables required by GCC 16. Compiler warning checks remain enabled.
Legacy SBUS CTBR and MSP-only direct-bench builds are still available through the
menuconfig output choice; they are not the installed shared-MSP image. Legacy
SBUS uses 100000 8E2 and cannot share the UART with MSP. On this raw RX4 wiring it
used uninverted ESP TX with Betaflight 4.5.1 `serialrx_inverted=ON`; that inversion
setting is irrelevant to the current normal MSP UART.

## Host and mode selection

Use a Python environment matching ROS (Ubuntu Jazzy uses `/usr/bin/python3`).
Install `eclipse-zenoh==1.10.1` from the sender requirements. From
`Sanity/src/ros2_sanity_ws`, install the pure Python packages:

```sh
python -m pip install -e src/udp_broadcaster -e src/sanity_zenoh
```

Start one command producer per drone. These finite examples remain disabled:

```sh
python -m sanity_zenoh.cli --mode ctbr --duration 10
python -m sanity_zenoh.cli --mode direct_thrust --duration 10
```

The Python API is `SanitySender(mode=CTBR|DIRECT)`, followed by
`.ctbr(armed, total_thrust_N, rates_rad_s)` or
`.direct_thrust(enabled, rotor_thrusts_N)`. Close the previous disabled sender
before starting another mode. For ROS, select the bridge's `mode` parameter when
starting the node; there is no FC configuration or ESP image change.

The ESP boots into disabled CTBR and subscribes to both command keys. A fresh
**disabled** packet can claim another mode only while the ESP output is disabled
and recent FC status confirms disarmed. It advances the receiver epoch, invalidates
old leases, then requires another fresh disabled acknowledgement. Buffered packets
from the previous mode cannot resume output. The host can learn the receiver clock
from another mode's status, but cannot enable until its own session has a fresh
matching-mode acknowledgement. Mode changes are not in-flight handovers.

For enabled operation: disabled → wait for own-session disabled acknowledgement →
enabled zero thrust → wait for enabled acknowledgement → positive thrust.
A fault/reconnect requires this sequence again. CLI `--enable` requests it;
direct bench also requires `--bench-ready`. Positive CTBR requires `--ctbr-map`;
positive direct requires `--actuator-model`, `--model-to-output 1 2 4 3` and
`--rotor-thrusts T_RR T_FR T_FL T_RL`. `--help` lists options. Closing sends STOP.

For ROS, build/source `sanity_msgs`, `udp_broadcaster` and `sanity_zenoh` with the
ROS-compatible interpreter. Load `config/zenoh.yaml`; for direct mode additionally
load `config/sanity01_direct.yaml` and set `actuator_model_path` to the absolute
`Sanity/sanity-hardware/parameters/actuators.json` path.

| ROS topic | Message |
| --- | --- |
| `sanity01/ctbr_command` | `sanity_msgs/msg/CtbrCommand` |
| `sanity01/direct_thrust_command` | `sanity_msgs/msg/DirectThrustCommand` |
| `sanity01/zenoh_status` | `std_msgs/msg/String`, JSON status/error |

Commands use strictly increasing fresh ROS timestamps and frame
`sanity01/base_link` (x forward, y left, z up); hardware mode rejects simulated ROS
time. Publish at the policy rate, usually 50 Hz. Repeating at 100 Hz does not extend
the source command's validity. The native sender's default/maximum transport
validity is now 1 s, with 1 s receiver-status freshness and 1 s receive-silence
watchdog. The takeoff/circle wrapper retains its separate 40 ms state and controller
checks. The generic ROS command bridge retains its own 100 ms source-command
timeout. Both host and ESP must be updated together; older ESP firmware rejects
the longer leases. STOP remains latched after expiry. Earlier measurements below
used the former 100 ms lease.

Voltage telemetry now reports valid nonzero MSP battery measurements even outside
the motor-fit envelope. Zero still denotes unavailable/invalid/stale telemetry.
Direct-thrust bench readiness continues to require a 2S battery within
7.514–8.112 V; host thrust-mapping voltage checks remain separately controlled.

## Physical-unit conversion

For CTBR, reuse `CtbrMapping` from the UDP bridge:

```text
RC_axis = round(1500 + 500 * sign_axis * degrees(rate_rad_s) / full_scale_deg_s)
```

Defaults: signs (+1,+1,-1), scale (600,600,600) deg/s, model limits
(2.3,2.3,2.6) rad/s, collective limit 3.2 N. The signs follow the established RC
convention. Match Betaflight's linear rate profile to the host scale and verify
axis signs. Sanity01 retains rate profile 0: ACTUAL centre and maximum rates
600 deg/s on all axes (`rc_rate=60`, `srate=60`), zero expo and deadband. At this
scale each RC count represents 1.2 deg/s, with rounding error at most 0.6 deg/s.
The policy's physical rate limits are unchanged. Explicit older 150 deg/s host
overrides must be updated to match this FC profile. Collective thrust
uses piecewise-linear `thrust_newtons`/`throttle_rc` points. The supplied table
contains only (0 N,1000), so **positive CTBR thrust remains blocked** until a
measured collective-RC table is supplied. The per-motor fit is not that table.

The 16 October settings check retained RC smoothing ON with existing AUTO
cutoffs/factor 30 and restored `vbat_sag_compensation=100` in PID profile 0 after
a temporary change to zero. Sag compensation is not incompatible with CTBR;
the collective-RC mapping must account for it as well as voltage and the motor
fit. The final full FC dump matches the original backup. PID gains, other
filters, motor setup and UART configuration were retained; no firmware flash,
arming or positive motor commands were performed. See the
[final setup receipt](../../../../outputs/sanity_betaflight_ctbr_setup_20261016/final_setup_receipt.json).

The user confirmed standard Betaflight QUAD X output order: M1=RR, M2=FR, M3=RL,
M4=FL. Therefore policy [RR,FR,FL,RL] maps to **[1,2,4,3]**, stored in the direct
ROS overlay. The current STOP-only test did not independently identify physical
motor positions. The host inverts each hardware motor's supplied schema-v2 fit:

```text
omega_i = sqrt(T_i / k)
(V/8) * (a_i + b_i*q_i + c_i*q_i^2) = omega_i
u_i = round(1350 + 100*q_i),  q_i in [-1,1]
```

The inverse is solved on its monotonic calibrated interval. Positive commands must
fit **1250..1450 counts**, the measured speed range and **7.513333..8.1125 V**;
receiver voltage quantisation restricts it to 7.514..8.112 V. Zero means explicit
STOP=1000. Out-of-range thrust/voltage is rejected; no extrapolation or startup
model is invented. These are MSP external motor counts, converted by the FC to
DSHOT300, not PWM pulses on UART4.

## Freshness and output interlocks

Keys: `sanity/09/command/ctbr`, `sanity/09/command/direct_thrust`,
`sanity/09/status`. Command size is 40 bytes, network byte order:

| Offset | Field |
| --- | --- |
| 0 | `SZC1` magic |
| 4..7 | Version=1, mode (1 CTBR / 2 direct), device ID, enabled flag (u8 each) |
| 8 | Receiver epoch (u64) |
| 16,20 | Sender session and sequence (u32 each) |
| 24 | Echoed receiver timestamp ms (u32) |
| 28,30 | Validity ms (u16, 1..100); reserved zero (u16) |
| 32 | Four u16 values: roll/pitch/yaw/throttle RC or M1/M2/M3/M4 counts |

Status is 36 bytes (`>4sBBBBQIIIIHH`): `SZS1`, version, mode, ID, flags, epoch,
receiver time, accepted session/sequence, rejected count, battery mV, FC-status age
ms. Flags: enabled=1, freshness/reset-required=2, FC-ready=4. Enabled reports the
accepted output command, not independent proof of FC arming; direct bench never
arms the FC. Unknown voltage=0,
unknown age=65535. Readiness is mode-specific; direct readiness includes battery
and disarmed checks, whereas CTBR may stay ready while the FC is armed.

At about 50 Hz the ESP publishes its monotonic timestamp. The sender echoes it
unchanged and supplies at most the source command's remaining lifetime. Acceptance
requires unsigned wrap-safe `(now - echoed_time) < validity`; output stops at that
deadline. This includes the status/command round trip without synchronised clocks.
Session/sequence/epoch checks reject stale or reordered traffic. A separate 100 Hz
output task enforces expiry independently of Zenoh callbacks.

Unified MSP always sends ARM-low RC in direct mode, and motor-test STOP values
in CTBR mode, clearing any previous direct-output cache. CTBR checks compatible
FC identity/configuration, RX_MSP selection, recent status and RC/motor STOP ACKs.
Direct bench additionally requires positively disarmed FC, four motors, DSHOT300,
3D off, measured battery envelope and fresh ACKs. Dynamic replies expire after
150 ms; static identity/configuration checks after 1500 ms. Enabled direct bench
time is limited to 60 seconds. No such time limit is imposed on CTBR.

**Stock MSP_SET_MOTOR is not a flight interface.** It updates `motor_disarmed`,
which Betaflight applies while disarmed. It can spin motors despite the disarmed
flag. If ESP power or UART fails, the FC may retain the last motor-test value.
An **FC-side command timeout and stop/kill mechanism** is required for direct
flight; no such FC firmware extension is included here. MSP RC input, by contrast,
feeds Betaflight's existing receiver/control path.

Application storage retains one latest command. REAL_TIME/Express/Drop settings
avoid application history/replay; TCP can still queue/retransmit. Receiver deadlines
prevent delayed bytes becoming fresh on arrival. For additional drones, assign
unique IDs/keys; the current setup needs one producer and one drone only.

## Validation and remaining limits

The shared image was built/flashed and all three host/ROS packages rebuilt.
All **35 targeted checks passed** after the robotlab clock-ordering fix: 26
receiver/bridge/configuration/benchmark checks and nine host mapping, real Zenoh
and live ROS checks, with no skips in these targeted checks. Native C tests
cover parser/deadlines, SBUS/MSP bytes, disarmed-only mode claims and epoch fencing;
real Python Zenoh ↔ native Zenoh-Pico integration covers both control encodings,
positive calibrated inverse/permutation, mode transitions and reconnect/reset.

The physical disabled-only test ran **CTBR → direct bench → CTBR**, ten seconds
per phase: **2,965 command packets** and 101 independent FC USB checks. UART4 MSP
and RX_MSP stayed unchanged throughout. FC RC readback matched
[roll,pitch,yaw,throttle,ARM,AUX2]=[1500,1500,1500,1000,1000,2000], motor counts were
[1000,1000,1000,1000] and all RPMs zero. CTBR readiness passed; direct positive
readiness stayed false because no flight battery was connected. Four sampled
receiver freshness flags occurred: this is a functional mode-switch check, not
an uninterrupted-control or latency pass. No arming/positive commands were sent.

[Shared deployment receipt](../../../../outputs/sanity_direct_msp_20261016/unified_validation.json),
[shared-MSP hardware capture](../../../../outputs/sanity_direct_msp_20261016/unified_modes_verification.json),
[one-time settings receipt](../../../../outputs/sanity_direct_msp_20261016/uart4_unified_applied/receipt.json),
and complete private before/after FC dumps are in
`outputs/sanity_direct_msp_20261016/`. Earlier SBUS arm/disarm and original flash
backups remain in their dated output directories.

The earlier two five-minute `wgb` streams saw 33/18 status gaps ≥100 ms and 52/99
receiver freshness episodes with PC Wi-Fi power saving on/off respectively.
All ten host reconnects and two ESP restarts recovered, but disabling PC power
saving alone did not remove freshness faults. ACK sequence skips are not packet
loss counts; acknowledgements include network and status scheduling, not one-way
control latency. Keep telemetry independent of command production. A wired host
and nearby dedicated AP were tested below; the 100 ms watchdog remains enabled.
[Timing comparison PDF](../../../../outputs/sanity_zenoh_stability_no_powersave_20261016/stability_comparison.pdf).

### Robotlab component timing and clock-ordering correction

The PC now uses the **robotlab Ethernet profile**, 10.0.0.200/24; the ESP uses
Wi-Fi 10.0.0.109. The department Ethernet profile retains DHCP and its department
DNS servers. Diagnostic captures used robotlab; the initial production baseline
used the department profile on the same physical robotlab link and DHCP address.
This profile difference is recorded in the raw evidence.

Separate diagnostic keys and four timestamps remove the measured ESP reply
residence from Zenoh RTT: `RTT = (PC_reply - PC_send) - (ESP_send - ESP_receive)`.
This includes host/ESP dispatch, Zenoh, TCP/IP, Ethernet/AP and Wi-Fi. It does not
isolate radio airtime or require symmetric paths. Optional instrumentation is
controlled by `CONFIG_SANITY_TIMING_DIAGNOSTICS=y`; it is **off in the deployed
normal image**. Normal command/status wire formats are unchanged.

| Measurement after correction (CTBR) | Median | p95 | p99 |
| --- | ---: | ---: | ---: |
| Residence-corrected Zenoh RTT | 5.54 ms | 8.35 ms | 10.88 ms |
| Host API to first publish | 5.05 ms | 9.60 ms | 10.02 ms |
| ESP receive to UART queue | 4.69 ms | 9.40 ms | 10.58 ms |

The capture contains **1,684/1,684 probe replies over 84.2 seconds**, with no
sampled freshness flags or expiry events. It was interrupted before the direct
phase; it is not a completed two-mode endurance test. One-way command transport
is approximately 2.91 ms median under a fitted clock model, with median causal
bounds 0–6.45 ms. API-to-FC-parse median bounds are 15.97–30.50 ms. These are
model-based observation bounds, not exact actuation times. MSP RC wire time is
1.91 ms at 115200 8N1; ESP ACK polling adds up to roughly one 10 ms output period.
Component percentiles must not be added as though their tails coincide.

The earlier full capture had **184/185 expiry events caused by a timestamp race**:
a caller sampled time before locking state, then a concurrent receive updated
`last_rx_ms` to a later millisecond. Unsigned subtraction made the fresh command
appear extremely old. Time is now sampled **inside the shared lock** in output,
receive and status paths. Command validity remains at most 100 ms and the other
FC/bench interlocks are unchanged. The older generic M4 flag lacked cause bits,
so this finding does not establish its specific historical cause.

A genuine **2.812 s corrected return-path RTT** occurred in the earlier direct
capture and remains unresolved. Its forward arrival was only a few milliseconds;
the delay accumulated on the reply path after the ESP timestamp. That capture
received 2,357/2,400 probes; its 43 missing replies match diagnostic reply-queue
overflow and must not be described as 43 lost network packets. Full before/after
captures and failed instrumentation pilots are retained.

After restoring normal firmware, a 30-second **CTBR → direct → CTBR** check sent
**3,006 disabled packets** with **101 independent FC USB checks** and no sampled
freshness flags. The FC remained disarmed, motor commands were all 1000 and RPM
zero. This verifies transport and mode switching; positive direct output and
loaded actuation were not retested. Temporary USB arming inhibit was restored
to its original value after independent stopped-output verification.

[Component timing PDF](../../../../outputs/sanity_robotlab_latency_20261016/robotlab_delay_report_final.pdf),
[clock fits and detailed statistics](../../../../outputs/sanity_robotlab_latency_20261016/fixed_analysis.json),
[deployment receipt](../../../../outputs/sanity_robotlab_latency_20261016/deployment_receipt.json).

With an optional timing image, both USBs connected and Configurator closed:

```sh
python scripts/measure_transport_timing.py \
  --expected-uid 2a0039000a47333135373835 --duration 120 \
  --output /path/to/NEW_transport_capture
```

The harness permits only disabled neutral commands, runs the normal 50 Hz source
and 100 Hz publisher, adds 20 Hz timestamp probes and applies/restores temporary
USB arming inhibit around independent FC checks. The exact harness used for the
reported captures is saved as `capture_harness_used.py` with the raw artifacts.
Its later automatic-inhibit cleanup refinement was syntax checked; those captures
used the separately applied inhibit and monitored it throughout.

### Command-to-FC and unloaded motor timing

The props-off benchmark uses the same production Zenoh/MSP image, with the FC's
independent USB port for readback and an explicit temporary MSP arming inhibit.
CTBR sends zero-thrust roll markers; it measures RC input application, not body-rate
response. Direct uses physical thrust inputs computed to produce bounded
1250–1300 motor-test counts, one motor at a time. Its RPM is unloaded and does
not validate the loaded actuator fit. No persistent FC setting or firmware was
changed for these tests; no battery-voltage exception was used.

Each USB query takes about 16 ms to return on this setup. Sending independent
queries together reduces the sampling overhead. The refined **readback upper
bounds**, conditional on observed targets, were:

| Path | Targets observed/attempted | Median upper bound | p95 upper bound |
| --- | ---: | ---: | ---: |
| CTBR → FC RC input | 21/24 | 41.3 ms | 54.3 ms |
| Direct → FC motor command | 10/10 | 37.0 ms | 66.2 ms |

Median application-interval width was 30.8 ms for both paths: these are **not
exact actuation delays**. The 10 direct targets span four runs; three runs aborted,
including one before its first timing step. One of two CTBR runs aborted and
three targets were unobserved. The p95 values have small samples and are not
worst-case guarantees. All failed attempts remain in the output directory.

Eight complete, nonfaulted direct steps gave median host-update to reported RPM
10%/50%/90% crossings of 42.2/60.5/90.1 ms. These are fractions of the observed
unloaded speed change, including ESC/FC telemetry and USB polling, not actuator
time constants. Propellers change the mechanical load and transient response.

Command freshness stops still occurred. Captured FC telemetry ages stayed at
0–20 ms; two source faults followed more than 100 ms without a status arrival.
The current test therefore does not establish uninterrupted control. The
watchdog was retained, and every run ended disarmed, ARM low, all motor counts
1000 and reported RPM zero before the USB arming inhibit was restored.

[Bench timing PDF](../../../../outputs/sanity_command_actuation_20261016/command_actuation_report.pdf),
[plots](../../../../outputs/sanity_command_actuation_20261016/timing_plots.png),
[complete statistics](../../../../outputs/sanity_command_actuation_20261016/analysis.json)
and per-run raw JSON captures are in `outputs/sanity_command_actuation_20261016/`.
The host inverse now tolerates only floating-point roundoff at exact calibrated
endpoints; genuine out-of-envelope inputs remain rejected.

Repeat only with props removed, both USBs connected, Configurator closed and
an active battery within the original measured voltage envelope for direct:

```sh
python scripts/measure_command_latency.py --mode ctbr --props-removed \
  --expected-uid 2a0039000a47333135373835 --trials 80 \
  --output /path/to/NEW_ctbr_timing
python scripts/measure_command_latency.py --mode direct-counts --props-removed \
  --expected-uid 2a0039000a47333135373835 --trials 6 --motors 1 \
  --output /path/to/NEW_direct_timing
```

The benchmark rejects commands outside zero-thrust CTBR or a single motor in
1250–1300 counts, retains query request/reply timestamps, aborts direct output on
an interlock fault, and verifies independent USB STOP/zero-RPM/ARM-low cleanup.
It holds arming inhibit throughout; direct MSP motor testing can spin motors
while the FC is disarmed. The original actuator voltage and command envelopes
remain enforced. CTBR uses explicit disabled recovery after freshness faults,
which is recorded rather than counted as uninterrupted operation.

From the receiver repository root with host packages available:

```sh
python -m unittest discover -s zenoh_receiver/tests -v
cmake -S zenoh_receiver/tests -B /tmp/sanity_zenoh_emulator_build \
  -DZENOH_PICO_SOURCE=/tmp/sanity-zenoh-pico-1.10.1
cmake --build /tmp/sanity_zenoh_emulator_build -j 4
export SANITY_ZENOH_EMULATOR=/tmp/sanity_zenoh_emulator_build/receiver_emulator
python -m unittest discover -s ../ros2_sanity_ws/src/sanity_zenoh/test -v
# Source the ROS overlay and set SANITY_TEST_ROS=1 for live ROS topic tests.
```

Repeat the disabled-only hardware cycle from this directory with both USB cables,
Configurator closed and the host environment active:

```sh
python scripts/verify_msp_modes.py \
  --expected-uid 2a0039000a47333135373835 \
  --duration 10 --output /path/to/NEW_modes_check.json
```

Sources: [Zenoh-Pico 1.10.1](https://github.com/eclipse-zenoh/zenoh-pico/tree/1.10.1),
[Betaflight MSP handler](https://github.com/betaflight/betaflight/blob/4.5.1/src/main/msp/msp.c#L2481),
[MSP serial processing](https://github.com/betaflight/betaflight/blob/4.5.1/src/main/msp/msp_serial.c#L524),
[serial task scheduling](https://github.com/betaflight/betaflight/blob/4.5.1/src/main/fc/tasks.c#L478),
[QUAD X motor order](https://github.com/betaflight/betaflight/blob/4.5.1/src/main/flight/mixer_init.c#L66).


## FC telemetry and ESP reset/power diagnostics —22 October2026

The unified image now includes bounded read-only FC gyro/motor/RPM telemetry and
ESP boot/reset/Wi-Fi health. Existing mission runners record it automatically.
See [TELEMETRY.md](TELEMETRY.md) for signal rates, timestamp limitations and the
read-only `run_sanity_telemetry.sh --duration 30` capture command. ESP supply rail
voltage is not measured; reset diagnostics include a reported brownout cause.


## Disarmed-only transport isolation experiments (2026-10-22)

The new comparison profiles are `tcp-shared`, `tcp-split` and `udp-split`. Their
firmware sets `CONFIG_SANITY_COMMS_TEST_ONLY=y`, rejects every enabled command,
and cannot fly. Normal build defaults and the one-second control lease remain
unchanged. `CONFIG_SANITY_SPLIT_TELEMETRY=y` moves FC records and bulk timing
traces onto an independent session/task; control/status/probes remain separate.
Optional isolated sender arguments are `telemetry_listen`, `telemetry_connect`,
and `diagnostics`; existing sender calls retain their original behavior.

From the drones workspace, with the corresponding test image installed:

```sh
Sanity/src/ros2_sanity_ws/scripts/run_sanity_comms_test.sh \
  --profile udp-split --duration 600
```

This runs ten minutes per telemetry phase (off then on), only disabled CTBR
commands, with a50Hz source and100Hz transmitter. Use `--telemetry on` or `off`
for a single phase, `--mode direct` for disabled motor-cache mode (powered FC
needed for a disarmed mode switch), and `--host-ip` to bind a different address.
The defaults bind only robotlab host10.0.0.200: UDP7447 control, TCP7448 telemetry.
TCP comparison images use TCP7447; shared telemetry also uses that session.
The existing ROS/DDS interfaces and PC network profiles are unchanged.

Every run records raw events, trace/configuration ACKs, plan and summary below
`outputs/sanity_comms_DATE_PROFILE/`. The runner requires an ACK from the
disarmed receiver owner before toggling FC telemetry. It never modifies FC
settings and never requests ARM. Ctrl-C stops testing and sends disabled output.
Its50ms gap limit is an offline qualification criterion, not a new watchdog.

Metrics include actual accepted-command gaps, callback duration, accepted receipt
to UART-write age, UART cadence and round-trip probes after ESP residence is
removed. No one-way latency or precise FC sensor/actuation timestamp is claimed.
Full traces are required; loss fails qualification. Trace type6 reports UART-poll
lateness over2ms, type7 counts unmatched ACK backlog overflows separately from
trace loss (expected with the FC unpowered). Firmware output remains disabled.

Build an image without flashing; base sdkconfig contains private Wi-Fi settings:

```sh
python Sanity/src/sanity_receiver/zenoh_receiver/scripts/build_comms_firmware.py \
  --profile udp-split --base-config /path/to/private/sdkconfig \
  --idf-path /path/to/esp-idf --pico-source /path/to/zenoh-pico-1.10.1 \
  --output /path/to/new/private/build --optimize
```

Use the same `--optimize` choice for every comparison image. It selects240MHz
and performance compilation; private build outputs have restricted permissions.
The pinned C unstable API is enabled in test builds only to select best-effort
status/probes explicitly on UDP. Completed image receipts give hashes and state
that flashing has not happened; flashing requires the normal battery-unplugged
USB setup. The complete non-secret ESP audit and local evidence are in
`outputs/sanity_comms_isolation_20261022/ESP_CONFIG_REVIEW.txt`.

Native test target `comms_emulator` uses the real pinned Zenoh-Pico and firmware
control/configuration code. Its optional delayed-callback injection verifies
that the qualification report detects a stall; successful loopback tests do
not qualify the actual Wi-Fi link.

The runner exits1 if a qualification criterion fails, while preserving all data.
A phase needs adequate full-rate trace coverage (at least90% of published commands),
no new trace drops, stable receiver epoch and accepted-command gaps<=50ms. These
are offline checks only. If trace loss occurs, a large gap between surviving traces
is not evidence of an equally large control gap: use the separate status frontier.
Delayed FC batches are attributed by their UART command sequence, rather than their
arrival phase. Unpaired MSP ACKs are separate from real trace-queue loss. Type5/code4
traces now time bulk FC publications; stall events take precedence over ordinary
traces when that diagnostic queue is full. This retains loss counters honestly.
The matched USB-only short comparison is in hardware_comparison.csv/json; none
of the three profiles passed every50ms phase criterion. USB-only FC-unpowered
checks do not qualify full powered-FC telemetry load, battery power or arena RF.

Final test instrumentation logs every command callback and primary output write
(CTBR:200; direct bench:214), samples secondary output/ACK traces1-in10, and
records publication stalls above5ms. Routine1-2ms put calls are not logged as
stalls. This limits observer load without changing any command or flight deadline.
The original short benchmark binaries are retained as bench_v1_application.bin;
final image receipts identify the reduced-trace revision. Keep firmware versions
matched when comparing profiles. Additional final-image smoke data are separate
from the matched short comparison; they must not be pooled as a fair A/B sample.


### Flight profile after the UDP/split communications tests

`build_comms_firmware.py --profile udp-split --flight --optimize ...` builds
a flight-capable image with the same UDP control/status endpoint (host7447),
independent TCP telemetry endpoint (host7448),240MHz/-O2 settings,command executor
priority14 and reduced diagnostic publication load as the test profile. It removes
the test-only enabled-command rejection and the bench telemetry-toggle endpoint;
FC telemetry remains enabled. Without `--flight`, the builder still produces a
disarmed-only test image. The helper never flashes hardware.

The shipped circle,experimental5m/s MPC,pitch/identification and ROS bridge YAML
profiles now select `udp/10.0.0.200:7447`,
`telemetry_listen: tcp/10.0.0.200:7448`,and `transport_diagnostics: true`.
Existing mission commands use those profiles. The read-only telemetry capture
defaults to the same pair; use `--telemetry-listen ''` with a legacy shared-session
receiver. Generic sender API defaults remain compatible with older TCP clients.
Simulation explicitly disables the external bulk listener/diagnostics and retains
its isolated localhost transport. No DDS interface or mocap configuration changed.

The one-second lease/watchdog,calibration,rate limits,UART4 MSP wiring and USB
passthrough remain unchanged. Flight mode still requires explicit disabled claim,
zero-thrust enable and normal FC readiness checks. Direct MSP motor output remains
a disarmed bench interface. Restoring flight capability does not qualify aggressive
missions or eliminate measured60..80ms command-reception tails.

### Split telemetry reconnection after a mission exits

The pinned Pico1.10.1 library can lose its TCP transport while
`z_session_is_closed()` remains false; interest filtering can also make
`z_publisher_put()` return success without transmitting. Split telemetry therefore
uses the public transport-connectivity listener (`session_watch.c`,history=true)
to recreate its own session after the peer disappears. Split builds enable
`Z_FEATURE_CONNECTIVITY=1`; global `Z_FEATURE_AUTO_RECONNECT` remains off and no
control epoch,arming sequence or command deadline is changed by this watcher.

Regression: build the native tests with `-DZ_FEATURE_CONNECTIVITY=1
-DZ_FEATURE_UNSTABLE_API=1`,then set `SANITY_SESSION_WATCH_EMULATOR` to the built
`session_watch_emulator` executable and run `tests/test_session_watch.py`. This
uses loopback only and reconnects four host sessions without restarting the peer.
Hardware verification should likewise use repeated disabled-only host sessions
with the same ESP boot ID. No power cycle should be required between missions.


### Extended tuning RPC: paired damping and isolated boost

With `SANITY_MSP_TUNING`, command0 capability reply retains the original7bytes
and appends feature bits: bit0=bounded roll/pitch D-min,bit1=bounded feedforward
boost. New hosts accept7/8bytes; new tuning profiles require the extension before
arming. Original API1.46/BF4.5.1/profile/disarm fields unchanged.

MSP95 additionally permits only bytes39/40(D-min roll/pitch) and52(boost).
D-min stays within100..110% of the captured original and<=active D ceiling;
boost can only decrease from original,by at most5units per transaction. Exact
original restore is allowed. I,yaw,other advanced fields remain protected.
Paired host updates use MSP202 D first,then MSP95 D-min,with complete readback.
Automatic lease-loss restore requires receiver disabled AND FC disarmed.
No wire command/rate mapping/watchdog/voltage conversion changes.
See ROS package AUTOTUNE.txt and outputs/sanity_rate_damping_20261022/.

Control/status reconnection also uses the connectivity watcher in connectivity-enabled builds. If the host peer disappears, output stops, the control session is recreated with a fresh epoch, and a new disabled claim is required. This addresses UDP status loss across launcher restarts as well as the separately watched TCP telemetry path. The reconnect regression covers four successive localhost hosts over both TCP and UDP.


### Session teardown ordering (Sanity24 PANIC fix, 26 October 2026)

The pinned Pico build can destroy a disconnected transport while the session
still reports open. Dropping a filtered publisher at that point may send an
interest-removal message through a destroyed transmit mutex. This was captured
on Sanity24 as `spinlock_acquire_impl` while `telemetry_task` dropped the trace
publisher, with disabled-only traffic.

Both receiver tasks now explicitly `z_close` the session before dropping any
publisher, subscriber or connectivity listener, then release the session and
retry. Closing first marks the session closed and joins its background runtime
and callbacks. Control loss still calls STOP before the potentially blocking
close. All optional control handles start empty, and declaration failures use
the same cleanup path instead of dropping entities from an open session or
aborting on partial initialization. A failed explicit close remains fail-fast;
cleanup must not proceed against an unquiesced session.

`tests/test_session_watch.py` exercises real pinned-Pico TCP/UDP restarts with
matched filtered publishers and receive callbacks, plus teardown at five partial
initialization stages per protocol. Set `SANITY_SESSION_WATCH_EMULATOR` to the
native executable built from `tests/CMakeLists.txt` with connectivity and unstable
API enabled. Hardware regression evidence and installed image receipts are in
`outputs/sanity24_teardown_fix_20261026/` at the workspace root. Firmware changes
apply only to receivers that are rebuilt/flashed; this does not update other
installed S3 or C5 images automatically. Bench restart success alone does not
qualify simultaneous aircraft radios or flight-power operation.


### Three-receiver test with installed flight firmware

The shared-listener runner `Sanity/src/ros2_sanity_ws/scripts/run_sanity_fleet_comms.sh`
exercises IDs01/15/24 simultaneously with disabled-zero CTBR and normal full
telemetry. It does not use COMMS_TEST_ONLY or its telemetry-toggle endpoint.
Reset without BOOT to leave passthrough, remove props and use normal flight-power
wiring for the hardware comparison. See ROS package `FLEET_COMMS.txt` for the
5minute command, independent per-receiver results and latency limitations.
No receiver image/settings update is needed for this test.

The receiver labelled Sanity15 (ID15,10.0.0.115) is now installed on the Sanity21
airframe; the original ID21 receiver is excluded from the comms-test configuration.


### Bounded diagnostics and UDP-unicast baseline —26 October2026

The new `build_comms_firmware.py --diagnostic-level minimal` profile samples FC
telemetry and reports actual callback-arrival counters instead of per-command
traces. `--backend udp-unicast` carries the same40-byte commands using explicit
ESP IPs/ports; it retains the shared receiver acceptance/calibration/lease code.
It is a comparison backend, not an automatic ROS deployment transport change.
The pinned Pico build also requires the transport-lifetime mutex patch applied
by `scripts/prepare_pico.py` in a private checkout. Existing images are not
updated automatically. Full characterization needs `--diagnostic-level full`.

Exact commands, limitations and firmware-profile matching are documented in
`Sanity/src/ros2_sanity_ws/src/sanity_zenoh/COMMS_COMPARE.txt`. The comparison
entry point is `scripts/run_sanity_comms_compare.sh` in the ROS workspace. Both
backends share one host transport, per-device state and disabled-only guards.


### Optional onboard HX711 quarter-bridge acquisition —27 October2026

`build_comms_firmware.py --hx711` enables a low-priority raw acquisition task
(A128), DOUT XIAO D0/GPIO1, SCK D1/GPIO2, 3.3V logic. It sends versioned JSON to
host UDP7460, independently of the command/status protocol. Disabled by default;
no auto-tare or force-feedback control. The prepared Sanity09 image in
`outputs/strain_gauge/sanity_onboard_tension_20261027/firmware_iram_recovery` is disarmed bench-only and
was flashed to Sanity09 on27 October2026; physical HX711 readout verified, force calibration pending. See [wiring, bridge math and hanging-drone calibration](../../../sanity-hardware/docs/onboard-tension-calibration.md).


HX711 timing update27October: GPIO functions selected into IRAM; clock reader
IRAM/noinline (ELF0x40376fe4) with cycle-counter upper bound at fixed240MHz,
PMdisabled.45us guard retained. New image installed toSanity09 at user request;
current receipt `outputs/strain_gauge/sanity_onboard_tension_20261027/recovery/flash_receipt_noinline.json`.
Host calibration automatically discards fault/settling fragments and retries
the same weight for a complete clean window; persistent failure pauses and
preserves completed measurements. No tension feedback/flight output enabled.
