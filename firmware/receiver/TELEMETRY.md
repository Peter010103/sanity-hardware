# Sanity FC and ESP telemetry

The receiver publishes read-only observations on `sanity/09/telemetry/fc` (replace
09 with the configured device ID). Both the normal deployment and pitch runners
subscribe automatically through `SanitySender`. Their `events.jsonl` contains
`zenoh_rx_fc_telemetry` events with raw `payload_hex`, decoded records, PC monotonic
nanoseconds and Unix nanoseconds. Existing mission commands need no new flags.

For a telemetry-only capture, with no aircraft commands or ROS dependency:

```sh
Sanity/src/ros2_sanity_ws/scripts/run_sanity_telemetry.sh --duration 30
```

Run from the drones workspace. It creates `outputs/sanity_telemetry_DATE_TIME/`
with raw/decoded `events.jsonl` and `summary.json`. Do not run another process
listening on port7447 concurrently. To attach to an existing local Zenoh listener,
use `--connect tcp/127.0.0.1:7447`. An existing listener may itself be controlling
the aircraft; this program is only an additional subscriber.

To summarize a recorded mission without networking:

```sh
Sanity/src/ros2_sanity_ws/scripts/run_sanity_telemetry.sh \
  --events outputs/YOUR_RUN/events.jsonl
```

The summary reports achieved response rates/gaps, RC UART-output cadence, telemetry
drops, decode errors, RPM quality and ESP boot/reset information. No received
telemetry is required by a flight interlock and none modifies the selected CTBR.

## Recorded signals

| Source | Target polling rate | Contents |
|---|---:|---|
| MSP_RAW_IMU102 |100Hz|Signed gyro counts; host applies the recorded sensor scale; accelerometer/magnetometer counts|
| MSP_MOTOR104 |50Hz|Eight FC motor outputs in Betaflight external output units|
| MSP_MOTOR_TELEMETRY139 |10Hz|Per-motor mechanical RPM, invalid-packet percentage and raw ESC fields|
| MSP_RC105 |10Hz|FC received input channels; these are not PID rate setpoints|
| MSP_BATTERY_STATE130 |Existing10Hz|FC voltage/current/consumption|
| MSP_SET_RAW_RC200 ACK |Existing100Hz|MSP response observation; no command sequence is echoed by FC|
| ESP UART writes |Existing100Hz each|Exact RC and motor-test-cache payloads plus accepted command session/sequence|
| ESP health |1Hz|Boot identity/count, reset reason, uptime, Wi-Fi reconnect history/RSSI, free/minimum heap|

The control output and essential compatibility/status requests are written before
optional telemetry queries on the existing115200baud UART. At most one optional
query per command ID is outstanding; a100ms timeout or MSP error backs that query
off for1s. A busy receive buffer suppresses optional reads. Missing/unsupported RPM
therefore becomes a telemetry timeout/error, not a controller error. Target rates
are upper targets, not promises of FC acquisition or delivery rates.

The64-record FreeRTOS queue never waits for network transmission. Batches are at
most1400bytes, published after receiver status with background priority and DROP
congestion policy. Queue/publication failures increment a cumulative drop counter.
The shared TCP connection can still experience head-of-line blocking; this design
does not claim complete isolation from network stalls. Verify output cadence in
an actual disarmed capture before flight with the new polling load.

`MSP_MOTOR104` describes FC motor outputs. `MSP_SET_MOTOR214` telemetry describes
the disarmed motor-test cache written by our existing unified bridge; its STOP
values during CTBR must not be interpreted as actual spinning motor outputs.
RC writes use the established AETR order: roll,pitch,throttle,yaw,ARM,ACRO,AUX3,AUX4.
Gyro readings remain in Betaflight-reported axes. Betaflight4.5.1 MSP_RAW_IMU
returns filtered gyro values divided by the sensor scale despite the helper name
`gyroRateDps`. The recorded Sanity ICM42688P uses2000/32768 degrees/s per count.
The host retains raw counts and applies that scale; other sensor configurations
need an explicit override. Logs before this correction retained raw packets but
labelled counts as degrees/s; re-decode their `payload_hex` before analysis.
 No assumed mocap/model-axis
transformation is applied to them by this diagnostic decoder.

## Timing and RPM interpretation

All ESP times are64-bit monotonic microseconds. Output records include the accepted
Zenoh callback entry time, UART write-call entry and return. The write return
means driver acceptance, not FC application or completed physical actuation.
FC response records include optional query-write time, first parser observation
and completed parser observation. UART is polled each10ms; parser timestamps do
not represent the FC sensor acquisition time or exact byte-arrival time. MSP has
no transaction IDs: query matching is by command ID, and an old reply after timeout
cannot be unambiguously associated with a retry. Raw ACKs are not assigned command
sequences. PC arrival timestamps must not be subtracted from ESP clocks to claim
one-way latency without a clock-mapping procedure.

Mechanical RPM is already converted by Betaflight; do not apply motor pole count
again. When the existing physical CTBR profile supplies kT, the host additionally
estimates `sum(kT*(RPM_i*2*pi/60)^2)` for four positive-RPM motors with invalid-packet
percentage below100%. This is an RPM/static-calibration force estimate, not measured
flight thrust. The FC can report100% when quality statistics are unavailable;
the decoder conservatively omits the force estimate while retaining raw RPM.
Raw ESC voltage/current are retained because extended DShot fields have different
availability/conversion paths. Zero values do not establish sensor availability.

## ESP power diagnosis

Health records contain `esp_reset_reason()`, including POWERON, BROWNOUT, PWR_GLITCH,
software reset, panic and watchdog causes. A boot ID changes on receiver startup;
NVS boot count increments once per normal receiver startup (passthrough startup
is excluded). A zero/unavailable NVS count is reported as null. Wi-Fi reconnects
within a running firmware retain boot ID/uptime and increment disconnect/connect
counters. A new Zenoh epoch alone is not evidence of a power reset.

ESP3.3V/USB rail voltage is not measured by this hardware connection. The reported
FC battery voltage is a separate rail. A total supply loss may reboot as POWERON,
which does not prove a brownout cause; BROWNOUT is positive reset evidence when
reported, but its absence does not exclude power loss. An oscilloscope/rail monitor
is needed to capture the voltage dip itself. Health is resent after reconnect;
there is no claim that a failing radio can report its last moments.

## Wire format

Network integers use big endian; raw MSP payloads retain Betaflight little endian.
Batch header12bytes: `>4sBBHI`: magicSZF1, version1, device, record count, cumulative
ESP telemetry drops. Each record header44bytes: `>BBBBQIIQQQ`: type, MSP command,
flags, payload length<=128, receiver epoch, session, sequence, time0,time1,time2.
Types:1UART output,2FC reply,3MSP error,4query timeout,5ESP health. Flag1 marks output
command context; flag2 means the parser had a matching outstanding query ID.
For type1 times are command receipt/write entry/write return; for replies they
are first parser observation/query write entry/parser completion. Session/sequence
are only meaningful for type1. Health payload48bytes is`>Q7IQbB2x`: boot ID,count,
reset reason,heap,min heap,disconnects,connections,last disconnect reason,last
disconnect time,RSSI,channel. Health uptime is the record's final timestamp.

Protocol reference: Betaflight4.5.1 `src/main/msp/msp.c` and `msp_protocol.h`:
https://github.com/betaflight/betaflight/blob/4.5.1/src/main/msp/msp.c
https://github.com/betaflight/betaflight/blob/4.5.1/src/main/msp/msp_protocol.h

No local rate/RPM feedback controller, thrust correction or MPC change is introduced
by telemetry. It supplies the observations needed to evaluate those changes.
