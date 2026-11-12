# HX711 raw telemetry and source timing

Prepared 27 October 2026 for Sanity09 (receiver9,10.0.0.109). No firmware was
flashed and no aircraft commands, arming, FC configuration writes or timing
probes were sent in this work. Only Sanity09 has verified physical strain data.

This is acquisition/health support for a HOST TAUT/SLACK/UNKNOWN estimator.
There is no tare, baseline tracker, count threshold, high-pass detector, drift
correction, force conversion, cable decision or tension-dependent flight action
in the firmware. A loaded signal declining with creep does not establish SLACK.
A positive signal cannot exclude a snag. Force feedback/load sharing stay disabled.

## Compatibility

Destination remains raw JSON over UDP10.0.0.200:7460, independent of Zenoh.
Schema remains `sanity_hx711_v1`, signed24-bit `raw` unchanged. Existing fields
and flag bits remain:1 missing/not-ready,2 ADC rail saturation,4 clock/stuck-data
fault,8 settling. ANY nonzero flags makes the conversion unusable. Missing
heartbeats have raw0/ready_us0; that zero is not a measured zero. No flag bits added.
Legacy validators that accept extra fields still work and still reject unknown
flag bits. New parsers accept legacy packets WITHOUT the extension. A partial
extension or unsupported `timing_v` is rejected explicitly.

Additive `timing_v:1` fields:

| Field | Meaning |
|---|---|
| `receiver_boot_id` |16-digit hex identity also published in receiver ESP-health telemetry. |
| `receiver_boot_count` |Same NVS boot counter as receiver health;0 means unavailable. |
| `ready_lower_us` |Timestamp BEFORE last observed DOUT-high poll preceding this ready observation;0 means no bracket available. |
| `shift_end_us` |Timestamp after all25 A128 pulses and the final low delay;0 for a missing-status packet. |
| `acquired_us` |Timestamp after sample health assessment and any commanded converter reset, before enqueue. |
| `reset_us` |Completion time of the last firmware-commanded HX711 reset;0 if GPIO initialization failed/no reset was issued. |
| `queue_drops` |u32 cumulative records discarded locally by queue overflow/age; distinct from send failures. |

Every present extension field is validated. New `record_onboard_tension.py`
preserves metadata in raw JSONL/CSV and records source intervals, local dispatch
age, health reasons and baseline-generation changes. No cable classification.
The old `samples.csv` columns remain; additions are appended. Historical files
have not been changed. Calibration clean-window collection now restarts after
sequence gaps/reordering as well as flags/reset changes; it does not fit across
those discontinuities.

## Exact timestamp and identity contract

All source microseconds come from `esp_timer_get_time()`: monotonic time since
ESP Timer initialization shortly before app_main, NOT Unix time and NOT since
HX711 task start. Same origin as receiver diagnostics/probe/health `*_us`.
Receiver-status `receiver_ms` uses `(uint32_t)(esp_timer_get_time()/1000)`.

`ready_us` retains the time just after observing DOUT low, before shifting24 data
bits plus the25th gain-selection pulse. It is a POLLING observation, not an
interrupt-captured falling edge, conversion start/midpoint or instantaneous force.
When `ready_lower_us>0`, the DOUT-ready edge lies between that lower observation
and ready_us (subject to GPIO/clock measurement resolution). Scheduling/polling
latency is in the bracket. A zero lower bound means unknown, not edge at boot.
Internal ADC filtering/settling and mechanical strain transfer are not timed by
this bracket. The observed~90.3Hz is not replaced with a nominal80Hz contract.
Use differences of actual source observations for filtering/dwell intervals.

`read_us` retains the legacy pre-JSON-format/UDP-dispatch timestamp, previously
misleadingly named "read". It is NOT the last SCK edge, completed network send,
host receive time or sensor force time. With separate tasks it includes local
queue/scheduling delay. `shift_end_us-ready_us` bounds the software shift span;
`acquired_us-shift_end_us` includes health processing and reset if faulted;
`read_us-acquired_us` is local age at preparation, not a pure measured queue wait.
Formatting and sendto happen AFTER read_us; no post-send timestamp is claimed.

`boot_id` keeps its original separate RANDOM ACQUISITION-instance identity. It
was not previously the receiver-health boot identity. New receiver_boot_id/count
explicitly associate it with health/probe timestamps. A Zenoh reconnect/epoch
change alone does not change this ESP clock or converter identity. ESP reboot
changes identities and resets source time. Deep sleep restarts the timer/app;
this receiver does not use it. NVS boot count is ancillary and may be unavailable.

Source times are signed64-bit microseconds (~292,000-year range). Status ms wrap
every49.71days: unwrap modulo2^32 ONLY within a verified common receiver boot,
with bounded observation intervals. Sequence, send_drops, queue_drops and resets
are u32 modulo2^32. `seq` advances for EVERY emitted conversion/status record
before enqueue, even one later discarded. Forward sequence deltas are valid only
in the half range0<delta<2^31; equal/old/ambiguous samples cannot refresh freshness.
`resets` starts1 after the startup reset and advances on each commanded clock
recovery reset;0 is possible on wrap (or GPIO init failure, with missing flags).
ANY reset-counter/reset_us change invalidates the host baseline and persistence.
The cycle-count HIGH guard uses unsigned cycle subtraction across cycle wrap;
fixed240MHz/PM disabled is enforced by the HX711 build checks.

These counters detect SOFTWARE-COMMANDED converter resets. An isolated HX711
power dip/reset is not independently observable from a scalar ADC stream and is
not guaranteed to increment them. Source gaps/ambiguous signal must invalidate
confidence; supply/reset monitoring would be needed to prove every such event.

Host monotonic arrival is a receive watchdog/transport observation, not a source
clock synchronization. UDP may delay, reorder, duplicate or silently lose a
successful send. Packet read/ready times cannot establish absolute packet age
without a separately verified source-host clock relation and latency bounds.
Existing receiver timestamped probes can help establish a bound AFTER matching
receiver_boot_id; ADC filter/mechanical delay still needs measurement. No host
arrival timestamps, nominal sample rate, RTT/2 or cue Enter times establish
synchronized strain/mocap timing by themselves.

## Failure isolation and timing protections

Acquisition task priority2 polls once per RTOS tick, leaves SCK LOW while waiting,
and issues exactly25 A128 pulses. Each short HIGH is separately interrupt-masked;
waiting, all LOW intervals, the80us reset HIGH and network work are not masked.
The noinline read loop and GPIO get/set remain in IRAM. HIGH software upper bound
>45us or stuck-low after the25th pulse flags4 and commands reset. Rail values flag2.
Startup/reset marks at least600ms PLUS four post-reset conversions as settling;
the fault conversion does not consume the new four-conversion discard allowance.
No flagged/settling conversion is presented as healthy.

A four-record nonblocking queue feeds a DIFFERENT UDP task at priority1. A full
queue discards oldest records, incrementing queue_drops; sequence gaps remain
observable. Records older than100ms at dequeue are discarded, never replayed as
fresh local evidence. Socket creation retries at most once per second;
O_NONBLOCK sendto makes one attempt, with no per-packet retry or acknowledgement.
Failure increments send_drops (unsent local datagrams); the next successful packet
reports preceding failures. A successful UDP send is not proof of reception.

Missing ADC produces missing-status heartbeats approximately once per second.
Source rates and watchdogs do not rely on exact heartbeat/RTOS timing. Socket or
UDP-task failure cannot stop acquisition. Optional GPIO/queue/task setup failures
log and leave the NORMAL RECEIVER running rather than abort/reset it. If heartbeat
creation/delivery itself fails, the host sees silence and must return UNKNOWN.
No sensor code holds the FC/state mutex or uses UART/Zenoh publishers. Normal
priorities remain FC15/control RX14/telemetry RX6/Zenoh5/diagnostics4. Pulse critical
sections and shared CPU/heap/Wi-Fi still REQUIRE actual workload qualification.
Nonblocking APIs are not a hard WCET guarantee for shared lwIP internals.

## Host handoff

`tools/acquisition/hx711_timing.py:HX711Health` is a causal HEALTH helper, not the
TAUT/SLACK/UNKNOWN classifier. Validate device/source/schema/pins/channel/gain,
then call observe(packet, host_monotonic_ns). On decoder failure call
health.invalidate(reason), without refreshing freshness. Poll check(now) independently even
when packets stop. Duplicates/old packets cannot refresh the watchdog. Faults,
loss/counter changes, source-clock reversal, stale local age and receive gaps
make the sample unusable; boot/converter changes advance baseline_generation.
The helper's configurable100ms source-gap/local-age and250ms receive-health
limits are diagnostic defaults, not validated pickup persistence or flight limits.
`clock_alignment_verified` deliberately stays false. `sample_usable`/`health_ok`
mean local acquisition/continuity evidence only, not a verified baseline,
absolute network freshness, synchronized geometry, safe force or cable state.

The estimator must independently require a verified unloaded reference, baseline
age/uncertainty, known loading polarity, causal robust filtering, count hysteresis
and ELAPSED-TIME dwell, plus unobstructed mocap cable geometry. Clear transition
persistence on transport/health discontinuity and return UNKNOWN when baseline,
health or timing confidence expires. Never use the classifier's own SLACK result
as authority to tare. Sustained load creep alone cannot become SLACK. Start in
shadow mode; no controller consumes a cable decision in this change.

## Separate build profiles (never flash automatically)

Preserved original installed bench receipt:
`outputs/strain_gauge/sanity_onboard_tension_20261027/recovery/flash_receipt_noinline.json`
records `bench_only=true`; do not assume it can fly. This task did not change it.

Run from /home/hw527/Projects/drones. Choose a NEW output directory:

```bash
IDFPY=/home/hw527/.espressif/python_env/idf6.2_py3.13_env/bin/python
$IDFPY Sanity/src/sanity_receiver/zenoh_receiver/scripts/build_comms_firmware.py \
  --backend zenoh --profile udp-split --diagnostic-level minimal \
  --base-config outputs/sanity09_id09_20261026/firmware/sdkconfig \
  --idf-path /home/hw527/Projects/kine2go/deployment/esp-idf \
  --pico-source outputs/sanity_comms_redesign_20261026/pico \
  --output outputs/strain_gauge/hx711_bench_NEW \
  --optimize --hx711 --hx711-profile bench
```

For a SEPARATE flight-compatible build, change the output and use
`--optimize --hx711 --hx711-profile flight --flight --enable-tuning`.
Bench/flight public presets are profiles/hx711_{bench,flight}.defaults. Flight
requires BOTH explicit flight selections; bench remains guarded. Sanity09 ID9,
IP.109, UART, mapping, gains, one-second leases and failsafes are inherited from
the normal receiver config. Wi-Fi credentials remain in private build directories.
Build success is not flight readiness. No flash command is provided by this task.

Prepared binaries/receipts: outputs/strain_gauge/hx711_causal_support_20261027/
{flight,bench,receiver_no_hx711}/. Flight and bench binaries are NOT installed.
See RESULTS.txt, validation.json, native_cases_final/, source manifests and test
logs for software evidence. The fake HAL runs production HX711 tasks and pure
receiver state/output logic; it does NOT reproduce FreeRTOS priorities, actual
UART/Wi-Fi/IRQs or flight dynamics. No aircraft command socket exists in that test.

## Remaining hardware qualification

Before any flight claim: inspect actual SCK HIGH/LOW/reset and ready edges with a
logic analyzer under the full normal receiver workload; measure task stack/heap,
command arrival/FC-output timing and watchdog behavior with sensor normal,
missing, saturated, unplugged/reconnected, reset and destination blocked. Verify
new metadata against receiver-health boot identity and status/probe clock wrap.
Measure delivered sensor/ESP supply stability under operating power and vibration.
Any later flashing/command test requires a separately authorized bench session.
The receive-only check here observes the OLD installed legacy stream, not these
new tasks. No current physical sample demonstrates new-image coexistence.

Clock/API references: [Espressif ESP Timer](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/esp_timer.html),
[Espressif nonblocking socket example](https://github.com/espressif/esp-idf/blob/master/examples/protocols/sockets/non_blocking/main/non_blocking_socket_example.c).
