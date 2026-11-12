# Three Grove HX711 modules on the Orin Nano

**Wiring verified by an ADC-only check, 20 October 2026.** Applies to NVIDIA's standard **P3768 Orin Nano developer-kit carrier**, including the Orin NX module identified on the acquisition computer. All three GPIO pairs below returned valid data simultaneously in a 15-second test. This confirms communication, not load-cell calibration, physical sensor identity or force/torque sign. The temporary clock-pad changes were restored after the check.

Use one HX711 per load cell, all at 80 SPS, with **independent DOUT and PD_SCK lines**. Six GPIOs are available without using I2C or serial ports. The HX711 interface is a clocked GPIO protocol, not I2C, despite the Grove connector and some wording in the Grove example. Do not tie DOUT lines together.

## Recommended physical-header assignment

These are **physical BOARD pin numbers on J12**, not Raspberry Pi BCM numbers, Linux global GPIO numbers or the chip's own pin numbers. Locate the carrier's pin-1 marking before connecting power.

| HX711 function | Thrust module | Torque module 1 | Torque module 2 |
|---|---|---|---|
| VCC | 3.3 V from J12 pin **1**, fanned out | Same supply | Same supply |
| GND | J12 pin **6**, fanned out | Same ground | Same ground |
| DOUT → Orin input | J12 pin **29** | J12 pin **7** | J12 pin **32** |
| PD_SCK ← Orin output | J12 pin **31** | J12 pin **15** | J12 pin **33** |

Pin 17 is another connection to the same 3.3 V rail if needed. Other ground pins can be used; do not connect sensor ground through a motor-current return lead. Use a small power/ground distribution terminal rather than forcing several wires into one header contact.

Power the Grove boards at **3.3 V** so their digital supply and DOUT levels match the Orin. The published Grove schematic connects VCC directly to HX711 DVDD; there is no 5 V-to-3.3 V translator on DOUT. Do not power these boards from header pins 2/4 (5 V) while directly wiring their signals to the Jetson. NVIDIA specifies a **100 mA total 3.3 V header budget**, shared with other attached devices; include all three ADC boards and bridge excitation in the measured load budget. If that is exceeded, redesign the 3.3 V supply arrangement rather than assuming pin 17 provides a second independent allowance.

## Grove cable and load-cell connections

The published Seeed schematic's **J1 Grove connector** is:

| Grove connector pin | Signal | Usual cable colour |
|---|---|---|
| 1 | PD_SCK (schematic calls it PSCK) | Yellow |
| 2 | DOUT | White |
| 3 | VCC | Red |
| 4 | GND | Black |

Verify connector pin numbering/continuity on the actual revision; wire colours are a convenience, not proof of signal assignment. In particular, **yellow is clock and white is data** for this schematic.

Each load cell connects only to its own board's E+, E−, INA+ and INA− terminals. Use the load cell's actual wiring specification/continuity checks to identify excitation and signal pairs; colours vary. Tare and direction checks establish measurement sign. Keep low-level cell wiring short, route it away from ESC/motor leads, and provide strain relief so cable movement does not apply force/torque to the rig.

For the 80 SPS modification, HX711 RATE is chip pin 15, not Orin header pin 15. The published board ties RATE to ground; isolate that connection before connecting RATE to the board's **DVDD/3.3 V**. Do not bridge an intact ground connection to VCC. Check the actual board revision and verify that the modification has not shorted adjacent pins. It is a hardware change, not a different GPIO clock frequency setting.

## GPIO configuration and reader changes

The first pair preserves the existing working assignment. For the standard carrier, NVIDIA's Jetson.GPIO table gives:

| Header pin | Signal | SoC line | Offset in `tegra234-gpio` | Required direction |
|---|---|---|---:|---|
| 29 | GPIO01 | PQ.05 | 105 | Input |
| 31 | GPIO11 | PQ.06 | 106 | Output, initially LOW |
| 7 | GPIO09 | PAC.06 | 144 | Input |
| 15 | GPIO12 / PWM alternate | PN.01 | 85 | GPIO output, initially LOW |
| 32 | GPIO07 / PWM alternate | PG.06 | 41 | GPIO input |
| 33 | GPIO13 / PWM alternate | PH.00 | 43 | GPIO output, initially LOW |

Offsets are within the GPIO controller, **not physical pins**. Resolve the controller by its label and verify against the installed board/JetPack GPIO table; do not hard-code a global GPIO base or assume `/dev/gpiochip0`.

The pad mux and direction must allow these functions. Pins 15, 32 and 33 must be GPIO rather than active PWM/peripheral outputs, and no other process may own the lines. GPIO-library direction requests alone do not necessarily fix pad mux settings. The existing `tools/hx711_pinmux.py` only knows clock pin 31; its register values must not be copied to other pads. Use the installed JetPack's supported pinmux configuration and validate each signal electrically. No pinmux writes are performed by this guide.

The packaged single-cell wrapper still uses offsets 105/106. The three-channel
`tools/torque_check.py` group and `tools/full_campaign.py` worker explicitly use
all three assignments. The raw single-cell command and calibration helper only
read the thrust pair. Wiring three modules does not make those single-cell
commands read torque.

The three-channel group checks P3768/Tegra234 identity and the expected input/
clock pad values, exclusively claims the GPIO lines, temporarily enables the
three clock outputs, then restores their entry values during cleanup. It also
restores the recorded CPU governor and releases its CPU latency request. Its
runtime register procedure and scheduling retain the eight-core Orin NX campaign
assumptions; verify the carrier/JetPack before execution. This is not boot-time
pinmux configuration. See [operation](operation.md) for privileges and shutdown.

Use a reader that services all ready modules promptly, with one owner per clock. Do not block the whole collector waiting on a failed sensor. Keep Tenma/MSP transactions outside the GPIO acquisition loop. Retain the clock-pulse guard: HX711 normal HIGH pulses must not exceed 50 microseconds, and a HIGH exceeding 60 microseconds enters power-down. Linux scheduling can stretch pulses; repeated resets invalidate any claimed continuous 80 SPS stream. If measured timing is unreliable, a microcontroller acquisition bridge is the fallback, not disabling the guard.

Independent modules have independent conversion timing. Timestamp each DOUT-ready observation and read completion; 80 SPS on all three does not imply simultaneous sampling. Align force/torque streams offline and retain alignment uncertainty. A common SCK connection is possible only with a coordinated multi-device driver and compatible channel/gain selection; it is not a drop-in change to three independent reader instances. Independent clocks are preferred for this initial installation.

## Bring-up before powered motor tests

1. Confirm the P3768 carrier and header orientation. With nothing connected, verify pin 1 to pin 6 is approximately 3.3 V; then shut down and remove power before wiring.
2. Wire one module at a time, confirming VCC/GND and data/clock continuity. Connect its load cell. With power restored, verify digital levels and clock output at the module, with no motor commands.
3. With each module at 80 SPS, check native raw counts, approximately 12.5 ms conversion spacing, zero drift and response to a gentle known load. Recheck calibration after the modification.
4. Acquire all three together for 30–60 seconds. Report per-channel fresh-sample rate, interval distribution, resets, timing rejections and missing samples. Confirm operation while Tenma/FC readers are active, without powering motors for this acquisition check.
5. Store measured throughput, pin mappings, individual tares/signs and rate settings in the run metadata. Require a successful simultaneous probe before using the powered experiment sequences.

The historical diagnostic below preceded the later three-channel collector now
included in this branch. Its observations describe that check, not a powered
validation of the current packaged code. Re-run a stopped probe on the actual rig.

## Live connection check — 20 October 2026

Host: `unitree@192.168.123.15`; installed library: `~/sanity_thrust_test/libhx711.so`; board compatible: `nvidia,p3768-0000+p3767-0000`; Jetson Linux R36.5. No FC USB device was present and no FC/motor interface was accessed.

All three modules were read concurrently for 15 seconds, with no retries or communication errors. Each returned non-saturated signed ADC values; zero samples were at the ADC rails.

| Logical channel | DOUT / SCK physical pins | Accepted samples | Observed samples/s | Maximum clock-HIGH software bound (µs) |
|---|---|---:|---:|---:|
| thrust | 29 / 31 | 1373 | 91.41 | 44.96 |
| torque_1 | 7 / 15 | 1332 | 88.69 | 33.19 |
| torque_2 | 32 / 33 | 1381 | 92.00 | 33.28 |

The approximately 89–92 SPS observed throughput is consistent with the HX711 high-rate setting. Use recorded timestamps rather than forcing a nominal 80 Hz timebase. Internal-clock variation, readiness polling and read overhead affect the observed interval; this is not an independent oscillator calibration. All measured clock-HIGH bounds stayed below the existing 50 µs guard, and no ready-observation gap exceeded 25 ms in this short check. Continuous operation under the final acquisition workload has not yet been validated.

Only the three SCK pad registers were temporarily changed, with all other fields preserved: physical pin 31 `0x58→0x08`, pin 15 `0x55→0x05`, pin 33 `0x54→0x04`. All three original values were restored and read back successfully. No boot-time configuration was installed. The user must not expect the original collector to drive the extra channels without the implementation/configuration work above.

The raw samples, exact diagnostic and restoration receipt are saved in the parent workspace under `outputs/hx711_three_channel_check_20261020/`. Channel names reflect wiring assignments; a known-load check must establish the actual thrust/torque sensor identity, tare, sensitivity and sign.

## Sources

- [NVIDIA P3768 carrier specification](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf): J12 physical pinout, 3.3 V signals and header power budget.
- [NVIDIA Jetson.GPIO pin definitions](https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/gpio_pin_data.py): Orin Nano/NX mapping, controller offsets and pad definitions.
- [Seeed Grove documentation and schematic download](https://wiki.seeedstudio.com/grove_adc_for_load_cell_hx711/). The linked Eagle schematic was inspected: J1.1=PSCK, J1.2=DOUT, J1.3=VCC, J1.4=GND; VCC feeds DVDD and RATE is grounded in the published design.
- [HX711 datasheet](https://www.mouser.com/datasheet/2/813/hx711_english-1022875.pdf): RATE selection and serial clock timing.
- Packaged [single-cell workflow](single-cell.md) and [operating procedure](operation.md).

## FC, supply and mechanical connections

| Connection | Required route |
|---|---|
| Jetson power | Its normal developer-kit input, independent of the motor supply |
| FC data, direct mode | Jetson USB → FC USB with MSP enabled; use a stable `/dev/serial/by-id/` path |
| FC data, Acro mode | A verified RX_MSP-capable serial path that permits arming; native USB may inhibit it. The original campaign used an ESP transparent bridge → FC MSP-configured UART. Bridge firmware and UART setup are external prerequisites. |
| PSU control | Jetson USB → Multicomp Pro 72-13350 V2.3 USB serial port; separate from FC data |
| Motor power, regulated mode | PSU rear +/− 30 A outputs → accessible physical disconnect → correctly polarised, rated aircraft battery-input leads |
| Motor power, battery mode | Flight battery → accessible physical disconnect → aircraft battery input; PSU physically disconnected |
| Torque measurement | One independently wired HX711 per opposing torque-reaction cell; no shared DOUT/SCK |

Do not parallel the PSU and battery. Do not feed motor power into Jetson header
power pins. Check polarity and voltage at the aircraft connector before connecting
it. USB ground and signal grounds are not substitutes for the rated motor-current
return lead. Verify back-power behaviour for the actual FC/bridge/USB arrangement.
This package does not specify unverified UART solder pads or an arbitrary cable's
wire colours; use the actual FC and bridge pin labels. For a separate UART bridge,
TX connects to the corresponding RX, RX to TX, with common signal ground and
compatible logic levels.

Mount the complete aircraft so thrust acts along the axial cell's intended
measurement axis. Restrain off-centre roll/pitch moments without allowing an
unknown axial/yaw bypass load path. The historical opposing torque-cell spacing
was 60 mm, giving assumed 30 mm perpendicular arms. Check the true arms, force
signs and both CW/CCW reactions with known loads; electrical communication alone
does not establish force calibration. Give sensor and power cables strain relief
and enough slack that they do not carry an unmeasured reaction.
