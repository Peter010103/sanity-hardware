# Hardware, receiver protocols and firmware

## Required connections

The setup is a Sanity quadrotor with a BETAFPVF405 flight controller, DSHOT300 and
bidirectional RPM telemetry, plus an ESP receiver on the experiment Wi-Fi. The
native benchmark's tested FC loop is 1 kHz with 2 kHz gyro sampling. The host runs
ROS 2 mocap reception and Zenoh command listeners. Normal flights do not use a USB
motor-test client or a thrust stand.

For the Seeed XIAO ESP32-S3 receiver used here:

| FC connection | ESP connection | Purpose |
|---|---|---|
| TX4 | GPIO3 / D2, ESP RX | FC → ESP MSP/status/observation |
| RX4 | GPIO4 / D3, ESP TX | ESP → FC MSP commands |
| GND | GND | Common signal reference |
| Aircraft battery input | Flight battery | Motor/FC power, correct polarity and aircraft voltage |
| ESP power | Board-appropriate regulated input | Check the actual aircraft power arrangement and USB back-power behavior |

The unified native observation build uses the matched **921600 baud 8N1** UART
configuration; historical ordinary MSP configurations used **115200 baud**. Match
both FC UART and receiver build. Do not set one end independently. The stock
README under `firmware/receiver/` contains dated profiles; select the build/profile
corresponding to the intended experiment.

ESP USB can power/configure the ESP and provide a transparent PC→ESP→FC UART
passthrough. It does not establish motor/FC battery power. For the XIAO build's
four-second startup selector, reset then briefly press BOOT to choose passthrough;
reset without that action returns to normal radio mode. Holding BOOT across reset
is the ROM firmware bootloader instead. Close Configurator/passthrough clients
before a radio flight. Do not apply the separate Stamp-P4 pin table to a XIAO.

The workstation needs a reachable address matching `lab.json` and the receiver's
firmware endpoint. Ordinary historical PID uses UDP7447 command/session listening
and TCP7448 telemetry. Current native profiles use shared TCP7447. Native clock
exchange normally uses UDP7461. These are distinct from motion-capture/NatNet and
ROS DDS ports. A Zenoh router is not required for the direct host listener/ESP
connection topology used by these recipes.

## Matched protocol and control selection

| Path | Required mapping/capability |
|---|---|
| Historical upright MPC + PID | Ordinary CTBR mapping and calibration, receiver ownership/lease and enable-zero handshake |
| Historical PID flips | SZC3, pitch authority 9.5 rad/s, thrust 1e-4 N and rate 1e-3 rad/s wire units |
| Native paired PID/INDI benchmark | SZC4 with explicit selected-inner request, native FC UID/status/permission and RPM/observation admission |
| Isolated 4–6 m/s INDI host | SZC6 all-axis high-rate capability, matching disabled session ACK, PIN1 telemetry support and native controller admission |

SZC3 high pitch does not carry the native INDI selector. SZC4 ordinary selected
INDI is limited to 2.3 rad/s roll/pitch. Raising a host-only bound does not change
receiver admission or the FC RC rate map. SZC6 allows ±9.5 rad/s on all three axes,
with mission-specific local caps and the expected linear ACTUAL 600°/s profile
(zero expo). Verify the active FC profile, actual emitted RC mapping and native
mode/weight rather than assuming a copied CLI fragment establishes them.

The supplied receiver source is the corrected all-axis SZC6 + PIN1 version. The
historical installed corrected application's SHA256 was
`0189505ec22955beb2b34543851fa9703d9db59067701c9352518a0cbaa83ab5`.
This identifies a recorded image, not a hash expected from every rebuild. No
credential-bearing firmware image is published in this branch. The earlier
high-rate image without PIN1 telemetry is superseded and cannot satisfy native
backend admission even though it acknowledges a high-rate command format.

## Rebuild the FC from public pinned sources

`firmware/fc/source.patch` is the complete current source delta against official
Betaflight 4.5.1, revision `77d01ba3b76a22909d5f09cb0628820141f95eaa`.
The board configuration is pinned to
`bc5da0e66be6d907c694448c3443fdcf6666c1ba`. It includes the native rotational
INDI/PIN1/observation path, measured-RPM model, rate feedforward and the recorded
timing-transient recovery changes, with third-party source notices retained.

```sh
.venv-sanity-deploy/bin/python firmware/fc/rebuild.py --work /absolute/path/to/NEW_BUILD
```

The script fetches the pinned sources, verifies/downloads GCC Arm
10.3-2021.10 by SHA256, applies the patch and builds INDI-capable and PID-only
reference HEX files. `--toolchain PATH` reuses that exact compiler; `--jobs N`
selects parallel compilation. It has no flashing operation. The native image
build options are:

```text
USE_SANITY_INDI USE_SANITY_SENSOR_STREAM SANITY_GYRO_2KHZ
SANITY_INDI_TRACE_DENOM=4 SP_FF_ACCEPT_TRANSFER=1
```

The native-capable image contains both the native PID-selected and INDI-selected
paths required by the paired benchmark. The plain PID-only reference build is
not a substitute for that native protocol firmware. The original October PID
flights used their recorded Betaflight PID configuration; exact binary equivalence
is not claimed for a new reference rebuild.

Reference FC restore/settings files are under `firmware/fc/configuration/`, and
the current gain fragment is `firmware/fc/recommended.cli`. Match the physical
UID and preserve the current aircraft's gyro orientation, motor/output ordering,
UART, receiver/AUX, voltage compensation, RPM poles and rate profiles. The retained
October restore file and November dump are dated evidence, not an instruction to
blindly overwrite a differently configured aircraft. Firmware flashing/restoring
settings is an explicit operator step, separate from the MPC flight scripts.

## Rebuild the receiver with private local settings

The source is under `firmware/receiver/`. Public Zenoh-Pico 1.10.1 is pinned to
`e1ab223a28aaebb5dec1e70d98eab152332f777a`; the included preparation helper applies
its GCC compatibility and transport-lifetime patches. The recorded ESP-IDF build
was a clean 6.2 development checkout at revision
`2067f3ae32b7e7b61a0f3a052df63d619cd7c61d` from
`https://github.com/espressif/esp-idf.git`. Check out that revision and initialize
its required submodules to reproduce the SDK source; record a compatible local ESP-IDF and
validate the resolved target/configuration before installation.

Copy `firmware/receiver/profiles/xiao_s3_native.example.defaults` to
`config/receiver.private.defaults` and replace `CHANGE_ME` with your own Wi-Fi
settings. The template retains the matched XIAO native feature/pin values.
Review its device ID, addresses and UART configuration for your aircraft.
Create a private base sdkconfig with your Wi-Fi credentials, device ID, target
board, UART pins/baud and endpoints. The receiver Kconfig and README describe the
fields; do not commit generated sdkconfigs or images containing credentials.
The supplied defaults contain no network credentials.

```sh
.venv-sanity-deploy/bin/python firmware/receiver/scripts/prepare_pico.py .deps/zenoh-pico
.venv-sanity-deploy/bin/python firmware/receiver/scripts/build_comms_firmware.py \
  --profile tcp-shared --base-config /absolute/path/to/private-sdkconfig \
  --idf-path /absolute/path/to/esp-idf --pico-source "$PWD/.deps/zenoh-pico" \
  --output "$PWD/build/receiver-NEW" \
  --flight --estimator --paper-native --mocap --command-timing --optimize
```

`--paper-native` is essential for this matched PIN1 build. Do not substitute the
older `--estimator-indi-hover` build profile: it disables the required PIN1
query/telemetry path. The ESKF is observation/logging support; selecting it here
does not make it the outer controller's state source.

The builder creates a new directory, records sources/configuration and never
flashes hardware. Install only on the positively identified ESP, using the generated
ESP-IDF flash instructions and the matching bootloader, partition table and
application. An FC USB port and an ESP USB port are different targets. After any
installation, use a disabled combined bench check and inspect capability/native
status before a later explicit flight request.

## Commissioning and failure handling

Verify actual motor numbering/direction, propellers, physical pole count, frame
signs, FC identity, calibration ID, receiver identity, active rate map, observation
and RPM health. The scripts check many of these electronically, but cannot infer
an incorrectly mounted marker or wrong propeller from a successful command ACK.

Host freshness, command leases, receiver session ownership, native FC faults and
recording health have separate roles. A solver failure can request landing; a hard
state/envelope/identity fault may disable immediately. No host cleanup code can
guarantee delivery after a network/host power failure. Use the experiment's physical
motor-power isolation and verify the actual disarmed state before handling the
aircraft. Do not bypass an admission fault solely to make the launch continue.
