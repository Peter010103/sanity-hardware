Current next-launch tilt guard revision:
ordinary benchmark speeds and3m/s:40deg soft/60deg hard;
4m/s:60deg/75deg;5m/s:72deg/82deg;6m/s:78deg/85deg.
These are measured-state fault thresholds, not commanded attitudes. MPC tilt
constraints, gains, thrust and body-rate caps are unchanged. Startup/takeoff
keeps15deg soft/35deg hard. Speed, position, freshness and identity guards stay.
Archived tuples remain available for exact replay. Current evidence:
  outputs/sanity15_tilt_envelopes_20261112/
Emergency braking revision is pending; its mandatory0.6rad/s cap is unchanged.

Matched receiver installation (authoritative root hardware receipt):
  outputs/sanity15_indi_flip_preparation_20261112/esp_build_all_axes_pin1/sanity_zenoh_receiver.bin
  SHA256 0189505ec22955beb2b34543851fa9703d9db59067701c9352518a0cbaa83ab5
  948672 bytes; PIN1 and SZC6 enabled
  outputs/sanity15_highspeed_esp_deployment_20261112/verification.json
The earlier 195ab650... image lacked PIN1 and is superseded; historical build
records remain for provenance. RECEIVER_MATCH.json records the current image.
This verified installation is not a claim of 4–6 m/s physical flight readiness.

Isolated 4–6 m/s INDI lemniscate host integration

The runners preserve the requested speed, R2 geometry, world centre, 1 m height,
three laps and two-second angular ramps. They select the matched SZC6 host
mapping; the receiver must acknowledge high-rate support in the same session.
Receiver installation and disarmed bench status are recorded separately by the
root deployment receipts. This integration performed no hardware/firmware action.

Reviewed airborne and braking limits:
Requested speed    Soft/hard tilt    Hard speed    MPC thrust cap
4 m/s              60° / 75°         5.0 m/s       1.8 weight
5 m/s              72° / 82°         6.5 m/s       2.2 weight
6 m/s              78° / 85°         7.5 m/s       2.9 weight

MPC tilt limits remain 50°/62°/72°, and all-axis rate bounds remain 9.5 rad/s.
The hard speed caps are interruption thresholds with headroom above requested
peaks and prior nominal model peaks; 7.5 m/s also exceeds the earlier severe
6 m/s model peak of 6.538 m/s. Reference peaks were not reduced.

The selected speed/tilt envelope latches after MPC admission and stays active
through worker, RPM and operator fault landing. Before admission, the original
2 m/s takeoff/hover guard remains. Ordinary lower-speed profiles and PD command
caps are unchanged. Wrong speed/envelope tuples are rejected. Genuine overspeed,
hard tilt, position/altitude, stale state/controller, FC identity and native
faults retain their existing actions. The native FC takeoff guard was not raised.

48 production mission.tick guard regressions passed. Independent physical-plant
screens now exercise actual mission admission, blend, all three laps, return,
hold and PD landing. All three completed normally, with no solver failures,
allocation clips or false 2 m/s hard stop:
Requested speed    Peak model speed    Peak model tilt    Tracking RMS
4 m/s              3.987 m/s           43.43°             5.63 cm
5 m/s              4.999 m/s           56.71°             6.88 cm
6 m/s              6.060 m/s           66.96°             8.85 cm

These screens begin at stable airborne MPC entry. They use fitted small-angle
rate dynamics, provisional inertia, assumed yaw, explicit command/collective
lag, motor allocation/headroom and simple ground contact. They are not physical
higher-speed flights or a demonstration of complete emergency recovery.

Remaining emergency-braking limitation — landing law intentionally unchanged:
At a full-speed centre crossing, the braking reference alone advances
15.00 / 23.44 / 33.75 m at 4 / 5 / 6 m/s. Its slow return then takes about
125 / 195 / 281 s. The current 120 s mission runtime can expire before touchdown.
Room and mocap coverage are not verified. Installing the matched receiver does
not resolve these emergency-braking/runtime limitations or establish higher-speed
flight readiness. No higher-speed physical flight was requested or run here.

Plans:
  bash outputs/sanity15_highspeed_lemniscates_20261112/run_4mps.sh --plan-only
  bash outputs/sanity15_highspeed_lemniscates_20261112/run_5mps.sh --plan-only
  bash outputs/sanity15_highspeed_lemniscates_20261112/run_6mps.sh --plan-only

Exact guard tests, full mission screen outputs, fault-distance calculations,
source hashes and cleanup are in ../mission_guard_integration/.
INTEGRATION_RECEIPT.json is the current evidence; earlier optimizer-only studies
do not substitute for these actual mission-path checks. Active production source
and the shared flip receiver/protocol files were not edited.
