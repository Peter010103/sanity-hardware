SZC6 shared isolated wire interface

API (use this release's `host/sanity_zenoh` first on PYTHONPATH):

```python
mapping = FirmwareCollective(profile, high_rate=True, selected_high_rate=True)
sender = SanitySender(device, ctbr_mapping=mapping, physical_units=True,
                      controller_selection=True, ...)
sender.ctbr(enabled, total_thrust_N, body_rates_FLU_rad_s,
            valid_for_s=.25, indi_requested=enabled)
```

`selected_high_rate=True` is required to opt into the general all-axis mapping. `controller_selection=True` selects version 6 when `high_rate=True`. Low-level functions are `ctbr_si_values(armed, thrust, rates, high_rate=True, selected_high_rate=True)` and `si_command_packet(..., high_rate=True, controller_selection=True, indi_requested=...)`.

The packet remains 40 bytes, the existing `>4sBBBBQIIIHH4H` layout. Magic/version are `SZC6`/6. CTBR mode 1, flags enabled bit0 / INDI-request bit1 (request while disabled is rejected), epoch/session/sequence/lease/TTL/calibration fields are unchanged. Signed 16-bit rates at bytes32/34/36 are 1e-3 rad/s with ±9500 integer bounds on all axes. Unsigned thrust at byte38 remains 1e-4 N, range0–32000. The ESP maps rates to its existing linear600°/s RC scale with FLU yaw sign reversal, so ±9.5 maps toRC1954/1046. Calibrated thrust inversion, AUX arm/request emission, stale-command fencing and lease expiry are unchanged.

The receiver sets SZS2 flag64 only after accepting a selected high-rate command; status echoes the command's session. The sender first publishes disabled SZC6 commands and requires a fresh same-session flag64 acknowledgement before enabled output. `ReceiverStatus.indi_high_rate_supported` exposes the bit (`indi_pitch_loop_supported` is a compatibility alias within this new release). Old receivers drop version6 and cannot satisfy the acknowledgement. Ordinary SZC1–5 commands do not gain flag64, so existing hosts and missions continue to work. SZC3 keeps roll±2.3/pitch±9.5/yaw±2.6; SZC4 keeps its existing ordinary ranges. All per-mission limits are independent of the new wire ceiling.

The saved FC dump restores ACTUAL rateprofile0: roll/pitch/yaw center and maximum rates600°/s, expo0, BF rate-limit1998°/s. 9.5rad/s is544.3°/s, within that RC path on all axes. This audits the saved installed configuration, not a later physical configuration change. INDI native authority still requires actual mode/weight acknowledgement and all original native health checks. The wire ceiling is not flight qualification.
