Sanity15 MPC and INDI update — 10 November 2026

This desktop update targets the current FC settings: Kv=20/20/8 per second,
rate-reference feedforward=0.25/0.25/0, H=12 Hz and derivative cutoff=20 Hz.
FC/ESP firmware and gains are unchanged. Measured flying mass is 71.51 g.

Implemented changes
-------------------
1. A dedicated plane_sanity15.xml supplies 71.51 g to the matched PD and MPC
   profiles. The shared historical plane.xml remains unchanged.
2. The INDI predictor has 16 states: position (3), quaternion (4), velocity (3)
   and six nominal rate-response states. Roll/pitch use a stable second-order
   response with a zero, fitted on 0.5–3 Hz and checked against held-out 4 Hz.
   The yaw time constant of 0.125 s is an engineering assumption.
3. Effective thrust gain 0.953 and body damping [0.587, 0.820, 0] per second
   come from the partial flights. They are effective model parameters, not
   independent physical thrust or aerodynamic calibrations. No fixed world
   acceleration bias is applied.
4. MPC uses 20 nodes at 0.05 s, internal integration steps no longer than
   10 ms, and angular-acceleration cost weights [0.002, 0.002, 0.001]. It keeps
   the 25 ms solve budget and existing physical command bounds.
5. Measured pose/velocity are projected to an expected command application
   epoch: 20 ms asynchronous selection plus a nominal 15 ms command stage.
   The latter is an explicit approximation informed by local timing data,
   not an identified end-to-end one-way latency. Projection uses timestamped
   selected CTBR history, not solver candidates. It adds no gyro observer or
   measurement filter and does not relax the 40 ms input-validity rule.
6. The explicit indi_shaped_angular schedule preserves the R=3.2 m figure
   eight, center, 1.5 m/s peak speed and three laps. It lengthens phase ramps
   and uses C3 septic transfers. Existing reference recipes remain available.
7. Targeted raw TF inputs, rejection reasons and accepted-state ages are now
   recorded. The disabled bench no longer starts an unused MPC worker.

Evidence and validation
-----------------------
rate_fit/: identification, held-out 4 Hz and independent trajectory replay.
drag_fit/: cross-flight fits, tail exclusion and attitude-consistency audits.
command_path_review/: local timing and actual delivered-demand spectra.
  Actual feedback commands strongly excite about 2 Hz even though the ideal
  position reference has a much lower fundamental frequency.
closed_loop_results.json: six complete three-lap airborne simulations.
  The independent plant integration includes fitted damping, uncompensated
  constant bias, 25 ms thrust lag, a 15 ms command stage, measured mass, rate
  dynamics and explicit attitude-noise stress. Legacy RMS is about 9 cm;
  the updated controller gives about 4 cm, including sensitivity cases.
  This is a calibrated engineering simulation, not a new flight result.
delay_sensitivity.json: actual command delays of 0/30 ms versus the nominal
  15 ms prediction, over two 30-second segments.
ipc_check.json: production solver and ordered IPC with synthetic stationary
  state. The worker is explicitly closed and reaped.
pytest_final.log: final regression results, including legacy MPC behavior.

Preview without hardware or network access:

  bash Sanity/src/ros2_sanity_ws/scripts/run_sanity_control_test.sh \
    --test trajectory --pattern lemniscate --schedule indi_shaped_angular \
    --speed 1.5 --inner indi --plan-only

Flight command:

  bash Sanity/src/ros2_sanity_ws/scripts/run_sanity_control_test.sh \
    --test trajectory --pattern lemniscate --schedule indi_shaped_angular \
    --speed 1.5 --inner indi --fly --frames-confirmed

Known sensing and validation limits
-----------------------------------
The full geometry still passes the location where previous runs lost state,
approximately x=-2.7 m, y=+1.5 m. This update improves prediction and logging;
it does not demonstrate restored mocap coverage. Freshness and stop behavior
are unchanged. Check coverage there before expecting a complete mission.
The simulation starts airborne and does not validate takeoff, occlusion or
physical equipment. No physical flight was started during this update.

PID comparison
--------------
PID retains the legacy ideal-rate predictor with the corrected mass. INDI
now has a different identified predictor, discretization and regularization.
Comparisons therefore test complete MPC+inner-loop combinations; they no
longer isolate only the inner controller with an identical outer predictor.
