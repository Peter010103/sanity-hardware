Explicit INDI-shaped variable-speed lemniscate
============================================
The new selectable schedule is indi_shaped_angular. It does not replace
constant_arclength or variable_angular and does not change their samples.

Geometry stays x=R sin(theta), y=(R/2)sin(2theta), R=3.2m, center[0,0],
three laps. Cruise angular speed and peak physical speed are preserved.
The quintic angular-speed entry/exit ramps are lengthened when needed.
Entry/return position transfers use a septic C3 profile only for this new
schedule, with analytic speed/acceleration/jerk duration bounds. They join
the stationary hold and lap with zero velocity, acceleration and jerk.
The same geometry still crosses the region where mocap state was lost.

The current limits are reference acceleration 1.5 m/s^2 and jerk 2 m/s^3.
These correspond to an ideal tilt below approximately 8.7 deg and a small-angle
fixed-yaw rate proxy jerk/g<=0.204rad/s. The latter is not an actual MPC or
FC demand bound; feedback corrections and disturbances can add rate demand.

Ramp duration uses analytic conservative bounds, not sampled extrema:
  |p_theta|<=R sqrt(2), |p_theta_theta|<=R sqrt(5),
  |p_theta_theta_theta|<=R sqrt(17).
The quintic angular-speed smoothstep has max|S'|=15/8 and
max|S''|=10/sqrt(3). The chain rule bounds acceleration and jerk in terms of
cruise angular speed and ramp duration. The larger duration satisfying both
inequalities is selected, while retaining at least the requested 2 s ramp.
If cruise alone cannot satisfy a limit, construction fails explicitly.

At1.5m/s, ramp duration increases2.000->4.251s. A50Hz sample of the exact
analytic reference gives acceleration peak1.387->0.747m/s^2, jerk peak
2.672->0.936m/s^3, geometric fixed-yaw ideal body-rate norm0.272->0.095rad/s.
Whole three-lap segment duration increases58.868->61.120s. This lap-segment comparison
excludes transfers, the MPC feedback loop, motors, sensing failures or
physical flight. It is reference-demand verification, not flight qualification.

At0.8m/s the requested2s ramps already meet the limits, so reference samples
are unchanged. At1.0m/s ramps become2.370s. Historical variable_angular keeps
its original2s ramps at all speeds.

Preview (no ROS/network/hardware):
  bash Sanity/src/ros2_sanity_ws/scripts/run_sanity_control_test.sh \
    --test trajectory --pattern lemniscate --schedule indi_shaped_angular \
    --speed 1.5 --inner indi --plan-only

Artifacts: comparison.json, per-case NPZ exact reference signals, tests.log,
and public_plan.json. Tests: see tests.log (67 tests after adding C3 transfers), including prior reference tests,
analytic derivative consistency, C3 joins, speed/geometry/three-lap preservation,
conservative acceleration/jerk limits, explicit config selection and rejection
of an unsupported shaped-circle combination. No lab processes were started.
