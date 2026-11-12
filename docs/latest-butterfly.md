# Latest butterfly flights — 12 November 2026

This update captures the later isolated releases and flight review completed after
this branch's original snapshot. The earlier planar and PID flip recipes remain
available with their original models and guards.

## Geometry and launch commands

The new reference occupies **4 × 3 m in XY**, with alternating inclined figure-eight
planes at **fresh ground + 0.7 through +1.3 m**. Three A/B cycles mean **six complete
figure eights**. Each eight stops at the center with zero velocity, acceleration
and jerk before changing plane. Peak speed means requested 3D reference speed.

```sh
./scripts/run_mpc.sh indi-butterfly-3 --plan-only
./scripts/run_mpc.sh indi-butterfly-4 --plan-only
./scripts/run_mpc.sh indi-butterfly-5 --prepare-only
./scripts/run_mpc.sh pid-butterfly-3 --plan-only
./scripts/run_mpc.sh pid-butterfly-5 --prepare-only

# Current local lab configuration and matched receiver/FC commissioning required.
./scripts/run_mpc.sh indi-butterfly-3 --lab-config config/lab.json --bench-check --frames-confirmed
./scripts/run_mpc.sh indi-butterfly-3 --lab-config config/lab.json --fly --frames-confirmed
./scripts/run_mpc.sh pid-butterfly-3 --lab-config config/lab.json --fly --frames-confirmed
```

The combined launcher repeats its disabled bench, measures fresh stationary ground,
then builds the actual flight configuration. The unchanged world altitude ceiling
is 1.5 m; planning rejects a ground/height combination that would cross it. No
paired PID 4 m/s flight is documented, so no named PID 4 m/s recipe is added.

`releases/butterfly-indi/` contains the activated release with the roll-only 5 m/s
rate-model refit and scoped emergency-braking change. `releases/butterfly-pid/`
contains the corresponding PID comparison implementation. PID uses ideal-rate
prediction with a separate causal projection from selected-command history to the
20 ms selection + nominal 15 ms output epoch. It does not use the identified INDI
rate poles. The worker-preparation command selects the matching isolated host
package as well as its solver, so it exercises the appropriate projection code.

## Emergency behavior and model changes

The activated 5 m/s INDI release allows **2 rad/s only during emergency braking**;
ordinary takeoff and settled landing retain 0.6 rad/s. The saved lower-speed INDI
3/4 m/s releases retain their existing 0.6 rad/s emergency cap. The PID comparison
uses its explicit 2 rad/s emergency controller for 3/4/5 m/s. These differences
are part of the actual compared systems, not a new global increase to command
bounds. FC/ESP firmware and native gains were not changed by this host update.

`ACTIVATION.json`, `RATE_MODEL_CHANGE.json` and exact per-flight configurations are
retained in `evidence/latest-butterfly/`. Some earlier preparation receipts describe
the refit/braking change as unactivated; the later activation and actual recorded
configurations establish which release was used. Offline recovery tests are not
physical recovery qualification. The original planar braking-distance limitation
in the older runbook/results describes the archived planar implementation, not the
new emergency controller; the new controller also has observed failure cases below.

## Observed flights

| Inner loop / reference | Run start UTC | Outcome |
|---|---|---|
| INDI 3 m/s | 09:52:31 | Completed six eights, `test_complete` |
| INDI 4 m/s | 09:54:26 | Completed six eights, `test_complete` |
| INDI 5 m/s | 09:48:21 | Native allocator fault; about 1.554/6 eights; fallback/braking landed |
| PID 3 m/s | 10:21:18 | Completed six eights, `test_complete` |
| PID 5 m/s | 10:23:05 | Stale MPC tick initiated abort; about 2.068/6 eights; recovery reached hard altitude ceiling |

The [final paired comparison](../evidence/latest-butterfly/MPC_INDI_vs_MPC_PID_butterfly_comparison.pdf)
contains the full findings and native appendix. Its [machine-readable assessment](../evidence/latest-butterfly/FINAL_COMPARISON.json)
and exact run configurations are also included.

At 3 m/s, recorded position RMS is **3.802 cm for MPC+INDI** and **5.347 cm for
MPC+PID**. Native frontend-target tracking RMS norm is 0.296 versus 0.426 rad/s.
At 5 m/s, neither run completes. The fair shared window is the first **9.98 s**,
before native INDI fallback/alignment uncertainty: position RMS is **7.605 versus
8.293 cm**, while native rate-target tracking RMS norm is **1.809 versus 0.635
rad/s**. Thus smaller position error in that window does not imply better rate
tracking or a completed-flight advantage. These are paired system observations;
outer predictors, takeoff and emergency behavior also differ.

For INDI 5 m/s, the host MPC remained healthy when the native allocator fault
latched. A bounded solution/motor target precedes fallback, but the initiating
1 kHz allocation rejection is absent from the 250 Hz recording. Its numerical,
convergence or rank cause remains unresolved; bound activity alone is not proof.

For PID 5 m/s, the first trigger is stale MPC selection. Valid results for two ticks
finished 25.897/25.268 ms after the control timestamp, missing the 20 ms selection
cadence, despite full-command computation times of 13.999/11.829 ms and optimizer
core times around 0.24 ms. The unprofiled overhead must not be attributed entirely
to the optimizer. Source-state age is a separate clock/validity measurement.

The PID emergency began at world Z1.283836 m with upward velocity 0.461 m/s. It
requested downward acceleration, but upward velocity initially rose; Z reached
1.500002 m after 0.460 s and the hard ceiling disabled output. The guard prediction
was about 1.36 m. The INDI recovery began lower and descending, so the different
recovery outcomes do not isolate inner-controller quality.

Native rate metrics compare the same-row frontend target and early gyro in FRD.
Host authority gyro is BF/FLU; these signals are not mixed. PID conclusions use
measured RPM and applied mixer counts, not unused INDI-shadow allocator targets,
servo errors or residuals. Static thrust inferred from RPM is model-derived rather
than a force measurement. No fitted signal shifts or added filtering were used
for the final paired comparison.
