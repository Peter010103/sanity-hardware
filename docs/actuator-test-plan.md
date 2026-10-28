# Voltage-controlled actuator identification plan

**Status: measurements collected 20 October 2026. See [results, exact coverage and limitations](actuator-modelling.md). Completed sections of interrupted runs are retained; this is not a claim that every planned repeat completed.**

The connected supply identifies as `Multicomp Pro 72-13350 V2.3`, correcting the initially supplied 72-13360 model. The local `tenma-serial` patch adds this model, fixes single-channel family detection and preserves three-decimal current-setpoint readback. Unloaded 7.6/7.8/8.0/8.2 V setting and output switching passed; original 8.0 V / 30 A settings were restored with output **off**. These unloaded checks do not validate current regulation or loaded voltage stability. Logs are in workspace `outputs/sanity_tenma_supply_20261020/`; battery results are in `outputs/sanity_torque_campaign_20261020/`.

Measure Sanity's voltage-dependent thrust, yaw reaction torque, current, efficiency, actuator dynamics and battery voltage sag, with the complete drone mounted on the existing rig. Identify both direct motor commands and the Betaflight control path. Preserve raw observations so lever-arm and calibration corrections can be applied without recollection.

## Confirmed setup and explicit assumptions

| Item | Plan |
|---|---|
| Power supply | Multicomp Pro **72-13350 V2.3**, controlled through `deps/tenma-serial`, base revision `d1b73c2` plus the local 72-13350 patch |
| Supply capability | Manufacturer specifies 0–30 V, 0–30 A, **300 W**; use only the aircraft's permitted voltage range. At 8 V the 30 A ceiling corresponds to 240 W. Wiring/ESC/rig limits and the selected output-terminal rating may be lower; hardware ratings are not experiment limits. |
| Acquisition | Orin acquisition host, three high-rate Grove HX711 modules; the host reports an Orin NX development kit. Native force sampling was approximately 11 ms per sample in the battery campaign. Intermittent transfer faults remain unresolved. See the [three-module wiring guide](hx711-orin-wiring.md). |
| Force sensing | One axial thrust cell and two opposing torque-reaction cells; reuse the previous force gradient provisionally, with independent zero and sign for every cell. |
| Torque geometry | User-measured torque-cell spacing is **60 mm**. Assume **30 mm perpendicular arm per cell**, with the yaw axis midway; perpendicular alignment and other torque paths remain unverified. |
| Aircraft | Complete drone remains mounted. Direct-mode tests can run one motor at a time, then all four. Individual tests require the rig to restrain off-centre roll/pitch moments and preserve axial force measurement. |
| Flight configuration | Betaflight Acro, RX_MSP, existing battery compensation and deployment settings retained and recorded. No default switch to SBUS or disabled compensation. |
| Weight reference | Current drone mass 69.47 g; nominal hover force is approximately **0.6813 N**. Installed motor/propeller and battery-replacement cable configuration must be recorded. |

The supply replaces the flight battery for the controlled-voltage measurements. A separate battery-powered block measures actual battery sag; disconnect the supply before connecting the pack. Do not connect a battery in parallel as an unmeasured buffer. Secure the full aircraft with its intended propellers for thrust measurements. Keep an accessible physical power interruption independent of software.

## 1. What each command path measures

| Path | Input and route | Identified behaviour |
|---|---|---|
| Direct motor bench | `MSP_SET_MOTOR` (214), disarmed; M1–M4 external motor counts | Command → ESC/motor/propeller → thrust and reaction torque, bypassing the armed rate controller/mixer |
| Betaflight in the loop | `MSP_SET_RAW_RC` (200), RX_MSP, armed Acro; RC throttle with zero body-rate reference initially | RC processing, active controller/mixer, compensation, motor dynamics and resulting total thrust |
| Deployment confirmation | Existing CTBR command → production conversion/limits → Zenoh → ESP → RX_MSP → Betaflight | Complete deployed command-to-force response, including any software slew limits |

Use USB MSP for the direct baseline. Use the verified RX_MSP command connection for armed tests; if USB inhibits arming, use the existing ESP/UART4 path and identify that transport explicitly. Do not clear unrelated arming faults. The two primary modes use the same MSP-configured UART; no RX_SERIAL/SBUS configuration switch is required.

**“Raw PWM” will be labelled accurately.** With DShot, 1000–2000 external MSP counts are not physical PWM pulse widths or ESC duty cycle. Save the raw counts, normalized command, actual motor outputs and RPM. Save RC throttle separately: the same numeric RC and motor count do not imply the same actuation. Record motor protocol and output conversion.

For independent motor identification, send raw motor counts directly. Do not pass new thrust targets through the existing fitted newtons-to-counts inverse and then treat that as an independent raw-command experiment.

## 2. Reconstructing thrust and torque

For each cell, retain raw counts $c_i$ and independent stopped tare $b_i$. Initially use the earlier gradient magnitude

$$
k_c=21036.44535\ \mathrm{counts/N},\qquad
F_i=s_i(c_i-b_i)/k_c,
$$

with sign $s_i$ established from a gentle known loading direction. Mark the two transferred gradients as **assumed**, including their unverified reverse-loading response. Equal gradients do not imply equal offsets. Re-tare between blocks and retain stopped readings before and after each run.

Torque comes from the actual force geometry, not from subtracting two nominal torque readings:

$$
Q_{\mathrm{support},z}=\sum_i(\mathbf r_i\times\mathbf F_i)\cdot\hat{\mathbf z}.
$$

For the symmetric assumed arrangement $r_{1,x}=+\ell$, $r_{2,x}=-\ell$, with signed forces along a common Y direction,

$$
\ell=0.030\ \mathrm m,\qquad Q_{\mathrm{support},z}=\ell(F_1-F_2).
$$

A pure symmetric couple gives $F_1\simeq-F_2$, so the magnitude is **$2\ell|F_1|$**. Each cell measures part of the support reaction; simply averaging the two $\ell F$ terms would lose a factor of two. Under steady equilibrium, the drone's applied yaw torque is the opposite support reaction, after tare and accounting for other torque paths. Establish the sign using a known CW/CCW torque.

Plot both signed forces, the sign-corrected pair, and imbalance $F_1+F_2$. This comparison assumes the symmetric pure-couple geometry; revise it when actual contacts are supplied. Avoid relative-error ratios near zero torque. A common calibration error can make both cells agree while giving the wrong absolute torque.

Run stopped zero checks, a known axial load, a known torque in both directions if available, and an axial-load test for torque cross-talk. Check whether supports, bearings or cables carry yaw reaction that bypasses the cells. These checks quantify the shared-gradient assumption; initial collection may proceed with provisional scaling. Unknown arm lengths remain in metadata and torque plots must be labelled provisional. Replacing an equal assumed arm by its true value rescales torque linearly, without changing raw data.

**Whole-drone torque is not per-rotor torque.** CW and CCW motor reactions largely cancel during collective operation. Keep all four mounted but run M1–M4 individually to obtain each steady torque/thrust curve, then use all-motor runs to measure net torque and interactions. Retain output order; the recorded deployment convention is M1=RR, M2=FR, M3=RL, M4=FL, to be checked against this aircraft. Verify rotation directions and physical pole count.

For a displaced thrust line, include its $\mathbf r\times\mathbf T$ contribution about the measured axis; do not attribute every rig moment to propeller reaction torque. During acceleration, measured reaction also includes rotor angular acceleration and fixture dynamics. Use steady torque, not transient net rig torque, for shaft-efficiency calculations.

## 3. Acquisition and timing

The supply holds a fixed voltage using its internal constant-voltage loop. Python selects voltages between blocks; it does not implement a fast voltage regulator.

The current Tenma library sleeps **200 ms per command**. A sequential actual-V, actual-I, status triplet therefore takes at least approximately 0.6 s, before other overhead. Keep this worker independent from motor commands and sensor acquisition. Use `runningVoltage()` / `runningCurrent()` for measured V/A; `readVoltage()` / `readCurrent()` return setpoints. Setter inputs are mV/mA. Normalize the shared single-channel driver's status keys, which include trailing spaces, and record CV/CC and output status.

Check the actual `*IDN?` response against the connected **Multicomp Pro 72-13350**; reject an unidentified unit rather than using the SDK's fallback model. Use bounded serial transactions. Record the dependency revision and local modifications; retain tenma-serial as an external GPL dependency rather than copying its implementation into the hardware repository. Current setters format voltage to 0.01 V despite the device's finer advertised resolution; the proposed grid does not need finer steps.

| Stream | Collection plan and limitation |
|---|---|
| Three load cells | Independent raw counts, ready/read-completion timestamps, gain, signs, tare and validity. Prefer one ADC per cell rather than multiplexing gain/channel. The current three-channel bench uses separate confirmed GPIO pairs; see the wiring guide. |
| Tenma V, I, status | Approximately 1–1.5 complete triplets/s initially; measured throughput determines the final schedule. Adequate for settled points, not rapid electrical transients. |
| FC telemetry | Per-motor RPM, output commands, battery V/I, gyro, arming/mode/status. Retain each query's request/reply timestamps; they bracket receipt, not internal actuation. Validate pole-count conversion and telemetry freshness. |
| Betaflight Blackbox | **Not used, at the user’s request.** Timestamped MSP telemetry provides the current observations; no native-rate spectrum or internal actuator timestamps are claimed. |
| Fast electrical channel | For transient power and voltage-sag analysis, use synchronized, calibrated current plus voltage at both the source/pack output and ESC/FC input, preferably ≥1 kHz with adequate bandwidth. Record the exact probe boundaries and sensor insertion resistance. Tenma readings alone omit lead drop and are too slow; battery runs have no Tenma readback. |
| Fast mechanical channel | For tens-of-ms force/torque dynamics, target roughly 500–1000 samples/s per channel with suitable sensor, ADC and anti-alias bandwidth. Sampling rate alone does not establish usable bandwidth. |

The previous HX711 setup ran near **10 samples/s**. HX711 supports 10 or 80 SPS by hardware RATE selection. At 10 SPS, use settled data and identify RPM dynamics; do not claim separately resolved 20–40 ms thrust/torque poles or dead time. At 80 SPS, resolution improves but the ADC filter and rig response still limit identification. Datasheet startup/gain-change settling times are not a direct measurement of the continuous force-channel group delay. Identify or bound sensor/fixture response separately using an instrumented reference-force step/impulse and an independent trigger or reference sensor; a manual Enter marker is not an accurate force-onset timestamp. Otherwise label results as combined actuator-plus-measurement dynamics.

Put acquisition and command generation on the Orin where practical, using one monotonic clock and one owner for each serial device. If PC and Orin split responsibilities, log clock mapping/trigger alignment and its error. Log native-rate streams separately; align offline. Record timing distributions, dropouts and sensor/CPU saturation. Any high-rate logger must not starve GPIO timing or command refresh.

## 4. Experiment matrix

Start with a finite pilot. Expand the envelope only after measuring current, force and mount behaviour. **A supply rating is not an aircraft or rig test limit.**

Initial proposed supply levels are **7.6, 7.8, 8.0 and 8.2 V**, subject to the aircraft's accepted range. Use actual ESC-terminal voltage in fits. Existing inverse-fit bounds around 7.52–8.11 V are historical model limits, not limits on a new raw-count characterization; 8.2 V would be new coverage. No 8.4 V test is scheduled without confirming the installed system's limits.

| Stage | Excitation | Main result |
|---|---|---|
| A. Instrument/rig check | Output off; three-cell signs/tare/cross-talk; supply identity/setpoint/readback; telemetry and timing probe | Confirm the measurement chain before powered sweeps |
| B. Powered pilot at 7.8 V | Stopped baseline, then narrow direct-count steps; previously used 1250–1450 is a starting region only, subject to present limits | Determine stable operating range, current margin, noise, settling and drift; capture startup separately |
| C. Individual static maps | At each voltage, M1–M4 individually; 8–12 stable levels covering the verified usable range, with extra points near idle and hover-equivalent thrust | $T_i(u,V), Q_i(u,V), I(u,V), \omega_i(u,V)$ and motor-to-motor variation |
| D. Collective static maps | All motors at equal direct counts, same voltage/level grid | Total force/current, net yaw reaction and difference from the sum of isolated results |
| E. Armed Acro static maps | RC throttle grid at the same voltages, zero rate targets, actual active compensation/mixer settings | Directly measured $T_{\rm total}(r_{\rm throttle},V)$; net torque, RPM/output imbalance, usable authority |
| F. Dynamic identification | Repeated up/down steps around low, hover and higher operating points, at low/middle/high voltage; direct individual, direct collective and armed collective | Delay, rise/fall dynamics and operating-point/voltage dependence |
| G. Held-out validation | Separate multilevel sequence, then bounded band-limited PRBS or multisine if rig/sampling permit | Predictive performance of static and dynamic models on unseen inputs |
| H. Deployment confirmation | Short hover-region collective sequence through production CTBR conversion and Zenoh/ESP | Check the final newtons-to-RC mapping and total command-to-force response |
| I. Voltage sag and recovery | Matched whole-drone load pulses on the Tenma, then the actual flight packs at high/middle/lower usable charge; direct collective and armed Acro | Separate source regulation, harness loss, battery sag/recovery and compensation effects; see the battery block below |

For C–E, use ascending and descending sweeps with **three repeat cycles**, provided thermal limits allow. Initially allow 3–5 s per plateau; extend or reject the point unless the settled window meets predeclared noise/slope tolerances. At 10 SPS, retain at least 20 settled samples where practical. Record temperature, use cooldowns and interleave a fixed reference point to expose drift. Balance/reverse voltage order across repeats instead of always confounding low voltage with cold motors.

Begin dynamic steps with ten rise/fall pairs per selected condition, small enough to stay in the local linear range. Vary dwell lengths relative to the ADC sampling period. Use approximately 1.5–2 s dwell initially, extending it if the measured settling requires it. Include a separate startup/stop sequence; do not mix startup deadband with the spinning small-signal model. Choose broadband frequencies only after observing the fixture response and sensor bandwidth; avoid exciting mount resonances or commanding full-range random steps.

Reserve complete sequences/cycles for validation. Also withhold **all 8.0 V runs** from the initial static fit to check interpolation between 7.8 and 8.2 V. Use separate repeat sequences at measured voltages to check repeatability. After model selection and reporting these scores, refit the selected model on all accepted data. Do not use neighboring samples from the same plateau as an independent validation set.

Mark constant-current, voltage-collapse, saturation, clipping, warm-up and unsettled intervals. They remain in raw logs and diagnostic plots but do not become nominal constant-voltage map points. Slow Tenma status polling can miss brief current limiting; high-rate terminal V/I or an instrument trigger is needed to exclude short excursions confidently. Stop rather than continue a sweep into sustained current limit. The connected front terminals are rated 10 A. The PSU set current limit remains 30 A as requested; a separate 8 A measured-current test stop bounds this campaign. It is not a fast hardware current limiter. Higher-current tests would require the appropriate terminals and wiring.

**A fixed rig cannot identify the free-flight rate loop.** Nonzero body-rate references on a clamped aircraft can cause integrator windup and saturation. Armed collective tests therefore start at zero rate demand and retain PID/output logs. Small zero-mean yaw bursts are optional only within a verified rig envelope and quantify command-to-reaction-torque behaviour. A later low-friction single-axis rotational rig with gyro/angle measurements is required to identify true closed-loop rate tracking and update the CTBR angular response. This campaign can already improve collective thrust mapping and lag without that extra rig.

## 4a. Voltage sag, recovery and available thrust

Keep **three phenomena separate**: battery internal voltage drop and recovery; cable/connector loss; and Tenma regulation/current limiting. A constant-voltage bench supply is not a flight battery, and its transient response must not become the battery model.

**Measurements.** Log $V_{\rm pack}$ at the defined pack output, $V_{\rm ESC}$ at the aircraft input and total discharge current $I$, synchronized with RPM, force, torque and commands. For the supply comparison, replace $V_{\rm pack}$ by independently measured supply-terminal voltage. Prefer differential/Kelvin voltage measurements with a common timebase and suitable input ranges; do not connect pack voltages directly to Orin GPIO. Record which pigtails/connectors lie between probes, instrument offsets, current-sensor burden and the temporary test harness. Extra bench cable resistance must not be silently transferred to the flight model. FC voltage/current and Tenma telemetry remain secondary cross-checks; they cannot replace validated fast sensing for subsecond sag.

**Sequence.**

1. Repeat the same bounded whole-drone load-pulse family using the Tenma in CV mode. Measure source-terminal droop and $V_{\rm source}-V_{\rm ESC}$ to establish supply/harness behaviour. Keep voltage fixed during each pulse block; do not deliberately enter current limit to imitate battery sag.
2. Disconnect the Tenma and use the intended flight battery and flight connector. Record pack identity, chemistry, nominal capacity, age/cycles where known, actual starting/resting voltage, temperature and cumulative discharged Ah/Wh. Measure cell voltages where accessible. Confirm charge/discharge limits for that pack; no universal minimum voltage is assumed.
3. At **high, middle and lower usable charge**, run direct-collective pulses from a low spinning baseline toward hover and a higher verified load. Begin with 0.25 s, 2 s and 10 s pulses, three repeats each, reducing amplitude/duration if pack or thermal limits require it. This captures rapid drop, slower droop and recovery. Use actual measured current as the excitation: motor commands do not create ideal rectangular current pulses. Stop on the earliest cell, pack, current or temperature limit; this is not a deep-discharge/capacity-to-cutoff test.
4. Allow at least an initial 30 s low-load recovery between pulse groups, extending it until voltage drift meets a declared tolerance or marking relaxation incomplete. Collect a genuinely unloaded/rested voltage before and after each charge-state block where practical. Avionics-on voltage is a loaded baseline, not open-circuit voltage; a fixed 30 s wait does not guarantee electrochemical equilibrium.
5. Repeat a representative pulse subset through armed Acro at zero rate targets, keeping existing voltage compensation active. Log the resulting motor outputs, RPM, actual current and thrust. Match comparisons by measured load/operating point rather than assuming the same numeric command produces the same load. Validate a held-out mission-like collective sequence through the production CTBR path.

Use at least the deployment pack; preferably repeat on three representative packs to assess pack-to-pack variation. For one pack, label the result pack-specific. Charge-state bins are initially indexed by measured resting voltage and discharged charge. Do not claim exact SOC from terminal voltage alone or normalize by nominal capacity as though usable capacity had been measured. Time/temperature/charge-state effects must not be mistaken for repeatability noise.

**Analysis.** For discharge-positive current, report incremental apparent resistance at explicitly stated times after a measured current change:

$$
R_{\rm app}(t)=-\frac{\Delta V_{\rm pack}(t)}{\Delta I(t)}.
$$

Show rapid voltage minimum, drop at 100 ms/1 s/end-of-pulse where supported, and recovery traces. Account for the current ramp, filtering and input capacitance; a finite-time ratio is not automatically the instantaneous ohmic resistance. Estimate harness resistance from the slope of $(V_{\rm pack}-V_{\rm ESC})$ versus current, accounting for offsets. Fit pack parameters from pack-terminal voltage; fitting ESC voltage alone combines battery and harness effects.

Start with a state-dependent resistance plus one polarization branch, adding a second branch only if held-out data justify it:

$$
V_{\rm pack}=V_{\rm OC}(z,\theta)-I R_0(z,\theta)-v_p,\qquad
\dot v_p=-\frac{v_p}{\tau_p}+\frac{R_p}{\tau_p}I,
$$

$$
V_{\rm ESC}=V_{\rm pack}-I R_{\rm harness},\qquad
\dot z=-\frac{I}{3600C_{\rm Ah}}.
$$

Here $z$ is fractional SOC, $\theta$ temperature and $C_{\rm Ah}$ usable capacity if independently known. If capacity or equilibrium OCV is not measured, retain it as uncertain and use an empirical voltage/charge-bin model instead of presenting a fully identified SOC model. The polarization state represents delayed voltage droop/recovery; it is distinct from motor/propeller lag. Compare resistance-only versus RC models using full held-out pulse/mission traces and unseen packs where available. Restrict temperature dependence to the temperatures actually observed.

Deliver sag/recovery overlays by pack and charge bin, apparent resistance versus charge state, separated harness drop, model validation and **thrust loss versus sag**. Feed the fitted $V_{\rm ESC}(t)$ into the independently measured voltage-dependent actuator maps. Armed-Acro predictions must also represent the recorded compensation settings. This gives realistic voltage-dependent thrust authority without double-counting sag or treating compensation's response as intrinsic motor dynamics. Until fast dual-point V/I sensing is confirmed, collect slower sag envelopes and label rapid battery/harness separation unresolved.

## 5. Curves, efficiency and model fitting

Produce per-voltage overlays and compact surfaces for thrust, signed torque, current, RPM and efficiency against raw motor counts and, separately, RC throttle. Publish per-rotor direct maps, collective direct maps and armed throttle maps as different datasets/models.

For steady samples,

$$
P_e=V_{\rm bus}I_{\rm bus},\qquad
E_T=T/P_e\quad[\mathrm{N/W}],
$$

and, for an individually characterized rotor with measured mechanical angular speed,

$$
P_{\rm shaft}=|Q|\omega,\qquad
\eta_{\rm drive}=P_{\rm shaft}/P_e.
$$

Also show g/W using $1000T/(gP_e)$. Thrust per watt is a useful static efficiency metric, not dimensionless propulsive efficiency. Whole-drone net torque cannot be used in $Q\omega$ to obtain total shaft power because the counter-rotating reactions cancel. Report the electrical measurement boundary: gross supply input includes lead losses and avionics; terminal power excludes lead losses. Provide a separately measured stopped-electronics baseline and optionally baseline-subtracted propulsion power, labelled explicitly. Tenma V/I samples are sequential; only use their pair as steady power when the plateau is stable. Fast power requires synchronized V/I samples.

First fit smooth, monotonic-in-command maps where appropriate:

$$
T_i=f_T(u_i,V),\quad Q_i=f_Q(u_i,V),\quad
I=f_I(\mathbf u,V),\quad \omega_i=f_\omega(u_i,V).
$$

Do not force collective current into a sum of four individual currents without subtracting the repeatedly counted electronics baseline. Compare speed-based models $T_i=k_{T,i}\omega_i^2$ and $|Q_i|=k_{Q,i}\omega_i^2$, including residual voltage dependence if data support it. Measured speed may explain most voltage dependence without voltage-dependent aerodynamic coefficients. Derive $Q_i/T_i$ for simulator yaw reaction coefficients; this ratio has units of metres. Record rotor signs independently.

For dynamics, compare a delayed first-order model against second-order candidates, including two real poles:

$$
G_1(s)=\frac{K e^{-Ls}}{1+\tau s},\qquad
G_2(s)=\frac{K\omega_n^2e^{-Ls}}{s^2+2\zeta\omega_ns+\omega_n^2},\qquad
G_{2r}(s)=\frac{K e^{-Ls}}{(1+\tau_1s)(1+\tau_2s)}.
$$

Fit around each operating point; investigate rising/falling asymmetry and voltage dependence. Prefer a nonlinear static map plus dynamic state model when one global linear transfer function is inadequate. Predict complete held-out traces without resetting hidden state at every edge. Select complexity from held-out error, rise/fall timing and residual structure, with run-level uncertainty. Do not force second order merely because an uncorrected force trace has two apparent poles.

Separate these times where measured: host scheduling/transmission; FC receipt/RC filtering; controller/mixer output; output-to-RPM; RPM-to-physical thrust/torque; ADC/filter/fixture response. Report unresolved combinations and confidence intervals rather than subtracting unrelated median delays. Snapshot the production thrust slew limit and measure its effect separately; it must not be absorbed into intrinsic motor lag. Avoid applying HX711 or fixture filtering to simulated physical thrust.

## 6. Planned implementation and outputs

Extend the existing [acquisition tools](../tools/acquisition/README.txt), retaining their finite-test, no-device-access-without-`--execute` convention:

1. Configurable three-cell reader with verified GPIO mapping, independent calibration/signs/timing and sample-rate probe. Existing native reader has hard-coded pins and cannot simply be instantiated three times unchanged.
2. Tenma worker with strict model detection, normalized status, limits, setpoint/readback verification and no blocking of command refresh. Hardware limits and experiment limits are separate.
3. Explicit direct-motor and armed-RC experiment modes, a shared experiment manifest, one serial owner per device, and recorded watchdog/stop outcomes. Do not silently change FC profiles to cross between them.
4. Separate battery/source experiment manifests and fast dual-point voltage/current acquisition, with pack identity, charge/temperature metadata, pulse/recovery analysis and battery-versus-harness model fitting.
5. Reducer, diagnostic plots and static/dynamic fit comparison, with held-out sequences and versioned JSON parameter exports. Preserve the current actuator profile until the new dataset is reviewed.

Each run stores the experiment manifest, aircraft/rig/prop/ESC identities, FC firmware and full CLI dump, PID/rate/mixer/filter/smoothing/compensation/idle settings, motor mapping/pole count, host and dependency versions, measured/assumed calibration and geometry, raw per-stream CSV and events (Blackbox excluded at user request) with monotonic timestamps. Record commanded versus actual voltage/current, raw motor/RC inputs, any newtons-to-RC conversion and slew limiting. New voltage maps must not overwrite historical raw data.

Use separate direct STOP and armed minimum-throttle/disarm cleanup paths. Abort on lost/stale commands, sensor failure, force/current/voltage limits, unexpected arming/mode or sustained saturation. Do not rely on a Python `finally` block alone to stop direct motor test outputs after a host crash: verify an independent stop mechanism and accessible power cut. Confirm every motor has stopped before switching experiment modes. PSU output-off is the last fallback and may leave USB-powered electronics running.

Deliver:

- Per-voltage thrust/torque curves versus raw motor counts and RC throttle, with repeats, uncertainty and excluded-point flags.
- Voltage-command surfaces, per-motor asymmetry, collective interaction and opposing-cell consistency plots.
- Gross/net electrical power, thrust-per-watt and defensible per-motor shaft-efficiency curves.
- Measured/predicted step and held-out traces, first/second-order comparison, delay attribution and unresolved bandwidth limits.
- Battery sag/recovery, harness resistance and supply-droop diagnostics, with pack/charge-state variation and voltage-dependent available thrust.
- A compact report plus versioned simulator-ready maps/dynamics with validity ranges, calibration assumptions and provenance. Updating MJX/Isaac and training configurations follows model validation; no policy retraining is launched by this plan.

**Next implementation order:** supply/readers and static pilot → direct static curves → armed collective curves → dynamics within measured bandwidth → battery pulse/sag characterization → deployment-path confirmation. Actual lever arms and GPIO/ADC configuration can be filled in later; placeholder values must remain visibly provisional.

## Sources and existing evidence

- [Battery equivalent-circuit model](https://www.mathworks.com/help/simscape-battery/ref/batteryequivalentcircuit.html) and [pulse-characterization method](https://www.mathworks.com/help/simscape-battery/ug/battery-cell-characterization-for-ev.html): resistance/polarization structure and pulse/recovery identification; the proposed drone pulse durations are campaign choices, not a standardized HPPC protocol.
- [72-13350/72-13360 manufacturer manual](https://www.farnell.com/datasheets/4509046.pdf): electrical ratings and interfaces.
- Local `deps/tenma-serial/tenma/tenmaDcLib.py`: 72-13360 API/status keys, units, command formatting and 200 ms command wait; checkout relative to the parent drone workspace.
- [AVIA HX711 datasheet](https://www.mouser.com/datasheet/2/813/hx711_english-1022875.pdf): hardware rates, gain/channel selection and startup settling limitations.
- [Betaflight 4.5.1 MSP source](https://github.com/betaflight/betaflight/blob/4.5.1/src/main/msp/msp.c): direct writes update `motor_disarmed`; raw RC is delivered to RX_MSP. Verify actual installed firmware at collection.
- [Previous force calibration](../data/raw/loadcell/calibration.json), [acquisition workflow](../tools/acquisition/README.txt), [prior actuator data](../data/README.md) and [current actuator parameters](../parameters/actuators.json).
