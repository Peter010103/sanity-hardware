# MPC model: rigid-body dynamics and delays

This document describes Sanity’s 3D quadrotor model and its two implemented CTBR MPC formulations, checked against the workspace sources on **23 October 2026**. CTBR means **collective thrust and body rates**: the host requests total thrust and angular rates, while Betaflight supplies the inner rate controller and mixer. The [actuator modelling guide](actuator-modelling.md) contains the measured force/torque models and calibration details.

| Implementation | Prediction state | Inner-loop treatment |
|---|---|---|
| Trajectory baseline, `CtbrMPC` | 10: position, quaternion, velocity | Commanded body rate is achieved immediately; no actuator lag or explicit delay |
| Experimental full-attitude, `PitchLoopMPC`, legacy mode | 14: add body rate and actual collective force | Assumed rate servo, inertia, torque/headroom limits and collective lag |
| Experimental full-attitude, `PitchLoopMPC`, flight mode | 22: add eight roll/pitch response states | Identified second-order roll/pitch response, approximate internal delay, and command-history prediction |

The experimental controller retains all three attitude/rate axes despite its pitch-loop name. Its small-upright flight identification does not validate inverted or high-rate performance. The equations below distinguish physical dynamics from the approximations actually used in prediction.

## 1. Coordinates, state and inputs

World frame W has Z up. Body frame B has **X forward, Y left, Z up**. Position and linear velocity are expressed in W; angular velocity and moments are expressed in B. Let

$$
x_{RB}=\begin{bmatrix}p_W\\q_{WB}\\v_W\\\omega_B\end{bmatrix}\in\mathbb R^{13},
\qquad q_{WB}=\begin{bmatrix}q_0\\q_v\end{bmatrix},\quad \|q\|=1.
$$

The scalar-first Hamilton quaternion `w,x,y,z` rotates body vectors into world coordinates. Its rotation matrix is

$$
R(q)=(q_0^2-q_v^Tq_v)I+2q_vq_v^T+2q_0[q_v]_\times,
\qquad [a]_\times b=a\times b.
$$

The physical rigid body has 12 independent states; the quaternion uses four numbers with one norm constraint. A world-frame angular-rate measurement must be converted by $\omega_B=R(q)^T\omega_W$ before applying the rotational equations. The pitch mission performs this conversion; baseline prediction ignores the measured rates.

CTBR input is

$$
u_c=\begin{bmatrix}F_c&\omega_{cx}&\omega_{cy}&\omega_{cz}\end{bmatrix}^T,
$$

with **total** requested thrust $F_c$ in N and requested body rates in rad/s. Actual thrust $F$ and actual rates $\omega_B$ need not equal their requests. Motor-test counts and RC throttle are different downstream quantities.

## 2. Full 3D rigid-body equations

With mass $m$, body-frame inertia $J$ about the centre of mass, total thrust $F\ge0$, and resultant body moment $\tau_B$:

$$
\boxed{\begin{aligned}
\dot p_W&=v_W,\\
\dot q_{WB}&=\tfrac12q_{WB}\otimes\begin{bmatrix}0\\\omega_B\end{bmatrix},\\
m\dot v_W&=-mg e_3+R(q)e_3F+f_{ext,W},\\
J\dot\omega_B&=\tau_B-\omega_B\times(J\omega_B),
\end{aligned}}
\qquad e_3=\begin{bmatrix}0&0&1\end{bmatrix}^T.
$$

Here $f_{ext,W}$ denotes any separately modelled external force. The experimental controller sets it to zero. The cross-product term is the rigid-body gyroscopic coupling; omitting it is a modelling simplification, not part of Newton–Euler dynamics. Explicitly,

$$
\dot q_0=-\tfrac12 q_v^T\omega_B,
\qquad
\dot q_v=\tfrac12(q_0\omega_B+q_v\times\omega_B).
$$

These equations remain valid through large rotations and inverted attitudes. Positive body-Z thrust rotates with the aircraft; gravity always points down in W. At level hover, $F=mg$, $\omega_B=0$ and $\tau_B=0$.

The [hardware URDF](../models/sanity.urdf) uses $m=0.06947$ kg and $J=\operatorname{diag}(3.32,4.28,5.43)\times10^{-5}$ kg m², with the documented CoM/fixture assumptions. The **experimental pitch configuration uses the newer all-up mass 0.06985 kg**, retaining the same inertia. The trajectory baseline reads mass from its selected XML. These are distinct parameter sources; the MPC does not automatically load the compact hardware JSON.

## 3. From rotor forces to the body wrench

For rotor $i$, let $r_i=[x_i,y_i,z_i]^T$ be its location relative to the CoM, $f_i$ its axial thrust, and $\sigma_i$ the sign of its reaction moment **on the body**. Assuming parallel rotor thrust axes,

$$
F=\sum_{i=1}^4 f_i,\qquad
\tau_B=\sum_{i=1}^4\left[r_i\times(e_3f_i)+\sigma_iQ_i e_3\right].
$$

The separate speed-based force and reaction laws are

$$
f_i=k_T\Omega_i^2,\qquad Q_i=k_Q\Omega_i^2,
\qquad \kappa=\frac{k_Q}{k_T}\quad[\mathrm m],
$$

where $\Omega_i$ is rotor angular speed, not body rate. Thus

$$
\begin{bmatrix}F\\\tau_x\\\tau_y\\\tau_z\end{bmatrix}
=Bf,\qquad
B=\begin{bmatrix}
1&1&1&1\\
y_1&y_2&y_3&y_4\\
-x_1&-x_2&-x_3&-x_4\\
\sigma_1\kappa&\sigma_2\kappa&\sigma_3\kappa&\sigma_4\kappa
\end{bmatrix}.
$$

The calibrated $k_Q=1.5888988282\times10^{-10}$ N m/(rad/s)² is a magnitude; physical rotor order/spin signs require a consistent mapping. The experimental virtual allocation uses **effective armed** $k_T=3.0676404012\times10^{-8}$ N/(rad/s)², giving $\kappa=0.0051795472$ m. This differs from the isolated-motor $k_T$ in the surface viewer. Its yaw allocation magnitude is $1/(4\kappa)=48.2667673\ \mathrm{m^{-1}}$.

The measured static surface $f_i=\mathcal T(c_i,V_{FC})$ maps **direct motor-test command** $c_i$ and delivered voltage to isolated-rotor thrust. It is not directly evaluated by either CTBR MPC. The current CTBR path uses its existing collective conversion; virtual rotor allocation below approximates available moment authority without reproducing Betaflight’s mixer/PID internals.

## 4. The 10-state trajectory baseline

The baseline eliminates rate and motor dynamics by assuming $\omega_B=\omega_c$ and $F=F_c$. Its prediction state is $x=[p_W,q_{WB},v_W]$. Internally its first optimisation input is **specific thrust** $a_T=F_c/m$ in m/s²:

$$
\dot p=v,\qquad
\dot q=\tfrac12q\otimes[0,\omega_c],\qquad
\dot v=R(q)e_3a_T-ge_3-R(q)D R(q)^Tv.
$$

$D$ is an optional diagonal body-axis drag coefficient in s⁻¹, zero in the checked profiles. The outgoing thrust is converted back to N by multiplying by $m$. Inertia and calibrated yaw torque do not enter this ideal-rate prediction; they remain part of the physical plant. No fitted thrust surface, motor lag or delay is inserted into this baseline.

## 5. The augmented CTBR model

The experimental prediction state starts with

$$
x_{14}=[p_W,q_{WB},v_W,\omega_B,F].
$$

### Collective-force dynamics

A first-order observation-based approximation distinguishes rising and falling collective force:

$$
F_t=\operatorname{clip}(F_c,0,F_{\max}),\qquad
\dot F=\frac{F_t-F}{\tau_F},\qquad
\tau_F=\begin{cases}
0.0438819\ \mathrm s,&F_t\ge F,\\
0.0485511\ \mathrm s,&F_t<F.
\end{cases}
$$

These time constants came from the armed Acro force response. They include measurement/fixture effects and are not independently identified motor-only poles. They are retained modelling approximations in this MPC; no additional fitted force deadtime is added here.

### Rate dynamics and moment authority

Legacy mode requests the body moment

$$
\tau_d=K_v(\omega_c-\omega_B),\qquad
K_v=\operatorname{diag}(0.00083,0.00107,0.000543).
$$

$K_v$ converts rate error to moment. Before limiting and ignoring cross-axis coupling, the associated rate time constants are $J_{aa}/K_{v,a}=[0.040,0.040,0.100]$ s. They are assumed servo parameters.

Both modes clip requested moment and reduce it near collective saturation. Let $A$ be the configured `allocation_per_m` matrix, such that $\Delta f=A\bar\tau$ is the virtual per-rotor thrust change needed for moment $\bar\tau$:

$$
\begin{aligned}
\bar\tau&=\operatorname{clip}(\tau_d,-\tau_{lim},\tau_{lim}),\\
H(F)&=\frac{\rho}{4}\max\{\min(F-F_{\min},F_{\max}-F),0\},\\
\lambda&=\min\left\{1,\frac{H(F)}{\max(\|A\bar\tau\|_\infty,10^{-8})}\right\},\\
\tau_B&=\lambda\bar\tau,\qquad
\dot\omega_B=J^{-1}\left[\lambda\bar\tau-\omega_B\times(J\omega_B)\right].
\end{aligned}
$$

The checked configuration has $\rho=0.7$, $\tau_{lim}=[0.032,0.032,0.011]$ N m and physical collective bounds $[0.031383,3.101025]$ N. These bounds are parameters and can be replaced by the runner’s voltage-dependent values. This construction reserves rotor headroom; it is not an exact model of actual motor mixing, imbalance or voltage-dependent torque saturation.

### Identified roll/pitch response and internal delay

Flight mode replaces the legacy roll/pitch servo with a measured closed-loop response, for $a\in\{x,y\}$:

$$
\frac{\omega_a(s)}{\omega_{c,a}(s)}
=K_a\frac{\omega_{n,a}^2(1+T_{z,a}s)}
{s^2+2\zeta_a\omega_{n,a}s+\omega_{n,a}^2}\,e^{-T_{d,a}s},
\qquad \omega_n=2\pi f_n.
$$

| Axis | Gain $K$ | $f_n$ [Hz] | Damping $\zeta$ | Zero $T_z$ [ms] | Internal delay $T_d$ [ms] |
|---|---:|---:|---:|---:|---:|
| Roll | 1.01809 | 16.3035 | 0.353584 | 7.34025 | 10.36224 |
| Pitch | 1.02577 | 12.1509 | 0.474232 | 13.46891 | 10.39952 |

These are the `query_proxy` timestamp estimates. The source profile also stores the alternative response-timestamp fit; acquisition timestamps do not independently identify FC sample time. Yaw retains the assumed first-order response, and collective retains the bench-derived lag.

The solver approximates each fitted internal deadtime using **three cascaded first-order states** with $\tau_d=T_d/3$:

$$
\dot d_1=\frac{\omega_c-d_1}{\tau_d},\qquad
\dot d_2=\frac{d_1-d_2}{\tau_d},\qquad
\dot d_3=\frac{d_2-d_3}{\tau_d},\qquad
 e^{-T_ds}\approx(1+T_ds/3)^{-3}.
$$

This is a finite-band approximation, not an exact time shift. One additional state $\ell_a$ per axis realises the second-order response together with the existing rate state:

$$
\dot\ell_a=\frac{\omega_a/K_a-\ell_a}{T_{z,a}},\qquad
\alpha_a=K_a\left[(1-2\zeta_a\omega_{n,a}T_{z,a})\dot\ell_a
+T_{z,a}\omega_{n,a}^2(d_{3,a}-\ell_a)\right].
$$

With no saturation, this is equivalent to $\ddot\ell+2\zeta\omega_n\dot\ell+\omega_n^2\ell=\omega_n^2d_3$ and $\omega=K(\ell+T_z\dot\ell)$. The commanded moment is

$$
\tau_d=J\alpha+\omega_B\times(J\omega_B),\qquad
\alpha_z=\frac{K_{v,z}}{J_{zz}}(\omega_{c,z}-\omega_z).
$$

It then passes through the same moment clipping/headroom model. Gyroscopic compensation makes the unconstrained rate response equal $\alpha$; saturation can change it. The complete flight prediction state is

$$
x_{22}=[x_{14},\ell_x,\ell_y,d_{1,x},d_{2,x},d_{3,x},d_{1,y},d_{2,y},d_{3,y}].
$$

## 6. Delay compensation and state preparation

The experimental runner keeps **selected outgoing commands**, including PD/hover commands. If a command was selected at $t_j$, its predicted application is shifted by $\delta_{out}$:

$$
u_{hist}(t)=u_j\quad\text{for the latest }t_j\le t-\delta_{out}.
$$

Let $t_s$ be the observation timestamp, $t$ the current host time, $a=t-t_s$ its age, and

$$
\delta_{out}=\delta_{host\ output}+\delta_{external},\qquad
\delta_{pred}=\delta_{selection}+\delta_{out}.
$$

The checked defaults are **20 ms selection + 15 ms host output**, with an optional explicitly supplied external command delay of 0–40 ms. These defaults describe host scheduling; they are not a measured Wi-Fi/ESP/FC latency decomposition. The runner constructs the initial MPC state by integrating the known command history from $t_s$ to $t+\delta_{pred}$:

$$
x_{0|t}=\Phi\left(t+\delta_{pred},t_s,\hat x(t_s),u_{hist}\right).
$$

Thus the propagation duration is $a+\delta_{pred}$, not just 35 ms. The reference preview is shifted to the same future time. Integration intervals split at command changes, rather than applying the newest command over the whole stale-observation interval. Collective force is estimated causally from that history, not measured directly in flight.

Measurement filtering is treated separately. The roll/pitch observer contains the configured **50 ms angular-rate low-pass**, with a measurement state satisfying $\tau_m\dot y_m=\omega-y_m$. A causal linear observer estimates the hidden rate/delay states from filtered rates and past commands; its covariance choices are engineering assumptions. This measurement-filter state is outside the 22-state optimisation model.

The worker also applies the configured pose-filter prediction and a **22.5 ms velocity extrapolation** using model acceleration. For position/attitude filtering, the runner’s default approximate lag is

$$
\delta_{pose}=\frac{h}{\exp(h/\tau_{pose})-1},\qquad
h=0.005\ \mathrm s.
$$

For $\tau_{pose}=0.010$ s this is about 7.71 ms. The worker advances position using velocity and advances attitude with $q\leftarrow q\otimes\operatorname{Exp}_q(\delta_{pose}\omega_B)$. Here $\operatorname{Exp}_q(r)=[\cos(\|r\|/2),\;\sin(\|r\|/2)r/\|r\|]$ with its identity limit at zero. These are observer/state-preparation approximations, not additional independently identified pure delays. In flight-rate mode the observer handles angular-rate filtering; the legacy angular-rate extrapolation is not also applied.

**Delay accounting:** internal FC delay is already represented by the three-stage states; host/output timing is represented by command history; observation age determines the rollout start. A filter time constant is not a pure deadtime. Do not add these quantities again as one extra motor lag or treat a serial query round trip as one-way actuation delay. The baseline `CtbrMPC` does not contain this experimental history-prediction path.

## 7. Discrete-time nonlinear MPC

For either continuous model $\dot x=f(x,u)$, prediction uses zero-order-held inputs and RK4:

$$
\begin{aligned}
k_1&=f(x,u),&k_2&=f(x+hk_1/2,u),\\
k_3&=f(x+hk_2/2,u),&k_4&=f(x+hk_3,u),\\
x^+&=x+\tfrac h6(k_1+2k_2+2k_3+k_4).
\end{aligned}
$$

The quaternion is renormalised after each complete integration substep. The baseline uses one RK4 step per prediction interval. The experimental legacy mode uses two substeps; flight mode uses $\max(2,\lceil\Delta t/0.005\rceil)$ substeps to resolve its faster rate/delay states. History forecasting also uses substeps of at most 5 ms.

The receding-horizon problem is

$$
\begin{aligned}
\min_{x_{0:N},u_{0:N-1}}\quad&
\sum_{k=0}^{N-1}\left(\|e(x_k,x_k^r)\|_Q^2+\|u_k-u_k^r\|_R^2\right)
+\|e(x_N,x_N^r)\|_P^2,\\
\text{subject to}\quad&x_0=\hat x_{application},\quad x_{k+1}=f_d(x_k,u_k),\\
&u_{min}\le u_k\le u_{max},\quad\text{configured state constraints}.
\end{aligned}
$$

Here $\|z\|_Q^2=z^TQz$. Baseline input/cost uses specific thrust; experimental input/cost uses newtons. The quaternion attitude error is

$$
e_R=2\operatorname{Im}\{(q^r)^{-1}\otimes q\},
$$

which approximates rotation-vector error for small discrepancies. The baseline residual stacks $[p-p^r,v-v^r,e_R]$. The experimental residual additionally includes $[\omega-\omega^r,F-F^r]$; its eight latent response states have no direct tracking penalty. Both use $P=2Q$ in the checked default configurations.

The checked diagonal cost weights are:

| Cost block | Baseline | Experimental |
|---|---|---|
| Position $Q_p$ | $[30,30,60]$ | $[80,80,120]$ |
| Velocity $Q_v$ | $[4,4,6]$ | $[8,8,12]$ |
| Attitude $Q_R$ | $[1,1,2]$ | $[12,12,12]$ |
| Body rate $Q_\omega$ | Not in the state | $[0.1,0.1,0.1]$ |
| Actual force $Q_F$ | Not in the state | $0.1$ |
| Input $R$ | $[0.02,0.1,0.1,0.1]$ | $[0.1,0.03,0.03,0.03]$ |

The experimental state weights are code defaults; its input weights are explicit in the checked configuration. Numerical weights depend on the stated units.

For the baseline position/velocity/acceleration preview, define $s^r=a^r+ge_3$. Feedforward uses $a_T^r=\|s^r\|$, body-Z direction $b_3^r=s^r/\|s^r\|$, and a yaw-zero attitude constructed with $b_2^r\propto b_3^r\times e_1$, $b_1^r=b_2^r\times b_3^r$. Its rate reference is $\omega_k^r=\operatorname{Log}(R_k^{rT}R_{k+1}^r)/\Delta t$, with Log returning a rotation vector.

The experimental runner supplies a complete reference $[p^r,q^r,v^r,\omega^r,F^r,u^r]$ at each node. The checked run-in reference sets $F^r=m\|a^r+ge_3\|$ and approximately compensates the collective lag with $F_c^r=F^r+\tau_F\dot F^r$. Its pitch feedforward remains $\omega_{cy}^r=\omega_y^r+(J_{yy}/K_{v,y})\dot\omega_y^r$, even when the MPC uses the identified flight-rate prediction. It is not an exact inverse of the identified second-order/delay model; optimisation can deviate from this feedforward through its input cost.

| Setting | Baseline default | Experimental configuration/defaults |
|---|---|---|
| Horizon | $N=10$, $\Delta t=0.1$ s: 1.0 s | $N=20$, $\Delta t=0.04$ s: 0.8 s |
| Applied control tick | 20 ms | 20 ms |
| Input bounds | Selected XML CTBR bounds; thrust divided by mass internally | $F_c\in[0,2.85]$ N; rate magnitudes $[2.3,9.5,2.6]$ rad/s |
| State constraints | $e_3^TR(q)e_3\ge\cos\theta_{max}$; 30° default, 60° high-bank profile | Height 0.12–2.0 m; no upright tilt constraint, allowing full rotations |
| Solver | acados SQP-RTI, one nonlinear iteration per tick | acados SQP-RTI, two requested iterations in checked configuration |

Both use Gauss–Newton least squares and HPIPM QPs. RK4 defines the nonlinear prediction; RTI linearises it around the current iterate to solve the constrained QP. RTI is not a nonlinear solve to convergence. The acados one-half cost convention is cancelled by using weight matrices $2Q,2R$; stage cost scaling is one, without an extra $\Delta t$ factor. Previous controls are shifted by a 20 ms tick for warm start. Only the first feasible command is selected, then the problem is reinitialised from the next observation. Solver status, raw bounds, nonlinear rollout and the command-computation deadline are checked; a command lease/watchdog is a separate transport mechanism, not a delay state.

## 8. Parameter ownership and source snapshot

This is a description of the checked implementation, not a change to dynamics, mission configuration or firmware. Static calibration does not identify drag, full Betaflight rate/mixer dynamics, high-rate/inverted response or every source of latency. In particular, a full rigid-body model can be structurally correct while its closed-loop actuator approximation remains conditional.

The following paths are relative to `/home/hw527/Projects/drones`; they belong to adjacent workspace repositories, not this portable hardware repo. Hashes identify the source snapshot (first 16 SHA-256 digits):

| Source | SHA-256 prefix |
|---|---|
| `loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_controller.py` | `3739f011e5e443d4` |
| `loco_mjx/experiments/sanity_pitch_mpc/controller.py` | `63f051c5284ba32c` |
| `loco_mjx/experiments/sanity_pitch_mpc/flight_prediction.py` | `0a526f933f9d7355` |
| `loco_mjx/experiments/sanity_pitch_mpc/run_in_reference.py` | `551648889cf12d13` |
| `Sanity/src/ros2_sanity_ws/src/sanity_zenoh/sanity_zenoh/pitch_mission.py` | `10aba7a9181aef60` |
| `Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc.json` | `fa556292ace4c576` |
| `Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_pitch_loop.json` | `78f9bf16d8d9c0ec` |
| `loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/flight_rate_20261022.json` | `942ee8fb83311bb3` |

See also [measured actuator models](actuator-modelling.md), [mass/inertia and their assumptions](inertia.md), and the [compact identified parameter file](../parameters/identified_actuators.json). The experimental flight-rate profile/configuration is separate from that compact static-calibration file.
