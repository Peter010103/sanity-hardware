#!/usr/bin/env python3
"""Nonlinear Geometric Controller with Incremental Nonlinear Dynamic Inversion (INDI)
for the Sanity quadrotor model with suspended cable (models/sanity_cable.xml).

This script provides a self-contained MuJoCo implementation of the INDI attitude and
position controller adapted from nphm_jax/env/2robots-manipulation-test_indi.py and
nphm_jax/env/utils.py.

Controller Architecture:
1. Incremental Nonlinear Dynamic Inversion (INDI):
   - Estimates external wrench (cable tension force and disturbance torque) acting on the
     aircraft by comparing model-predicted forces/moments with numerical accelerations (Δv/dt, Δω/dt).
   - Injects the estimated disturbance into the command law to actively reject cable dynamics.
2. Geometric SO(3) Attitude Control:
   - Computes desired body z-axis (z_b_des) from the compensated acceleration command.
   - Uses reduced quaternion tracking error on SO(3) for singularity-free large-angle attitude control.
3. Control Allocation Matrix (G_1):
   - Maps total thrust and body moments [T, Mx, My, Mz]^T to individual rotor thrusts and reaction
     torques according to the Sanity Quad-X rotor geometry.
"""

import argparse
import math
import os
from pathlib import Path

import numpy as np

# Set environment defaults for headless rendering
os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------- #
# Quaternion and SO(3) Math Utilities (Pure NumPy)
# --------------------------------------------------------------------------- #

def skew(vec: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix from a 3D vector for cross-product representation."""
    return np.array([
        [0.0, -vec[2], vec[1]],
        [vec[2], 0.0, -vec[0]],
        [-vec[1], vec[0], 0.0],
    ], dtype=np.float64)


def quat2rotm(quat: np.ndarray) -> np.ndarray:
    """Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix (body -> world)."""
    w, x, y, z = quat
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_product(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product of q1 and q2 (both [w, x, y, z])."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def quat_inverse(q: np.ndarray) -> np.ndarray:
    """Inverse of a unit quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_to_roll_pitch(quat: np.ndarray) -> tuple[float, float]:
    """Convert quaternion [w, x, y, z] to (roll, pitch) in radians."""
    w, x, y, z = [float(v) for v in quat]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def quat_to_euler(quat: np.ndarray) -> tuple[float, float, float]:
    """Convert quaternion [w, x, y, z] to (roll, pitch, yaw) in radians."""
    roll, pitch = quat_to_roll_pitch(quat)
    w, x, y, z = [float(v) for v in quat]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# --------------------------------------------------------------------------- #
# Control Allocation Matrix for Sanity Quad-X
# --------------------------------------------------------------------------- #

def build_allocation_matrix(
    d: float = 0.0284610479,
    kappa: float = 0.005,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the 4x4 allocation matrix G_1 mapping rotor thrusts to [T, Mx, My, Mz]^T.

    In models/sanity_cable.xml:
      Rotor 1 (actuator 0): Rear Right  pos = (-d, -d, 0)
      Rotor 2 (actuator 1): Front Right pos = (+d, -d, 0)
      Rotor 3 (actuator 2): Front Left  pos = (+d, +d, 0)
      Rotor 4 (actuator 3): Rear Left   pos = (-d, +d, 0)

    Moments tau_i = r_i x [0, 0, T_i]:
      Rotor 1 (RR): Mx = -d * T1, My = +d * T1, Mz = +kappa * T1
      Rotor 2 (FR): Mx = -d * T2, My = -d * T2, Mz = -kappa * T2
      Rotor 3 (FL): Mx = +d * T3, My = -d * T3, Mz = +kappa * T3
      Rotor 4 (RL): Mx = +d * T4, My = +d * T4, Mz = -kappa * T4
    """
    G_1 = np.array([
        [1.0, 1.0, 1.0, 1.0],           # Collective thrust T
        [-d, -d, d, d],                 # Roll moment Mx
        [d, -d, -d, d],                 # Pitch moment My
        [kappa, -kappa, kappa, -kappa], # Yaw reaction moment Mz
    ], dtype=np.float64)
    G_1_inv = np.linalg.inv(G_1)
    return G_1, G_1_inv


# --------------------------------------------------------------------------- #
# Incremental Nonlinear Dynamic Inversion (INDI) Controller (Pure NumPy)
# --------------------------------------------------------------------------- #

class SanityINDIController:
    """Nonlinear Geometric INDI Controller for Sanity Quadrotor.

    Estimates external linear forces (cable tension disturbance) and external
    torques acting on the aircraft using IMU/state differentiation and motor history.
    """

    def __init__(
        self,
        drone_mass: float = 0.06609,
        inertia_diag: tuple[float, float, float] = (5.75e-05, 6.64e-05, 9.31e-05),
        dt: float = 0.002,
        rotor_arm_d: float = 0.0284610479,
        kappa: float = 0.005,
        f_ext_cutoff_hz: float = 3.0,
        tau_ext_cutoff_hz: float = 3.0,
        f_ext_max: float = 50.0,
        tau_ext_max: float = 5.0,
        thrust_min_rotor: float = 0.0,
        thrust_max_rotor: float = 1.0,
        kp_att_xy: float = 150.0,
        kp_att_z: float = 10.0,
        kp_rate: tuple[float, float, float] = (25.0, 25.0, 8.0),
    ):
        self.m = float(drone_mass)
        self.inertia_diag = np.array(inertia_diag, dtype=np.float64)
        self.inertia_mat = np.diag(self.inertia_diag)
        self.dt = float(dt)
        self.kappa = float(kappa)

        self.G_1, self.G_1_inv = build_allocation_matrix(rotor_arm_d, kappa)

        # First-order Low-Pass Filter smoothing factors
        self.f_ext_alpha = dt / (1.0 / (2.0 * math.pi * f_ext_cutoff_hz) + dt)
        self.tau_ext_alpha = dt / (1.0 / (2.0 * math.pi * tau_ext_cutoff_hz) + dt)

        self.f_ext_max = float(f_ext_max)
        self.tau_ext_max = float(tau_ext_max)

        self.thrust_min_rotor = float(thrust_min_rotor)
        self.thrust_max_rotor = float(thrust_max_rotor)
        self.thrust_min_coll = self.thrust_min_rotor * 4.0
        self.thrust_max_coll = self.thrust_max_rotor * 4.0

        self.kp_att_xy = float(kp_att_xy)
        self.kp_att_z = float(kp_att_z)
        self.kp_rate = np.array(kp_rate, dtype=np.float64)

        self.g_vec = np.array([0.0, 0.0, -9.80665], dtype=np.float64)

        # State memory for INDI numerical differentiation & filtering
        hover_per_rotor = self.m * 9.80665 / 4.0
        self.prev_thrusts = np.full(4, hover_per_rotor, dtype=np.float64)
        self.prev_vel_lin = np.zeros(3, dtype=np.float64)
        self.prev_omega = np.zeros(3, dtype=np.float64)
        self.prev_f_ext = np.zeros(3, dtype=np.float64)
        self.prev_tau_ext = np.zeros(3, dtype=np.float64)

    def reset(
        self,
        init_vel_lin: np.ndarray | None = None,
        init_omega: np.ndarray | None = None,
    ):
        """Reset internal filter states and history."""
        hover_per_rotor = self.m * 9.80665 / 4.0
        self.prev_thrusts = np.full(4, hover_per_rotor, dtype=np.float64)
        self.prev_vel_lin = np.zeros(3, dtype=np.float64) if init_vel_lin is None else init_vel_lin.copy()
        self.prev_omega = np.zeros(3, dtype=np.float64) if init_omega is None else init_omega.copy()
        self.prev_f_ext = np.zeros(3, dtype=np.float64)
        self.prev_tau_ext = np.zeros(3, dtype=np.float64)

    def update(
        self,
        quat: np.ndarray,
        vel_lin: np.ndarray,
        omega: np.ndarray,
        action: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute motor thrusts and reaction torques from current state and action command.

        Parameters
        ----------
        quat : np.ndarray (4,)
            Current base_link quaternion [w, x, y, z].
        vel_lin : np.ndarray (3,)
            Linear velocity in world frame [vx, vy, vz].
        omega : np.ndarray (3,)
            Angular velocity in body frame [wx, wy, wz].
        action : np.ndarray (6,)
            Desired linear acceleration and angular velocity [ax, ay, az, wx_ref, wy_ref, wz_ref].

        Returns
        -------
        thrusts : np.ndarray (4,)
            Individual rotor thrust commands in Newtons.
        f_ext : np.ndarray (3,)
            Estimated external acceleration in world frame [m/s^2].
        tau_ext : np.ndarray (3,)
            Estimated external disturbance torque in body frame [Nm].
        """
        R = quat2rotm(quat)

        # ------------------------------------------------------------------ #
        # 1. External Wrench Estimation
        # ------------------------------------------------------------------ #
        mu_prev = self.G_1 @ self.prev_thrusts
        T_prev = mu_prev[0]
        tau_prev = mu_prev[1:4]

        # Numerical linear acceleration in world frame
        a_est_world = (vel_lin - self.prev_vel_lin) / self.dt
        f_model_world = R @ np.array([0.0, 0.0, T_prev], dtype=np.float64) / self.m
        f_ext_raw = a_est_world - f_model_world - self.g_vec

        # Low-pass filter external force estimate
        f_ext_filt = self.prev_f_ext + self.f_ext_alpha * (f_ext_raw - self.prev_f_ext)
        f_ext_world = np.clip(f_ext_filt, -self.f_ext_max, self.f_ext_max)

        # Numerical angular acceleration in body frame
        gyro = np.cross(omega, self.inertia_mat @ omega)
        alpha_est = (omega - self.prev_omega) / self.dt
        tau_ext_raw = self.inertia_mat @ alpha_est - tau_prev

        # Low-pass filter external torque estimate
        tau_ext_filt = self.prev_tau_ext + self.tau_ext_alpha * (tau_ext_raw - self.prev_tau_ext)
        tau_ext = np.clip(tau_ext_filt, -self.tau_ext_max, self.tau_ext_max)

        # ------------------------------------------------------------------ #
        # 2. Desired Acceleration & Collective Thrust
        # ------------------------------------------------------------------ #
        des_acc = action[0:3]
        omega_b_ref = action[3:6]

        # Total acceleration command: subtract gravity and cancel external force disturbance
        acc_cmd = des_acc - self.g_vec - f_ext_world
        collective_thrust_des = float(np.dot(acc_cmd, R[:, 2]) * self.m)
        T_cmd = np.clip(collective_thrust_des, self.thrust_min_coll, self.thrust_max_coll)

        # ------------------------------------------------------------------ #
        # 3. Geometric SO(3) Attitude Target
        # ------------------------------------------------------------------ #
        acc_norm = math.sqrt(float(np.sum(acc_cmd**2))) + 1e-8
        z_b_des = acc_cmd / acc_norm

        # Preserve current yaw angle when computing tilt
        yaw_current = math.atan2(
            2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
            1.0 - 2.0 * (quat[2]**2 + quat[3]**2) + 1e-8,
        )
        x_intermediate = np.array([math.cos(yaw_current), math.sin(yaw_current), 0.0], dtype=np.float64)
        y_b_des_un = np.cross(z_b_des, x_intermediate)
        y_b_des_norm = math.sqrt(float(np.sum(y_b_des_un**2))) + 1e-8
        y_b_des = y_b_des_un / y_b_des_norm
        x_b_des = np.cross(y_b_des, z_b_des)

        R_des = np.column_stack([x_b_des, y_b_des, z_b_des])

        # Quaternion representation of R_des
        tr = R_des[0, 0] + R_des[1, 1] + R_des[2, 2]
        q_cmd_w = math.sqrt(max(1.0 + tr, 1e-8)) / 2.0
        q_cmd_x = (R_des[2, 1] - R_des[1, 2]) / (4.0 * q_cmd_w)
        q_cmd_y = (R_des[0, 2] - R_des[2, 0]) / (4.0 * q_cmd_w)
        q_cmd_z = (R_des[1, 0] - R_des[0, 1]) / (4.0 * q_cmd_w)
        q_cmd = np.array([q_cmd_w, q_cmd_x, q_cmd_y, q_cmd_z], dtype=np.float64)
        q_cmd /= (math.sqrt(float(np.sum(q_cmd**2))) + 1e-8)

        # Reduced quaternion tracking error
        q_diff = quat_product(quat_inverse(quat), q_cmd)
        q_e_w, q_e_x, q_e_y, q_e_z = q_diff

        norm_factor = 2.0 / (math.sqrt(q_e_w**2 + q_e_z**2) + 1e-8)
        q_e_red = norm_factor * np.array([
            q_e_w * q_e_x - q_e_y * q_e_z,
            q_e_w * q_e_y + q_e_x * q_e_z,
            0.0,
        ], dtype=np.float64)
        q_e_yaw = norm_factor * np.array([0.0, 0.0, q_e_z], dtype=np.float64)

        alpha_b_des = (
            self.kp_att_xy * q_e_red
            + self.kp_att_z * np.sign(q_e_w) * q_e_yaw
            + self.kp_rate * (omega_b_ref - omega)
        )

        # ------------------------------------------------------------------ #
        # 4. INDI Moment Calculation & Motor Allocation
        # ------------------------------------------------------------------ #
        moments = self.inertia_mat @ alpha_b_des + gyro - tau_ext
        mu_indi = np.array([T_cmd, moments[0], moments[1], moments[2]], dtype=np.float64)

        thrusts = self.G_1_inv @ mu_indi
        thrusts = np.clip(thrusts, self.thrust_min_rotor, self.thrust_max_rotor)

        # Update history for next step
        self.prev_vel_lin = vel_lin.copy()
        self.prev_omega = omega.copy()
        self.prev_thrusts = thrusts.copy()
        self.prev_f_ext = f_ext_world.copy()
        self.prev_tau_ext = tau_ext.copy()

        return thrusts, f_ext_world, tau_ext


# --------------------------------------------------------------------------- #
# Main MuJoCo Simulation Loop
# --------------------------------------------------------------------------- #

def run_simulation(
    model_path: str = "models/sanity_cable.xml",
    duration: float = 5.0,
    mode: str = "hover",
    render: bool = True,
    video_path: str = "simulation_output.mp4",
    framerate: int = 60,
    gui: bool = False,
):
    """Run self-contained MuJoCo simulation of Sanity with cable and INDI control."""
    xml_file = (ROOT / model_path).resolve() if not Path(model_path).is_absolute() else Path(model_path)
    if not xml_file.is_file():
        raise FileNotFoundError(f"Model file not found: {xml_file}")

    import mujoco

    print(f"Loading MuJoCo model from: {xml_file}")
    model = mujoco.MjModel.from_xml_path(str(xml_file))
    data = mujoco.MjData(model)

    dt = model.opt.timestep
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    drone_mass = float(model.body_mass[base_id])
    total_mass = float(sum(model.body_mass))
    inertia_diag = tuple(float(x) for x in model.body_inertia[base_id])

    print(f"Model parameters: base drone mass = {drone_mass:.4f} kg, total suspended mass = {total_mass:.4f} kg")
    print(f"Simulation timestep: {dt*1e3:.2f} ms ({1.0/dt:.0f} Hz), Duration: {duration:.1f} s")

    # Geometry & allocation
    d = 0.0284610479
    kappa = 0.005

    # Position controller gains (outer-loop PD generating desired acceleration)
    Kp_pos = np.array([3.0, 3.0, 8.0])
    Kd_pos = np.array([1.0, 1.0, 3.0])
    target_pos = np.array([0.0, 0.0, 1.5])

    frames = []
    sim_steps_per_frame = max(1, int((1.0 / framerate) / dt))

    controller = SanityINDIController(
        drone_mass=drone_mass,
        inertia_diag=inertia_diag,
        dt=dt,
        rotor_arm_d=d,
        kappa=kappa,
    )

    renderer = None
    if render and not gui:
        renderer = mujoco.Renderer(model, height=480, width=640)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.distance = 1.6
        camera.elevation = -20
        camera.azimuth = 135
        camera.lookat = [0.0, 0.0, 1.2]

    print("Starting MuJoCo simulation loop...")
    step_count = 0

    viewer_context = None
    if gui:
        import mujoco.viewer
        viewer_context = mujoco.viewer.launch_passive(model, data)

    try:
        while data.time < duration:
            pos = data.qpos[0:3]
            quat = data.qpos[3:7]
            vel = data.qvel[0:3]
            omega = data.qvel[3:6]

            if mode == "demo_indi":
                # Action = 0 demonstrates pure INDI disturbance estimation & weight compensation
                action = np.zeros(6, dtype=np.float64)
            elif mode == "track":
                # Smooth 3D figure-8 / circular trajectory
                t = data.time
                cur_target = np.array([
                    0.2 * math.sin(t),
                    0.2 * math.cos(t) - 0.2,
                    1.5 + 0.1 * math.sin(2.0 * t),
                ])
                des_acc = Kp_pos * (cur_target - pos) - Kd_pos * vel
                action = np.concatenate([des_acc, np.zeros(3)])
            else:
                # Hover at target setpoint
                des_acc = Kp_pos * (target_pos - pos) - Kd_pos * vel
                action = np.concatenate([des_acc, np.zeros(3)])

            thrusts, f_ext, tau_ext = controller.update(quat, vel, omega, action)

            # Set rotor thrusts (0..3) and reaction torques (4..7)
            data.ctrl[0:4] = thrusts
            data.ctrl[4] = kappa * thrusts[0]
            data.ctrl[5] = -kappa * thrusts[1]
            data.ctrl[6] = kappa * thrusts[2]
            data.ctrl[7] = -kappa * thrusts[3]

            mujoco.mj_step(model, data)
            step_count += 1

            if step_count % int(0.25 / dt) == 0:
                roll, pitch = quat_to_roll_pitch(quat)
                print(
                    f"Time: {data.time:.2f}s | Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | "
                    f"Thrust: {np.sum(thrusts):.3f}N | f_ext: [{f_ext[0]:.2f}, {f_ext[1]:.2f}, {f_ext[2]:.2f}] | "
                    f"Roll: {math.degrees(roll):.1f}° Pitch: {math.degrees(pitch):.1f}°"
                )

            if viewer_context is not None and viewer_context.is_running():
                viewer_context.sync()

            if render and renderer is not None and step_count % sim_steps_per_frame == 0:
                renderer.update_scene(data, camera=camera)
                frames.append(renderer.render().copy())

    finally:
        if viewer_context is not None:
            viewer_context.close()
        if renderer is not None:
            renderer.close()

    # Save video if frames were recorded
    if render and frames:
        out_path = Path(video_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Simulation complete. Saving {len(frames)} frames to {out_path}...")
        import imageio
        imageio.mimsave(str(out_path), frames, fps=framerate)
        print(f"Saved video to '{out_path}'.")

    print("Finished simulation successfully.")


def main():
    parser = argparse.ArgumentParser(
        description="Nonlinear Geometric INDI Controller for Sanity Quadrotor with Cable (MuJoCo)"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="models/sanity_cable.xml",
        help="Path to MuJoCo XML model (default: models/sanity_cable.xml)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Simulation duration in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["hover", "track", "demo_indi"],
        default="hover",
        help="Control mode: 'hover' (setpoint hold), 'track' (trajectory tracking), 'demo_indi' (zero commanded acc, pure disturbance compensation)",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        default=True,
        help="Render video output of simulation (default: True)",
    )
    parser.add_argument(
        "--no-render",
        action="store_false",
        dest="render",
        help="Disable video rendering",
    )
    parser.add_argument(
        "--video-path",
        type=str,
        default="simulation_output.mp4",
        help="Path to save output video (default: simulation_output.mp4)",
    )
    parser.add_argument(
        "--framerate",
        type=int,
        default=60,
        help="Frame rate for video recording (default: 60)",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        default=False,
        help="Open interactive passive viewer window (requires DISPLAY)",
    )

    args = parser.parse_args()

    run_simulation(
        model_path=args.model,
        duration=args.duration,
        mode=args.mode,
        render=args.render,
        video_path=args.video_path,
        framerate=args.framerate,
        gui=args.gui,
    )


if __name__ == "__main__":
    main()
